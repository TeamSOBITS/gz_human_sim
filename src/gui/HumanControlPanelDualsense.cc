#include "HumanControlPanel.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <dlfcn.h>
#include <fstream>
#include <functional>
#include <memory>
#include <regex>
#include <sstream>
#include <utility>
#include <vector>

#include <QGuiApplication>
#include <QKeyEvent>
#include <QPointer>
#include <QTimer>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

// DualSense/gamepad polling only -- SDL_INIT_GAMECONTROLLER (never
// SDL_INIT_VIDEO), so this never touches windowing/GL and cannot conflict
// with the already-running Ogre2/Qt scene. See PollDualsense(). Mirrors
// guide_robot's GuiderRobotManager, which this mode is modeled on.
#include <SDL2/SDL.h>

#include "HumanControlPanelInternal.hh"

// DualSense/gamepad polling and its orbit camera.
//
// Split out of the single-file HumanControlPanel.cc; the code below is
// unchanged from that file.
//
// ---------------------------------------------------------------------------
// このファイルは unified_entity_control へ移管予定です
// ---------------------------------------------------------------------------
// 移管先: unified_entity_control/src/GamepadDevice.{hh,cc}
//         （軌道カメラの部分は同パッケージのカメラ担当へ）
//
// gz_human_sim は「人物をスポーンし、世界に配置し、状態を持つ」パッケージへ
// 戻り、「操作する」責務を持たなくなります。ゲームパッド操作は人物・ロボット・
// 生き物・動く物体を区別しない統合パッケージ側へ集約されます。
// 経緯は 構想書/gz_human_sim再設計構想.md §11 を参照。
//
// それまでは、このファイルはこのまま動き続けます。先に消さないでください。
//
// ここに新機能を足さないでください。検討済みで保留中のもの:
//   ×ボタン = ジャンプ / L2・R2 = ダッシュ・スロー / 旋回を L1・R1 へ /
//   リバインド画面
// いずれも unified_entity_control 側で実装します。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
bool HumanControlPanel::DualsenseModeEnabled() const
{
  return this->dualsenseModeState;
}

void HumanControlPanel::SetDualsenseModeEnabled(bool _enabled)
{
  if (_enabled == this->dualsenseModeState)
    return;
  this->dualsenseModeState = _enabled;
  if (_enabled)
    this->OpenDualsenseController();
  else
    this->CloseDualsenseController();
  this->dualsenseModeChanged();
}

QString HumanControlPanel::DualsenseStatusText() const
{
  return this->dualsenseStatusTextState;
}

bool HumanControlPanel::InvertCameraY() const
{
  return this->invertCameraYState;
}

void HumanControlPanel::SetInvertCameraY(bool _enabled)
{
  if (this->invertCameraYState == _enabled)
    return;
  this->invertCameraYState = _enabled;
  this->invertCameraYChanged();
}

namespace
{
/// \brief Deadzoned, normalized SDL axis reading. Sticks report
/// -32768..32767; triggers report 0..32767 (rest at 0), so the negative
/// branch below never fires for a trigger but is harmless either way.
/// Duplicated from guide_robot's GuiderRobotManager (same helper, no
/// shared header between the two independent GUI plugins).
double NormalizeAxis(Sint16 _raw)
{
  constexpr double kDeadzone = 0.10;
  const double value = _raw / (_raw < 0 ? 32768.0 : 32767.0);
  if (std::abs(value) < kDeadzone)
    return 0.0;
  const double sign = value < 0.0 ? -1.0 : 1.0;
  return sign * (std::abs(value) - kDeadzone) / (1.0 - kDeadzone);
}
}  // namespace

void HumanControlPanel::OpenDualsenseController()
{
  // See guide_robot's GuiderRobotManager::OpenDualsenseController() for why
  // this hint is needed: this plugin commonly runs inside a container with
  // no udevd of its own, and SDL's default Linux joystick backend
  // enumerates via udev, silently finding nothing even though
  // /dev/input/eventN is perfectly readable. Forcing the non-udev fallback
  // (raw /dev/input scan) works both inside such a container and on a
  // normal host. Set by its literal hint string, not a SDL_HINT_* macro --
  // this SDL2 build's <SDL_hints.h> doesn't expose one for it. Must be set
  // before SDL_InitSubSystem().
  SDL_SetHint("SDL_JOYSTICK_DISABLE_UDEV", "1");

  if (SDL_WasInit(SDL_INIT_GAMECONTROLLER) == 0 &&
      SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER) != 0)
  {
    this->dualsenseStatusTextState =
        QString("コントローラ初期化に失敗しました: %1").arg(SDL_GetError());
    this->dualsenseStatusChanged();
    return;
  }

  for (int i = 0; i < SDL_NumJoysticks(); ++i)
  {
    if (!SDL_IsGameController(i))
      continue;
    auto *controller = SDL_GameControllerOpen(i);
    if (!controller)
      continue;
    this->dualsenseController =
        reinterpret_cast<_SDL_GameController *>(controller);
    const char *name = SDL_GameControllerName(controller);
    this->dualsenseStatusTextState =
        QString("%1 接続中").arg(name ? name : "コントローラ");
    this->dualsenseStatusChanged();
    return;
  }

  this->dualsenseStatusTextState = "コントローラが見つかりません";
  this->dualsenseStatusChanged();
}

void HumanControlPanel::CloseDualsenseController()
{
  if (this->dualsenseController)
  {
    SDL_GameControllerClose(
        reinterpret_cast<SDL_GameController *>(this->dualsenseController));
    this->dualsenseController = nullptr;
  }
  if (this->dualsenseWasMoving && this->activeHumanIndex >= 0 &&
      this->activeHumanIndex < static_cast<int>(this->humans.size()))
    this->teleopStop(this->activeHumanIndex);
  this->dualsenseWasMoving = false;
  this->dualsenseOrbitTarget.clear();
  this->dualsenseLastPollValid = false;
  this->dualsenseStatusTextState = "未接続";
  this->dualsenseStatusChanged();
}

void HumanControlPanel::PollDualsense()
{
  if (!this->dualsenseModeState || !this->dualsenseController)
    return;
  if (this->activeHumanIndex < 0 ||
      this->activeHumanIndex >= static_cast<int>(this->humans.size()))
    return;

  auto *controller =
      reinterpret_cast<SDL_GameController *>(this->dualsenseController);
  SDL_GameControllerUpdate();

  const double leftX = NormalizeAxis(
      SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX));
  const double leftY = NormalizeAxis(
      SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY));
  const double rightX = NormalizeAxis(
      SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTX));
  const double rightY = NormalizeAxis(
      SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY));
  const double triggerLeft = NormalizeAxis(SDL_GameControllerGetAxis(
      controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT));
  const double triggerRight = NormalizeAxis(SDL_GameControllerGetAxis(
      controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT));

  // Humans have no non-holonomic type to branch on (unlike
  // GuiderRobotManager's sobit_mini/edu/light diff-drive robots) --
  // ActorCommandPlugin's plain Twist always accepts an independent lateral
  // component (see DirectionToTwist()'s J strafe mode / kTeleopSpeed
  // table), so the left stick alone always gives continuous-angle
  // translation, same as GuiderRobotManager's holonomic branch. Up on the
  // stick reports a negative Y in SDL's convention, hence the negation for
  // "forward = positive linear". Sign conventions match DirectionToTwist()'s
  // A/D mapping (A = left = +lateral) so DualSense mode and the keyboard
  // strafe the same way for the same stick/key side.
  const double speed = this->EffectiveSpeedMultiplier();
  const double linear = -leftY * kTeleopSpeed * speed;
  const double lateral = -leftX * kTeleopSpeed * speed;
  const double angular = (triggerLeft - triggerRight) * kTeleopTurnRate * speed;

  const bool moving = linear != 0.0 || lateral != 0.0 || angular != 0.0;
  if (moving)
  {
    ++this->humans.at(this->activeHumanIndex).teleopGeneration;
    this->teleopMove(this->activeHumanIndex, linear, lateral, angular);
    this->dualsenseWasMoving = true;
  }
  else if (this->dualsenseWasMoving)
  {
    this->teleopStop(this->activeHumanIndex);
    this->dualsenseWasMoving = false;
  }

  this->ApplyDualsenseOrbit(rightX, rightY);
}

void HumanControlPanel::ApplyDualsenseOrbit(double _rightX, double _rightY)
{
  if (!this->userCamera)
    return;

  const auto &human = this->humans.at(this->activeHumanIndex);
  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!scene)
    return;

  // Rendering node names may carry "id::" scope prefixes; accept both the
  // plain model name and a scoped suffix match (same lookup ApplyViewpoint()
  // uses).
  gz::rendering::NodePtr target = scene->NodeByName(human.name);
  if (!target)
  {
    const std::string suffix = "::" + human.name;
    for (unsigned int i = 0; i < scene->VisualCount(); ++i)
    {
      auto visual = scene->VisualByIndex(i);
      if (!visual)
        continue;
      const std::string &name = visual->Name();
      if (name.size() >= suffix.size() &&
          name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        target = visual;
        break;
      }
    }
  }
  if (!target)
    return;

  if (this->dualsenseOrbitTarget != human.name)
  {
    // Newly (re)engaged target -- start from the same offset kViewBehind
    // uses (yaw=0, pitch=0) rather than inheriting a leftover angle from
    // whatever was orbited before.
    this->dualsenseOrbitTarget = human.name;
    this->dualsenseOrbitYaw = 0.0;
    this->dualsenseOrbitPitch = 0.0;
    this->dualsenseLastPollValid = false;
  }

  const auto now = std::chrono::steady_clock::now();
  double dt = 1.0 / 30.0;
  if (this->dualsenseLastPollValid)
    dt = std::chrono::duration<double>(now - this->dualsenseLastPollTime).count();
  this->dualsenseLastPollTime = now;
  this->dualsenseLastPollValid = true;
  dt = std::clamp(dt, 0.0, 0.25);

  // rad/s at full stick deflection; pitch clamp avoids flipping over the
  // top/bottom pole (same concern kViewTop's comment flags).
  constexpr double kOrbitRate = 1.5;
  constexpr double kPitchLimit = 1.3;
  this->dualsenseOrbitYaw -= _rightX * kOrbitRate * dt;
  // SDL reports stick-up as a negative Y. Adding it therefore lowers the
  // pitch, swinging the camera down and its gaze up -- the third-person
  // default. invertCameraY flips back to the flight-sim sense; see the
  // Q_PROPERTY comment.
  const double pitchSign = this->invertCameraYState ? -1.0 : 1.0;
  this->dualsenseOrbitPitch = std::clamp(
      this->dualsenseOrbitPitch + pitchSign * _rightY * kOrbitRate * dt,
      -kPitchLimit, kPitchLimit);

  // Spherical offset around the human's local frame (worldFrame = false,
  // matching ApplyViewpoint()'s kViewFirstPerson convention): yaw=0/pitch=0
  // reproduces kViewBehind's own offset exactly, so DualSense mode's
  // starting view matches the existing "後方追従" viewpoint.
  const double cosPitch = std::cos(this->dualsenseOrbitPitch);
  const gz::math::Vector3d offset(
      -this->dualsenseOrbitDistance * cosPitch *
          std::cos(this->dualsenseOrbitYaw),
      -this->dualsenseOrbitDistance * cosPitch *
          std::sin(this->dualsenseOrbitYaw),
      0.5 + this->dualsenseOrbitDistance *
          std::sin(this->dualsenseOrbitPitch));

  this->userCamera->SetFollowTarget(target, offset, false);
  this->userCamera->SetFollowPGain(kChasePGain);
  this->userCamera->SetTrackTarget(target, {0.0, 0.0, 0.6}, false);
  this->userCamera->SetTrackPGain(kChasePGain);
}
}  // namespace gz_human_sim
