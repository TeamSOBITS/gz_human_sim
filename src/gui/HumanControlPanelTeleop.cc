// HumanControlPanelTeleop.cc -- テレオペ。キー/パッドから Twist と経路指令を作って publish する。
// DualSense のポーリングと軌道カメラの回し込みもここ。
//
// 3808行あった単一の HumanControlPanel.cc を責務ごとに割ったもの
// （構想書/これからやるやつ/gz_human_sim再設計構想.md §13 段階2）。
// **中身は移動しただけで、振る舞いは変えていない。**
// クラス宣言は HumanControlPanel.hh、ファイル間で共有する表と定数は
// HumanControlPanelInternal.hh にある。

#include "HumanControlPanel.hh"
#include "HumanControlPanelInternal.hh"

// include は分割前の単一ファイルと同じものを揃えてある。責務ごとに
// 削るのは「動くこと」を確認してからでよい（段階2は移動だけ）。
#include "GuiderDeleteRecoverEvent.hh"
#include "GuiderViewpointRequestEvent.hh"

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
// gz/plugin/Register.hh はここでは include しない。**翻訳単位ごとに
// GzPluginHook を定義する**ので、複数の .cc から include すると
// リンクが multiple definition で落ちる。GZ_ADD_PLUGIN を持つ
// HumanControlPanel.cc の1本だけが include する。
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

namespace gz_human_sim
{
double HumanControlPanel::JumpHeight() const
{
  return this->jumpHeightState;
}

double HumanControlPanel::SpeedMultiplier() const
{
  return this->speedMultiplierState;
}

void HumanControlPanel::setJumpHeight(double _height)
{
  const double clamped = std::clamp(_height, kJumpHeightMin, kJumpHeightMax);
  if (std::abs(clamped - this->jumpHeightState) < 1e-9)
    return;
  this->jumpHeightState = clamped;
  this->jumpHeightChanged();
}

void HumanControlPanel::setSpeedMultiplier(double _value)
{
  const double clamped = std::clamp(_value, kSpeedMultiplierMin, kSpeedMultiplierMax);
  if (std::abs(clamped - this->speedMultiplierState) < 1e-9)
    return;
  this->speedMultiplierState = clamped;
  this->speedMultiplierChanged();
}

double HumanControlPanel::EffectiveSpeedMultiplier() const
{
  const double factor = this->ctrlHeldState
      ? kSlowFactor : (this->shiftHeldState ? kRunFactor : 1.0);
  return this->speedMultiplierState * factor;
}

void HumanControlPanel::teleopJump(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.jumpPublisher.Valid())
    return;

  gz::msgs::Double message;
  message.set_data(this->jumpHeightState);
  human.jumpPublisher.Publish(message);
}

void HumanControlPanel::teleopRotate(int _index, bool _counterClockwise)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  // Pure-angular Twist -- no linear/lateral -- so the actor spins on the
  // spot rather than walking. Sign matches DirectionToTwist()'s A/D
  // convention (A: +lateral -> +yaw -> counterclockwise; D: -lateral ->
  // -yaw -> clockwise), same as ApplyHeldDirectionKeys()'s steering sign.
  const double angular =
      (_counterClockwise ? kTeleopTurnRate : -kTeleopTurnRate) *
      this->EffectiveSpeedMultiplier();
  ++this->humans.at(_index).teleopGeneration;
  this->teleopMove(_index, 0.0, 0.0, angular);
}

void HumanControlPanel::teleopMove(
    int _index, double _linear, double _lateral, double _angular)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;

  gz::msgs::Twist message;
  message.mutable_linear()->set_x(_linear);
  message.mutable_linear()->set_y(_lateral);
  message.mutable_angular()->set_z(_angular);
  human.velocityPublisher.Publish(message);
}

void HumanControlPanel::teleopStop(int _index)
{
  if (_index >= 0 && _index < static_cast<int>(this->humans.size()))
    ++this->humans.at(_index).teleopGeneration;
  this->teleopMove(_index, 0.0, 0.0, 0.0);
}

void HumanControlPanel::PublishTurnToFace(int _index, double _speed, double _targetHeadingRad)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;

  // angular.x doubles as the turn-to-face flag ActorCommandPlugin::
  // VelocityCallback() checks -- see the comment there. angular.z here is
  // an ABSOLUTE world heading (radians), not a turn rate.
  gz::msgs::Twist message;
  message.mutable_linear()->set_x(_speed);
  message.mutable_angular()->set_x(1.0);
  message.mutable_angular()->set_z(_targetHeadingRad);
  human.velocityPublisher.Publish(message);
}

void HumanControlPanel::teleopDirection(
    int _index, const QString &_direction, bool _turnToFace)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;

  const std::string direction = _direction.toStdString();
  double linear = 0.0;
  double lateral = 0.0;
  double angular = 0.0;
  if (!DirectionToTwist(direction, linear, lateral, angular))
    return;
  const double speed = this->EffectiveSpeedMultiplier();

  if (_turnToFace)
  {
    // Steer toward this direction's ABSOLUTE world heading (fixed per-key
    // angle table -- see DirectionHeadingAngle()) while continuously
    // walking forward, instead of the old stop-in-place-then-walk two
    // phases: ActorCommandPlugin::PreUpdate() now does the smooth
    // in-motion arc itself, every tick, for as long as this Twist keeps
    // being the latest one received. Because the target is an ABSOLUTE
    // heading rather than a turn relative to wherever the actor currently
    // happens to be facing, pressing the same direction key again (or a
    // live speed change re-publishing this, see RefreshHeldMovementSpeed())
    // is always idempotent -- it never stacks another 90 degrees on top,
    // unlike the old relative-angle design. magnitude is kTeleopSpeed for
    // every entry in DirectionToTwist()'s table by construction (see its
    // comment), so this is exactly the walking speed regardless of which
    // key it was.
    ++this->humans.at(_index).teleopGeneration;
    this->PublishTurnToFace(
        _index, kTeleopSpeed * speed, DirectionHeadingAngle(direction));
    return;
  }

  linear *= speed;
  lateral *= speed;
  ++this->humans.at(_index).teleopGeneration;
  this->teleopMove(_index, linear, lateral, angular);
}

void HumanControlPanel::PressDirectionKey(const std::string &_direction)
{
  auto &keys = this->heldDirectionKeys;
  if (std::find(keys.begin(), keys.end(), _direction) == keys.end())
  {
    // A 3rd simultaneous direction key has no defined combo meaning here;
    // ignore it rather than guess (the first two keep driving the human).
    if (keys.size() >= 2)
      return;
    keys.push_back(_direction);
  }
  // Solo key, not in J strafe-mode: turn the body to face this direction,
  // then walk -- teleopDirection()'s turnToFace path already handles
  // "already facing that way" (W) as an instant walk, no visible turn
  // needed. This bypasses ApplyHeldDirectionKeys()'s pure-strafe Twist
  // entirely for the solo case; that function is only still used below
  // for the 2-key curving combo, or here in strafe mode.
  if (keys.size() == 1 && !this->jHeldState)
  {
    this->teleopDirection(
        this->activeHumanIndex, QString::fromStdString(_direction), true);
    return;
  }
  this->ApplyHeldDirectionKeys();
}

void HumanControlPanel::ReleaseDirectionKey(const std::string &_direction)
{
  auto &keys = this->heldDirectionKeys;
  keys.erase(std::remove(keys.begin(), keys.end(), _direction), keys.end());
  if (keys.empty())
  {
    this->teleopStop(this->activeHumanIndex);
  }
  else if (keys.size() == 1 && !this->jHeldState)
  {
    // Back down to a single held key -- same turn-to-face-then-walk
    // restart as PressDirectionKey()'s solo-key branch (the 2-key combo
    // we may have just been doing can leave the body facing a heading
    // that doesn't match this key's own).
    this->teleopDirection(
        this->activeHumanIndex, QString::fromStdString(keys.front()), true);
  }
  else
  {
    this->ApplyHeldDirectionKeys();
  }
}

void HumanControlPanel::ApplyHeldDirectionKeys()
{
  const auto &keys = this->heldDirectionKeys;
  if (keys.empty())
    return;

  const int index = this->activeHumanIndex;
  if (index < 0 || index >= static_cast<int>(this->humans.size()))
    return;

  if (keys.size() == 2 && !this->jHeldState)
  {
    // Curving combo: steer toward the SECOND (most recently pressed)
    // key's absolute heading via the same turn-to-face mechanism
    // teleopDirection() uses for a solo key, instead of publishing a
    // fixed turn RATE that never got zeroed back out -- that old version
    // kept rotating for as long as both keys stayed held, so holding e.g.
    // W then D never settled into moving right, it just spiralled
    // forever. This version curves in and then walks straight the moment
    // it reaches the second key's heading, exactly like a solo
    // turn-to-face key. Releasing either key still falls through to
    // ReleaseDirectionKey()'s solo-key retarget, unchanged.
    ++this->humans.at(index).teleopGeneration;
    this->PublishTurnToFace(index, kTeleopSpeed * this->EffectiveSpeedMultiplier(),
        DirectionHeadingAngle(keys.back()));
    return;
  }

  // J strafe mode (1 or 2 keys -- a 2nd key's direction is ignored here,
  // strafe mode never turns): the original body-relative Twist, angular
  // always 0.
  double linear = 0.0, lateral = 0.0, angular = 0.0;
  DirectionToTwist(keys.front(), linear, lateral, angular);
  const double speed = this->EffectiveSpeedMultiplier();
  linear *= speed;
  lateral *= speed;
  ++this->humans.at(index).teleopGeneration;
  this->teleopMove(index, linear, lateral, angular);
}

void HumanControlPanel::RefreshHeldMovementSpeed()
{
  const int index = this->activeHumanIndex;
  if (index < 0 || index >= static_cast<int>(this->humans.size()))
    return;
  auto &keys = this->heldDirectionKeys;
  if (keys.empty())
    return;

  if (keys.size() == 2 || this->jHeldState)
  {
    // 2-key curve combo, or solo J strafe -- both are the
    // ApplyHeldDirectionKeys() Twist, always safe/idempotent to just
    // recompute and republish, including switching INTO strafe mode
    // mid-hold (J newly held while a solo key was already in turn-to-face
    // mode below).
    this->ApplyHeldDirectionKeys();
    return;
  }

  // Solo key, turn-to-face mode: always safe/idempotent to just reissue
  // at the current speed, whether or not the actor has finished turning
  // yet -- teleopDirection()'s target is an absolute world heading, not a
  // turn relative to wherever the actor currently is facing, so this
  // never adds extra rotation, it just updates the walking speed (and,
  // if J was just released, switches back into turn-to-face mode from
  // strafe mode).
  this->teleopDirection(index, QString::fromStdString(keys.front()), true);
}

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

  // **左スティックは「その向きを向いて歩け」**。キーボードの W/A/D/X が
  // 通っている PublishTurnToFace() と同じ経路で、ActorCommandPlugin が
  // 歩きながらの向き変えを自分で作る。
  //
  // 以前はここが素のストレイフ（linear/lateral/angular をそのまま送る）
  // だった。キーボードだけが turn-to-face を通っていたので、**同じ人物が
  // 入力によって別物のように動いていた**: パッドだと前を向いたまま
  // 横歩きし、向きを変えるには L2/R2 を押す必要があった（ムーンウォーク）。
  // guide_robot の GuiderPadController も同じ理由で直してある。
  //
  // 方位の求め方: スティック上 = world +X（絶対world方位。カメラ相対では
  // ない）。SDL はスティック上を負の Y で返すので -leftY、方位の回転向きは
  // 画面方位と逆なので atan2(x, -y) の符号を反転する。
  // guide_robot 側の同じ変換と一致させてあること。
  const double speed = this->EffectiveSpeedMultiplier();
  const double magnitude = std::hypot(leftX, leftY);
  const double angular = (triggerLeft - triggerRight) * kTeleopTurnRate * speed;

  // NormalizeAxis() が既にデッドゾーンの中を 0 に潰しているので、
  // ここでの閾値は「厳密に 0 でない」でよい。
  if (magnitude > 0.0)
  {
    const double bearing = std::atan2(leftX, -leftY);
    ++this->humans.at(this->activeHumanIndex).teleopGeneration;
    this->PublishTurnToFace(this->activeHumanIndex,
        std::min(magnitude, 1.0) * kTeleopSpeed * speed, -bearing);
    this->dualsenseWasMoving = true;
  }
  else if (angular != 0.0)
  {
    // スティックが中央のときだけ L2/R2 をその場旋回として残す。歩かずに
    // 向きだけ変えたい場面（ポーズ・目線合わせ）が turn-to-face では
    // できないため。
    ++this->humans.at(this->activeHumanIndex).teleopGeneration;
    this->teleopMove(this->activeHumanIndex, 0.0, 0.0, angular);
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
