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

// User-camera viewpoints: follow/track offsets and the render-thread
// application of pending view commands.
//
// Split out of the single-file HumanControlPanel.cc; the code below is
// unchanged from that file.

namespace gz_human_sim
{
void HumanControlPanel::setViewpoint(int _index, int _viewIndex, double _distance)
{
  if (_viewIndex < 0 || _viewIndex >= kViewCount)
    return;

  // Record this as _index's current view (for ActiveViewIndex()/
  // ActiveViewDistance(), which the global viewpoint combo/distance field
  // bind to) whenever _index is a real human, regardless of which branch
  // below actually runs -- including kViewFree, so switching back to this
  // human later shows "自由視点" rather than a stale prior selection.
  if (_index >= 0 && _index < static_cast<int>(this->humans.size()))
  {
    this->humans.at(_index).viewIndex = _viewIndex;
    this->humans.at(_index).viewDistance = _distance;
    if (_index == this->activeHumanIndex)
      this->activeViewIndexChanged();
  }

  if (_viewIndex == kViewFree)
  {
    this->viewpointTarget.clear();
    std::lock_guard<std::mutex> lock(this->viewMutex);
    this->viewCommand = ViewCommand();
    this->viewCommand.pending = true;
    this->viewCommand.engage = false;
    this->viewCommand.label = "カメラを自由視点に戻しました";
    return;
  }

  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
  {
    this->SetStatus("視点変更：対象の人物がありません");
    return;
  }

  const double distance = std::clamp(_distance, 0.3, 50.0);
  // Eye level scales with distance: near views sit low, far views look
  // down a little.
  const double height = std::clamp(distance * 0.5, 0.4, 2.5);

  ViewCommand command;
  command.pending = true;
  command.engage = true;
  command.target = this->humans.at(_index).name;

  switch (_viewIndex)
  {
    case kViewFirstPerson:
    {
      // Camera right at the actor's own head, looking out at a point far
      // ahead in its own (local) frame: turns with the actor like its own
      // eyes -- the one view where that's actually wanted, unlike every
      // other case below (see ViewCommand::worldFrame). The distance box
      // doesn't apply to this view (same as GuiderRobotManager's
      // equivalent).
      //
      // eyeOffset is height ABOVE THIS MODEL'S OWN ORIGIN, not the flat
      // absolute kEyeHeight -- see kEyeHeight's own comment for why: the
      // origin itself already sits partway up the body for most models.
      const double eyeOffset = std::max(kMinEyeOffset,
          kEyeHeight - DefaultZForModelName(this->humans.at(_index).model));
      command.followOffset = {0.1, 0.0, eyeOffset};
      command.trackOffset = {3.0, 0.0, eyeOffset - 0.05};
      command.worldFrame = false;
      break;
    }
    case kViewBehind:
      command.followOffset = {-distance, 0.0, height};
      break;
    case kViewFront:
      command.followOffset = {distance, 0.0, height};
      break;
    case kViewRight:
      command.followOffset = {0.0, -distance, height};
      break;
    case kViewLeft:
      command.followOffset = {0.0, distance, height};
      break;
    case kViewTop:
      // A touch of forward offset avoids the straight-down singularity.
      command.followOffset = {std::max(0.3, distance * 0.1), 0.0, distance};
      command.trackOffset = {0.0, 0.0, 0.0};
      break;
    case kViewFrontRightUp:
    case kViewFrontLeftUp:
    {
      const double lateral = _viewIndex == kViewFrontRightUp
        ? -distance * 0.7 : distance * 0.7;
      const double up = std::clamp(distance * 0.8, 0.6, 4.0);
      command.followOffset = {distance * 0.7, lateral, up};
      break;
    }
    default:
      return;
  }

  command.label = QString("カメラ視点：%1（%2）")
      .arg(kViewpointLabels[_viewIndex],
           QString::fromStdString(this->humans.at(_index).name));

  this->viewpointTarget = command.target;
  std::lock_guard<std::mutex> lock(this->viewMutex);
  this->viewCommand = command;
}

void HumanControlPanel::resetToInitialView()
{
  this->viewpointTarget.clear();
  // The active human's combo should reflect reality: nothing is being
  // followed anymore once this takes effect.
  if (this->activeHumanIndex >= 0 &&
      this->activeHumanIndex < static_cast<int>(this->humans.size()))
  {
    this->humans.at(this->activeHumanIndex).viewIndex = kViewFree;
    this->activeViewIndexChanged();
  }
  std::lock_guard<std::mutex> lock(this->viewMutex);
  this->viewCommand = ViewCommand();
  this->viewCommand.pending = true;
  this->viewCommand.engage = false;
  this->viewCommand.resetPose = true;
  this->viewCommand.label = this->initialCameraPoseCaptured
      ? "初期の全体俯瞰視点に戻しました"
      : "初期視点をまだ取得できていません（3Dビューの読み込み待ち）";
}

void HumanControlPanel::ApplyViewpoint()
{
  ViewCommand command;
  bool hasCommand = false;
  {
    std::lock_guard<std::mutex> lock(this->viewMutex);
    hasCommand = this->viewCommand.pending;
    if (hasCommand)
      command = this->viewCommand;
  }
  if (!hasCommand)
    return;

  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!scene)
    return;

  if (!this->EnsureUserCamera(scene))
    return;

  const auto finish = [this]()
  {
    std::lock_guard<std::mutex> lock(this->viewMutex);
    this->viewCommand.pending = false;
  };

  if (!command.engage)
  {
    this->userCamera->SetFollowTarget(nullptr);
    this->userCamera->SetTrackTarget(nullptr);
    if (command.resetPose && this->initialCameraPoseCaptured)
      this->userCamera->SetWorldPose(this->initialCameraPose);
    finish();
    this->SetStatus(command.label);
    return;
  }

  // Rendering node names may carry "id::" scope prefixes; accept both the
  // plain model name and a scoped suffix match.
  gz::rendering::NodePtr target = scene->NodeByName(command.target);
  if (!target)
  {
    const std::string suffix = "::" + command.target;
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
  {
    // The model may still be spawning; retry for ~10 s of frames.
    std::lock_guard<std::mutex> lock(this->viewMutex);
    if (++this->viewCommand.retries > 600)
    {
      this->viewCommand.pending = false;
      this->SetStatus(
          QString("視点変更：%1 が見つかりません")
              .arg(QString::fromStdString(command.target)));
    }
    return;
  }

  // command.worldFrame (see the .hh) is false only for kViewFirstPerson --
  // every other view keeps its offset fixed in world axes so the human's
  // own body rotation doesn't drag the camera/background around with it.
  this->userCamera->SetFollowTarget(target, command.followOffset, command.worldFrame);
  this->userCamera->SetFollowPGain(kChasePGain);
  this->userCamera->SetTrackTarget(target, command.trackOffset, command.worldFrame);
  this->userCamera->SetTrackPGain(kChasePGain);

  finish();
  this->SetStatus(command.label);
}

bool HumanControlPanel::EnsureUserCamera(const gz::rendering::ScenePtr &_scene)
{
  // The MinimalScene plugin tags the GUI camera with this user data.
  if (!this->userCamera)
  {
    for (unsigned int i = 0; i < _scene->NodeCount(); ++i)
    {
      auto camera = std::dynamic_pointer_cast<gz::rendering::Camera>(
          _scene->NodeByIndex(i));
      if (!camera || !camera->HasUserData("user-camera"))
        continue;
      const auto data = camera->UserData("user-camera");
      const auto *flag = std::get_if<bool>(&data);
      if (flag && *flag)
      {
        this->userCamera = camera;
        break;
      }
    }
    if (!this->userCamera)
      return false;
    // First time the camera is found, it's still wherever gui.config's
    // MinimalScene <camera_pose> placed it -- nothing has engaged
    // follow/track yet at this point in a fresh launch. Save it so
    // resetToInitialView() has a real pose to snap back to.
    this->initialCameraPose = this->userCamera->WorldPose();
    this->initialCameraPoseCaptured = true;
  }

  // Kept fresh every frame for ProbeSafeSpawnPosition(), which runs on the
  // Qt thread and can't touch the scene itself -- see the cameraX/cameraY
  // members' comment for why the probe wants the camera's position.
  const auto pose = this->userCamera->WorldPose();
  {
    std::lock_guard<std::mutex> lock(this->cameraPosMutex);
    this->cameraX = pose.Pos().X();
    this->cameraY = pose.Pos().Y();
    this->cameraPosValid = true;
  }
  return true;
}
}  // namespace gz_human_sim
