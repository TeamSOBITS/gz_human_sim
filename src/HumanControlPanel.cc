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
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

namespace gz_human_sim
{
// Index order backs both the QML ComboBox and defaultName()/isCustomHuman().
// person_walking (formerly here) was dropped: it's the same "Mingfei"
// generic-actor mesh as walking_actor (compare model.config authorship),
// just fetched at spawn time from a remote Fuel URL instead of the copy
// walking_actor already vendors locally, and with no ActorCommandPlugin of
// its own -- a strictly worse duplicate of walking_actor, not a second
// distinct avatar.
static const char *const kHumanModels[] = {
  "walking_actor", "DoctorFemaleWalk",
  "person_standing", "custom_human",
};
static const char *const kHumanModelDescriptions[] = {
  "汎用の歩行アクター。矢印パッド/キーボードで移動可",
  "女性医師の歩行スキン。矢印パッド/キーボードで移動可",
  "静止した立ち姿。移動不可",
  "ポーズ指定できる静止人物。移動不可",
};
// Suggested spawn Z per model, shown as the QML spawn form's default when
// that model is selected. DoctorFemaleWalk's own model.sdf already poses
// the actor at z=0 (its mesh origin sits at ground level, unlike
// walking_actor's, which needs the +0.86 the other three models default
// to) -- keep in sync with kHumanModels above.
static const double kHumanModelDefaultZ[] = {
  1.0, 0.0, 1.0, 1.0,
};
static constexpr int kHumanModelCount =
    static_cast<int>(sizeof(kHumanModels) / sizeof(kHumanModels[0]));
static constexpr int kCustomHumanIndex = 3;

// Raw ActorCommandPlugin follow_mode values the GUI exposes, index-matched
// with FollowModeLabels()'s QML-facing Japanese labels. "velocity" (ignore
// any cmd_path, teleop-only) is deliberately left out here -- it's nearly
// indistinguishable from "auto" for anyone who never sends this human a
// path, so it only added a confusing third choice. It's still available at
// the launch-argument level (`ros2 launch gz_human_sim spawn_human.launch.py
// follow_mode:=velocity ...`) for the rare case that actually wants it.
static const char *const kFollowModeValues[] = {"auto", "path"};
static constexpr int kFollowModeCount =
    static_cast<int>(sizeof(kFollowModeValues) / sizeof(kFollowModeValues[0]));

// Path templates sendPathTemplate() can generate, index-matched with
// PathTemplateLabels(); keep this in sync with scripts/path_template.py's
// --shape choices (that script is the standalone-CLI version of the same
// shapes; this one publishes straight to gz-transport for the GUI button
// instead of going through ROS + a subprocess).
enum PathTemplateIndex
{
  kPathTemplateCircle = 0,
  kPathTemplateSquare,
  kPathTemplateCount,
};
static const char *const kPathTemplateLabels[kPathTemplateCount] = {"円", "四角"};

// Spawn position auto-offset: fans consecutive default-position spawns out
// in a grid instead of stacking them on top of each other at (0, 0).
static constexpr double kSpawnGridSpacing = 1.2;
static constexpr int kSpawnGridColumns = 4;

// ProbeSafeSpawnPosition()/CheckProbeSettle(): how many grid slots to try
// before giving up and spawning at the original position regardless; how
// fast the probe walks in from kSpawnGridSpacing south of the candidate
// (same order of magnitude as normal teleop walking speed); how long it
// gets to complete that walk-in before its pose is trusted (must clear
// kSpawnGridSpacing / kSpawnProbeSpeed = 1.2 s at the values below, plus
// margin for acceleration and the discovery-race republishes just before
// it); and how much of that kSpawnGridSpacing walk-in it's allowed to have
// come up short by (measured from where it STARTED, not the candidate --
// see CheckProbeSettle()'s comment for why) and still count as "made it",
// vs. "got stopped partway by something".
static constexpr int kSpawnSafetyMaxAttempts = 8;
static constexpr double kSpawnProbeSpeed = 1.0;
static constexpr int kSpawnSafetySettleMs = 2200;
static constexpr double kSpawnSafetyDisplacementMeters = 0.3;

// PollProbeRemoval(): how often to re-check whether a just-removed probe
// is actually gone (QueryEntityExists()), and how many times to check
// before giving up and spawning the real collision-body companion anyway.
// 150ms * 20 = 3s, comfortably more than removal should ever realistically
// take -- this is a safety net for a stuck removal, not the expected path.
static constexpr int kProbeRemovalPollIntervalMs = 150;
static constexpr int kProbeRemovalMaxAttempts = 20;

// Entity-existence polling: gz-sim's create/remove services ack the
// request, not the outcome, so spawn/removal are confirmed by polling
// /world/<w>/state instead of trusting the ack. 20 * 500ms = 10s.
static constexpr int kEntityPollIntervalMs = 500;
static constexpr int kEntityPollMaxAttempts = 20;
static constexpr unsigned int kStateQueryTimeoutMs = 800u;

// ApplyCollisionVisibility(): transparency values (0 = opaque, 1 = fully
// transparent, see gz::rendering::Material::SetTransparency()) for the
// collision-body debug capsule's two states. Hidden isn't 1.0 outright --
// slightly short of fully invisible reads as more intentional ("this is
// dimmed") than a value that could be mistaken for the visual having
// failed to apply at all.
static constexpr float kCollisionHiddenTransparency = 0.9f;
static constexpr float kCollisionVisibleTransparency = 0.25f;
// Same idea as kSpawnSafetyMaxAttempts's 8-attempt cap, but per-frame
// instead of per-spawn-attempt: gives up searching the render scene for a
// human's collision-body visual after this many failed Render-event
// ticks (the companion model may take a few frames to actually appear
// after being spawned/respawned).
static constexpr int kCollisionVisualMaxRetries = 600;

static bool IsActorIndex(int _index)
{
  // walking_actor, DoctorFemaleWalk -- keep in sync with kHumanModels above.
  return _index >= 0 && _index < 2;
}

// Only walking_actor ships sit_down/sitting/stand_up meshes (see
// models/walking_actor/meshes/) -- DoctorFemaleWalk has a single walk-only
// mesh, so the sit feature (K key / Space lock / QML toggle) is narrower
// than IsActorIndex() above.
static bool IsSitCapableIndex(int _index)
{
  return _index == 0;
}

// Movement layout (Q W E / A _ D / Z X C, center vacated -- S and N are
// held modifier / stop keys now, not directions), shared by the on-screen
// pad and the keyboard shortcuts in eventFilter(). Diagonal components are
// scaled by 1/sqrt(2) so diagonal moves aren't faster than straight ones.
// X is the odd one out: rather than a pure backward strafe (which would
// moonwalk the actor away without it ever turning around), it combines
// backward motion with a turn rate so the actor visibly spins to face the
// direction it's retreating toward -- but only when _turnToFace is
// requested (S held); a plain key press is always a pure strafe, including
// X (straight-back strafe, i.e. moonwalking), same as the other 7
// directions.
static constexpr double kTeleopSpeed = 1.0;
static constexpr double kTeleopDiagonal = kTeleopSpeed * 0.70710678;
static constexpr double kTeleopTurnRate = 2.5;

// Jump height (meters) range the QML slider / setJumpHeight() clamp to.
static constexpr double kJumpHeightMin = 0.05;
static constexpr double kJumpHeightMax = 1.5;

// Baseline speed-multiplier range setSpeedMultiplier() (the QML slider)
// clamps to.
static constexpr double kSpeedMultiplierMin = 0.1;
static constexpr double kSpeedMultiplierMax = 4.0;

// Extra factor EffectiveSpeedMultiplier() applies on top of the baseline
// speedMultiplierState while Ctrl (slow walk) or Shift (run) is held --
// see eventFilter()'s Key_Control/Key_Shift tracking and teleopDirection()/
// ApplyHeldDirectionKeys(), which are the only two places that call it.
static constexpr double kSlowFactor = 0.5;
static constexpr double kRunFactor = 2.0;

// Every entry's (linear, lateral) magnitude is kTeleopSpeed by construction
// (cardinal directions put it all on one axis; diagonals split it
// kTeleopDiagonal/kTeleopDiagonal, which is kTeleopSpeed/sqrt(2) per axis,
// i.e. kTeleopSpeed once recombined) -- teleopDirection()'s _turnToFace
// path relies on that to reuse this table for "how far to turn, then walk
// forward at this same speed" instead of a separate direction-angle table.
static bool DirectionToTwist(
    const std::string &_direction, double &_linear, double &_lateral,
    double &_angular)
{
  _linear = 0.0;
  _lateral = 0.0;
  _angular = 0.0;
  if (_direction == "W") { _linear = kTeleopSpeed; }
  else if (_direction == "A") { _lateral = kTeleopSpeed; }
  else if (_direction == "D") { _lateral = -kTeleopSpeed; }
  else if (_direction == "X") { _linear = -kTeleopSpeed; }
  else if (_direction == "Q") { _linear = kTeleopDiagonal; _lateral = kTeleopDiagonal; }
  else if (_direction == "E") { _linear = kTeleopDiagonal; _lateral = -kTeleopDiagonal; }
  else if (_direction == "Z") { _linear = -kTeleopDiagonal; _lateral = kTeleopDiagonal; }
  else if (_direction == "C") { _linear = -kTeleopDiagonal; _lateral = -kTeleopDiagonal; }
  else { return false; }
  return true;
}

// The 4 cardinal keys eligible for steering-combo tracking (see
// PressDirectionKey()/ApplyHeldDirectionKeys()). Diagonals (Q/E/Z/C) stay
// on the old single-shot immediate path.
static bool IsSteerableDirection(const std::string &_direction)
{
  return _direction == "W" || _direction == "A" ||
      _direction == "D" || _direction == "X";
}

// atan2(lateral, linear) for a direction's DirectionToTwist() vector --
// same convention teleopDirection()'s turnToFace path already uses for
// headingOffset, reused here so the turn direction sign logic in
// ApplyHeldDirectionKeys() matches it exactly.
static double DirectionHeadingAngle(const std::string &_direction)
{
  double linear = 0.0, lateral = 0.0, angular = 0.0;
  DirectionToTwist(_direction, linear, lateral, angular);
  return std::atan2(lateral, linear);
}

// Keyboard shortcuts only fire when focus isn't on a text-editable QML item
// (name field, x/y/z/yaw fields, ...) -- otherwise typing "human1" into the
// name box would also drive the active human around.
static bool IsTextEditFocused()
{
  auto *focusObject = qGuiApp ? qGuiApp->focusObject() : nullptr;
  if (!focusObject)
    return false;
  const QString className = focusObject->metaObject()->className();
  return className.contains("TextInput") || className.contains("TextEdit");
}

// Keep in sync with setViewpoint()'s switch below and the QML ComboBox.
enum ViewIndex
{
  kViewFree = 0,
  kViewFirstPerson,
  kViewBehind,
  kViewFront,
  kViewRight,
  kViewLeft,
  kViewTop,
  kViewFrontRightUp,
  kViewFrontLeftUp,
  kViewCount,
};
static const char *const kViewpointLabels[kViewCount] = {
  "自由視点", "一人称（本人視点）", "後方追従", "前方から", "右横から", "左横から",
  "俯瞰（真上）", "右奥上から（斜め上）", "左奥上から（斜め上）",
};
// Eye height for kViewFirstPerson -- walking_actor/DoctorFemaleWalk are
// human-scale (roughly 1.6-1.8m), unlike guide_robot's robots, so this
// differs from GuiderRobotManager::setViewpoint()'s equivalent value.
static constexpr double kEyeHeight = 1.6;

// How eagerly the chase camera (SetFollowTarget/SetTrackTarget's pgain)
// catches up to the human's current world position each frame. The
// background-swings-when-the-body-turns problem this used to be tuned for
// is now fixed properly via ViewCommand::worldFrame instead (see the .hh
// and ApplyViewpoint()) -- the human's own rotation no longer moves the
// camera at all, so this only smooths the camera's translation as the
// human actually walks somewhere. Kept a bit below the original 0.35 for
// a gentle trailing feel without being sluggish to snap onto a
// freshly-selected human.
static constexpr double kChasePGain = 0.25;

// dladdr anchor: resolves to the shared library this code was loaded
// from, so LoadConfig() can find human_pose_presets.yaml under this
// package's own share/ directory without hardcoding an install prefix.
static void ThisLibraryAnchor()
{
}

HumanControlPanel::HumanControlPanel()
  : gz::gui::Plugin()
{
}

HumanControlPanel::~HumanControlPanel()
{
  for (auto &human : this->humans)
    this->TerminateProcessGroup(human.process);
}

void HumanControlPanel::LoadConfig(const tinyxml2::XMLElement *)
{
  this->title = "Human Control";

  // Read the human_pose_presets.yaml top-level keys with a plain regex
  // scan rather than pulling in a YAML library just for this: the file's
  // shape (2-space-indented "name:" keys under pose_presets:) is stable
  // and already relied on elsewhere (spawn_human.launch.py's Python side
  // does load it with PyYAML; this is only the GUI's picker list).
  std::string packagePrefix;
  {
    // This library installs to <prefix>/lib/gz_human_sim/gz-gui/.
    Dl_info info;
    if (dladdr(reinterpret_cast<void *>(&ThisLibraryAnchor), &info) && info.dli_fname)
    {
      std::string libPath(info.dli_fname);
      const auto libDir = libPath.find_last_of('/');
      if (libDir != std::string::npos)
        packagePrefix = libPath.substr(0, libDir) + "/../../..";
    }
  }
  const std::string presetsPath =
      packagePrefix + "/share/gz_human_sim/config/human_pose_presets.yaml";
  std::ifstream presetsFile(presetsPath);
  if (presetsFile)
  {
    std::regex keyPattern(R"(^  ([A-Za-z0-9_]+):\s*$)");
    std::string line;
    while (std::getline(presetsFile, line))
    {
      std::smatch match;
      if (std::regex_match(line, match, keyPattern))
        this->posePresetList << QString::fromStdString(match[1].str());
    }
  }
  if (this->posePresetList.isEmpty())
    this->posePresetList << "cross_arms";

  // ProbeSafeSpawnPosition()'s throwaway probes reuse this template
  // verbatim (each attempt only substitutes a fresh model name) -- see
  // its own comment for why a probe is just this same collision body.
  const std::string collisionBodyPath =
      packagePrefix + "/share/gz_human_sim/models/human_collision_body/model.sdf";
  std::ifstream collisionBodyFile(collisionBodyPath);
  if (collisionBodyFile)
  {
    std::ostringstream buffer;
    buffer << collisionBodyFile.rdbuf();
    this->collisionBodyTemplate = buffer.str();
  }

  // Viewpoint commands touch the Ogre2 scene, which is only safe from the
  // render thread; watch Render events like GuiderRobotManager does. Render
  // events are only ever sent to MainWindow, so a filter there is enough
  // for that.
  auto *mainWindow = gz::gui::App()->findChild<gz::gui::MainWindow *>();
  if (mainWindow)
    mainWindow->installEventFilter(this);

  // QWEASDZXC keyboard teleop needs real key events, which target whichever
  // QQuickItem currently has focus (e.g. the 3D scene), not MainWindow --
  // installing on the Application object itself instead catches every
  // event application-wide (QApplication is the one QObject whose
  // installEventFilter() acts globally rather than per-object), so the
  // shortcuts work no matter which panel/item is focused.
  gz::gui::App()->installEventFilter(this);

  this->DiscoverWorld();
}

bool HumanControlPanel::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->ApplyViewpoint();
    this->ApplyCollisionVisibility();
  }
  else if (_event->type() == QEvent::KeyPress || _event->type() == QEvent::KeyRelease)
  {
    auto *keyEvent = static_cast<QKeyEvent *>(_event);
    const bool pressed = _event->type() == QEvent::KeyPress;
    // Shift/Ctrl/S are tracked as plain held-state (not read off
    // modifiers()/a per-key switch at the moment a direction key fires) so
    // QML buttons can also read shiftHeld/ctrlHeld/sHeld/jHeld live for
    // mouse-driven clicks, and so S -- not a real Qt modifier -- can be
    // tracked the same way as Shift/Ctrl. Not gated on IsTextEditFocused()
    // (unlike the direction dispatch below) so these stay accurate even
    // while a name/x/y/z field has focus elsewhere in the panel.
    if (keyEvent->key() == Qt::Key_Shift && !keyEvent->isAutoRepeat() &&
        pressed != this->shiftHeldState)
    {
      this->shiftHeldState = pressed;
      this->shiftHeldChanged();
      // Live speed/mode react to Shift toggling mid-hold -- e.g. holding W
      // and THEN pressing Shift starts running immediately, no need to
      // release and re-press W. See RefreshHeldMovementSpeed().
      this->RefreshHeldMovementSpeed();
    }
    else if (keyEvent->key() == Qt::Key_Control && !keyEvent->isAutoRepeat() &&
        pressed != this->ctrlHeldState)
    {
      this->ctrlHeldState = pressed;
      this->ctrlHeldChanged();
      this->RefreshHeldMovementSpeed();
    }
    else if (keyEvent->key() == Qt::Key_S && !keyEvent->isAutoRepeat() &&
        pressed != this->sHeldState)
    {
      this->sHeldState = pressed;
      this->sHeldChanged();
    }
    else if (keyEvent->key() == Qt::Key_J && !keyEvent->isAutoRepeat() &&
        pressed != this->jHeldState)
    {
      this->jHeldState = pressed;
      this->jHeldChanged();
      // J is the strafe-mode modifier -- live-switch a currently-held key
      // between turn-to-face and strafe the same way Shift/Ctrl live-swap
      // speed above.
      this->RefreshHeldMovementSpeed();
    }
    else if (keyEvent->key() == Qt::Key_K && !keyEvent->isAutoRepeat() &&
        pressed != this->kHeldState)
    {
      // Hold-to-sit: K down publishes "sit", K up publishes "stand" again
      // (unless the active human's sit is currently locked, see Key_Space
      // below/UpdateSitIntent()). Tracked as plain held-state, same as
      // Shift/Ctrl/S/J above, so a QML button could read kHeld too.
      this->kHeldState = pressed;
      this->kHeldChanged();
      this->UpdateSitIntent(this->activeHumanIndex);
    }
    if (!keyEvent->isAutoRepeat() && !IsTextEditFocused())
    {
      if (pressed && keyEvent->key() == Qt::Key_Space)
      {
        // Space toggles the active human's sit lock -- on its own (K never
        // held) this is enough to sit the human down and keep them seated,
        // since UpdateSitIntent() publishes "locked OR K held". Pressing it
        // again releases the lock, same Q_INVOKABLE the QML sit button uses.
        this->toggleSitLock(this->activeHumanIndex);
      }
      else if (pressed && keyEvent->key() >= Qt::Key_1 && keyEvent->key() <= Qt::Key_9)
      {
        this->setActiveHuman(keyEvent->key() - Qt::Key_1);
      }
      else if (pressed &&
          (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter))
      {
        // Jump in place, or -- if a direction key is still held -- while
        // continuing to move that way (teleopJump() only adds a Z arc, the
        // held key(s)' horizontal Twist keeps applying unchanged).
        this->teleopJump(this->activeHumanIndex);
      }
      else if (pressed && keyEvent->key() == Qt::Key_N)
      {
        // Immediate stop -- this is S's old job; S is now the
        // turn-to-face modifier (see sHeldState above/teleopDirection()).
        this->heldDirectionKeys.clear();
        this->teleopStop(this->activeHumanIndex);
      }
      else
      {
        std::string direction;
        switch (keyEvent->key())
        {
          case Qt::Key_Q: direction = "Q"; break;
          case Qt::Key_W: direction = "W"; break;
          case Qt::Key_E: direction = "E"; break;
          case Qt::Key_A: direction = "A"; break;
          case Qt::Key_D: direction = "D"; break;
          case Qt::Key_Z: direction = "Z"; break;
          case Qt::Key_X: direction = "X"; break;
          case Qt::Key_C: direction = "C"; break;
          default: break;
        }
        if (!direction.empty())
        {
          if (this->sHeldState && (direction == "A" || direction == "D"))
          {
            // S+A / S+D: spin in place (A = counterclockwise, D =
            // clockwise) instead of walking -- see teleopRotate(). This
            // is S's whole remaining job now that plain movement below
            // always turns to face where it's going anyway.
            this->heldDirectionKeys.clear();
            if (pressed)
              this->teleopRotate(this->activeHumanIndex, direction == "A");
            else
              this->teleopStop(this->activeHumanIndex);
          }
          else if (IsSteerableDirection(direction))
          {
            // W/A/D/X go through the held-key tracker: PressDirectionKey()/
            // ReleaseDirectionKey() decide there whether a solo key turns
            // to face and walks, or (J strafe mode, or a 2nd key joining
            // for the curving combo) falls through to the old strafe/curve
            // Twist via ApplyHeldDirectionKeys().
            if (pressed)
              this->PressDirectionKey(direction);
            else
              this->ReleaseDirectionKey(direction);
          }
          else if (pressed)
          {
            // Diagonals (Q/E/Z/C): always single-shot immediate. Turn to
            // face and walk by default, same as W/A/D/X; J held switches
            // to the original no-turn strafe (see PressDirectionKey()'s
            // matching override).
            this->heldDirectionKeys.clear();
            const bool turnToFace = !this->jHeldState;
            this->teleopDirection(
                this->activeHumanIndex, QString::fromStdString(direction), turnToFace);
          }
          else
          {
            this->heldDirectionKeys.clear();
            this->teleopStop(this->activeHumanIndex);
          }
        }
      }
    }
  }
  return QObject::eventFilter(_obj, _event);
}

QStringList HumanControlPanel::ViewpointLabels() const
{
  QStringList result;
  for (int i = 0; i < kViewCount; ++i)
    result << kViewpointLabels[i];
  return result;
}

void HumanControlPanel::DiscoverWorld()
{
  gz::msgs::StringMsg_V worlds;
  bool result{false};
  const bool executed = this->node.Request("/gazebo/worlds", 500u, worlds, result);

  if (executed && result && worlds.data_size() > 0)
  {
    this->worldName = worlds.data(0);
    this->SetStatus(
        QString::fromStdString("接続済み · world: " + this->worldName));
    // CheckProbeSettle() needs live poses of the throwaway spawn-safety
    // probes (see ProbeSafeSpawnPosition()); this is the same
    // dynamic_pose/info topic guide_robot's GuiderRobotManager already
    // subscribes to for its own pose cache.
    if (!this->poseSubscribed)
    {
      this->poseSubscribed = this->node.Subscribe(
          "/world/" + this->worldName + "/dynamic_pose/info",
          &HumanControlPanel::OnPoseInfo, this);
    }
    return;
  }

  QTimer::singleShot(500, this, &HumanControlPanel::DiscoverWorld);
}

void HumanControlPanel::OnPoseInfo(const gz::msgs::Pose_V &_message)
{
  std::lock_guard<std::mutex> lock(this->poseMutex);
  for (int i = 0; i < _message.pose_size(); ++i)
  {
    const auto &pose = _message.pose(i);
    auto &cached = this->poses[pose.name()];
    cached.x = pose.position().x();
    cached.y = pose.position().y();
    cached.z = pose.position().z();
    const auto &q = pose.orientation();
    cached.yaw = std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    cached.valid = true;

    // See activeProbes' own comment: stop a spawn-safety probe as soon as
    // it reaches its candidate point instead of letting it keep walking
    // past it until CheckProbeSettle()'s timer deletes it.
    const auto probeIt = this->activeProbes.find(pose.name());
    if (probeIt != this->activeProbes.end() && !probeIt->second.stopped)
    {
      const double travelled = std::hypot(
          cached.x - probeIt->second.startX, cached.y - probeIt->second.startY);
      if (travelled >= kSpawnGridSpacing && probeIt->second.publisher)
      {
        probeIt->second.publisher->Publish(gz::msgs::Twist());
        probeIt->second.stopped = true;
      }
    }
  }
}

QStringList HumanControlPanel::HumanModels() const
{
  QStringList result;
  for (int i = 0; i < kHumanModelCount; ++i)
    result << kHumanModels[i];
  return result;
}

QStringList HumanControlPanel::PosePresets() const
{
  return this->posePresetList;
}

QStringList HumanControlPanel::FollowModeLabels() const
{
  return {"テレオペ", "経路専用（テレオペ無視）"};
}

QStringList HumanControlPanel::PathTemplateLabels() const
{
  QStringList result;
  for (int i = 0; i < kPathTemplateCount; ++i)
    result << kPathTemplateLabels[i];
  return result;
}

QStringList HumanControlPanel::HumanList() const
{
  QStringList result;
  for (const auto &human : this->humans)
    result << QString::fromStdString(human.name + "  (" + human.model + ")");
  return result;
}

QString HumanControlPanel::Status() const
{
  return this->statusText;
}

int HumanControlPanel::ActiveHumanIndex() const
{
  return this->activeHumanIndex;
}

bool HumanControlPanel::ShiftHeld() const
{
  return this->shiftHeldState;
}

bool HumanControlPanel::CtrlHeld() const
{
  return this->ctrlHeldState;
}

bool HumanControlPanel::SHeld() const
{
  return this->sHeldState;
}

bool HumanControlPanel::JHeld() const
{
  return this->jHeldState;
}

bool HumanControlPanel::KHeld() const
{
  return this->kHeldState;
}

bool HumanControlPanel::ActiveSitLocked() const
{
  if (this->activeHumanIndex < 0 ||
      this->activeHumanIndex >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(this->activeHumanIndex).sitLocked;
}

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

int HumanControlPanel::ActiveViewIndex() const
{
  if (this->activeHumanIndex < 0 ||
      this->activeHumanIndex >= static_cast<int>(this->humans.size()))
    return kViewFree;
  return this->humans.at(this->activeHumanIndex).viewIndex;
}

double HumanControlPanel::ActiveViewDistance() const
{
  if (this->activeHumanIndex < 0 ||
      this->activeHumanIndex >= static_cast<int>(this->humans.size()))
    return 2.0;
  return this->humans.at(this->activeHumanIndex).viewDistance;
}

int HumanControlPanel::ActiveFollowModeIndex() const
{
  if (this->activeHumanIndex < 0 ||
      this->activeHumanIndex >= static_cast<int>(this->humans.size()))
    return 0;
  return this->humans.at(this->activeHumanIndex).followModeIndex;
}

void HumanControlPanel::setActiveHuman(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  if (this->activeHumanIndex == _index)
    return;
  this->activeHumanIndex = _index;
  this->activeHumanChanged();
  this->activeFollowModeChanged();
  // Same auto-focus as a fresh spawn (see PollSpawnConfirmation): jump the
  // camera to a behind/chase view of whichever human just became "対象".
  // This also updates the human's stored viewIndex/viewDistance and fires
  // activeViewIndexChanged() itself (since _index == activeHumanIndex by
  // now), so no separate call to that is needed here.
  this->setViewpoint(_index, kViewBehind, 2.0);
  this->SetStatus(QString::fromStdString(this->humans.at(_index).name) +
      " をキーボード/視点操作対象にしました（後方追従視点に切替）");
}

void HumanControlPanel::SetStatus(const QString &_status)
{
  this->statusText = _status;
  this->StatusChanged();
}

QString HumanControlPanel::defaultName(int _modelIndex) const
{
  (void)_modelIndex;
  return QString("human%1").arg(this->humans.size() + 1);
}

QString HumanControlPanel::modelDescription(int _modelIndex) const
{
  if (_modelIndex < 0 || _modelIndex >= kHumanModelCount)
    return {};
  return kHumanModelDescriptions[_modelIndex];
}

double HumanControlPanel::defaultZ(int _modelIndex) const
{
  if (_modelIndex < 0 || _modelIndex >= kHumanModelCount)
    return 1.0;
  return kHumanModelDefaultZ[_modelIndex];
}

double HumanControlPanel::nextSpawnX() const
{
  const int index = static_cast<int>(this->humans.size()) % kSpawnGridColumns;
  return index * kSpawnGridSpacing;
}

double HumanControlPanel::nextSpawnY() const
{
  const int index = static_cast<int>(this->humans.size()) / kSpawnGridColumns;
  return index * kSpawnGridSpacing;
}

bool HumanControlPanel::isCustomHuman(int _modelIndex) const
{
  return _modelIndex == kCustomHumanIndex;
}

bool HumanControlPanel::isActorModel(int _modelIndex) const
{
  return IsActorIndex(_modelIndex);
}

bool HumanControlPanel::isSitCapableModel(int _modelIndex) const
{
  return IsSitCapableIndex(_modelIndex);
}

bool HumanControlPanel::isSitCapableHumanAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(_index).sitPublisher.Valid();
}

QString HumanControlPanel::followModeValue(int _followModeIndex) const
{
  if (_followModeIndex < 0 || _followModeIndex >= kFollowModeCount)
    return "auto";
  return kFollowModeValues[_followModeIndex];
}

void HumanControlPanel::setFollowMode(int _index, int _followModeIndex)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  if (_followModeIndex < 0 || _followModeIndex >= kFollowModeCount)
    return;
  auto &human = this->humans.at(_index);
  if (!human.followModePublisher.Valid())
    return;

  gz::msgs::StringMsg message;
  message.set_data(kFollowModeValues[_followModeIndex]);
  human.followModePublisher.Publish(message);

  human.followModeIndex = _followModeIndex;
  if (_index == this->activeHumanIndex)
    this->activeFollowModeChanged();
  this->SetStatus(QString::fromStdString(human.name) + " の動作モードを「" +
      this->FollowModeLabels().value(_followModeIndex) + "」に変更しました");
}

void HumanControlPanel::toggleSitLock(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.sitPublisher.Valid())
    return;

  human.sitLocked = !human.sitLocked;
  if (_index == this->activeHumanIndex)
    this->activeSitLockedChanged();
  this->SetStatus(QString::fromStdString(human.name) +
      (human.sitLocked ? " の着席を固定しました" : " の着席固定を解除しました"));
  this->UpdateSitIntent(_index);
}

void HumanControlPanel::UpdateSitIntent(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.sitPublisher.Valid())
    return;

  // Effective intent: locked (Space) OR K currently held. Locking alone
  // (K never pressed, e.g. from the QML button) is enough to sit; K alone
  // (no lock) sits only while held; both together behave the same as K
  // alone until the lock is released, at which point it falls back to
  // whatever K is doing right then -- see the Q_PROPERTY comments in the
  // header for the full K+Space design.
  const bool wantsSit = human.sitLocked ||
      (_index == this->activeHumanIndex && this->kHeldState);
  gz::msgs::StringMsg message;
  message.set_data(wantsSit ? "sit" : "stand");
  human.sitPublisher.Publish(message);
}

bool HumanControlPanel::isHumanActorAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(_index).velocityPublisher.Valid();
}

bool HumanControlPanel::showCollisionAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(_index).showCollision;
}

void HumanControlPanel::setShowCollision(int _index, bool _value)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;
  human.showCollision = _value;
  // ApplyCollisionVisibility() (render thread, driven off Render events)
  // picks this up and applies it next frame -- see its own comment for
  // why this can't just be done synchronously here. humansChanged() lets
  // every row's QML checkbox (not just this one, e.g. after
  // setShowCollisionAll()) resync its displayed checked state.
  this->humansChanged();
}

void HumanControlPanel::setShowCollisionAll(bool _value)
{
  for (auto &human : this->humans)
  {
    if (human.velocityPublisher.Valid())
      human.showCollision = _value;
  }
  this->humansChanged();
}

double HumanControlPanel::collisionRadiusAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return 0.25;
  return this->humans.at(_index).collisionRadius;
}

double HumanControlPanel::collisionLengthAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return 1.2;
  return this->humans.at(_index).collisionLength;
}

void HumanControlPanel::applyCollisionSize(int _index, double _radius, double _length)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;  // Static (non-actor) humans have no collision-body companion.
  if (this->worldName.empty() || this->collisionBodyTemplate.empty())
    return;

  // Same naming convention spawn_human.launch.py's _spawn_human_cmd()
  // uses (collision_model_name = f'{model_name}_collision', topic =
  // f'/model/{collision_model_name}/cmd_vel') -- derived here rather than
  // stored on Human, since it's fully determined by the human's own name.
  const std::string collisionModelName = human.name + "_collision";
  const std::string collisionCmdVelTopic = "/model/" + collisionModelName + "/cmd_vel";

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->poseMutex);
    const auto it = this->poses.find(collisionModelName);
    if (it != this->poses.end() && it->second.valid)
    {
      x = it->second.x;
      y = it->second.y;
      z = it->second.z;
    }
  }

  // Same template ProbeSafeSpawnPosition() reuses for its throwaway
  // probes, but with the real per-human name/topic substituted (not a
  // probe placeholder) plus the new capsule dimensions.
  std::string sdf = this->collisionBodyTemplate;
  const std::string namePlaceholder = "<model name=\"human_collision_body\">";
  const auto namePos = sdf.find(namePlaceholder);
  if (namePos != std::string::npos)
  {
    sdf.replace(namePos, namePlaceholder.size(),
        "<model name=\"" + collisionModelName + "\">");
  }
  const std::string topicPlaceholder =
      "<topic>/model/human_collision_body/cmd_vel</topic>";
  const auto topicPos = sdf.find(topicPlaceholder);
  if (topicPos != std::string::npos)
  {
    sdf.replace(topicPos, topicPlaceholder.size(),
        "<topic>" + collisionCmdVelTopic + "</topic>");
  }
  // <radius>/<length> each appear twice in the template (the <collision>
  // and its matching debug <visual>, see human_collision_body/model.sdf)
  // -- replace every occurrence, same pragmatic literal-value
  // substitution scripts/human_model_utils.py already relies on for topics.
  const std::string radiusStr = std::to_string(_radius);
  const std::string radiusPlaceholder = "<radius>0.25</radius>";
  const std::string newRadius = "<radius>" + radiusStr + "</radius>";
  for (auto pos = sdf.find(radiusPlaceholder); pos != std::string::npos;
      pos = sdf.find(radiusPlaceholder, pos + newRadius.size()))
    sdf.replace(pos, radiusPlaceholder.size(), newRadius);
  const std::string lengthStr = std::to_string(_length);
  const std::string lengthPlaceholder = "<length>1.2</length>";
  const std::string newLength = "<length>" + lengthStr + "</length>";
  for (auto pos = sdf.find(lengthPlaceholder); pos != std::string::npos;
      pos = sdf.find(lengthPlaceholder, pos + newLength.size()))
    sdf.replace(pos, lengthPlaceholder.size(), newLength);

  this->RequestEntityRemoval(collisionModelName);
  // The old cached material belongs to the entity that's about to be
  // destroyed -- ApplyCollisionVisibility() re-finds and re-clones a
  // fresh one for the new entity once it appears.
  human.collisionMaterial.reset();
  human.appliedCollisionTransparency = -1.0f;
  human.collisionVisualRetries = 0;
  human.collisionRadius = _radius;
  human.collisionLength = _length;

  // A create request for this name landing before the remove request has
  // actually been processed server-side would collide with the entity
  // still being torn down -- give it a moment, same "async operations
  // need explicit settling time" approach kSpawnSafetySettleMs already
  // uses elsewhere in this file.
  QTimer::singleShot(500, this,
      [this, sdf, collisionModelName, x, y, z]()
      {
        this->RequestEntityCreation(sdf, collisionModelName, x, y, z);
      });
  this->SetStatus(QString::fromStdString(human.name) +
      " の当たり判定サイズを変更しました（半径" + QString::number(_radius, 'f', 2) +
      "m・長さ" + QString::number(_length, 'f', 2) + "m）");
}

QProcess *HumanControlPanel::StartLaunchProcess(const QStringList &_arguments)
{
  auto *process = new QProcess(this);
  // setsid makes the child (ros2 launch, plus every node it spawns) its
  // own process group, so TerminateProcessGroup() can signal all of them
  // at once on removal instead of leaving orphaned bridge/spawn nodes
  // behind. Same pattern as guide_robot's GuiderRobotManager.
  process->setProgram("setsid");
  process->setArguments(QStringList{"ros2"} + _arguments);
  process->setStandardOutputFile(QProcess::nullDevice());
  process->setStandardErrorFile(QProcess::nullDevice());
  process->start();
  if (!process->waitForStarted(3000))
  {
    process->deleteLater();
    return nullptr;
  }
  return process;
}

void HumanControlPanel::TerminateProcessGroup(QProcess *_process)
{
  if (!_process)
    return;
  const qint64 pid = _process->processId();
  if (pid > 0)
  {
    ::kill(static_cast<pid_t>(-pid), SIGINT);
    QTimer::singleShot(4000, this, [pid]()
    {
      if (::kill(static_cast<pid_t>(-pid), 0) == 0)
        ::kill(static_cast<pid_t>(-pid), SIGTERM);
    });
    QTimer::singleShot(8000, this, [pid]()
    {
      if (::kill(static_cast<pid_t>(-pid), 0) == 0)
        ::kill(static_cast<pid_t>(-pid), SIGKILL);
    });
  }
  QTimer::singleShot(9000, this, [guard = QPointer<QProcess>(_process)]()
  {
    if (guard)
      guard->deleteLater();
  });
}

void HumanControlPanel::RequestEntityRemoval(const std::string &_name)
{
  gz::msgs::Entity request;
  request.set_name(_name);
  request.set_type(gz::msgs::Entity::MODEL);

  const std::string service = "/world/" + this->worldName + "/remove";
  std::function<void(const gz::msgs::Boolean &, const bool)> callback =
      [](const gz::msgs::Boolean &, const bool) { /* fire and forget */ };
  this->node.Request(service, request, callback);
}

void HumanControlPanel::RequestEntityCreation(const std::string &_sdf,
    const std::string &_name, double _x, double _y, double _z)
{
  gz::msgs::EntityFactory request;
  request.set_sdf(_sdf);
  request.set_name(_name);
  request.mutable_pose()->mutable_position()->set_x(_x);
  request.mutable_pose()->mutable_position()->set_y(_y);
  request.mutable_pose()->mutable_position()->set_z(_z);

  const std::string service = "/world/" + this->worldName + "/create";
  // Fire and forget, same as RequestEntityRemoval() above -- if this
  // fails, ActorCommandPlugin simply keeps not finding a collision entity
  // by name (same as before the human_collision_body companion first
  // spawns) rather than anything crashing, so there's nothing useful to
  // do with a failure callback here.
  std::function<void(const gz::msgs::Boolean &, const bool)> callback =
      [](const gz::msgs::Boolean &, const bool) { /* fire and forget */ };
  this->node.Request(service, request, callback);
}

bool HumanControlPanel::QueryEntityExists(const std::string &_name)
{
  if (this->worldName.empty())
    return false;
  // /world/<w>/scene/info and the `gz model` CLI only enumerate MODEL-type
  // entities -- actors (what every human_model this panel spawns actually
  // is) are a distinct entity kind in gz-sim's ECS and never show up
  // there, spawned-at-load-time or not (verified: even `gz model -m
  // <name> -p` reports "No model named <name> was found" for a live,
  // just-spawned, teleoperable actor). /world/<w>/state is the one
  // service that does include actors -- its response is the raw
  // serialized ECS component set, so rather than hand-walking gz-sim's
  // component type IDs to decode it, this just substring-searches the
  // serialized bytes for the entity's Name component string. Good enough
  // for an existence check: after a real removal every component
  // mentioning that name (Name, the plugin's own vel_topic/path_topic
  // strings, ...) is gone from the ECS too.
  gz::msgs::Empty request;
  gz::msgs::SerializedStepMap response;
  bool result = false;
  const bool executed = this->node.Request(
      "/world/" + this->worldName + "/state", request,
      kStateQueryTimeoutMs, response, result);
  if (!executed || !result)
    return false;
  const std::string serialized = response.SerializeAsString();
  return serialized.find(_name) != std::string::npos;
}

void HumanControlPanel::spawnHuman(
    int _modelIndex, const QString &_name, const QString &_posePreset,
    const QString &_followMode, double _x, double _y, double _z, double _yaw)
{
  if (_modelIndex < 0 || _modelIndex >= kHumanModelCount)
    return;
  if (this->worldName.empty())
  {
    this->SetStatus("ワールド未検出のためspawnできません");
    return;
  }

  const QString name = _name.trimmed();
  if (name.isEmpty())
  {
    this->SetStatus("名前を入力してください");
    return;
  }
  const std::string nameStd = name.toStdString();
  for (const auto &human : this->humans)
  {
    if (human.name == nameStd)
    {
      this->SetStatus(name + " は既に存在します。別名にしてください");
      return;
    }
  }
  // A name this panel doesn't know about may still be live in gz (a ghost
  // from an earlier session/crash, or another tool). Spawning on top of it
  // silently corrupts gz-sim's name lookup -- both copies become
  // unfindable by name afterward, which is what made removal fail. Refuse
  // instead of reproducing that.
  if (this->QueryEntityExists(nameStd))
  {
    this->SetStatus(name + " は既にワールドに存在します。別名にするか、"
        "先にGazeboを再起動してゴーストエンティティを解消してください");
    return;
  }

  // Actor-backed models get the collision-body probe first (rcjo2025_arena
  // spawning its first human at the (0,0) grid default -- which happens to
  // land inside that world's center wall -- is exactly the case this
  // exists for); static models (person_standing/custom_human) have no
  // paired collision body to probe with, so they just spawn where asked,
  // same as before this feature existed.
  if (IsActorIndex(_modelIndex))
    this->ProbeSafeSpawnPosition(_modelIndex, name, _posePreset, _followMode, _x, _y, _z, _yaw, 0);
  else
    this->StartRealSpawn(_modelIndex, name, _posePreset, _followMode, _x, _y, _z, _yaw);
}

void HumanControlPanel::StartRealSpawn(
    int _modelIndex, QString _name, QString _posePreset, QString _followMode,
    double _x, double _y, double _z, double _yaw)
{
  const QString model = kHumanModels[_modelIndex];
  QStringList arguments;
  arguments << "launch" << "gz_human_sim" << "spawn_human.launch.py"
            << "world_name:=" + QString::fromStdString(this->worldName)
            // namespace must match what this panel publishes teleop Twist
            // messages to below (velocityTopic/pathTopic): spawn_human.launch.py
            // derives the actor's vel_topic/path_topic SDF params from this
            // argument, so without it every spawned actor ends up wired to
            // the unnamespaced /cmd_vel and the teleop pad silently does
            // nothing (also breaks human_teleop_switcher.py, which publishes
            // to the namespaced /human1,2/cmd_vel ROS topics by name).
            << "namespace:=" + _name
            << "model_name:=" + _name
            << "human_model:=" + model
            << "enable_teleop:=false"
            << "x:=" + QString::number(_x) << "y:=" + QString::number(_y)
            << "z:=" + QString::number(_z) << "yaw:=" + QString::number(_yaw);
  if (_modelIndex == kCustomHumanIndex && !_posePreset.isEmpty())
    arguments << "human_pose:=" + _posePreset;
  if (IsActorIndex(_modelIndex) && !_followMode.isEmpty())
    arguments << "follow_mode:=" + _followMode;

  auto *process = this->StartLaunchProcess(arguments);
  if (!process)
  {
    this->SetStatus(_name + " の起動に失敗しました");
    return;
  }

  this->SetStatus(_name + " をspawn中…（存在確認待ち）");
  QTimer::singleShot(kEntityPollIntervalMs, this,
      [this, _name, model, followMode = _followMode, process = QPointer<QProcess>(process),
       _x, _y, _z, _yaw]()
      {
        this->PollSpawnConfirmation(_name, model, followMode, process, _x, _y, _z, _yaw, 0);
      });
}

std::pair<double, double> HumanControlPanel::SpawnSpiralOffset(int _attempt) const
{
  // Built once: every integer grid cell in a radius-4 block (9x9 = 81
  // cells, comfortably more than kSpawnSafetyMaxAttempts even if that cap
  // grows later), sorted by ascending Euclidean distance from (0, 0) so
  // index 0 is (0, 0) itself and each following index is the next-nearest
  // cell -- ties (e.g. the 4 axis neighbours all at distance 1) broken by
  // angle so same-ring cells still come out in a clean spiral order
  // rather than an arbitrary one.
  static const std::vector<std::pair<int, int>> kOffsets = []()
  {
    constexpr int kRadius = 4;
    std::vector<std::pair<int, int>> offsets;
    for (int i = -kRadius; i <= kRadius; ++i)
    {
      for (int j = -kRadius; j <= kRadius; ++j)
        offsets.emplace_back(i, j);
    }
    std::sort(offsets.begin(), offsets.end(),
        [](const std::pair<int, int> &_a, const std::pair<int, int> &_b)
        {
          const double distA = std::hypot(_a.first, _a.second);
          const double distB = std::hypot(_b.first, _b.second);
          if (distA != distB)
            return distA < distB;
          return std::atan2(_a.second, _a.first) < std::atan2(_b.second, _b.first);
        });
    return offsets;
  }();
  const auto &offset = kOffsets[std::min<std::size_t>(
      static_cast<std::size_t>(std::max(_attempt, 0)), kOffsets.size() - 1)];
  return {offset.first * kSpawnGridSpacing, offset.second * kSpawnGridSpacing};
}

void HumanControlPanel::ProbeSafeSpawnPosition(
    int _modelIndex, QString _name, QString _posePreset, QString _followMode,
    double _x, double _y, double _z, double _yaw, int _attempt)
{
  if (_attempt >= kSpawnSafetyMaxAttempts || this->collisionBodyTemplate.empty())
  {
    // Gave up finding a clear spot (or never had a template to probe
    // with) -- spawn at the originally-requested position anyway rather
    // than refusing outright; a possibly-embedded spawn the user can see
    // and fix is better than a silent no-op.
    if (_attempt >= kSpawnSafetyMaxAttempts)
    {
      this->SetStatus(_name + "：安全なスポーン地点が見つからなかったため、"
          "指定座標にそのままスポーンします");
    }
    this->StartRealSpawn(_modelIndex, _name, _posePreset, _followMode, _x, _y, _z, _yaw);
    return;
  }

  // Nearest-first spiral around the user-requested (_x, _y) -- attempt 0
  // always tests the original position first, then the closest untried
  // cell in any direction. See SpawnSpiralOffset()'s own comment for why
  // this replaced the old one-quadrant row-major grid.
  const auto [offsetX, offsetY] = this->SpawnSpiralOffset(_attempt);
  const double candidateX = _x + offsetX;
  const double candidateY = _y + offsetY;

  const std::string probeName = "__spawn_probe_" + _name.toStdString() +
      "_" + std::to_string(_attempt);
  const std::string probeTopic = "/model/" + probeName + "/cmd_vel";
  std::string probeSdf = this->collisionBodyTemplate;
  const std::string namePlaceholder = "<model name=\"human_collision_body\">";
  const auto namePos = probeSdf.find(namePlaceholder);
  if (namePos != std::string::npos)
  {
    probeSdf.replace(namePos, namePlaceholder.size(),
        "<model name=\"" + probeName + "\">");
  }
  // Give this probe its OWN VelocityControl topic -- reusing the
  // template's unmodified placeholder topic would make every simultaneous
  // probe (and any real, already-spawned collision body still using the
  // template's literal default) fight over the same one.
  const std::string topicPlaceholder =
      "<topic>/model/human_collision_body/cmd_vel</topic>";
  const auto topicPos = probeSdf.find(topicPlaceholder);
  if (topicPos != std::string::npos)
  {
    probeSdf.replace(topicPos, topicPlaceholder.size(),
        "<topic>" + probeTopic + "</topic>");
  }

  // Confirmed by direct testing (see the commit this landed in): a probe
  // simply DROPPED at an already-overlapping candidate does NOT get
  // pushed back out by gz-sim's own contact resolution -- a symmetric,
  // velocity-free interpenetration with a static body just sits there
  // indefinitely instead of separating. So instead, this spawns the probe
  // one grid cell short of the candidate and drives it toward the
  // candidate at a normal walking speed -- exactly the already-verified
  // "a moving body cleanly stops at a wall's surface" behavior real
  // teleop movement relies on (see actor_command_plugin.cpp).
  // CheckProbeSettle() then checks how far it actually got.
  //
  // Approach direction: radially outward from the original requested
  // (_x, _y), i.e. straight along this candidate's own spiral offset --
  // NOT a fixed south->north walk-in like before the spiral search
  // existed. A fixed direction only made sense back when every candidate
  // sat in the same +X/+Y quadrant from the request point; the spiral
  // now puts candidates in every direction, so the approach itself must
  // rotate with it, or a southward approach would cut across untested
  // (or already-rejected) territory instead of the same straight line the
  // candidate was reached by. attempt 0 (offset (0, 0), the exact
  // requested point) has no direction to derive from, so it keeps the
  // original fixed south->north approach.
  double approachDirX = 0.0;
  double approachDirY = 1.0;
  const double offsetLength = std::hypot(offsetX, offsetY);
  if (offsetLength > 1e-6)
  {
    approachDirX = offsetX / offsetLength;
    approachDirY = offsetY / offsetLength;
  }
  const double approachStartX = candidateX - approachDirX * kSpawnGridSpacing;
  const double approachStartY = candidateY - approachDirY * kSpawnGridSpacing;

  gz::msgs::EntityFactory request;
  request.set_sdf(probeSdf);
  request.set_name(probeName);
  request.mutable_pose()->mutable_position()->set_x(approachStartX);
  request.mutable_pose()->mutable_position()->set_y(approachStartY);
  request.mutable_pose()->mutable_position()->set_z(0.0);

  gz::msgs::Boolean response;
  bool result = false;
  const bool executed = this->node.Request(
      "/world/" + this->worldName + "/create", request, 2000u, response, result);
  if (!executed || !result || !response.data())
  {
    // Couldn't even spawn the probe -- don't block the real spawn on a
    // safety check that isn't working right now.
    this->StartRealSpawn(_modelIndex, _name, _posePreset, _followMode, _x, _y, _z, _yaw);
    return;
  }

  // gz-transport publisher/subscriber discovery is asynchronous -- a
  // Publish() called immediately after Advertise() can easily lose the
  // race and never reach the probe's VelocityControl system before this
  // handle would otherwise go out of scope. Kept alive (shared_ptr,
  // captured by value below) and re-published a couple of times over the
  // first moment of the settle window to survive that race; a one-shot
  // command is otherwise enough since VelocityControl holds the last
  // velocity it received, same as real teleop relies on.
  auto probePublisher = std::make_shared<gz::transport::Node::Publisher>(
      this->node.Advertise<gz::msgs::Twist>(probeTopic));
  // Registered before the first Publish() below so OnPoseInfo() (transport
  // thread) can stop this probe the moment it has walked the full approach
  // distance, instead of leaving it to keep walking (VelocityControl holds
  // the last Twist forever) until CheckProbeSettle()'s timer deletes it --
  // see activeProbes' own comment for why that overshoot read as "the
  // tested spot and the actual spawn spot don't match".
  {
    std::lock_guard<std::mutex> lock(this->poseMutex);
    this->activeProbes[probeName] = ActiveProbe{approachStartX, approachStartY, probePublisher, false};
  }
  gz::msgs::Twist probeTwist;
  probeTwist.mutable_linear()->set_x(approachDirX * kSpawnProbeSpeed);
  probeTwist.mutable_linear()->set_y(approachDirY * kSpawnProbeSpeed);
  probePublisher->Publish(probeTwist);
  QTimer::singleShot(100, this, [probePublisher, probeTwist]() { probePublisher->Publish(probeTwist); });
  QTimer::singleShot(300, this, [probePublisher, probeTwist]() { probePublisher->Publish(probeTwist); });

  QTimer::singleShot(kSpawnSafetySettleMs, this,
      [this, _modelIndex, _name, _posePreset, _followMode, _x, _y, _z, _yaw, _attempt,
       probeName = QString::fromStdString(probeName), candidateX, candidateY,
       approachStartX, approachStartY, probePublisher]()
      {
        this->CheckProbeSettle(_modelIndex, _name, _posePreset, _followMode, _x, _y, _z, _yaw,
            _attempt, probeName, candidateX, candidateY, approachStartX, approachStartY);
      });
}

void HumanControlPanel::CheckProbeSettle(
    int _modelIndex, QString _name, QString _posePreset, QString _followMode,
    double _x, double _y, double _z, double _yaw, int _attempt,
    QString _probeName, double _candidateX, double _candidateY,
    double _approachStartX, double _approachStartY)
{
  const std::string probeNameStd = _probeName.toStdString();
  // Safe means "travelled roughly the full kSpawnGridSpacing walk in from
  // _approachStartX/Y" (the same point ProbeSafeSpawnPosition() dropped
  // and launched the probe from -- radially outward from the original
  // request, see its own comment for why that replaced a fixed
  // south->north walk-in), checked as distance from its OWN START point
  // rather than proximity to the candidate. OnPoseInfo() now stops the
  // probe (zero Twist) as soon as it covers that same distance -- see
  // activeProbes' own comment -- so this mostly reads back that stopped
  // position; still measuring from the start (rather than
  // distance-from-candidate) costs nothing and keeps this correct even
  // for the rare case where this timer fires before OnPoseInfo() has
  // caught up. No pose yet (never confirmed moving at all) counts as NOT
  // safe -- a probe that should have had a full kSpawnSafetySettleMs to
  // walk in and simply never reported a position is more likely stuck
  // than fine.
  bool safe = false;
  {
    std::lock_guard<std::mutex> lock(this->poseMutex);
    const auto it = this->poses.find(probeNameStd);
    if (it != this->poses.end() && it->second.valid)
    {
      const double distanceFromStart =
          std::hypot(it->second.x - _approachStartX, it->second.y - _approachStartY);
      safe = distanceFromStart >= (kSpawnGridSpacing - kSpawnSafetyDisplacementMeters);
    }
    this->poses.erase(probeNameStd);
    this->activeProbes.erase(probeNameStd);
  }
  // Always clean up the probe, safe or not -- it was only ever a test.
  this->RequestEntityRemoval(probeNameStd);

  if (safe)
  {
    // Confirm the probe is actually gone -- not just guess at a delay --
    // before spawning the real collision-body companion at essentially
    // the same spot the probe was just occupying. See PollProbeRemoval()'s
    // own comment for why: a real capsule (same mass, same collision
    // shape, spawned by a brand new ros_gz_sim create call that knows
    // nothing about the probe) starting out overlapping a probe that
    // hasn't actually been torn down yet gets thrown apart by contact
    // resolution the instant physics runs, which is what "the person
    // gets launched sideways right after spawning" was.
    this->PollProbeRemoval(QString::fromStdString(probeNameStd), _modelIndex, _name,
        _posePreset, _followMode, _candidateX, _candidateY, _z, _yaw, 0);
  }
  else
  {
    this->SetStatus(_name + "：候補地点(" + QString::number(_candidateX) + ", " +
        QString::number(_candidateY) + ")が障害物と重なっていたため、別の位置を試します");
    this->ProbeSafeSpawnPosition(
        _modelIndex, _name, _posePreset, _followMode, _x, _y, _z, _yaw, _attempt + 1);
  }
}

void HumanControlPanel::PollProbeRemoval(
    QString _probeName, int _modelIndex, QString _name, QString _posePreset,
    QString _followMode, double _candidateX, double _candidateY, double _z, double _yaw,
    int _pollAttempt)
{
  const std::string probeNameStd = _probeName.toStdString();
  if (this->QueryEntityExists(probeNameStd) && _pollAttempt < kProbeRemovalMaxAttempts)
  {
    QTimer::singleShot(kProbeRemovalPollIntervalMs, this,
        [this, _probeName, _modelIndex, _name, _posePreset, _followMode,
         _candidateX, _candidateY, _z, _yaw, _pollAttempt]()
        {
          this->PollProbeRemoval(_probeName, _modelIndex, _name, _posePreset, _followMode,
              _candidateX, _candidateY, _z, _yaw, _pollAttempt + 1);
        });
    return;
  }
  // Either confirmed gone, or gave up waiting for it (spawning anyway
  // rather than blocking forever on a removal that's stuck -- same
  // trade-off ProbeSafeSpawnPosition() itself makes when it gives up
  // finding a clear spot).
  this->StartRealSpawn(
      _modelIndex, _name, _posePreset, _followMode, _candidateX, _candidateY, _z, _yaw);
}

void HumanControlPanel::PollSpawnConfirmation(
    QString _name, QString _model, QString _followMode, QPointer<QProcess> _process,
    double _x, double _y, double _z, double _yaw, int _attempt)
{
  const std::string nameStd = _name.toStdString();
  if (!this->QueryEntityExists(nameStd))
  {
    if (_attempt + 1 >= kEntityPollMaxAttempts)
    {
      this->SetStatus(_name + " のspawnに失敗しました（"
          + QString::number(kEntityPollMaxAttempts * kEntityPollIntervalMs / 1000)
          + "秒待っても見つかりません）");
      this->TerminateProcessGroup(_process);
      return;
    }
    QTimer::singleShot(kEntityPollIntervalMs, this,
        [this, _name, _model, _followMode, _process, _x, _y, _z, _yaw, _attempt]()
        {
          this->PollSpawnConfirmation(
              _name, _model, _followMode, _process, _x, _y, _z, _yaw, _attempt + 1);
        });
    return;
  }

  const int modelIndex = static_cast<int>(
      std::find(std::begin(kHumanModels), std::end(kHumanModels),
          _model.toStdString()) - std::begin(kHumanModels));

  Human human;
  human.name = nameStd;
  human.model = _model.toStdString();
  human.process = _process;
  if (IsActorIndex(modelIndex))
  {
    const std::string velocityTopic = "/" + nameStd + "/cmd_vel";
    const std::string pathTopic = "/" + nameStd + "/cmd_path";
    const std::string jumpTopic = "/" + nameStd + "/cmd_jump";
    const std::string removeTopic = "/" + nameStd + "/remove_actor";
    const std::string followModeTopic = "/" + nameStd + "/set_follow_mode";
    human.velocityPublisher =
        this->node.Advertise<gz::msgs::Twist>(velocityTopic);
    human.pathPublisher =
        this->node.Advertise<gz::msgs::Pose_V>(pathTopic);
    human.jumpPublisher =
        this->node.Advertise<gz::msgs::Double>(jumpTopic);
    human.removePublisher =
        this->node.Advertise<gz::msgs::Empty>(removeTopic);
    human.followModePublisher =
        this->node.Advertise<gz::msgs::StringMsg>(followModeTopic);
    const std::string followModeStd = _followMode.isEmpty()
        ? "auto" : _followMode.toStdString();
    const auto followModeIt = std::find(
        std::begin(kFollowModeValues), std::end(kFollowModeValues), followModeStd);
    human.followModeIndex = followModeIt != std::end(kFollowModeValues)
        ? static_cast<int>(followModeIt - std::begin(kFollowModeValues)) : 0;
    if (IsSitCapableIndex(modelIndex))
    {
      const std::string sitTopic = "/" + nameStd + "/cmd_sit";
      human.sitPublisher = this->node.Advertise<gz::msgs::StringMsg>(sitTopic);
    }
  }
  this->humans.push_back(std::move(human));
  const int newIndex = static_cast<int>(this->humans.size()) - 1;
  this->humansChanged();
  this->SetStatus(_name + " (" + _model + ") を (" +
      QString::number(_x) + ", " + QString::number(_y) + ", " +
      QString::number(_z) + ") yaw=" + QString::number(_yaw) + " にspawnしました");

  // Newly spawned human becomes the active target for both the keyboard
  // (QWEASDZXC only actually moves actor-backed humans, but 1-9/"対象に
  // する" still work either way) and the single global viewpoint control
  // below -- the common case is spawn-then-immediately-look-at-it, and
  // both stay switchable afterward.
  this->activeHumanIndex = newIndex;
  this->activeHumanChanged();
  this->activeFollowModeChanged();

  // 人物の初期デフォルト視点は後方追従（kViewBehind）。activeHumanIndex
  // をこの直前に設定しているので、setViewpoint()内のactiveViewIndexChanged()
  // 発火でグローバルの視点コンボにも即反映される。
  this->setViewpoint(newIndex, kViewBehind, 2.0);
}

void HumanControlPanel::removeHuman(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  if (this->worldName.empty())
  {
    this->SetStatus("ワールド未検出のため削除できません");
    return;
  }

  Human human = std::move(this->humans.at(_index));
  this->humans.erase(this->humans.begin() + _index);
  this->humansChanged();

  // Keep the keyboard target pointing at the same logical human across the
  // index shift caused by erase(), or clear it if that's the one removed.
  if (this->activeHumanIndex == _index)
    this->activeHumanIndex = -1;
  else if (this->activeHumanIndex > _index)
    --this->activeHumanIndex;
  this->activeHumanChanged();
  this->activeViewIndexChanged();
  this->activeFollowModeChanged();

  this->TerminateProcessGroup(human.process);

  const QString name = QString::fromStdString(human.name);
  if (!this->QueryEntityExists(human.name))
  {
    // Already gone (or never actually existed -- a ghost list entry from
    // an earlier collision): nothing to ask gz to remove, and doing so
    // anyway just adds another confusing "not found" line to the server
    // log for no reason.
    this->SetStatus(name + " は既にワールドから存在しません（一覧から削除）");
    return;
  }

  if (human.removePublisher.Valid())
  {
    // Actor-backed human: /world/<w>/remove can't take an ACTOR (see
    // RequestEntityRemoval()'s comment), so ask the actor's own
    // ActorCommandPlugin to remove itself via the ECM instead.
    gz::msgs::Empty message;
    human.removePublisher.Publish(message);
  }
  else
  {
    this->RequestEntityRemoval(human.name);
  }
  this->SetStatus(name + " を削除中…（確認待ち）");
  QTimer::singleShot(kEntityPollIntervalMs, this,
      [this, name]() { this->PollRemovalConfirmation(name, 0); });
}

void HumanControlPanel::PollRemovalConfirmation(QString _name, int _attempt)
{
  const std::string nameStd = _name.toStdString();
  if (!this->QueryEntityExists(nameStd))
  {
    this->SetStatus(_name + " を削除しました");
    return;
  }
  if (_attempt + 1 >= kEntityPollMaxAttempts)
  {
    this->SetStatus(_name + " の削除を確認できませんでした（Gazebo側に残っている可能性があります）");
    return;
  }
  QTimer::singleShot(kEntityPollIntervalMs, this,
      [this, _name, _attempt]()
      {
        this->PollRemovalConfirmation(_name, _attempt + 1);
      });
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

void HumanControlPanel::sendWaypoint(int _index, double _x, double _y)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.pathPublisher.Valid())
    return;

  gz::msgs::Pose_V message;
  auto *pose = message.add_pose();
  pose->mutable_position()->set_x(_x);
  pose->mutable_position()->set_y(_y);
  pose->mutable_orientation()->set_w(1.0);
  human.pathPublisher.Publish(message);
}

void HumanControlPanel::sendPathTemplate(
    int _index, int _templateIndex, double _centerX, double _centerY,
    double _size, int _numWaypoints, bool _clockwise)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.pathPublisher.Valid())
    return;
  if (_templateIndex < 0 || _templateIndex >= kPathTemplateCount)
    return;

  // (x, y, yaw) tuples, same math as scripts/path_template.py's
  // _circle_waypoints()/_square_waypoints() -- kept in sync by hand since
  // one is Python (for headless/CLI use) and this is C++ (for the GUI
  // button), not sharable code between the two.
  std::vector<std::array<double, 3>> waypoints;
  const double direction = _clockwise ? -1.0 : 1.0;

  if (_templateIndex == kPathTemplateCircle)
  {
    const double radius = std::max(0.1, _size);
    const int count = std::clamp(_numWaypoints, 3, 200);
    for (int i = 0; i < count; ++i)
    {
      const double angle = direction * i * (2.0 * M_PI / count);
      const double tangent = angle + direction * (M_PI / 2.0);
      waypoints.push_back({_centerX + radius * std::cos(angle),
          _centerY + radius * std::sin(angle), tangent});
    }
  }
  else
  {
    const double half = std::max(0.1, _size) / 2.0;
    std::vector<std::pair<double, double>> corners = {
      {_centerX + half, _centerY + half}, {_centerX - half, _centerY + half},
      {_centerX - half, _centerY - half}, {_centerX + half, _centerY - half}};
    if (_clockwise)
      std::reverse(corners.begin() + 1, corners.end());
    for (std::size_t i = 0; i < corners.size(); ++i)
    {
      const auto &[x, y] = corners[i];
      const auto &[nextX, nextY] = corners[(i + 1) % corners.size()];
      waypoints.push_back({x, y, std::atan2(nextY - y, nextX - x)});
    }
  }

  gz::msgs::Pose_V message;
  for (const auto &[x, y, yaw] : waypoints)
  {
    auto *pose = message.add_pose();
    pose->mutable_position()->set_x(x);
    pose->mutable_position()->set_y(y);
    pose->mutable_orientation()->set_z(std::sin(yaw * 0.5));
    pose->mutable_orientation()->set_w(std::cos(yaw * 0.5));
  }
  human.pathPublisher.Publish(message);
  this->SetStatus(QString::fromStdString(human.name) + " に" +
      kPathTemplateLabels[_templateIndex] + "の経路（" +
      QString::number(waypoints.size()) + "点）を送信しました");
}

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
      // Camera right at the actor's own head, looking out at a point far
      // ahead in its own (local) frame: turns with the actor like its own
      // eyes -- the one view where that's actually wanted, unlike every
      // other case below (see ViewCommand::worldFrame). The distance box
      // doesn't apply to this view (same as GuiderRobotManager's
      // equivalent).
      command.followOffset = {0.1, 0.0, kEyeHeight};
      command.trackOffset = {3.0, 0.0, kEyeHeight - 0.05};
      command.worldFrame = false;
      break;
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

  // The MinimalScene plugin tags the GUI camera with this user data.
  if (!this->userCamera)
  {
    for (unsigned int i = 0; i < scene->NodeCount(); ++i)
    {
      auto camera = std::dynamic_pointer_cast<gz::rendering::Camera>(
          scene->NodeByIndex(i));
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
      return;
    // First time the camera is found, it's still wherever gui.config's
    // MinimalScene <camera_pose> placed it -- nothing has engaged
    // follow/track yet at this point in a fresh launch. Save it so
    // resetToInitialView() has a real pose to snap back to.
    this->initialCameraPose = this->userCamera->WorldPose();
    this->initialCameraPoseCaptured = true;
  }

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

gz::rendering::VisualPtr HumanControlPanel::FindCollisionCapsuleVisual(
    const gz::rendering::ScenePtr &_scene, const std::string &_modelName) const
{
  // Exact lookup first -- the same one ApplyViewpoint() already relies on
  // successfully for model-level nodes -- then the same "id::"-scoped
  // suffix fallback it uses for the same reason.
  gz::rendering::VisualPtr modelVisual = _scene->VisualByName(_modelName);
  if (!modelVisual)
  {
    const std::string suffix = "::" + _modelName;
    for (unsigned int i = 0; i < _scene->VisualCount(); ++i)
    {
      auto visual = _scene->VisualByIndex(i);
      if (!visual)
        continue;
      const std::string &name = visual->Name();
      if (name.size() >= suffix.size() &&
          name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        modelVisual = visual;
        break;
      }
    }
  }
  if (!modelVisual)
    return nullptr;

  // modelVisual is the model's own top-level node -- it has no geometry
  // of its own, the capsule geometry/material live on a Visual some
  // number of levels below it (link, then visual). Recurse instead of
  // assuming a fixed depth, since that fixed-depth assumption ("<model
  // name>_collision::" as a plain substring, matching against a supposed
  // "<model>::<link>::visual" name) is what silently never matched
  // anything before this -- 当たり判定表示 doing nothing, and the capsule
  // staying stuck at model.sdf's own baked-in 0.55 transparency (visible
  // by default) instead of the 0.9 "hidden" state ApplyCollisionVisibility()
  // was supposed to apply from tick one, both trace back to this search
  // never finding its target.
  std::function<gz::rendering::VisualPtr(const gz::rendering::VisualPtr &)> findGeometryVisual =
      [&](const gz::rendering::VisualPtr &_visual) -> gz::rendering::VisualPtr
  {
    if (_visual->GeometryCount() > 0 && _visual->Material())
      return _visual;
    for (unsigned int i = 0; i < _visual->ChildCount(); ++i)
    {
      auto child = std::dynamic_pointer_cast<gz::rendering::Visual>(_visual->ChildByIndex(i));
      if (!child)
        continue;
      auto found = findGeometryVisual(child);
      if (found)
        return found;
    }
    return nullptr;
  };
  return findGeometryVisual(modelVisual);
}

void HumanControlPanel::ApplyCollisionVisibility()
{
  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!scene)
    return;

  for (auto &human : this->humans)
  {
    // Only actor-backed humans get a human_collision_body companion at
    // all (see spawn_human.launch.py) -- same check isHumanActorAt() uses.
    if (!human.velocityPublisher.Valid())
      continue;

    const float desired = human.showCollision
        ? kCollisionVisibleTransparency : kCollisionHiddenTransparency;

    if (human.collisionMaterial)
    {
      if (human.appliedCollisionTransparency != desired)
      {
        human.collisionMaterial->SetTransparency(desired);
        human.appliedCollisionTransparency = desired;
      }
      continue;
    }

    if (human.collisionVisualRetries > kCollisionVisualMaxRetries)
      continue;

    const std::string collisionModelName = human.name + "_collision";
    gz::rendering::VisualPtr found = this->FindCollisionCapsuleVisual(scene, collisionModelName);
    if (!found || !found->Material())
    {
      // Diagnostic (fires once, on this human's very first failed search,
      // not every retried frame): if FindCollisionCapsuleVisual() still
      // can't find a geometry-bearing visual under collisionModelName even
      // with the exact-name + suffix + recursive-descent lookup, dumping
      // every scene visual whose name contains "collision" shows whatever
      // naming scheme this gz-sim version/scene actually uses so the
      // lookup can be adjusted to match it.
      if (human.collisionVisualRetries == 0)
      {
        gzmsg << "[HumanControlPanel] collision-visual '" << collisionModelName
              << "' not found among " << scene->VisualCount()
              << " scene visuals. Names containing \"collision\": ";
        unsigned int logged = 0;
        for (unsigned int i = 0; i < scene->VisualCount() && logged < 40; ++i)
        {
          auto visual = scene->VisualByIndex(i);
          if (visual && visual->Name().find("collision") != std::string::npos)
          {
            gzmsg << "[" << visual->Name() << "] ";
            ++logged;
          }
        }
        if (logged == 0)
          gzmsg << "(none)";
        gzmsg << std::endl;
      }
      ++human.collisionVisualRetries;
      continue;
    }
    // Clone rather than mutate the shared material directly -- otherwise
    // every human's capsule (they all reference the same SDF-defined
    // material) would flip transparency together instead of independently.
    auto material = found->Material()->Clone();
    found->SetMaterial(material, false);
    human.collisionMaterial = material;
    human.collisionMaterial->SetTransparency(desired);
    human.appliedCollisionTransparency = desired;
  }
}
}  // namespace gz_human_sim

GZ_ADD_PLUGIN(
  gz_human_sim::HumanControlPanel,
  gz::gui::Plugin)
