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
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>


#include "HumanControlPanelInternal.hh"

// Core: construction, config load, world discovery, the roster feed, and
// the Q_PROPERTY/model-table accessors QML reads.
//
// Split out of the single-file HumanControlPanel.cc; the code below is
// unchanged from that file.

namespace gz_human_sim
{
HumanControlPanel::HumanControlPanel()
  : gz::gui::Plugin()
{
  // The roster goes out whenever the list changes (so the pad reacts to a
  // spawn immediately) and every 2s regardless (so a pad controller that
  // was loaded after the humans were spawned still learns about them --
  // gz-transport has no retained-message/latching semantics to lean on).
  this->rosterPublisher =
      this->node.Advertise<gz::msgs::StringMsg_V>("/guider/targets/humans");
  QObject::connect(
      this, &HumanControlPanel::humansChanged,
      this, &HumanControlPanel::PublishRoster);
  this->rosterTimer = new QTimer(this);
  this->rosterTimer->setInterval(2000);
  QObject::connect(
      this->rosterTimer, &QTimer::timeout,
      this, &HumanControlPanel::PublishRoster);
  this->rosterTimer->start();
}

void HumanControlPanel::PublishRoster()
{
  if (!this->rosterPublisher.Valid())
    return;

  gz::msgs::StringMsg_V message;
  for (const auto &human : this->humans)
  {
    // Only actor-backed humans have a velocity publisher, and only they
    // can be driven -- see spawnHumans(). A human without one would
    // silently swallow every Twist the pad sent.
    if (!human.velocityPublisher.Valid())
      continue;

    // Only pose-capable actors have a cmd_pose publisher (see
    // IsPoseCapableIndex()); for the rest the channel stays empty and the
    // pad's ○△×□ do nothing rather than publishing into the void.
    std::string poseChannel;
    std::string poses[4] = {"", "", "", ""};
    if (human.posePublisher.Valid())
    {
      poseChannel = "gztopic:/" + human.name + "/cmd_pose";
      // Bound only if the loaded human_pose_presets.yaml actually defines
      // them -- that file is the user's to edit, so a hardcoded name can
      // go stale, and UpdatePoseIntent() would then publish a pose the
      // actor plugin does not know.
      const char *const preferred[4] = {
        "raise_right_hand", "cross_arms", "initial_pose", "sit_on_chair"};
      for (int i = 0; i < 4; ++i)
      {
        if (this->posePresetList.contains(QString(preferred[i])))
          poses[i] = preferred[i];
      }
    }

    // kind|type|name|cmdVelTopic|holonomic|yawSign|maxLinear|maxAngular
    //   |poseChannel|pose1|pose2|pose3|pose4|label
    // `type` is empty for humans: it keys guide_robot's joint table, which
    // only covers robot arms.
    // -- format owned by guide_robot's src/GuiderTargetRoster.hh; see the
    // rosterPublisher comment in the header. Humans strafe (holonomic=1)
    // and need no yaw correction (yawSign=1). The speeds are the same
    // constants PollDualsense() applies, before speedMultiplier: that is a
    // live slider, and baking its current value into a 2s-stale roster
    // entry would make the pad drive at whatever it was two seconds ago.
    message.add_data(
        "human||" + human.name + "|/" + human.name + "/cmd_vel|1|1.000000|" +
        std::to_string(kAdvertisedMaxLinear) + "|" +
        std::to_string(kAdvertisedMaxAngular) + "|" + poseChannel + "|" +
        poses[0] + "|" + poses[1] + "|" + poses[2] + "|" + poses[3] +
        "|" + human.name + "（人物）");
  }

  this->rosterPublisher.Publish(message);
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
  // Magic-circle texture for the spawn markers (see CreateMarkerVisual).
  // Resolved from the same prefix; regenerate the asset itself with
  // `ros2 run guider_multifloor_builder guider_make_spawn_marker`, which
  // writes this copy and guide_robot's identical one together.
  {
    const std::string markerPath =
        packagePrefix + "/share/gz_human_sim/media/spawn_marker.png";
    if (std::ifstream(markerPath))
      this->spawnMarkerTexturePath = markerPath;
    else
      gzwarn << "[HumanControlPanel] spawn marker texture not found at "
             << markerPath << "; markers will be drawn untextured.\n";
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

  // A single application-wide filter covers everything eventFilter() needs:
  // Render and LeftClickToScene (both only ever sent to MainWindow) as well
  // as the QWEASDZXC keyboard shortcuts (which target whichever QQuickItem
  // currently has focus, e.g. the 3D scene, not MainWindow). Installing on
  // the Application object -- the one QObject whose installEventFilter()
  // acts application-wide rather than per-object -- reaches all of them.
  //
  // This used to ALSO install a second filter directly on MainWindow ("Render
  // events are only ever sent to MainWindow, so a filter there is enough for
  // that"), which was true but redundant: MainWindow is exactly the kind of
  // object the Application-wide filter already covers. Since Qt calls every
  // installed filter for a given event (global ones first, then the target
  // object's own), that second filter didn't watch anything the first one
  // missed -- it just ran eventFilter() a second time for every event
  // MainWindow received, which for Render was harmless (ApplyViewpoint() et
  // al. are idempotent) but for LeftClickToScene meant every single click
  // appended two identical spawn/route points instead of one.
  gz::gui::App()->installEventFilter(this);

  this->DiscoverWorld();
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
      if (travelled >= kSpawnProbeApproach && probeIt->second.publisher)
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







QString HumanControlPanel::poseLabel(const QString &_pose) const
{
  const std::string pose = _pose.toStdString();
  for (int i = 0; i < kPoseShortcutCount; ++i)
  {
    if (pose == kPoseShortcuts[i].pose)
      return kPoseShortcuts[i].label;
  }
  return kNoPoseLabel;
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

bool HumanControlPanel::RouteRecording() const
{
  return this->routeRecordingState;
}

QStringList HumanControlPanel::PendingRoutePoints() const
{
  QStringList result;
  for (const auto &point : this->pendingRoute)
  {
    result << QString("%1, %2")
        .arg(point.first, 0, 'f', 2).arg(point.second, 0, 'f', 2);
  }
  return result;
}

QStringList HumanControlPanel::PendingSpawnPoints() const
{
  QStringList result;
  for (const auto &point : this->pendingSpawnPoints)
  {
    result << QString("%1, %2, %3")
        .arg(point.x, 0, 'f', 2).arg(point.y, 0, 'f', 2)
        .arg(point.z, 0, 'f', 2);
  }
  return result;
}

bool HumanControlPanel::SpawnPicking() const
{
  return this->spawnPickingState;
}

int HumanControlPanel::RouteTargetCount() const
{
  int count = 0;
  for (const auto &human : this->humans)
  {
    if (human.routeTarget)
      ++count;
  }
  return count;
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
  // 姿勢は「対象」の付け替えで republish していたが、姿勢を含む操作は
  // このパッケージから外へ出た（構想書 §11）。いまはサーバーが状態を
  // publish し、このパネルはそれを表示するだけ。
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

bool HumanControlPanel::isPoseCapableModel(int _modelIndex) const
{
  return IsPoseCapableIndex(_modelIndex);
}

bool HumanControlPanel::isPoseCapableHumanAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(_index).posePublisher.Valid();
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

bool HumanControlPanel::isHumanActorAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(_index).velocityPublisher.Valid();
}
}  // namespace gz_human_sim


GZ_ADD_PLUGIN(
  gz_human_sim::HumanControlPanel,
  gz::gui::Plugin)
