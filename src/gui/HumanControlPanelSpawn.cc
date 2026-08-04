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


#include "HumanControlPanelInternal.hh"
#include "LaunchProcess.hh"
#include "WorldEntityService.hh"

// Spawn/removal pipeline: launch-process management, the collision probe
// that finds a clear spawn position, and entity-existence confirmation.
//
// Split out of the single-file HumanControlPanel.cc; the code below is
// unchanged from that file.

namespace gz_human_sim
{

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
  if (world_entity::Exists(this->node, this->worldName, nameStd))
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

void HumanControlPanel::spawnHumans(
    int _modelIndex, const QString &_baseName, int _count,
    const QString &_posePreset, const QString &_followMode,
    double _x, double _y, double _z, double _yaw)
{
  const QString baseName = _baseName.trimmed();
  if (baseName.isEmpty())
  {
    this->SetStatus("名前を入力してください");
    return;
  }

  // Copy the picked points before spawning: spawnHuman() -> ... ->
  // PollSpawnConfirmation() eventually fires humansChanged(), and clearing
  // the list below would invalidate anything we were still walking.
  const auto points = this->pendingSpawnPoints;
  const bool usePicked = !points.empty();
  const int count = usePicked
      ? static_cast<int>(points.size()) : std::clamp(_count, 1, 50);

  // Numbering continues past whoever already exists, so a second batch
  // never collides with the first one's names.
  const int offset = static_cast<int>(this->humans.size());
  for (int i = 0; i < count; ++i)
  {
    const QString name = baseName + QString::number(offset + i + 1);
    double x = 0.0;
    double y = 0.0;
    // Typed coordinates treat the form's z as an absolute world height,
    // which is what it has always meant. A picked point instead treats it
    // as the model's ground offset (defaultZ(): 1.0 for walking_actor,
    // 0.0 for DoctorFemaleWalk, ...) and adds it to the height of the
    // surface that was clicked -- that sum is what puts the human on top
    // of the floor they were dropped on, on any storey.
    double z = _z;
    if (usePicked)
    {
      // Exactly where each one was clicked.
      const auto &point = points[static_cast<std::size_t>(i)];
      x = point.x;
      y = point.y;
      z = point.z + _z;
    }
    else
    {
      // Spread kSpawnGridSpacing apart on the same grid nextSpawnX()/Y()
      // already uses, re-based at (_x, _y) -- so this batch's own members
      // are pre-separated before each one's individual
      // ProbeSafeSpawnPosition() furniture-avoidance probe ever runs (that
      // probe protects against pre-existing furniture/humans, not against N
      // brand new spawns landing on top of each other, since it only ever
      // reads pose state that already exists in gz-sim).
      x = _x + (i % kSpawnGridColumns) * kSpawnGridSpacing;
      y = _y + (i / kSpawnGridColumns) * kSpawnGridSpacing;
    }
    this->spawnHuman(_modelIndex, name, _posePreset, _followMode, x, y, z, _yaw);
  }

  if (usePicked)
  {
    this->pendingSpawnPoints.clear();
    this->spawnMarkers.InvalidatePending();
    this->pendingSpawnPointsChanged();
  }
  this->SetStatus(QString("%1人のspawnを開始しました（%2%3〜%2%4・%5）")
      .arg(count).arg(baseName)
      .arg(offset + 1).arg(offset + count)
      .arg(usePicked ? "選択した地点" : "座標指定"));
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

  auto *process = launch_process::Start(this, arguments);
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
  if (_attempt >= kSpawnSafetyMaxAttempts || !this->collisionBody.HasTemplate())
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
  // 寸法はテンプレートの既定値のまま。プローブは「人物と同じ太さのものが
  // そこに入るか」を見るためのものなので、変えてはいけない。
  const std::string probeSdf = this->collisionBody.BuildSdf(probeName);

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
  // candidate was reached by.
  //
  // Attempt 0 (offset (0, 0), the exact requested point) has no offset to
  // derive a direction from. It approaches from the GUI camera's side
  // instead of the old fixed +Y: the operator picked this point by clicking
  // it, so the camera's line of sight to it is by definition unobstructed,
  // making that the one direction the walk-in can't be blocked from by the
  // very furniture the probe exists to detect. A fixed +Y walk-in is what
  // made the check effectively test a point off to one side of the clicked
  // one.
  double approachDirX = 0.0;
  double approachDirY = 1.0;
  const double offsetLength = std::hypot(offsetX, offsetY);
  if (offsetLength > 1e-6)
  {
    approachDirX = offsetX / offsetLength;
    approachDirY = offsetY / offsetLength;
  }
  else
  {
    double camX = 0.0;
    double camY = 0.0;
    if (this->cameraController.GroundPosition(camX, camY))
    {
      const double toPointX = candidateX - camX;
      const double toPointY = candidateY - camY;
      const double length = std::hypot(toPointX, toPointY);
      if (length > 1e-3)
      {
        approachDirX = toPointX / length;
        approachDirY = toPointY / length;
      }
    }
  }
  const double approachStartX = candidateX - approachDirX * kSpawnProbeApproach;
  const double approachStartY = candidateY - approachDirY * kSpawnProbeApproach;

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
      // Judged by ARRIVAL at the candidate, not by distance covered from
      // the start: the question this check exists to answer is "is the
      // candidate point itself free?", and a probe stopped by something
      // between the two ends up short of the candidate either way. Measuring
      // from the start instead let a probe that had been shoved sideways
      // (travelled the distance, but not to the right place) count as
      // success.
      const double distanceToCandidate =
          std::hypot(it->second.x - _candidateX, it->second.y - _candidateY);
      safe = distanceToCandidate <= kSpawnSafetyDisplacementMeters;
    }
    this->poses.erase(probeNameStd);
    this->activeProbes.erase(probeNameStd);
  }
  (void)_approachStartX;
  (void)_approachStartY;
  // Always clean up the probe, safe or not -- it was only ever a test.
  world_entity::Remove(this->node, this->worldName, probeNameStd);

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
  if (world_entity::Exists(this->node, this->worldName, probeNameStd) && _pollAttempt < kProbeRemovalMaxAttempts)
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
  if (!world_entity::Exists(this->node, this->worldName, nameStd))
  {
    if (_attempt + 1 >= kEntityPollMaxAttempts)
    {
      this->SetStatus(_name + " のspawnに失敗しました（"
          + QString::number(kEntityPollMaxAttempts * kEntityPollIntervalMs / 1000)
          + "秒待っても見つかりません）");
      launch_process::TerminateGroup(this, _process);
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
  // Where this human actually ended up (after any spawn-safety adjustment
  // -- _x/_y are already the adjusted values by the time this runs), which
  // is what the spawn marker marks. Colour by spawn order, wrapping.
  human.marker.x = _x;
  human.marker.y = _y;
  human.marker.z = _z;
  human.marker.colorIndex = static_cast<int>(this->humans.size()) % kMarkerColorCount;
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
    if (IsPoseCapableIndex(modelIndex))
    {
      const std::string poseTopic = "/" + nameStd + "/cmd_pose";
      human.posePublisher = this->node.Advertise<gz::msgs::StringMsg>(poseTopic);
    }
    // sfm_enable_topic: matches exactly what SfmCrowdSystem::
    // ApplyRegistration() derives from a plain name ("/<name>/sfm_enable"),
    // see confirmRoute()/setSfmEnabled(). Advertised for every actor-backed
    // human regardless of useSfm so the toggle already works the moment a
    // route is later confirmed under SFM mode -- no separate re-wiring step.
    human.route.sfmEnablePublisher =
        this->node.Advertise<gz::msgs::Boolean>("/" + nameStd + "/sfm_enable");
  }
  this->humans.push_back(std::move(human));
  const int newIndex = static_cast<int>(this->humans.size()) - 1;
  // サーバーが publish する状態を購読する（構想書 §3）。この人物が
  // いま何をしているかは、このパネルではなくサーバーが決めます。
  this->SubscribeCharacterState(newIndex);
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
  // 状態の購読を解除してから消す。残しておくと、同じ名前で再 spawn した
  // ときに古いハンドラが二重に走ります。
  if (!human.server.topic.empty())
    this->node.Unsubscribe(human.server.topic);
  this->humans.erase(static_cast<std::size_t>(_index));
  this->humansChanged();

  // Scene nodes may only be destroyed on the render thread; hand this
  // human's spawn marker over to ApplySpawnMarkers() to clean up.
  {
    this->spawnMarkers.QueueRemoval("__spawn_marker_" + human.name);
  }

  // Harmless no-op if this human was never registered with SfmCrowdSystem
  // (it simply won't match any tracked name there) -- always sent anyway
  // so a removed-then-recreated same-named human never inherits a stale
  // route left over from before.
  this->EnsureSfmPublishers();
  gz::msgs::StringMsg unregisterMessage;
  unregisterMessage.set_data(human.name);
  this->sfmUnregisterPublisher.Publish(unregisterMessage);

  // Keep the keyboard target pointing at the same logical human across the
  // index shift caused by erase(), or clear it if that's the one removed.
  if (this->activeHumanIndex == _index)
    this->activeHumanIndex = -1;
  else if (this->activeHumanIndex > _index)
    --this->activeHumanIndex;
  this->activeHumanChanged();
  this->activeViewIndexChanged();
  this->activeFollowModeChanged();

  // Nobody left to follow: the camera would otherwise stay locked onto the
  // last human's final position, leaving the operator staring at an empty
  // patch of floor with no obvious way back. Return it to the world's own
  // opening overview instead.
  if (this->humans.empty())
    this->resetToInitialView();

  launch_process::TerminateGroup(this, human.process);

  const QString name = QString::fromStdString(human.name);
  if (!world_entity::Exists(this->node, this->worldName, human.name))
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
    // world_entity::Remove() のコメント参照), so ask the actor's own
    // ActorCommandPlugin to remove itself via the ECM instead.
    gz::msgs::Empty message;
    human.removePublisher.Publish(message);
  }
  else
  {
    world_entity::Remove(this->node, this->worldName, human.name);
  }
  this->SetStatus(name + " を削除中…（確認待ち）");
  QTimer::singleShot(kEntityPollIntervalMs, this,
      [this, name]() { this->PollRemovalConfirmation(name, 0); });
}

void HumanControlPanel::PollRemovalConfirmation(QString _name, int _attempt)
{
  const std::string nameStd = _name.toStdString();
  if (!world_entity::Exists(this->node, this->worldName, nameStd))
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
}  // namespace gz_human_sim
