#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector2.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace gz_human_sim
{
class ActorCommandPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->actor = gz::sim::Actor(_entity);
    this->entity = _entity;
    if (_ecm.Component<gz::sim::components::Actor>(_entity) == nullptr)
    {
      gzerr << "ActorCommandPlugin must be attached to an <actor>." << std::endl;
      return;
    }
    this->velocityTopic = _sdf->Get<std::string>("vel_topic", "/cmd_vel").first;
    this->pathTopic = _sdf->Get<std::string>("path_topic", "/cmd_path").first;
    this->removeTopic = _sdf->Get<std::string>("remove_topic", "/remove_actor").first;
    this->followModeTopic =
        _sdf->Get<std::string>("follow_mode_topic", "/set_follow_mode").first;
    this->animationName = _sdf->Get<std::string>("animation_name", "walk").first;
    this->animationFactor = _sdf->Get<double>("animation_factor", 4.0).first;
    this->linearVelocity = _sdf->Get<double>("linear_velocity", 1.0).first;
    this->linearTolerance = _sdf->Get<double>("linear_tolerance", 0.1).first;
    // "auto" (default): path takes over whenever one is queued/active,
    // otherwise velocity. "path"/"velocity" pin the actor to just one
    // command source, matching gazebo-ros-actor-plugin's follow_mode
    // concept (that plugin only offers path/velocity; "auto" is
    // gz_human_sim's own addition so the GUI teleop pad and cmd_path
    // waypoints can both just work without the caller having to pick
    // a mode up front). Also changeable at runtime via follow_mode_topic,
    // not just this spawn-time SDF default -- see SetFollowMode().
    this->SetFollowMode(_sdf->Get<std::string>("follow_mode", "auto").first);
    auto animationName = _ecm.Component<gz::sim::components::AnimationName>(
        this->entity);
    if (animationName == nullptr)
      _ecm.CreateComponent(this->entity,
          gz::sim::components::AnimationName(this->animationName));
    else
      *animationName = gz::sim::components::AnimationName(this->animationName);
    _ecm.SetChanged(this->entity, gz::sim::components::AnimationName::typeId,
        gz::sim::ComponentState::OneTimeChange);

    if (_ecm.Component<gz::sim::components::AnimationTime>(this->entity) == nullptr)
      _ecm.CreateComponent(this->entity,
          gz::sim::components::AnimationTime(this->animationTime));

    // The actor's spawn pose (from `ros_gz_sim create -x/-y/-z/-Y`) lands in
    // the base Pose component. Without this block, TrajectoryPose used to
    // start at literal (0,0,0,yaw=0) on the first PreUpdate() tick
    // regardless of where/how the actor was actually spawned. That yaw
    // mismatch is the real-world impact: PreUpdate() reads its current
    // facing via trajectoryPose->Data().Rot().Yaw(), so a rotated spawn
    // (`-Y` != 0) would compute its very first velocity step using yaw=0
    // instead of the actor's real facing, sending it off in the wrong
    // direction until its internally-tracked yaw caught up. Seeding
    // TrajectoryPose with the actual spawn pose fixes that from tick one.
    // X/Y are then zeroed on the base Pose so the spawn offset isn't also
    // counted a second time underneath the TrajectoryPose offset. Z stays
    // on the base Pose (it encodes the model's static height correction,
    // e.g. walking_actor's mesh offset) and is zeroed on the TrajectoryPose
    // side instead, since PreUpdate() never changes Z.
    //
    // Ported from gz_human_sim's own develop/yk branch (which fixed this
    // independently of gazebo-ros-actor-plugin -- that plugin's
    // Configure() does the same base-Pose/TrajectoryPose seeding, for the
    // same reason).
    auto poseComponent = _ecm.Component<gz::sim::components::Pose>(this->entity);
    gz::math::Pose3d initialPose = gz::math::Pose3d::Zero;
    if (poseComponent != nullptr)
    {
      initialPose = poseComponent->Data();
      gz::math::Pose3d basePose = initialPose;
      basePose.Pos().X(0.0);
      basePose.Pos().Y(0.0);
      *poseComponent = gz::sim::components::Pose(basePose);
    }
    // Create TrajectoryPose here rather than lazily in PreUpdate: PreUpdate
    // returns immediately while the world is paused (e.g. a launch file
    // that doesn't pass `-r`), which would otherwise leave this actor with
    // AnimationName/AnimationTime but no TrajectoryPose. gz-sim's render
    // thread assumes actors with animation components also have a
    // TrajectoryPose, and segfaults in RenderUtil::UpdateAnimation when
    // that assumption doesn't hold.
    if (_ecm.Component<gz::sim::components::TrajectoryPose>(this->entity) == nullptr)
    {
      gz::math::Pose3d initialTrajectoryPose = initialPose;
      initialTrajectoryPose.Pos().Z(0.0);
      _ecm.CreateComponent(this->entity,
          gz::sim::components::TrajectoryPose(initialTrajectoryPose));
    }

    if (!this->transportNode.Subscribe(this->velocityTopic,
          &ActorCommandPlugin::VelocityCallback, this))
      gzerr << "Failed to subscribe to velocity topic: " << this->velocityTopic << std::endl;
    if (!this->transportNode.Subscribe(this->pathTopic,
          &ActorCommandPlugin::PathCallback, this))
      gzerr << "Failed to subscribe to path topic: " << this->pathTopic << std::endl;
    if (!this->transportNode.Subscribe(this->removeTopic,
          &ActorCommandPlugin::RemoveCallback, this))
      gzerr << "Failed to subscribe to remove topic: " << this->removeTopic << std::endl;
    if (!this->transportNode.Subscribe(this->followModeTopic,
          &ActorCommandPlugin::FollowModeCallback, this))
      gzerr << "Failed to subscribe to follow_mode topic: " << this->followModeTopic
            << std::endl;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      if (this->removalRequested)
      {
        // /world/<w>/remove (gz-sim's UserCommands system) only accepts
        // MODEL or LIGHT entities -- it rejects an ACTOR outright with
        // "Entity [n] is not a model or a light, so it can't be removed.",
        // regardless of what Entity::Type is set on the request. There is
        // no server-side service that can remove an actor. Removing it via
        // EntityComponentManager::RequestRemoveEntity() from inside the
        // actor's own System (this PreUpdate) isn't subject to that
        // restriction, so that's what HumanControlPanel's remove button
        // actually triggers for actor-backed humans (see remove_topic).
        gzmsg << "ActorCommandPlugin: removing self on request." << std::endl;
        _ecm.RequestRemoveEntity(this->entity, true);
        return;
      }
    }
    if (_info.paused)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
      return;
    auto trajectoryPose = _ecm.Component<gz::sim::components::TrajectoryPose>(
        this->entity);
    if (trajectoryPose == nullptr)
      return;
    const gz::math::Pose3d currentPose = trajectoryPose->Data();
    gz::math::Pose3d nextPose = currentPose;
    double distanceTravelled = 0.0;
    this->ApplyNewestVelocity();
    // Snapshot under the lock rather than reading this->followMode
    // directly below: FollowModeCallback() (SetFollowMode()) can now
    // rewrite it at runtime from the transport thread, concurrently with
    // this PreUpdate() running on the simulation thread.
    std::string followMode;
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      followMode = this->followMode;
    }
    // follow_mode gates which command source may drive the actor this tick:
    //   "auto"     -- path takes over whenever one is queued/active, otherwise
    //                 velocity (this is the pre-existing behaviour, unchanged).
    //   "path"     -- velocity commands are never applied, even if no path is
    //                 active (the actor simply stands still, matching upstream).
    //   "velocity" -- path commands are accepted on the topic but never applied.
    bool pathApplied = false;
    if (followMode != "velocity")
      pathApplied = this->ApplyPathCommand(nextPose, dt, distanceTravelled);
    if (!pathApplied && followMode != "path")
    {
      const double yaw = currentPose.Rot().Yaw();
      const double dx = (this->velocity.X() * std::cos(yaw) -
          this->velocity.Y() * std::sin(yaw)) * dt;
      const double dy = (this->velocity.X() * std::sin(yaw) +
          this->velocity.Y() * std::cos(yaw)) * dt;
      nextPose.Pos().X(currentPose.Pos().X() + dx);
      nextPose.Pos().Y(currentPose.Pos().Y() + dy);
      nextPose.Rot() = gz::math::Quaterniond(0.0, 0.0,
          yaw + this->velocity.Z() * dt);
      distanceTravelled = std::hypot(dx, dy);
    }
    *trajectoryPose = gz::sim::components::TrajectoryPose(nextPose);
    _ecm.SetChanged(this->entity, gz::sim::components::TrajectoryPose::typeId,
        gz::sim::ComponentState::OneTimeChange);
    if (distanceTravelled > 0.0)
    {
      const auto animationStep = std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(distanceTravelled * this->animationFactor));
      this->animationTime += animationStep;
      auto animationTime = _ecm.Component<gz::sim::components::AnimationTime>(
          this->entity);
      if (animationTime != nullptr)
      {
        *animationTime = gz::sim::components::AnimationTime(this->animationTime);
        _ecm.SetChanged(this->entity, gz::sim::components::AnimationTime::typeId,
            gz::sim::ComponentState::OneTimeChange);
      }
    }
  }

  private: void VelocityCallback(const gz::msgs::Twist &_message)
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->velocityCommands.emplace(_message.linear().x(), _message.linear().y(),
        _message.angular().z());
  }

  private: void RemoveCallback(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->removalRequested = true;
  }

  /// \brief Validate and apply a follow_mode value, used both for the
  /// spawn-time SDF default (Configure()) and for runtime changes received
  /// on follow_mode_topic (FollowModeCallback()) -- switching an
  /// already-spawned actor between teleop-reactive ("auto") and
  /// path-locked ("path"/"velocity") without having to despawn/respawn it.
  private: void SetFollowMode(const std::string &_mode)
  {
    std::string mode = _mode;
    if (mode != "auto" && mode != "path" && mode != "velocity")
    {
      gzwarn << "ActorCommandPlugin: unknown follow_mode '" << mode
             << "', falling back to 'auto'. Valid values: auto, path, velocity."
             << std::endl;
      mode = "auto";
    }
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->followMode = mode;
    gzmsg << "ActorCommandPlugin: follow_mode is now '" << this->followMode << "'."
          << std::endl;
  }

  private: void FollowModeCallback(const gz::msgs::StringMsg &_message)
  {
    this->SetFollowMode(_message.data());
  }

  private: void PathCallback(const gz::msgs::Pose_V &_message)
  {
    std::vector<gz::math::Pose3d> path;
    path.reserve(_message.pose_size());
    for (int i = 0; i < _message.pose_size(); ++i)
    {
      const auto &pose = _message.pose(i);
      path.emplace_back(gz::math::Vector3d(pose.position().x(), pose.position().y(),
          pose.position().z()), gz::math::Quaterniond(pose.orientation().w(),
          pose.orientation().x(), pose.orientation().y(), pose.orientation().z()));
    }
    if (!path.empty())
    {
      const std::size_t waypointCount = path.size();
      {
        std::lock_guard<std::mutex> lock(this->commandMutex);
        this->pathCommands.push(std::move(path));
      }
      gzmsg << "ActorCommandPlugin: received new path with " << waypointCount
            << " waypoint(s)." << std::endl;
    }
    else
    {
      gzwarn << "ActorCommandPlugin: received an empty path, ignoring."
             << std::endl;
    }
  }

  private: void ApplyNewestVelocity()
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    while (!this->velocityCommands.empty())
    {
      this->velocity = this->velocityCommands.front();
      this->velocityCommands.pop();
    }
  }

  private: bool ApplyPathCommand(gz::math::Pose3d &_pose, double _dt,
      double &_distanceTravelled)
  {
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      if (!this->pathCommands.empty())
      {
        this->path = std::move(this->pathCommands.front());
        this->pathCommands.pop();
        this->pathIndex = 0u;
        this->velocity = gz::math::Vector3d::Zero;
      }
    }
    if (this->pathIndex >= this->path.size())
      return false;
    const auto &target = this->path[this->pathIndex];
    gz::math::Vector2d offset(target.Pos().X() - _pose.Pos().X(),
        target.Pos().Y() - _pose.Pos().Y());
    const double remainingDistance = offset.Length();
    if (remainingDistance <= this->linearTolerance)
    {
      _pose.Pos().X(target.Pos().X());
      _pose.Pos().Y(target.Pos().Y());
      _pose.Rot() = target.Rot();
      ++this->pathIndex;
      if (this->pathIndex >= this->path.size())
        gzmsg << "ActorCommandPlugin: path completed (" << this->path.size()
              << " waypoint(s))." << std::endl;
      return true;
    }
    const double step = std::min(this->linearVelocity * _dt, remainingDistance);
    offset /= remainingDistance;
    _pose.Pos().X(_pose.Pos().X() + offset.X() * step);
    _pose.Pos().Y(_pose.Pos().Y() + offset.Y() * step);
    _pose.Rot() = gz::math::Quaterniond(0.0, 0.0, std::atan2(offset.Y(), offset.X()));
    _distanceTravelled = step;
    return true;
  }

  private: gz::sim::Entity entity{gz::sim::kNullEntity};
  private: gz::sim::Actor actor;
  private: gz::transport::Node transportNode;
  private: std::mutex commandMutex;
  private: std::queue<gz::math::Vector3d> velocityCommands;
  private: std::queue<std::vector<gz::math::Pose3d>> pathCommands;
  private: std::vector<gz::math::Pose3d> path;
  private: std::size_t pathIndex{0u};
  private: gz::math::Vector3d velocity{0.0, 0.0, 0.0};
  private: std::string velocityTopic;
  private: std::string pathTopic;
  private: std::string removeTopic;
  private: std::string followModeTopic;
  private: bool removalRequested{false};
  private: std::string animationName;
  private: std::string followMode{"auto"};
  private: double animationFactor{4.0};
  private: double linearVelocity{1.0};
  private: double linearTolerance{0.1};
  private: std::chrono::steady_clock::duration animationTime{
      std::chrono::steady_clock::duration::zero()};
};
}  // namespace gz_human_sim

GZ_ADD_PLUGIN(gz_human_sim::ActorCommandPlugin, gz::sim::System,
    gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz_human_sim::ActorCommandPlugin,
    "gz_human_sim::ActorCommandPlugin")
