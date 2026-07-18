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
#include <gz/msgs/pose_v.pb.h>
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
    this->animationName = _sdf->Get<std::string>("animation_name", "walk").first;
    this->animationFactor = _sdf->Get<double>("animation_factor", 4.0).first;
    this->linearVelocity = _sdf->Get<double>("linear_velocity", 1.0).first;
    this->linearTolerance = _sdf->Get<double>("linear_tolerance", 0.1).first;
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
    if (!this->transportNode.Subscribe(this->velocityTopic,
          &ActorCommandPlugin::VelocityCallback, this))
      gzerr << "Failed to subscribe to velocity topic: " << this->velocityTopic << std::endl;
    if (!this->transportNode.Subscribe(this->pathTopic,
          &ActorCommandPlugin::PathCallback, this))
      gzerr << "Failed to subscribe to path topic: " << this->pathTopic << std::endl;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
      return;
    auto trajectoryPose = _ecm.Component<gz::sim::components::TrajectoryPose>(
        this->entity);
    if (trajectoryPose == nullptr)
    {
      _ecm.CreateComponent(this->entity,
          gz::sim::components::TrajectoryPose(gz::math::Pose3d::Zero));
      trajectoryPose = _ecm.Component<gz::sim::components::TrajectoryPose>(
          this->entity);
    }
    if (trajectoryPose == nullptr)
      return;
    const gz::math::Pose3d currentPose = trajectoryPose->Data();
    gz::math::Pose3d nextPose = currentPose;
    double distanceTravelled = 0.0;
    this->ApplyNewestVelocity();
    if (!this->ApplyPathCommand(nextPose, dt, distanceTravelled))
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
      std::lock_guard<std::mutex> lock(this->commandMutex);
      this->pathCommands.push(std::move(path));
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
  private: std::string animationName;
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
