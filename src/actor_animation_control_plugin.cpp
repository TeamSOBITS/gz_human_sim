#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace gz_human_sim
{
class ActorAnimationControlPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate
{
  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->entity = _entity;
    this->actor = gz::sim::Actor(_entity);

    if (_sdf->HasElement("animation_name"))
      this->animationName = _sdf->Get<std::string>("animation_name");

    if (_sdf->HasElement("playback_speed"))
      this->playbackSpeed = _sdf->Get<double>("playback_speed");

    if (_sdf->HasElement("linear_threshold"))
      this->linearThreshold = _sdf->Get<double>("linear_threshold");

    if (_sdf->HasElement("angular_threshold"))
      this->angularThreshold = _sdf->Get<double>("angular_threshold");

    this->actor.SetAnimationName(_ecm, this->animationName);
    this->actor.SetAnimationTime(_ecm, this->animationTime);
  }

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    this->actor.SetAnimationName(_ecm, this->animationName);

    if (this->movingLastStep)
    {
      const auto scaledNanoseconds =
          std::llround(static_cast<double>(_info.dt.count()) * this->playbackSpeed);
      this->animationTime += std::chrono::nanoseconds(scaledNanoseconds);
    }

    this->actor.SetAnimationTime(_ecm, this->animationTime);
  }

  public: void PostUpdate(
              const gz::sim::UpdateInfo & /*_info*/,
              const gz::sim::EntityComponentManager &_ecm) override
  {
    const auto worldPose = this->actor.WorldPose(_ecm);
    if (!worldPose.has_value())
      return;

    if (!this->previousWorldPose.has_value())
    {
      this->previousWorldPose = worldPose;
      this->movingLastStep = false;
      return;
    }

    const auto linearDistance =
        worldPose->Pos().Distance(this->previousWorldPose->Pos());
    const double yawDifference =
        worldPose->Rot().Yaw() - this->previousWorldPose->Rot().Yaw();
    const double angularDistance = std::atan2(
        std::sin(yawDifference), std::cos(yawDifference));

    this->movingLastStep =
        linearDistance > this->linearThreshold ||
        std::abs(angularDistance) > this->angularThreshold;

    this->previousWorldPose = worldPose;
  }

  private: gz::sim::Entity entity{gz::sim::kNullEntity};
  private: gz::sim::Actor actor;
  private: std::string animationName{"walk"};
  private: double playbackSpeed{1.0};
  private: double linearThreshold{1e-4};
  private: double angularThreshold{1e-4};
  private: bool movingLastStep{false};
  private: std::optional<gz::math::Pose3d> previousWorldPose;
  private: std::chrono::steady_clock::duration animationTime{
      std::chrono::steady_clock::duration::zero()};
};
}  // namespace gz_human_sim

GZ_ADD_PLUGIN(
    gz_human_sim::ActorAnimationControlPlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz_human_sim::ActorAnimationControlPlugin,
    "gz_human_sim::ActorAnimationControlPlugin")
