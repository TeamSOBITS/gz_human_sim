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
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
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
    this->entity = _entity;
    if (_ecm.Component<gz::sim::components::Actor>(_entity) == nullptr)
    {
      gzerr << "ActorCommandPlugin must be attached to an <actor>." << std::endl;
      return;
    }
    this->velocityTopic = _sdf->Get<std::string>("vel_topic", "/cmd_vel").first;
    this->pathTopic = _sdf->Get<std::string>("path_topic", "/cmd_path").first;
    this->jumpTopic = _sdf->Get<std::string>("jump_topic", "/cmd_jump").first;
    this->removeTopic = _sdf->Get<std::string>("remove_topic", "/remove_actor").first;
    this->followModeTopic =
        _sdf->Get<std::string>("follow_mode_topic", "/set_follow_mode").first;
    this->sitTopic = _sdf->Get<std::string>("sit_topic", "/cmd_sit").first;
    this->animationName = _sdf->Get<std::string>("animation_name", "walk").first;
    this->animationFactor = _sdf->Get<double>("animation_factor", 4.0).first;
    this->linearVelocity = _sdf->Get<double>("linear_velocity", 1.0).first;
    this->linearTolerance = _sdf->Get<double>("linear_tolerance", 0.1).first;
    this->turnRate = _sdf->Get<double>("turn_rate", 2.5).first;
    // Physics collision body (see models/human_collision_body/model.sdf
    // and HumanControlPanel/spawn_human.launch.py's spawn flow): this
    // actor is a pure kinematic TrajectoryPose teleport with no collision
    // of its own, so when these are set, PreUpdate() drives that separate
    // dynamic model over collisionCmdVelTopic instead of self-integrating
    // its own X/Y/yaw, and copies that body's physics-resolved pose back
    // each tick. Left empty (the default for any actor spawned without
    // this wiring, e.g. a hand-authored world SDF), PreUpdate() falls
    // back to the original self-integrated motion, uncollided.
    this->collisionModelName = _sdf->Get<std::string>("collision_model_name", "").first;
    this->collisionCmdVelTopic =
        _sdf->Get<std::string>("collision_cmd_vel_topic", "").first;
    if (!this->collisionCmdVelTopic.empty())
      this->collisionPublisher =
          this->transportNode.Advertise<gz::msgs::Twist>(this->collisionCmdVelTopic);
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
    // Tracks whatever name is currently written to the AnimationName
    // component, so PreUpdate()'s sit state machine only rewrites it on an
    // actual change -- see the guard comment on the sit-state switch below
    // for why (gz-sim rebuilds the render-thread PoseAnimation on every
    // write, even a same-value one, which segfaults mid-interpolation).
    this->appliedAnimationName = this->animationName;

    if (_ecm.Component<gz::sim::components::AnimationTime>(this->entity) == nullptr)
      _ecm.CreateComponent(this->entity,
          gz::sim::components::AnimationTime(this->animationTime));

    // Create TrajectoryPose here rather than lazily in PreUpdate: PreUpdate
    // returns immediately while the world is paused (e.g. a launch file
    // that doesn't pass `-r`), which would otherwise leave this actor with
    // AnimationName/AnimationTime but no TrajectoryPose. gz-sim's render
    // thread assumes actors with animation components also have a
    // TrajectoryPose, and segfaults in RenderUtil::UpdateAnimation when
    // that assumption doesn't hold.
    //
    // Seeded at identity, NOT at the spawn pose: at Configure() time the
    // base Pose component still holds the actor's own model.sdf <pose>,
    // because gz-sim's UserCommands create path applies the requested
    // `-x/-y/-z/-Y` spawn pose only AFTER an entity's system plugins have
    // been configured (verified directly on gz-sim 8: Configure() reads
    // walking_actor's own `0 0 0.86`, and the first PreUpdate() tick then
    // sees the real spawn pose). Splitting the spawn pose between the base
    // Pose and TrajectoryPose therefore cannot happen here -- see
    // NormalizeSpawnPose(), which does it on the first PreUpdate() tick
    // instead, and its comment for what goes wrong when it isn't done.
    if (_ecm.Component<gz::sim::components::TrajectoryPose>(this->entity) == nullptr)
    {
      _ecm.CreateComponent(this->entity,
          gz::sim::components::TrajectoryPose(gz::math::Pose3d::Zero));
    }

    if (!this->transportNode.Subscribe(this->velocityTopic,
          &ActorCommandPlugin::VelocityCallback, this))
      gzerr << "Failed to subscribe to velocity topic: " << this->velocityTopic << std::endl;
    if (!this->transportNode.Subscribe(this->pathTopic,
          &ActorCommandPlugin::PathCallback, this))
      gzerr << "Failed to subscribe to path topic: " << this->pathTopic << std::endl;
    if (!this->transportNode.Subscribe(this->jumpTopic,
          &ActorCommandPlugin::JumpCallback, this))
      gzerr << "Failed to subscribe to jump topic: " << this->jumpTopic << std::endl;
    if (!this->transportNode.Subscribe(this->removeTopic,
          &ActorCommandPlugin::RemoveCallback, this))
      gzerr << "Failed to subscribe to remove topic: " << this->removeTopic << std::endl;
    if (!this->transportNode.Subscribe(this->followModeTopic,
          &ActorCommandPlugin::FollowModeCallback, this))
      gzerr << "Failed to subscribe to follow_mode topic: " << this->followModeTopic
            << std::endl;
    if (!this->transportNode.Subscribe(this->sitTopic,
          &ActorCommandPlugin::SitCallback, this))
      gzerr << "Failed to subscribe to sit topic: " << this->sitTopic << std::endl;
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
        // Take the companion collision body (see Configure()'s comment)
        // down with it -- it's a plain MODEL, so unlike the actor itself
        // it has no restriction against RequestRemoveEntity(), it would
        // just otherwise be left behind as an orphaned invisible body.
        const gz::sim::Entity resolvedCollisionEntity =
            this->ResolveCollisionEntity(_ecm);
        if (resolvedCollisionEntity != gz::sim::kNullEntity)
          _ecm.RequestRemoveEntity(resolvedCollisionEntity, true);
        _ecm.RequestRemoveEntity(this->entity, true);
        return;
      }
    }
    // Before the paused check on purpose: this only rearranges which
    // component the actor's spawn pose is stored in, without moving it, so
    // it's safe (and desirable) to have the invariant established even in a
    // world that starts paused.
    this->NormalizeSpawnPose(_ecm);
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

    // Sit state machine: sitIntent ("does the GUI currently want this actor
    // seated") only ever flips a bool -- SittingDown/Sitting/StandingUp are
    // this plugin's own bookkeeping of where the sit_down/stand_up
    // transition clips currently are, entered/left below. Interrupting
    // mid-transition (sitIntent flips back before a clip finishes) just
    // reverses direction from wherever the clip currently is, rather than
    // waiting for it to complete first.
    bool sitIntent;
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      sitIntent = this->sitIntent;
    }
    // Accumulate once per tick for whichever sit-related clip is currently
    // playing (SittingDown/Sitting/StandingUp all use this same counter,
    // reset to zero every time the state below changes) -- kept separate
    // from this->animationTime (the walk cycle's own distance-driven
    // clock) so switching back and forth doesn't corrupt either one.
    if (this->sitState != SitState::Standing)
      this->sitAnimationTime += _info.dt;

    switch (this->sitState)
    {
      case SitState::Standing:
        if (sitIntent)
        {
          // Freeze the actor's footprint for the whole sit/stand cycle --
          // it has no legs to plant, so unlike a real person it can't also
          // walk while sitting down/standing up.
          this->frozenPose = currentPose;
          this->sitState = SitState::SittingDown;
          this->sitAnimationTime = std::chrono::steady_clock::duration::zero();
        }
        break;
      case SitState::SittingDown:
        if (!sitIntent)
        {
          this->sitState = SitState::StandingUp;
          this->sitAnimationTime = std::chrono::steady_clock::duration::zero();
        }
        else if (this->sitAnimationTime >= kSitDownDuration)
        {
          this->sitState = SitState::Sitting;
          this->sitAnimationTime = std::chrono::steady_clock::duration::zero();
        }
        break;
      case SitState::Sitting:
        if (!sitIntent)
        {
          this->sitState = SitState::StandingUp;
          this->sitAnimationTime = std::chrono::steady_clock::duration::zero();
        }
        break;
      case SitState::StandingUp:
        if (sitIntent)
        {
          this->sitState = SitState::SittingDown;
          this->sitAnimationTime = std::chrono::steady_clock::duration::zero();
        }
        else if (this->sitAnimationTime >= kStandUpDuration)
        {
          this->sitState = SitState::Standing;
          this->sitAnimationTime = std::chrono::steady_clock::duration::zero();
          // Restart the walk cycle from its beginning next time the actor
          // actually moves, rather than resuming from wherever it was
          // parked before sitting down.
          this->animationTime = std::chrono::steady_clock::duration::zero();
        }
        break;
    }

    // Drained every tick regardless of sit state: while sitting this just
    // discards whatever arrived on velocityTopic (never applied to
    // this->velocity below), instead of letting the queue grow unbounded
    // for as long as the actor stays seated -- the newest command is still
    // what's in effect the moment PreUpdate resumes using it, once back to
    // Standing, since only the latest queued entry survives either way.
    this->ApplyNewestVelocity();

    if (this->sitState != SitState::Standing)
    {
      // Sitting/transitioning: no path/velocity/jump integration at all,
      // the actor just holds the pose it had the moment it started sitting
      // down. See the animation/AnimationName block below (after the
      // existing walk-cycle AnimationTime write) for what actually plays.
      nextPose = this->frozenPose;
      // The companion physics body (see Configure()'s comment and the
      // collision-body block below) is driven by gz-sim-velocity-control-
      // system, which holds whatever velocity it was last told to use
      // until a new command arrives -- it does NOT stop on its own just
      // because this plugin stops republishing every tick. Left alone, it
      // would keep coasting on the last pre-sit velocity for the entire
      // sit/stand cycle, drifting away from frozenPose, and PreUpdate would
      // then read that drifted pose back and teleport the actor to it the
      // moment it stands back up (see collisionEntity/syncedFromCollisionBody
      // below). Explicitly pinning it to zero here, every tick, is what
      // keeps it (and therefore the actor, once movement resumes) actually
      // parked at frozenPose for the whole sit.
      this->DriveCollisionBody(_ecm, gz::msgs::Twist(), nullptr);
      // Jump requests received while sitting would otherwise sit in
      // jumpRequested (only consumed by the jump block inside the Standing
      // branch below) and fire as a surprise jump the instant the actor
      // stands back up. Discard them instead -- sitting has no jump.
      {
        std::lock_guard<std::mutex> lock(this->commandMutex);
        this->jumpRequested = false;
      }
    }
    else
    {
      this->ApplyStandingMotion(_ecm, currentPose, nextPose, dt, distanceTravelled);
    }

    *trajectoryPose = gz::sim::components::TrajectoryPose(nextPose);
    _ecm.SetChanged(this->entity, gz::sim::components::TrajectoryPose::typeId,
        gz::sim::ComponentState::OneTimeChange);
    if (this->sitState == SitState::Standing && distanceTravelled > 0.0)
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

    // Pick which named clip should be playing right now and, only if that
    // differs from what's already applied, push both AnimationName and a
    // freshly-scoped AnimationTime for it. The change-detection guard here
    // is mandatory, not an optimization: gz-sim rebuilds the render-thread
    // PoseAnimation any time AnimationName is written, even to the same
    // value, and doing that every tick segfaults inside
    // gz::common::Animation::Time()/PoseAnimation::InterpolatedKeyFrame()
    // while it's mid-interpolation (see actor_animation_control_plugin.cpp
    // for the same guard on its own, otherwise-unrelated, animation path).
    std::string desiredAnimationName = this->animationName;
    std::chrono::steady_clock::duration desiredAnimationTime = this->animationTime;
    switch (this->sitState)
    {
      case SitState::Standing:
        break;
      case SitState::SittingDown:
        desiredAnimationName = "sit_down";
        desiredAnimationTime = std::min(this->sitAnimationTime, kSitDownDuration);
        break;
      case SitState::Sitting:
      {
        desiredAnimationName = "sitting";
        // sitting.dae is a short clip meant to be held/looped for as long
        // as the actor stays seated; loop it here explicitly rather than
        // assuming gz-sim wraps AnimationTime past a clip's own duration
        // on its own.
        const double loopSeconds = std::fmod(
            std::chrono::duration<double>(this->sitAnimationTime).count(),
            std::chrono::duration<double>(kSittingLoopDuration).count());
        desiredAnimationTime = std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(loopSeconds));
        break;
      }
      case SitState::StandingUp:
        desiredAnimationName = "stand_up";
        desiredAnimationTime = std::min(this->sitAnimationTime, kStandUpDuration);
        break;
    }
    if (desiredAnimationName != this->appliedAnimationName)
    {
      auto animationNameComponent = _ecm.Component<gz::sim::components::AnimationName>(
          this->entity);
      if (animationNameComponent != nullptr)
      {
        *animationNameComponent = gz::sim::components::AnimationName(desiredAnimationName);
        _ecm.SetChanged(this->entity, gz::sim::components::AnimationName::typeId,
            gz::sim::ComponentState::OneTimeChange);
        this->appliedAnimationName = desiredAnimationName;
      }
    }
    if (this->sitState != SitState::Standing)
    {
      auto animationTimeComponent = _ecm.Component<gz::sim::components::AnimationTime>(
          this->entity);
      if (animationTimeComponent != nullptr)
      {
        *animationTimeComponent =
            gz::sim::components::AnimationTime(desiredAnimationTime);
        _ecm.SetChanged(this->entity, gz::sim::components::AnimationTime::typeId,
            gz::sim::ComponentState::OneTimeChange);
      }
    }
  }

  /// \brief One-shot, on the first PreUpdate() tick: moves the actor's
  /// spawn X/Y/yaw out of its base Pose component and into TrajectoryPose,
  /// leaving the base Pose as nothing but the model's static standing
  /// height (0, 0, z, no rotation).
  ///
  /// gz-sim renders an actor at `TrajectoryPose ⊕ Pose` -- i.e. the base
  /// Pose is a frame the TrajectoryPose is expressed IN, not an alternative
  /// to it (gz-sim's RenderUtil.cc composes the two). This plugin drives
  /// TrajectoryPose with absolute world coordinates (read straight off the
  /// collision body's physics-resolved world pose, see
  /// ApplyStandingMotion()), so unless the base Pose is neutralised first,
  /// every actor renders at spawnPose ⊕ worldPose instead of worldPose:
  /// displaced by its own spawn offset and turned by twice its spawn yaw.
  /// That is exactly the "the collision capsule stops at the wall but the
  /// human model keeps going and ends up inside the furniture" symptom --
  /// the physics and the read-back were right all along, only the drawn
  /// mesh was off by the spawn position.
  ///
  /// Has to happen here rather than in Configure() because Configure() runs
  /// before gz-sim applies the create-service spawn pose -- see the comment
  /// on the TrajectoryPose creation there.
  private: void NormalizeSpawnPose(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->spawnPoseNormalized)
      return;
    auto poseComponent = _ecm.Component<gz::sim::components::Pose>(this->entity);
    auto trajectoryPose = _ecm.Component<gz::sim::components::TrajectoryPose>(
        this->entity);
    if (poseComponent == nullptr || trajectoryPose == nullptr)
      return;
    this->spawnPoseNormalized = true;

    const gz::math::Pose3d spawnPose = poseComponent->Data();
    // Z is the one component that stays on the base Pose: it encodes the
    // model's static height correction (walking_actor's mesh origin sits
    // ~0.86 m above its feet, DoctorFemaleWalk's sits at ground level), and
    // PreUpdate() only ever writes Z on TrajectoryPose as the jump arc's
    // offset on top of it.
    gz::math::Pose3d basePose = gz::math::Pose3d::Zero;
    basePose.Pos().Z(spawnPose.Pos().Z());
    *poseComponent = gz::sim::components::Pose(basePose);
    _ecm.SetChanged(this->entity, gz::sim::components::Pose::typeId,
        gz::sim::ComponentState::OneTimeChange);

    // Seeding the yaw (not just X/Y) matters from tick one: PreUpdate()
    // reads the actor's current facing back out of TrajectoryPose, so an
    // actor spawned with `-Y != 0` would otherwise compute its very first
    // velocity step as if it were facing +X.
    *trajectoryPose = gz::sim::components::TrajectoryPose(
        gz::math::Pose3d(spawnPose.Pos().X(), spawnPose.Pos().Y(), 0.0,
            0.0, 0.0, spawnPose.Rot().Yaw()));
    _ecm.SetChanged(this->entity, gz::sim::components::TrajectoryPose::typeId,
        gz::sim::ComponentState::OneTimeChange);
  }

  private: void VelocityCallback(const gz::msgs::Twist &_message)
  {
    // angular.x doubles as a "turn-to-face" mode flag (nonzero = true) --
    // gz::msgs::Twist has no field for this otherwise, and angular.x is
    // never meaningful for a ground-plane actor's own direct-rate Twist
    // (that mode only ever uses angular.z, see VelocityCommand::angular).
    // See HumanControlPanel::PublishTurnToFace()/teleopMove() for the two
    // message shapes this parses.
    VelocityCommand command;
    command.turnToFace = _message.angular().x() != 0.0;
    command.linear = _message.linear().x();
    if (command.turnToFace)
      command.targetHeading = _message.angular().z();
    else
    {
      command.lateral = _message.linear().y();
      command.angular = _message.angular().z();
    }
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->velocityCommands.push(command);
  }

  /// \brief Jump request from the GUI's Enter key / jump button. Allowed
  /// twice per airborne cycle (jumpCount 0->1 from the ground, 1->2 as a
  /// mid-air double jump); a 3rd press before landing is ignored. The
  /// height carried on _message becomes the new kick's peak (from
  /// wherever the actor currently is, not from the ground -- see the
  /// PreUpdate() jump block), read live off HumanControlPanel's jumpHeight
  /// slider at the moment Enter was pressed.
  private: void JumpCallback(const gz::msgs::Double &_message)
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    if (this->jumpCount >= 2)
      return;
    this->jumpHeightParam = std::clamp(_message.data(), 0.05, 3.0);
    this->jumpRequested = true;
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

  /// \brief Sit/stand intent from the GUI's K (hold) / K+Space (lock) keys
  /// or its sit toggle button -- see the sit state machine at the top of
  /// PreUpdate() for how "sit" vs "stand" actually drives the
  /// sit_down/sitting/stand_up clips.
  private: void SitCallback(const gz::msgs::StringMsg &_message)
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->sitIntent = (_message.data() == "sit");
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

  /// \brief Normal (non-sitting) movement: path/velocity command source
  /// selection, the collision-body command, and the jump Z-overlay -- exactly
  /// what PreUpdate() always did before the sit state machine existed,
  /// just pulled out into its own method (a) so PreUpdate() itself reads
  /// as "sitting freeze, or else normal motion" instead of a page of
  /// nested logic, and (b) since it's meaningful on its own: this is the
  /// only place _nextPose/_distanceTravelled get touched when
  /// `sitState == Standing`.
  private: void ApplyStandingMotion(gz::sim::EntityComponentManager &_ecm,
      const gz::math::Pose3d &_currentPose, gz::math::Pose3d &_nextPose,
      double _dt, double &_distanceTravelled)
  {
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
      pathApplied = this->ApplyPathCommand(_nextPose, _dt, _distanceTravelled);
    if (pathApplied)
    {
      // Path following integrates waypoints open-loop -- the companion body
      // isn't what's driving here, so drag it along to the actor instead of
      // leaving it behind. Without this the capsule keeps coasting on
      // whatever velocity it was last given (gz-sim-velocity-control-system
      // holds the last command indefinitely) and drifts away from the human
      // for as long as the path runs.
      this->DriveCollisionBody(_ecm, gz::msgs::Twist(), &_nextPose);
    }
    else if (followMode == "path")
    {
      // follow_mode "path" with no path currently active: the actor stands
      // still, so the companion has to be told to stand still too -- same
      // "VelocityControl holds its last command forever" reason as above.
      this->DriveCollisionBody(_ecm, gz::msgs::Twist(), nullptr);
    }
    else
    {
      const double yaw = _currentPose.Rot().Yaw();
      double bodyLinear = this->velocity.linear;
      double bodyLateral = this->velocity.lateral;
      double bodyAngularRate = this->velocity.angular;
      double kinematicYaw = yaw;
      if (this->velocity.turnToFace)
      {
        // Steer the actual current yaw toward an ABSOLUTE world target
        // heading (not a relative turn-then-walk phase, and not a direct
        // turn RATE like the else branch below) while continuously
        // walking forward every tick -- this is what makes direction
        // changes read as a natural in-motion arc instead of a
        // stop-spin-then-walk, and makes repeating the same direction key
        // idempotent (same absolute target every time) instead of
        // stacking another relative turn on top of wherever the actor
        // happens to be currently facing. See HumanControlPanel::
        // PublishTurnToFace()/teleopDirection() on the sending side.
        double diff = this->velocity.targetHeading - yaw;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff <= -M_PI) diff += 2.0 * M_PI;
        const double maxStep = this->turnRate * _dt;
        const double turnStep = std::clamp(diff, -maxStep, maxStep);
        bodyLateral = 0.0;
        bodyAngularRate = _dt > 0.0 ? turnStep / _dt : 0.0;
        kinematicYaw = yaw + turnStep;
      }
      else
      {
        // Direct body-frame Twist: linear/lateral velocity plus a fixed
        // turn RATE (not a target), used by strafe mode (J), the 2-key
        // steering combo, and the S+A/S+D in-place spin -- all of which
        // already want continuous rotation without any "steer toward a
        // fixed heading" behavior.
        kinematicYaw = yaw + bodyAngularRate * _dt;
      }

      bool syncedFromCollisionBody = false;
      if (!this->collisionCmdVelTopic.empty())
      {
        // Drive the invisible physics body (see models/
        // human_collision_body/model.sdf) with the exact same body-frame
        // velocity this tick would otherwise have self-integrated
        // directly, then read back wherever physics/collisions actually
        // let it end up, instead of trusting our own uncollided math --
        // this is what stops the actor at walls/furniture instead of
        // walking through them. Resolved lazily/every tick until found,
        // since the companion model may still be mid-spawn for the first
        // few ticks after this actor's own Configure() runs.
        gz::msgs::Twist collisionTwist;
        collisionTwist.mutable_linear()->set_x(bodyLinear);
        collisionTwist.mutable_linear()->set_y(bodyLateral);
        collisionTwist.mutable_angular()->set_z(bodyAngularRate);
        // No teleport here on purpose (see DriveCollisionBody()): this is
        // the branch where the physics body -- not the actor -- decides
        // where the human ends up.
        this->DriveCollisionBody(_ecm, collisionTwist, nullptr);
        const gz::sim::Entity resolvedCollisionEntity =
            this->ResolveCollisionEntity(_ecm);
        if (resolvedCollisionEntity != gz::sim::kNullEntity)
        {
          auto collisionPose = _ecm.Component<gz::sim::components::Pose>(
              resolvedCollisionEntity);
          if (collisionPose != nullptr)
          {
            const gz::math::Pose3d resolved = collisionPose->Data();
            const double dx = resolved.Pos().X() - _currentPose.Pos().X();
            const double dy = resolved.Pos().Y() - _currentPose.Pos().Y();
            _nextPose.Pos().X(resolved.Pos().X());
            _nextPose.Pos().Y(resolved.Pos().Y());
            _nextPose.Rot() = gz::math::Quaterniond(0.0, 0.0, resolved.Rot().Yaw());
            _distanceTravelled = std::hypot(dx, dy);
            syncedFromCollisionBody = true;
          }
          else
          {
            // Entity existed under that name but isn't a real pose-bearing
            // model (or vanished) -- stop trusting the cached id and fall
            // back to uncollided motion below until a fresh lookup finds
            // it again.
            this->collisionEntity = gz::sim::kNullEntity;
          }
        }
      }
      if (!syncedFromCollisionBody)
      {
        // No collision body configured, or it hasn't appeared in the ECM
        // yet -- fall back to the original uncollided kinematic estimate
        // so the actor still moves smoothly instead of freezing while
        // waiting for the companion to spawn.
        const double dx = (bodyLinear * std::cos(yaw) - bodyLateral * std::sin(yaw)) * _dt;
        const double dy = (bodyLinear * std::sin(yaw) + bodyLateral * std::cos(yaw)) * _dt;
        _nextPose.Pos().X(_currentPose.Pos().X() + dx);
        _nextPose.Pos().Y(_currentPose.Pos().Y() + dy);
        _nextPose.Rot() = gz::math::Quaterniond(0.0, 0.0, kinematicYaw);
        _distanceTravelled = std::hypot(dx, dy);
      }
    }
    // Jump is a Z-only overlay on top of whatever X/Y/yaw motion the path or
    // velocity branch above just computed -- so "jump while walking" needs
    // no special handling: the horizontal component keeps coming from
    // this->velocity (or the active path) exactly as it already would, and
    // this just adds a projectile-motion Z arc on top. Integrated as a
    // running (jumpZ, jumpVelocityZ) state each tick (Euler step) rather
    // than a closed-form parabola keyed on elapsed-time-since-launch, so a
    // double jump can simply re-kick jumpVelocityZ from wherever the actor
    // currently is mid-air, instead of needing to reason about restarting a
    // parabola from a nonzero height. jumpCount (0 = grounded, 1 = single
    // jump used, 2 = double jump used) gates JumpCallback() -- see there.
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      if (this->jumpRequested)
      {
        this->jumpVelocityZ = std::sqrt(2.0 * kGravity * this->jumpHeightParam);
        ++this->jumpCount;
        this->jumpRequested = false;
      }
    }
    if (this->jumpCount > 0)
    {
      this->jumpVelocityZ -= kGravity * _dt;
      this->jumpZ += this->jumpVelocityZ * _dt;
      if (this->jumpZ <= 0.0)
      {
        this->jumpZ = 0.0;
        this->jumpVelocityZ = 0.0;
        this->jumpCount = 0;
      }
      _nextPose.Pos().Z(this->jumpZ);
    }
    else
    {
      _nextPose.Pos().Z(0.0);
    }
  }

  /// \brief Lazily resolves and caches the companion collision-body entity
  /// by name (it may still be mid-spawn for the first few ticks after this
  /// actor's own Configure() runs). Shared by the removal path (PreUpdate's
  /// removalRequested block) and ApplyStandingMotion() -- previously each
  /// had its own copy of this same null-check-then-EntityByComponents()
  /// lookup.
  private: gz::sim::Entity ResolveCollisionEntity(
      gz::sim::EntityComponentManager &_ecm)
  {
    if (this->collisionEntity == gz::sim::kNullEntity &&
        !this->collisionModelName.empty())
    {
      this->collisionEntity = _ecm.EntityByComponents(
          gz::sim::components::Name(this->collisionModelName));
    }
    return this->collisionEntity;
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
        this->velocity = VelocityCommand();
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

  /// \brief Sends the companion collision body (see
  /// ResolveCollisionEntity()) its command for this tick: always the
  /// body-frame Twist gz-sim-velocity-control-system integrates, plus --
  /// only when _teleportTo is non-null -- a hard world-pose teleport on top
  /// of it.
  ///
  /// The teleport goes through components::WorldPoseCmd, the only pose
  /// channel gz-sim's Physics system actually consumes for a dynamic model.
  /// An earlier version of this wrote components::Pose directly, which has
  /// no effect whatsoever: Physics recomputes that component from the
  /// engine's own result later in the same iteration, so the write was
  /// silently discarded every single tick.
  ///
  /// Teleporting is only ever right where the ACTOR, not the physics body,
  /// is deciding where to be (path following, or standing still). The
  /// teleop branch in ApplyStandingMotion() deliberately does not teleport
  /// -- it reads the body's physics-resolved pose back instead, which is
  /// exactly what lets walls and furniture stop it.
  private: void DriveCollisionBody(gz::sim::EntityComponentManager &_ecm,
      const gz::msgs::Twist &_twist, const gz::math::Pose3d *_teleportTo)
  {
    if (this->collisionPublisher.Valid())
      this->collisionPublisher.Publish(_twist);
    if (_teleportTo == nullptr)
      return;
    const gz::sim::Entity resolvedCollisionEntity = this->ResolveCollisionEntity(_ecm);
    if (resolvedCollisionEntity == gz::sim::kNullEntity)
      return;
    auto collisionPose =
        _ecm.Component<gz::sim::components::Pose>(resolvedCollisionEntity);
    if (collisionPose == nullptr)
    {
      // Entity existed under that name but isn't a real pose-bearing model
      // (or vanished) -- same recovery as ApplyStandingMotion()'s velocity
      // branch: stop trusting the cached id so the next tick looks it up
      // again instead of silently doing nothing forever.
      this->collisionEntity = gz::sim::kNullEntity;
      return;
    }
    // Only X/Y/yaw move -- the companion's own Z stays whatever its
    // model.sdf spawned it at (its <link> pose already bakes in standing
    // height from a ground-level model root, independent of the actor's own
    // Z -- same reasoning as spawn_human.launch.py's collision-body spawn
    // always using z=0.0 regardless of the actor's spawn z).
    gz::math::Pose3d target = collisionPose->Data();
    target.Pos().X(_teleportTo->Pos().X());
    target.Pos().Y(_teleportTo->Pos().Y());
    target.Rot() = gz::math::Quaterniond(0.0, 0.0, _teleportTo->Rot().Yaw());
    _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(
        resolvedCollisionEntity, target);
  }

  /// \brief One parsed cmd_vel command. turnToFace selects which of the
  /// two mutually-exclusive interpretations PreUpdate() uses: true means
  /// (linear, targetHeading) -- walk forward while steering yaw toward an
  /// absolute world heading; false means (linear, lateral, angular) --
  /// the original direct body-frame velocity + turn-rate Twist. See
  /// VelocityCallback()/PreUpdate().
  private: struct VelocityCommand
  {
    double linear{0.0};
    double lateral{0.0};
    double angular{0.0};
    bool turnToFace{false};
    double targetHeading{0.0};
  };

  private: gz::sim::Entity entity{gz::sim::kNullEntity};
  // One-shot latch for NormalizeSpawnPose() -- see there.
  private: bool spawnPoseNormalized{false};
  private: gz::transport::Node transportNode;
  private: std::mutex commandMutex;
  private: std::queue<VelocityCommand> velocityCommands;
  private: std::queue<std::vector<gz::math::Pose3d>> pathCommands;
  private: std::vector<gz::math::Pose3d> path;
  private: std::size_t pathIndex{0u};
  private: VelocityCommand velocity;
  private: std::string velocityTopic;
  private: std::string pathTopic;
  private: std::string jumpTopic;
  private: std::string removeTopic;
  private: std::string followModeTopic;
  private: std::string sitTopic;
  private: bool removalRequested{false};
  private: std::string animationName;
  private: std::string followMode{"auto"};
  private: double animationFactor{4.0};
  private: double linearVelocity{1.0};
  private: double linearTolerance{0.1};
  // Max rad/s the turnToFace path (see PreUpdate()) steers yaw toward its
  // target heading -- SDF-configurable (turn_rate) so it can be tuned
  // independently of linear_velocity; defaults to the same rate the old
  // GUI-side two-phase turn used (kTeleopTurnRate in HumanControlPanel.cc).
  private: double turnRate{2.5};
  // Companion physics collision body wiring -- see Configure()'s comment
  // and the collision-body block in PreUpdate(). collisionEntity is
  // resolved lazily (kNullEntity until EntityByComponents() finds a match)
  // since the companion model may spawn a few ticks after this actor.
  private: std::string collisionModelName;
  private: std::string collisionCmdVelTopic;
  private: gz::transport::Node::Publisher collisionPublisher;
  private: gz::sim::Entity collisionEntity{gz::sim::kNullEntity};
  // Jump state, integrated as a Z-only parabolic arc each PreUpdate tick --
  // see the jump block there and JumpCallback(). kGravity only needs to be
  // physically plausible (it just shapes the arc), not an exact match to
  // the world's <gravity>, since actors are kinematic (TrajectoryPose-
  // driven), not physics-simulated.
  private: static constexpr double kGravity = 9.81;
  private: bool jumpRequested{false};
  private: double jumpZ{0.0};
  private: double jumpVelocityZ{0.0};
  // 0 = grounded, 1 = single jump in progress, 2 = double jump used --
  // see JumpCallback()/the PreUpdate() jump block.
  private: int jumpCount{0};
  private: double jumpHeightParam{1.0};
  private: std::chrono::steady_clock::duration animationTime{
      std::chrono::steady_clock::duration::zero()};
  // Name currently written to the AnimationName component -- see the
  // change-detection guard in PreUpdate()'s animation-selection block for
  // why this has to be tracked instead of writing unconditionally.
  private: std::string appliedAnimationName;

  /// \brief Sit/stand cycle driven by sit_topic (SitCallback()) -- see the
  /// state machine at the top of PreUpdate(). Standing is the plugin's
  /// original walk/path/velocity behaviour, unchanged; the other three
  /// states play sit_down/sitting/stand_up in order while freezing the
  /// actor's footprint at whatever pose it had when it started sitting.
  private: enum class SitState { Standing, SittingDown, Sitting, StandingUp };
  private: SitState sitState{SitState::Standing};
  private: bool sitIntent{false};
  private: gz::math::Pose3d frozenPose{gz::math::Pose3d::Zero};
  private: std::chrono::steady_clock::duration sitAnimationTime{
      std::chrono::steady_clock::duration::zero()};
  // Clip lengths (measured from the .dae files under models/walking_actor/
  // meshes/), used to know when to advance sitState past sit_down/stand_up
  // and to loop sitting.dae manually -- see PreUpdate()'s animation block.
  private: static constexpr std::chrono::steady_clock::duration kSitDownDuration =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::milliseconds(6625));
  private: static constexpr std::chrono::steady_clock::duration kStandUpDuration =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::milliseconds(6625));
  private: static constexpr std::chrono::steady_clock::duration kSittingLoopDuration =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::milliseconds(4083));
};
}  // namespace gz_human_sim

GZ_ADD_PLUGIN(gz_human_sim::ActorCommandPlugin, gz::sim::System,
    gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz_human_sim::ActorCommandPlugin,
    "gz_human_sim::ActorCommandPlugin")
