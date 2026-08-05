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
#include <gz/msgs/param.pb.h>
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

#include "gz_human_sim/CharacterState.hh"

namespace gz_human_sim
{
/// \brief One named pose the actor can be put into and held in, beyond its
/// normal walk/idle behaviour. Adding another pose (wave, crouch, lie down,
/// ...) is meant to be exactly one more row in kPoseClips below plus the
/// matching <animation> entries in the model's SDF -- the state machine in
/// PreUpdate() is written against this table, never against "sit"
/// specifically.
///
/// Each pose is a three-clip cycle: enter (standing -> posed), hold (a short
/// clip looped for as long as the pose is held), exit (posed -> standing).
///
/// heightOffset is the part gz-sim cannot do for us. gz-sim strips an
/// actor animation's root-node motion out of the skeleton and expects the
/// TrajectoryPose component to supply it instead -- which is why an actor's
/// origin behaves as its hip rather than its feet, and why the model's
/// spawn z is ~1.0. This plugin writes X/Y/yaw into TrajectoryPose but has
/// no idea what the clip's own vertical root motion was, so without this
/// the actor plays the sit-down animation while its hip stays pinned at
/// standing height -- i.e. it sits down in mid-air. Measured directly off
/// the .dae files (the Hips node's animated transform):
///
///   walk.dae      hip z ~= 1.015          (standing reference)
///   sit_down.dae  hip z 1.031 -> 0.641    (drops 0.390)
///   sitting.dae   hip z ~= 0.641          (held)
///   stand_up.dae  hip z 0.641 -> 1.031    (exact reverse of sit_down)
///
/// so a seated actor's origin has to be driven 0.390 m below wherever
/// standing puts it. Expressed as a delta rather than an absolute height on
/// purpose: it stays correct regardless of the per-model spawn z offset.
struct PoseClip
{
  const char *name;
  const char *enterAnimation;
  const char *holdAnimation;
  const char *exitAnimation;
  // Clip lengths in seconds, measured from the .dae keyframe timelines.
  double enterSeconds;
  double holdSeconds;
  double exitSeconds;
  double heightOffset;
};

static constexpr PoseClip kPoseClips[] = {
  {"sit", "sit_down", "sitting", "stand_up", 6.625, 4.083, 6.625, -0.390},
};
static constexpr int kPoseClipCount =
    static_cast<int>(sizeof(kPoseClips) / sizeof(kPoseClips[0]));

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
    this->poseTopic = _sdf->Get<std::string>("pose_topic", "/cmd_pose").first;
    // 状態の publish 先（構想書 §3・§13 段階3）。**真実はサーバー側にある**の
    // で、GUI に「自分が送った指令から推測」させるのをやめるための出口。
    // 既定は他のトピックと同じ組み立て方（人物名が名前空間）。
    this->stateTopic = _sdf->Get<std::string>(
        "state_topic", kStateTopicSuffix).first;
    // How much faster than real time the enter/hold/exit clips in kPoseClips
    // are played. The sit_down/stand_up clips are 6.6 s of mocap each, which
    // reads as slow-motion next to the walk cycle; ~2.5x puts a full sit at
    // roughly 2.6 s, which is about how long sitting down actually takes.
    // SDF-tunable rather than hardcoded since it's a taste/realism knob.
    this->poseSpeed = std::max(0.1,
        _sdf->Get<double>("pose_speed", 2.5).first);
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
    // gz.msgs.Param（string -> Any の map）を使う。キーを足しても既存の
    // 読み手が壊れないため。roster の '|' 区切り固定長とは対照的
    // （CharacterState.hh の冒頭と CLAUDE.md 罠4）。
    this->statePublisher =
        this->transportNode.Advertise<gz::msgs::Param>(this->stateTopic);
    if (!this->statePublisher)
    {
      gzerr << "Failed to advertise state topic: " << this->stateTopic
            << std::endl;
    }
    if (!this->transportNode.Subscribe(this->poseTopic,
          &ActorCommandPlugin::PoseCallback, this))
      gzerr << "Failed to subscribe to pose topic: " << this->poseTopic << std::endl;
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

    // Pose state machine. requestedPose ("which named pose, if any, does the
    // GUI currently want this actor held in") is the only input; Entering/
    // Holding/Exiting is this plugin's own bookkeeping of where the current
    // pose's enter/exit clips are. See kPoseClips for the table this is
    // driven by -- nothing below is specific to sitting.
    std::string requestedPose;
    double requestedSupportHeight = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      requestedPose = this->requestedPose;
      requestedSupportHeight = this->requestedSupportHeight;
    }
    const int requestedIndex = PoseIndex(requestedPose);
    // Playback clock for whichever enter/hold/exit clip is running, scaled by
    // poseSpeed so the table can keep the .dae files' true lengths. Kept
    // separate from this->animationTime (the walk cycle's own
    // distance-driven clock) so switching back and forth doesn't corrupt
    // either one.
    if (this->poseState != PoseState::None)
      this->poseClipTime += dt * this->poseSpeed;

    switch (this->poseState)
    {
      case PoseState::None:
        if (requestedIndex >= 0)
        {
          // Freeze the actor's footprint for the whole enter/hold/exit
          // cycle -- it has no legs to plant, so unlike a real person it
          // can't also walk while sitting down or standing back up.
          this->frozenPose = currentPose;
          this->activePoseIndex = requestedIndex;
          this->activeSupportHeight = requestedSupportHeight;
          this->poseState = PoseState::Entering;
          this->poseClipTime = 0.0;
        }
        break;
      case PoseState::Entering:
        if (requestedIndex != this->activePoseIndex)
        {
          // Reversing mid-transition: start the exit clip at the point that
          // MIRRORS how far the enter clip got, instead of restarting it
          // from frame 0. enter and exit are the same mocap take played in
          // opposite directions (verified against the .dae keyframes:
          // stand_up is sit_down reversed, frame for frame), so this makes
          // an interrupted sit reverse smoothly out of wherever the body
          // currently is -- rather than snapping to the fully-seated pose
          // first and standing up from there, which is what made a quick
          // K tap look broken.
          this->poseClipTime = ExitSeconds(this->activePoseIndex) *
              (1.0 - Progress(this->poseClipTime, EnterSeconds(this->activePoseIndex)));
          this->poseState = PoseState::Exiting;
        }
        else if (this->poseClipTime >= EnterSeconds(this->activePoseIndex))
        {
          this->poseState = PoseState::Holding;
          this->poseClipTime = 0.0;
        }
        break;
      case PoseState::Holding:
        if (requestedIndex != this->activePoseIndex)
        {
          this->poseState = PoseState::Exiting;
          this->poseClipTime = 0.0;
        }
        break;
      case PoseState::Exiting:
        if (requestedIndex == this->activePoseIndex)
        {
          // Same mirroring as the Entering case above, in the other
          // direction.
          this->poseClipTime = EnterSeconds(this->activePoseIndex) *
              (1.0 - Progress(this->poseClipTime, ExitSeconds(this->activePoseIndex)));
          this->poseState = PoseState::Entering;
        }
        else if (this->poseClipTime >= ExitSeconds(this->activePoseIndex))
        {
          this->poseState = PoseState::None;
          this->activePoseIndex = -1;
          this->activeSupportHeight = 0.0;
          this->poseClipTime = 0.0;
          // Restart the walk cycle from its beginning next time the actor
          // actually moves, rather than resuming from wherever it was
          // parked before the pose started.
          this->animationTime = std::chrono::steady_clock::duration::zero();
        }
        break;
    }

    // Drained every tick regardless of pose state: while posed this just
    // discards whatever arrived on velocityTopic (never applied to
    // this->velocity below), instead of letting the queue grow unbounded
    // for as long as the actor stays posed -- the newest command is still
    // what's in effect the moment PreUpdate resumes using it, once back to
    // None, since only the latest queued entry survives either way.
    this->ApplyNewestVelocity();

    if (this->poseState != PoseState::None)
    {
      // Posed/transitioning: no path/velocity/jump integration at all, the
      // actor just holds the footprint it had the moment the pose started.
      // See the animation/AnimationName block below (after the existing
      // walk-cycle AnimationTime write) for what actually plays.
      nextPose = this->frozenPose;
      // ...but the actor's HEIGHT does have to move, because gz-sim threw
      // the clip's own vertical root motion away -- see PoseClip's comment.
      // Ramped with the clip rather than snapped, so the body descends into
      // the pose instead of teleporting down on the first frame.
      //
      // Measured from frozenPose's own Z (the surface the actor was standing
      // on when the pose began) rather than from an assumed z=0. Now that Z
      // is read back from physics like X/Y are (see ApplyStandingMotion()),
      // that surface can be a tabletop or a step, not just the floor -- so
      // sitting down on something the actor climbed onto lands the hips
      // relative to THAT, instead of driving them down to floor-relative
      // seated height and burying the actor in whatever it was standing on.
      nextPose.Pos().Z(
          this->frozenPose.Pos().Z() + this->CurrentPoseHeightOffset());
      // The companion physics body (see Configure()'s comment and the
      // collision-body block below) is driven by gz-sim-velocity-control-
      // system, which holds whatever velocity it was last told to use
      // until a new command arrives -- it does NOT stop on its own just
      // because this plugin stops republishing every tick. Left alone, it
      // would keep coasting on the last pre-pose velocity for the entire
      // enter/hold/exit cycle, drifting away from frozenPose, and PreUpdate
      // would then read that drifted pose back and teleport the actor to it
      // the moment the pose ends (see collisionEntity/
      // syncedFromCollisionBody below). Explicitly pinning it to zero here,
      // every tick, is what keeps it (and therefore the actor, once movement
      // resumes) actually parked at frozenPose for the whole pose.
      this->DriveCollisionBody(_ecm, gz::msgs::Twist(), nullptr);
      // Jump requests received while posed would otherwise sit in
      // jumpRequested (only consumed by ApplyStandingMotion's jump block)
      // and fire as a surprise jump the instant the pose ends. Discard them
      // instead -- a held pose has no jump. Any jump still in flight when
      // the pose started is cancelled for the same reason.
      {
        std::lock_guard<std::mutex> lock(this->commandMutex);
        this->jumpRequested = false;
      }
      this->jumpZ = 0.0;
      this->jumpVelocityZ = 0.0;
      this->jumpCount = 0;
    }
    else
    {
      this->ApplyStandingMotion(_ecm, currentPose, nextPose, dt, distanceTravelled);
    }

    *trajectoryPose = gz::sim::components::TrajectoryPose(nextPose);
    _ecm.SetChanged(this->entity, gz::sim::components::TrajectoryPose::typeId,
        gz::sim::ComponentState::OneTimeChange);
    if (this->poseState == PoseState::None && distanceTravelled > 0.0)
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
    if (this->poseState != PoseState::None)
    {
      const PoseClip &clip = kPoseClips[this->activePoseIndex];
      double clipSeconds = this->poseClipTime;
      switch (this->poseState)
      {
        case PoseState::None:
          break;
        case PoseState::Entering:
          desiredAnimationName = clip.enterAnimation;
          clipSeconds = std::min(clipSeconds, clip.enterSeconds);
          break;
        case PoseState::Holding:
          desiredAnimationName = clip.holdAnimation;
          // The hold clip is short and meant to be looped for as long as
          // the pose is held; wrap it here explicitly rather than assuming
          // gz-sim wraps AnimationTime past a clip's own duration on its own.
          clipSeconds = std::fmod(clipSeconds, clip.holdSeconds);
          break;
        case PoseState::Exiting:
          desiredAnimationName = clip.exitAnimation;
          clipSeconds = std::min(clipSeconds, clip.exitSeconds);
          break;
      }
      desiredAnimationTime = std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(clipSeconds));
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
    if (this->poseState != PoseState::None)
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

    this->PublishState(_info);
  }

  /// \brief いまの状態を1つに畳んで publish する（構想書 §3）。
  ///
  /// **ここが状態の持ち主。** GUI は表示するだけで、推測しない。
  /// 状態は bool の組み合わせではなく enum ひとつ -- 「座っているのに歩いて
  /// いる」のような矛盾を型として作れなくするため（CharacterState.hh）。
  ///
  /// 送るのは「変化した瞬間」と「変化が無くても kStateHeartbeatMs ごと」の
  /// 両方。後者が無いと、後から起動した GUI が最初の変化まで何も掴めない
  /// （roster を2秒ごとに出しているのと同じ理由）。
  private: void PublishState(const gz::sim::UpdateInfo &_info)
  {
    if (!this->statePublisher)
      return;

    // 優先順位は「その瞬間その人物を実際に支配しているもの」の順。
    // 削除 > ポーズ > ジャンプ > 経路 > 速度 > 静止。
    // ポーズがジャンプより上なのは、ポーズ中は足元を凍結していて
    // ジャンプ指令が入っても弧を描かないため（PreUpdate のポーズブロック）。
    CharacterState state = CharacterState::Standing;
    std::string pose;
    double speed = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      if (this->removalRequested)
      {
        state = CharacterState::Removing;
      }
      else if (this->poseState == PoseState::Entering)
      {
        state = CharacterState::PoseEntering;
      }
      else if (this->poseState == PoseState::Holding)
      {
        state = CharacterState::PoseHolding;
      }
      else if (this->poseState == PoseState::Exiting)
      {
        state = CharacterState::PoseExiting;
      }
      else if (this->jumpCount > 0)
      {
        state = CharacterState::Jumping;
      }
      else if (this->pathIndex < this->path.size())
      {
        state = CharacterState::Following;
        speed = this->linearVelocity;
      }
      else
      {
        // VelocityCommand は前後(linear)・左右(lateral)・旋回(angular) の
        // スカラ3つ。前後と左右の合成が並進速度。
        const double vx = this->velocity.linear;
        const double vy = this->velocity.lateral;
        speed = std::sqrt(vx * vx + vy * vy);
        // 並進がゼロでも旋回だけ入っていることがある（その場旋回、
        // turnToFace の向き直し）。見た目は止まっていないので Moving に含める。
        if (speed > 1e-6 || std::abs(this->velocity.angular) > 1e-6 ||
            this->velocity.turnToFace)
        {
          state = CharacterState::Moving;
        }
      }
      if (this->activePoseIndex >= 0)
        pose = this->requestedPose;
    }

    const auto now = _info.simTime;
    const bool changed = (state != this->publishedState) ||
        (pose != this->publishedPose);
    const auto sinceLast = now - this->lastStatePublish;
    if (!changed &&
        sinceLast < std::chrono::milliseconds(kStateHeartbeatMs))
    {
      return;
    }

    gz::msgs::Param message;
    (*message.mutable_params())["state"].set_string_value(ToString(state));
    (*message.mutable_params())["pose"].set_string_value(pose);
    (*message.mutable_params())["speed"].set_double_value(speed);
    this->statePublisher.Publish(message);

    this->publishedState = state;
    this->publishedPose = pose;
    this->lastStatePublish = now;
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

  /// \brief Which named pose (see kPoseClips) the GUI currently wants this
  /// actor held in -- from a pose hold key (K = sit) or the panel's pose
  /// lock. See the pose state machine at the top of PreUpdate().
  ///
  /// Payload is `<pose name>`, optionally followed by a support height in
  /// metres: `"sit"` sits on the floor, `"sit 0.45"` sits on something
  /// 0.45 m off the floor (a chair seat, a step, a bed edge). Anything that
  /// isn't a known pose name -- including the empty string and the literal
  /// "stand" the older sit-only protocol used -- means "no pose", which
  /// exits whatever pose is currently held.
  private: void PoseCallback(const gz::msgs::StringMsg &_message)
  {
    std::string name = _message.data();
    double supportHeight = 0.0;
    const auto separator = name.find(' ');
    if (separator != std::string::npos)
    {
      try
      {
        supportHeight = std::stod(name.substr(separator + 1));
      }
      catch (const std::exception &)
      {
        gzwarn << "ActorCommandPlugin: could not parse a support height out of '"
               << name << "', treating it as 0." << std::endl;
      }
      name = name.substr(0, separator);
    }
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->requestedPose = name;
    this->requestedSupportHeight = supportHeight;
  }

  /// \brief Index into kPoseClips for a pose name, or -1 for "no pose"
  /// (unknown name, empty string, or the legacy "stand").
  private: static int PoseIndex(const std::string &_name)
  {
    for (int i = 0; i < kPoseClipCount; ++i)
    {
      if (_name == kPoseClips[i].name)
        return i;
    }
    return -1;
  }

  /// \brief 0..1 through a clip of _seconds length, clamped. Guards against
  /// a zero-length clip in the table rather than dividing by zero.
  private: static double Progress(double _elapsed, double _seconds)
  {
    if (_seconds <= 0.0)
      return 1.0;
    return std::clamp(_elapsed / _seconds, 0.0, 1.0);
  }

  /// \brief Bound, in poseClipTime's own units, for how far the enter clip
  /// runs -- i.e. just the raw table value. poseClipTime already ticks in
  /// clip-time (it accumulates dt * poseSpeed, see PreUpdate()), so it can
  /// be compared directly against kPoseClips[_poseIndex].enterSeconds; NOT
  /// divided by poseSpeed again here, which used to double-apply the speed
  /// factor. That bug shrank the real (wall-clock) enter phase by a factor
  /// of poseSpeed^2 instead of poseSpeed, cutting the sit_down clip off at
  /// ~1/poseSpeed of the way through (40% at the default 2.5x) while
  /// CurrentPoseHeightOffset() -- using the same broken bound -- had
  /// already ramped the hip all the way down. The result was the actor's
  /// root snapping to fully-seated height while the skeleton was still
  /// mid-crouch, i.e. exactly the "sitting down looks broken" symptom this
  /// fixes.
  private: double EnterSeconds(int _poseIndex) const
  {
    return kPoseClips[_poseIndex].enterSeconds;
  }

  /// \brief Same fix as EnterSeconds(), for the exit clip.
  private: double ExitSeconds(int _poseIndex) const
  {
    return kPoseClips[_poseIndex].exitSeconds;
  }

  /// \brief How far below its normal standing height the actor's origin has
  /// to be right now, given where the current pose's clip has got to.
  ///
  /// See PoseClip's comment for why an offset is needed at all. Ramped
  /// linearly across the enter/exit clips: the real hip curve isn't quite
  /// linear (measured 1.031 -> 0.816 -> 0.641 across sit_down, vs 0.836 at
  /// the linear midpoint), but 2 cm of error mid-transition is invisible
  /// next to the 39 cm of float this exists to remove, and it keeps the
  /// table to one number per pose instead of a sampled curve.
  ///
  /// activeSupportHeight raises the whole thing for a pose taken on top of
  /// something: sitting on a 0.45 m chair seat puts the hip 0.45 m higher
  /// than sitting on the floor does.
  private: double CurrentPoseHeightOffset() const
  {
    if (this->poseState == PoseState::None || this->activePoseIndex < 0)
      return 0.0;
    const double held =
        kPoseClips[this->activePoseIndex].heightOffset + this->activeSupportHeight;
    switch (this->poseState)
    {
      case PoseState::Entering:
        return held * Progress(this->poseClipTime,
            this->EnterSeconds(this->activePoseIndex));
      case PoseState::Exiting:
        return held * (1.0 - Progress(this->poseClipTime,
            this->ExitSeconds(this->activePoseIndex)));
      case PoseState::Holding:
      case PoseState::None:
        break;
    }
    return held;
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

  /// \brief Normal (un-posed) movement: path/velocity command source
  /// selection, the collision-body command, and the jump Z-overlay -- exactly
  /// what PreUpdate() always did before the pose state machine existed,
  /// just pulled out into its own method (a) so PreUpdate() itself reads
  /// as "held-pose freeze, or else normal motion" instead of a page of
  /// nested logic, and (b) since it's meaningful on its own: this is the
  /// only place _nextPose/_distanceTravelled get touched when
  /// `poseState == None`.
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
    // Before the movement branches, not after: the velocity branch needs
    // this tick's jumpZ to tell the collision body how high to be. See
    // IntegrateJump().
    this->IntegrateJump(_ecm, _dt);
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
        // Vertical component so the capsule rises and falls with the actor's
        // jump instead of staying planted on the floor -- see
        // CollisionJumpRate(). Zero whenever the actor is grounded, so this
        // costs nothing in the normal walking case.
        collisionTwist.mutable_linear()->set_z(this->CollisionJumpRate(_ecm, _dt));
        // No teleport here on purpose (see DriveCollisionBody()): this is
        // the branch where the physics body -- not the actor -- decides
        // where the human ends up.
        this->DriveCollisionBody(_ecm, collisionTwist, nullptr);
        gz::math::Pose3d resolved;
        if (this->CollisionBodyPose(_ecm, resolved))
        {
          const double dx = resolved.Pos().X() - _currentPose.Pos().X();
          const double dy = resolved.Pos().Y() - _currentPose.Pos().Y();
          _nextPose.Pos().X(resolved.Pos().X());
          _nextPose.Pos().Y(resolved.Pos().Y());
          _nextPose.Rot() = gz::math::Quaterniond(0.0, 0.0, resolved.Rot().Yaw());
          _distanceTravelled = std::hypot(dx, dy);
          syncedFromCollisionBody = true;
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
    // Z, for EVERY branch above, comes from the same place X/Y already do:
    // whatever the physics body actually resolved to. This used to be an
    // unconditional `_nextPose.Pos().Z(this->jumpZ)` -- i.e. X/Y were read
    // back from physics while Z was computed independently, from a jump arc
    // that assumes the actor always lands back on the height it took off
    // from. Those two sources agree on flat floor and nowhere else, which is
    // exactly the reported bug: jump onto a table and physics correctly
    // stops the capsule on the tabletop while jumpZ keeps counting down to
    // the takeoff height, so the drawn human sinks back to floor level and
    // the collision volume is left floating above it.
    //
    // Reading Z back here instead makes the physics body the single source
    // of truth for all of X/Y/Z/yaw, so standing on a table, a step, or the
    // floor are all the same case and none of them need special handling.
    // No offset arithmetic is involved (and so no double-counting is
    // possible): the companion's model origin sits at its own feet -- its
    // link pose 0.85 minus the capsule's 0.85 half-height, see
    // human_collision_body/model.sdf -- and TrajectoryPose.Z is likewise
    // measured from the actor's feet, because NormalizeSpawnPose() left the
    // mesh-origin height on the base Pose component. The two are literally
    // the same quantity in the same frame.
    gz::math::Pose3d groundTruthPose;
    if (this->CollisionBodyPose(_ecm, groundTruthPose))
      _nextPose.Pos().Z(groundTruthPose.Pos().Z());
    else
      _nextPose.Pos().Z(this->jumpZ);
  }

  /// \brief Advances the jump arc for this tick. Jump is a Z-only overlay on
  /// top of whatever X/Y/yaw motion the path or velocity branch computes --
  /// so "jump while walking" needs no special handling: the horizontal
  /// component keeps coming from this->velocity (or the active path) exactly
  /// as it already would, and this just adds a projectile-motion Z arc on
  /// top. Integrated as a running (jumpZ, jumpVelocityZ) state each tick
  /// (Euler step) rather than a closed-form parabola keyed on
  /// elapsed-time-since-launch, so a double jump can simply re-kick
  /// jumpVelocityZ from wherever the actor currently is mid-air, instead of
  /// needing to reason about restarting a parabola from a nonzero height.
  /// jumpCount (0 = grounded, 1 = single jump used, 2 = double jump used)
  /// gates JumpCallback() -- see there.
  ///
  /// The arc shapes the jump; it does NOT decide when the jump ends. Landing
  /// is detected off the companion body's real collision result, so the
  /// actor lands on whatever it actually came down on (floor, table, step)
  /// rather than on an assumed flat return to takeoff height -- see the
  /// landing block below.
  ///
  /// Runs BEFORE the movement branches rather than after them (where it used
  /// to live) so that jumpZ is already current for this tick when the
  /// velocity branch builds the collision body's Twist -- that's what lets
  /// the debug capsule rise and fall with the actor instead of staying
  /// planted on the floor while the human hops out of it.
  private: void IntegrateJump(gz::sim::EntityComponentManager &_ecm, double _dt)
  {
    bool launched = false;
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      if (this->jumpRequested)
      {
        this->jumpVelocityZ = std::sqrt(2.0 * kGravity * this->jumpHeightParam);
        ++this->jumpCount;
        this->jumpRequested = false;
        launched = this->jumpCount == 1;
      }
    }
    // Taking off from the ground (not a mid-air double jump): remember where
    // the collision body was resting, so the arc below is measured from
    // whatever it was standing on rather than assuming z=0. Jumping off a
    // step or a kerb then lands the capsule back on that step.
    if (launched)
    {
      gz::math::Pose3d collisionPose;
      this->jumpCollisionBaseZ = this->CollisionBodyPose(_ecm, collisionPose)
          ? collisionPose.Pos().Z() : 0.0;
    }
    if (this->jumpCount == 0)
      return;
    this->jumpVelocityZ -= kGravity * _dt;
    this->jumpZ += this->jumpVelocityZ * _dt;

    // Landing. Only ever checked while descending, so a jump can't "land"
    // on the way up.
    if (this->jumpVelocityZ >= 0.0)
      return;
    gz::math::Pose3d collisionPose;
    if (this->CollisionBodyPose(_ecm, collisionPose))
    {
      // Physics -- not this arc -- decides where the ground is. The arc
      // keeps descending past whatever it took off from; the capsule can't,
      // because a floor/tabletop/step is in the way. So once the body sits
      // measurably ABOVE where the arc wants it, something solid is holding
      // it up and that is the landing surface. Rebasing jumpCollisionBaseZ
      // onto it is what lets the actor end a jump standing on a table
      // instead of being dragged back down to takeoff height.
      //
      // The old test was `jumpZ <= 0.0`, i.e. "have I fallen back to the
      // height I left from" -- structurally unable to notice a tabletop,
      // and half of the reported bug (the other half being that the actor's
      // Z ignored the physics body entirely, see ApplyStandingMotion()).
      if (collisionPose.Pos().Z() - (this->jumpCollisionBaseZ + this->jumpZ) >
          kLandingContactTolerance)
      {
        this->jumpCollisionBaseZ = collisionPose.Pos().Z();
        this->jumpZ = 0.0;
        this->jumpVelocityZ = 0.0;
        this->jumpCount = 0;
      }
      return;
    }
    // No companion body to ask (none configured, or still mid-spawn): fall
    // back to the original flat-ground assumption, which is the best that
    // can be done without collision information.
    if (this->jumpZ <= 0.0)
    {
      this->jumpZ = 0.0;
      this->jumpVelocityZ = 0.0;
      this->jumpCount = 0;
    }
  }

  /// \brief Vertical velocity to command the collision body with this tick so
  /// it tracks the actor's jump arc.
  ///
  /// Feed-forward (jumpVelocityZ, the plugin's own Euler-integrated jump
  /// velocity for this tick) plus a SMALL, CLAMPED correction for whatever
  /// gap has opened between the capsule's actual Z and the arc's target --
  /// not a bare (target - actual) / dt term, which is what this used to be.
  /// That divides the position error by the physics step size, so at a
  /// typical ~1 ms step it commands 1000x the error as a velocity every
  /// single tick: a deadbeat controller with effectively infinite gain.
  /// It only stays stable if the engine reproduces our Euler integration
  /// exactly, which it doesn't -- this capsule is a real dynamic body with
  /// gravity and floor contacts of its own, and any tiny mismatch between
  /// the commanded velocity and what physics actually resolves (contact
  /// solver iterations, gravity acting the same step, ordinary float
  /// error) gets amplified by that gain on the very next tick. Over a
  /// jump's much larger excursion that snowballs into exactly the
  /// "vibrates and launches itself" behaviour this fixes; it was already
  /// happening at rest too, just too small (sub-mm gravity sag) to notice.
  /// Bounding the correction with kJumpCorrectionGain/kJumpCorrectionMax
  /// keeps it self-correcting (still closes the gap, just over several
  /// ticks instead of one) without the runaway.
  ///
  /// While GROUNDED this instead commands a slow, constant descent
  /// (kGroundSeekRate) and lets contact stop it, rather than servoing to a
  /// remembered height. VelocityControl sets the body's velocity outright
  /// every step, so a real <gravity> can never move this body on its own --
  /// something has to command the downward motion, and a fixed height
  /// target would actively hold the capsule hovering in mid-air the moment
  /// the actor walked off the tabletop it landed on. A gentle downward seek
  /// is what "gravity" means for a velocity-controlled body: it settles onto
  /// whatever is underneath, steps down off ledges, and costs nothing while
  /// already resting on a surface (contact simply cancels it).
  private: double CollisionJumpRate(gz::sim::EntityComponentManager &_ecm,
      double _dt)
  {
    gz::math::Pose3d collisionPose;
    if (_dt <= 0.0 || !this->CollisionBodyPose(_ecm, collisionPose))
      return 0.0;
    if (this->jumpCount == 0)
    {
      // Grounded: seek downward and let physics decide where that stops.
      // Keeping jumpCollisionBaseZ pinned to where the body actually is
      // means the next jump launches its arc from the real current surface
      // (see IntegrateJump()'s `launched` block, which reads the same
      // value) without this needing to know how the actor got there --
      // walked down a step, rode a moving surface, or landed on a table.
      this->jumpCollisionBaseZ = collisionPose.Pos().Z();
      return -kGroundSeekRate;
    }
    const double target = this->jumpCollisionBaseZ + this->jumpZ;
    const double error = target - collisionPose.Pos().Z();
    const double correction = std::clamp(error * kJumpCorrectionGain,
        -kJumpCorrectionMax, kJumpCorrectionMax);
    return this->jumpVelocityZ + correction;
  }

  /// \brief Lazily resolves and caches the companion collision-body entity
  /// by name (it may still be mid-spawn for the first few ticks after this
  /// actor's own Configure() runs). Shared by the removal path (PreUpdate's
  /// removalRequested block) and ApplyStandingMotion() -- previously each
  /// had its own copy of this same null-check-then-EntityByComponents()
  /// lookup.
  /// \brief Reads the companion collision body's current world pose, i.e.
  /// wherever physics actually let it end up last step. False means there
  /// isn't one to read (none configured, still mid-spawn, or just removed) --
  /// every caller has an uncollided fallback for that case.
  ///
  /// Also self-heals the cached entity id: an id that resolves to something
  /// without a Pose component is a stale/vanished entity, so it's dropped
  /// and the next tick looks the name up again rather than silently doing
  /// nothing forever.
  private: bool CollisionBodyPose(gz::sim::EntityComponentManager &_ecm,
      gz::math::Pose3d &_pose)
  {
    const gz::sim::Entity resolved = this->ResolveCollisionEntity(_ecm);
    if (resolved == gz::sim::kNullEntity)
      return false;
    auto collisionPose = _ecm.Component<gz::sim::components::Pose>(resolved);
    if (collisionPose == nullptr)
    {
      this->collisionEntity = gz::sim::kNullEntity;
      return false;
    }
    _pose = collisionPose->Data();
    return true;
  }

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
  private: std::string poseTopic;
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
  // Bounds for CollisionJumpRate()'s position-error correction term -- see
  // its own comment for why this is clamped rather than a bare error/dt.
  private: static constexpr double kJumpCorrectionGain = 10.0;
  private: static constexpr double kJumpCorrectionMax = 2.0;
  // How far the descending jump arc has to sink below where the capsule
  // actually is before IntegrateJump() calls it a landing. Has to clear the
  // servo's own normal tracking error (millimetres -- the feed-forward term
  // means there's no steady-state lag to speak of) without being so large
  // that a landing goes unnoticed for several ticks; 5 cm sits comfortably
  // between the two.
  private: static constexpr double kLandingContactTolerance = 0.05;
  // Downward velocity commanded while grounded so the capsule settles onto
  // whatever is beneath it -- this package's stand-in for gravity, which
  // cannot act on a VelocityControl-driven body. See CollisionJumpRate().
  // Slow on purpose: it only ever has to close contact gaps and walk the
  // body down off ledges, and a fast descent would let it tunnel a step
  // deeper into a surface before the contact solver catches it.
  private: static constexpr double kGroundSeekRate = 0.8;
  private: bool jumpRequested{false};
  // Height of the jump arc ABOVE jumpCollisionBaseZ (the surface it launched
  // from) -- not the actor's world Z, which is read back from the physics
  // body instead. jumpZ only reaches the actor directly as the fallback for
  // when there is no companion body to read. See ApplyStandingMotion().
  private: double jumpZ{0.0};
  private: double jumpVelocityZ{0.0};
  // 0 = grounded, 1 = single jump in progress, 2 = double jump used --
  // see JumpCallback()/the PreUpdate() jump block.
  private: int jumpCount{0};
  private: double jumpHeightParam{1.0};
  // World Z of the surface the actor is standing on: continuously tracked
  // from the collision body while grounded, frozen at the takeoff surface
  // for the duration of a jump, and re-pinned to the landing surface the
  // moment contact is detected. The arc is measured from this rather than
  // from an assumed z=0, which is what lets a jump start and finish on a
  // table or a step. See CollisionJumpRate()/IntegrateJump().
  private: double jumpCollisionBaseZ{0.0};
  private: std::chrono::steady_clock::duration animationTime{
      std::chrono::steady_clock::duration::zero()};
  // Name currently written to the AnimationName component -- see the
  // change-detection guard in PreUpdate()'s animation-selection block for
  // why this has to be tracked instead of writing unconditionally.
  private: std::string appliedAnimationName;

  /// \brief Named-pose cycle driven by pose_topic (PoseCallback()) -- see
  /// kPoseClips for the pose table and the state machine at the top of
  /// PreUpdate(). None is the plugin's original walk/path/velocity
  /// behaviour, unchanged; the other three play the active pose's
  /// enter/hold/exit clips in order while freezing the actor's footprint at
  /// whatever pose it had when the pose started.
  private: enum class PoseState { None, Entering, Holding, Exiting };
  private: PoseState poseState{PoseState::None};
  // What the GUI is asking for right now (transport thread) vs. what's
  // actually playing (simulation thread). requestedSupportHeight/
  // activeSupportHeight are the "what is it being done on top of" height
  // from the pose command -- 0 for the floor, a seat height for a chair.
  private: std::string requestedPose;
  private: double requestedSupportHeight{0.0};
  private: int activePoseIndex{-1};
  private: double activeSupportHeight{0.0};
  // Seconds into the current enter/hold/exit clip, already scaled by
  // poseSpeed -- i.e. this ticks in clip-time, the same units as the
  // table's raw clip lengths, so it's compared directly against
  // EnterSeconds()/ExitSeconds() (which just return those raw lengths).
  private: double poseClipTime{0.0};
  private: double poseSpeed{2.5};
  private: gz::math::Pose3d frozenPose{gz::math::Pose3d::Zero};

  /// \brief 状態の publish（構想書 §3・§13 段階3）。PublishState() を参照。
  /// published* は「最後に送った内容」で、変化検出だけに使う
  /// -- 状態そのものは上のメンバから毎回導出する（二重管理を避けるため）。
  private: std::string stateTopic;
  private: gz::transport::Node::Publisher statePublisher;
  private: CharacterState publishedState{CharacterState::Unknown};
  private: std::string publishedPose;
  private: std::chrono::steady_clock::duration lastStatePublish{
      std::chrono::steady_clock::duration::zero()};
};
}  // namespace gz_human_sim

GZ_ADD_PLUGIN(gz_human_sim::ActorCommandPlugin, gz::sim::System,
    gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz_human_sim::ActorCommandPlugin,
    "gz_human_sim::ActorCommandPlugin")
