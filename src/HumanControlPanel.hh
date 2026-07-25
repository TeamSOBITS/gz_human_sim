#ifndef GZ_HUMAN_SIM_HUMAN_CONTROL_PANEL_HH_
#define GZ_HUMAN_SIM_HUMAN_CONTROL_PANEL_HH_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <QObject>
#include <QPointer>
#include <QProcess>
#include <QString>
#include <QStringList>

#include <gz/gui/Plugin.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/rendering/RenderTypes.hh>
#include <gz/transport/Node.hh>

namespace gz_human_sim
{
/// \brief Sidebar plugin that spawns, removes and teleoperates gz_human_sim
/// human models from inside the Gazebo GUI, replacing the rcjo2025_arena
/// world's old practice of baking human_actor_1/2 directly into the world
/// SDF. Modeled on guide_robot's GuiderRobotManager: spawning runs
/// `ros2 launch gz_human_sim spawn_human.launch.py` as a child process
/// (so the same bridges/topics come up as they would from a terminal),
/// removal despawns the gz entity via /world/<w>/remove, teleop publishes
/// gz.msgs.Twist directly to the actor's vel_topic (the ActorCommandPlugin
/// subscribes over gz-transport, no ROS bridge needed for this panel's own
/// controls), and viewpoint switching drives the GUI camera's follow/track
/// targets directly on the render thread (same technique as
/// GuiderRobotManager::ApplyViewpoint).
class HumanControlPanel : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QStringList humanModels READ HumanModels CONSTANT)
  Q_PROPERTY(QStringList posePresets READ PosePresets CONSTANT)
  // Labels for the spawn form's "動作モード" combo (actor models only --
  // see isActorModel()); index order matches followModeValue()'s raw
  // values ("auto"/"path", i.e. ActorCommandPlugin's follow_mode SDF
  // parameter). "velocity" (ignore any cmd_path) isn't offered here --
  // see kFollowModeValues in HumanControlPanel.cc -- but still works as a
  // spawn_human.launch.py follow_mode:= argument for anyone who wants it.
  Q_PROPERTY(QStringList followModeLabels READ FollowModeLabels CONSTANT)
  // Labels for the global "経路テンプレート" combo (see sendPathTemplate()).
  // A maintained list here, same pattern as humanModels/viewpointLabels/
  // followModeLabels above -- add one label + one case in
  // sendPathTemplate()'s switch whenever scripts/path_template.py grows a
  // new --shape, rather than trying to auto-discover shapes from the
  // filesystem at runtime.
  Q_PROPERTY(QStringList pathTemplateLabels READ PathTemplateLabels CONSTANT)
  Q_PROPERTY(QStringList viewpointLabels READ ViewpointLabels CONSTANT)
  Q_PROPERTY(QStringList humanList READ HumanList NOTIFY humansChanged)
  Q_PROPERTY(QString status READ Status NOTIFY StatusChanged)
  Q_PROPERTY(int activeHumanIndex READ ActiveHumanIndex NOTIFY activeHumanChanged)
  // Held state of Shift -- now the "run" modifier (see kRunFactor in the
  // .cc); no longer means turn-to-face (that moved to sHeld/Key_S below).
  Q_PROPERTY(bool shiftHeld READ ShiftHeld NOTIFY shiftHeldChanged)
  // Held state of Ctrl -- the "slow walk" modifier (kSlowFactor).
  Q_PROPERTY(bool ctrlHeld READ CtrlHeld NOTIFY ctrlHeldChanged)
  // Held state of S -- now a pure modifier key (no longer "stop", see
  // Key_N for that): S+direction turns the actor to face that direction
  // first, then walks forward, exactly what Shift+direction used to do.
  Q_PROPERTY(bool sHeld READ SHeld NOTIFY sHeldChanged)
  // Held state of J -- the "strafe mode" modifier: J+direction reproduces
  // the original body-relative strafe (no turning, body stays facing
  // forward) instead of the default turn-to-face-then-walk. See
  // PressDirectionKey()/the eventFilter() diagonal-key branch.
  Q_PROPERTY(bool jHeld READ JHeld NOTIFY jHeldChanged)
  // Which named pose (see kPoseShortcuts in the .cc) the operator is holding
  // a key down for right now, or "" for none. Currently K = "sit"; the point
  // of carrying a NAME rather than one bool per pose is that adding a second
  // pose later is one row in that table plus its clips in the model SDF,
  // with no new property, signal, or QML binding.
  Q_PROPERTY(QString heldPose READ HeldPose NOTIFY heldPoseChanged)
  // The pose the active human has REGISTERED (L key / the panel button):
  // the pose it holds on its own, without a key being held down, until it's
  // unregistered. "" means nothing registered. Mirrors
  // activeFollowModeIndex's pattern: per-human state (Human::lockedPose),
  // reflected here only for whichever human is "対象" right now, for the QML
  // button/legend.
  Q_PROPERTY(QString activeLockedPose READ ActiveLockedPose NOTIFY activeLockedPoseChanged)
  // Jump peak height (meters) used by teleopJump()/the Enter-key shortcut --
  // a global setting like shiftHeld above, not per-human. Adjustable live
  // from the QML slider (setJumpHeight()).
  Q_PROPERTY(double jumpHeight READ JumpHeight NOTIFY jumpHeightChanged)
  // Baseline multiplier on kTeleopSpeed/kTeleopDiagonal for every teleop
  // Twist this panel publishes, adjustable from the QML slider
  // (setSpeedMultiplier()). Ctrl/Shift apply a further kSlowFactor/
  // kRunFactor on top of this at the moment a direction key is pressed --
  // see EffectiveSpeedMultiplier() in the .cc.
  Q_PROPERTY(double speedMultiplier READ SpeedMultiplier NOTIFY speedMultiplierChanged)
  // Current viewpoint state of the active human, i.e. whatever setViewpoint()
  // last set for humans[activeHumanIndex]. Single global viewpoint controls
  // (see the QML) bind to these instead of each spawned human owning its
  // own combo box, mirroring guide_robot's GuiderRobotManager (one
  // viewpointCombo driven by whichever robot robotCombo currently selects,
  // not one per robot).
  Q_PROPERTY(int activeViewIndex READ ActiveViewIndex NOTIFY activeViewIndexChanged)
  Q_PROPERTY(double activeViewDistance READ ActiveViewDistance NOTIFY activeViewIndexChanged)
  // Same pattern as activeViewIndex above, but for follow_mode: lets the
  // global combo in the QML change an already-spawned human's mode at
  // runtime (setFollowMode()) and shows whichever mode is actually active
  // for the current "対象" human, not just what was picked at spawn time.
  Q_PROPERTY(int activeFollowModeIndex READ ActiveFollowModeIndex NOTIFY activeFollowModeChanged)

  public: HumanControlPanel();
  public: ~HumanControlPanel() override;

  public: QStringList HumanModels() const;
  public: QStringList PosePresets() const;
  public: QStringList FollowModeLabels() const;
  public: QStringList PathTemplateLabels() const;
  public: QStringList ViewpointLabels() const;
  public: QStringList HumanList() const;
  public: QString Status() const;
  public: int ActiveHumanIndex() const;
  public: bool ShiftHeld() const;
  public: bool CtrlHeld() const;
  public: bool SHeld() const;
  public: bool JHeld() const;
  public: QString HeldPose() const;
  public: QString ActiveLockedPose() const;
  public: double JumpHeight() const;
  public: double SpeedMultiplier() const;
  public: int ActiveViewIndex() const;
  public: double ActiveViewDistance() const;
  public: int ActiveFollowModeIndex() const;

  /// \brief Suggested spawn name for a model index ("human1", "human2", …).
  public: Q_INVOKABLE QString defaultName(int _modelIndex) const;

  /// \brief One-line description of a model index, shown in parentheses
  /// next to its name in the spawn dropdown.
  public: Q_INVOKABLE QString modelDescription(int _modelIndex) const;

  /// \brief Suggested spawn Z for a model index, used to refresh the QML
  /// spawn form's z field when the model dropdown changes. Models differ
  /// in where their mesh origin sits relative to the ground (e.g.
  /// DoctorFemaleWalk needs 0.0, walking_actor needs 1.0).
  public: Q_INVOKABLE double defaultZ(int _modelIndex) const;

  /// \brief Suggested spawn X/Y for the next spawn, fanned out so
  /// consecutive spawns at default coordinates don't land on top of each
  /// other. Recompute (and refresh the QML fields) after every spawn/removal.
  public: Q_INVOKABLE double nextSpawnX() const;
  public: Q_INVOKABLE double nextSpawnY() const;

  /// \brief Whether a model index is custom_human (its human_pose preset
  /// picker only applies to this one).
  public: Q_INVOKABLE bool isCustomHuman(int _modelIndex) const;

  /// \brief Whether a model index (walking_actor, DoctorFemaleWalk, ...) is
  /// actor-backed -- drives the spawn form's follow-mode combo visibility,
  /// same idea as isCustomHuman() for the pose preset combo. Static models
  /// (person_standing, custom_human) have no ActorCommandPlugin, so
  /// follow_mode means nothing for them.
  public: Q_INVOKABLE bool isActorModel(int _modelIndex) const;

  /// \brief Whether a model index ships the extra pose clips the named-pose
  /// feature needs (currently walking_actor only, which is the only one with
  /// sit_down/sitting/stand_up meshes) -- narrower than isActorModel(),
  /// drives the spawn form's/panel's pose control visibility.
  public: Q_INVOKABLE bool isPoseCapableModel(int _modelIndex) const;

  /// \brief Same as isHumanActorAt() but for the named-pose feature --
  /// whether the spawned human at this humanList row can be posed.
  public: Q_INVOKABLE bool isPoseCapableHumanAt(int _index) const;

  /// \brief Register/unregister _index's current pose (L key, or the QML
  /// button). Registering pins whatever pose the human is in right now --
  /// including "standing", which simply means nothing is registered -- so it
  /// keeps holding it once the pose key is released. Pressing it again
  /// unregisters, dropping back to whatever a held key is currently asking
  /// for. See UpdatePoseIntent() in the .cc.
  ///
  /// Deliberately "register the current pose" rather than "toggle sitting":
  /// once there are more poses than sit, one key that means "stay like that"
  /// keeps working for all of them without needing a lock key each.
  public: Q_INVOKABLE void togglePoseLock(int _index);

  /// \brief Human-readable label for a pose name ("sit" -> "着席"), for
  /// status lines and the QML button. Empty name gives the "standing"/no
  /// pose label.
  public: Q_INVOKABLE QString poseLabel(const QString &_pose) const;

  /// \brief Raw ActorCommandPlugin follow_mode string ("auto"/"path"/
  /// "velocity") for a FollowModeLabels() index, to pass into spawnHuman().
  public: Q_INVOKABLE QString followModeValue(int _followModeIndex) const;

  /// \brief Change an already-spawned human's follow_mode at runtime, via
  /// ActorCommandPlugin's follow_mode_topic -- unlike the spawn form's
  /// follow-mode combo (which only sets the SDF's initial value once, at
  /// spawn time), this reaches a human that's already walking around.
  /// No-op for non-actor humans (no ActorCommandPlugin to reach).
  public: Q_INVOKABLE void setFollowMode(int _index, int _followModeIndex);

  /// \brief Whether the spawned human at this row (an index into
  /// humanList, not a humanModels index) is an actor model — drives the
  /// QML teleop pad's visibility per-row.
  public: Q_INVOKABLE bool isHumanActorAt(int _index) const;

  /// \brief Current showCollision state for row _index, for the per-row
  /// QML checkbox to read back (e.g. after setShowCollisionAll() changes
  /// it out from under a row that isn't "対象" right now).
  public: Q_INVOKABLE bool showCollisionAt(int _index) const;

  /// \brief Show/hide _index's collision-body debug capsule (hidden by
  /// default) -- actually applied on the render thread by
  /// ApplyCollisionVisibility(), not here. No-op for non-actor humans (no
  /// collision-body companion to show).
  public: Q_INVOKABLE void setShowCollision(int _index, bool _value);

  /// \brief setShowCollision() for every currently-spawned human at once.
  public: Q_INVOKABLE void setShowCollisionAll(bool _value);

  /// \brief Current capsule dimensions for row _index (see
  /// Human::collisionRadius/collisionLength), for the QML size sliders to
  /// initialize/resync from when switching which human is active.
  public: Q_INVOKABLE double collisionRadiusAt(int _index) const;
  public: Q_INVOKABLE double collisionLengthAt(int _index) const;

  /// \brief Rebuild _index's collision-body companion at a new capsule
  /// size. gz-sim has no way to resize a shape in place (see the
  /// implementation's own comment), so this removes the existing
  /// companion and spawns a fresh one from human_collision_body/
  /// model.sdf's template with new <radius>/<length> values, at the same
  /// position the old one was at. ActorCommandPlugin re-resolves its
  /// cached collision entity by name on its own the next tick, so nothing
  /// on that side needs to know this happened.
  public: Q_INVOKABLE void applyCollisionSize(int _index, double _radius, double _length);

  /// \brief _followMode is only meaningful when isActorModel(_modelIndex)
  /// is true; ignored (may be empty) for static models. Raw ActorCommandPlugin
  /// value from followModeValue(), typically "auto" or "path" -- see
  /// FollowModeLabels() for the QML-facing labels in the same order.
  public: Q_INVOKABLE void spawnHuman(
      int _modelIndex, const QString &_name, const QString &_posePreset,
      const QString &_followMode, double _x, double _y, double _z, double _yaw);
  public: Q_INVOKABLE void removeHuman(int _index);
  public: Q_INVOKABLE void teleopMove(
      int _index, double _linear, double _lateral, double _angular);
  public: Q_INVOKABLE void teleopStop(int _index);

  /// \brief Enter-key / jump-button shortcut: publishes a one-shot jump
  /// request (current jumpHeight) to _index's cmd_jump topic. Purely a
  /// Z-axis overlay on the ActorCommandPlugin side -- whatever horizontal
  /// velocity is already active (held W/A/D/X, teleopDirection(), a path,
  /// ...) keeps driving X/Y/yaw unchanged, so holding a direction key and
  /// pressing Enter jumps while still moving that way, same as in a 3D
  /// game. Works a 2nd time while still airborne (double jump); a 3rd
  /// press before landing is a no-op -- see
  /// ActorCommandPlugin::JumpCallback()'s jumpCount guard.
  public: Q_INVOKABLE void teleopJump(int _index);

  /// \brief S+A / S+D shortcut: publishes a pure-angular Twist (no linear/
  /// lateral) so the actor spins in place instead of walking --
  /// _counterClockwise true for S+A (left), false for S+D (right). Scaled
  /// by EffectiveSpeedMultiplier() same as any other movement, so Ctrl/
  /// Shift still slow down/speed up the spin. Release (KeyRelease/
  /// onReleased) should call teleopStop(), same as any other held key.
  public: Q_INVOKABLE void teleopRotate(int _index, bool _counterClockwise);

  /// \brief Manual, continuous jump-height set from the QML slider (meters,
  /// clamped).
  public: Q_INVOKABLE void setJumpHeight(double _height);

  /// \brief Manual, continuous baseline speed-multiplier set from the QML
  /// slider. Ctrl/Shift scale on top of whatever this is set to -- see
  /// EffectiveSpeedMultiplier() in the .cc.
  public: Q_INVOKABLE void setSpeedMultiplier(double _value);

  /// \brief Move human _index one step of the movement layout (Q W E / A D
  /// / Z X C around a vacated center -- matching the on-screen pad and the
  /// keyboard shortcuts handled in eventFilter()). _turnToFace (the
  /// default for ordinary movement now -- see PressDirectionKey()) sends
  /// PublishTurnToFace()'s absolute-heading Twist, which
  /// ActorCommandPlugin::PreUpdate() uses to smoothly steer the actor's
  /// yaw toward that direction's fixed world heading while it keeps
  /// walking forward the whole time -- a natural in-motion arc, not a
  /// stop-in-place-then-walk, and since the target is an absolute world
  /// angle (not relative to wherever the actor currently happens to be
  /// facing), repeating the same direction key is always idempotent.
  /// Passing false instead reproduces the original body-relative strafe
  /// with no turning (X included -- straight-back strafe, i.e.
  /// moonwalking), via the ordinary teleopMove() Twist; that's now only
  /// reachable via the J "strafe mode" modifier (see PressDirectionKey()/
  /// the eventFilter() diagonal-key branch). Either way, speed is
  /// EffectiveSpeedMultiplier() (Ctrl slows, Shift speeds up). Stopping is
  /// teleopStop(), not a "direction" here -- see Key_N in eventFilter().
  public: Q_INVOKABLE void teleopDirection(
      int _index, const QString &_direction, bool _turnToFace = false);

  /// \brief Which spawned human (index into humanList) the keyboard
  /// shortcuts (movement keys, 1-9 to switch target) currently drive.
  /// Sits alongside, not instead of, the per-row GUI pad, which always
  /// targets its own row regardless of this selection.
  public: Q_INVOKABLE void setActiveHuman(int _index);

  public: Q_INVOKABLE void sendWaypoint(int _index, double _x, double _y);

  /// \brief Compute and publish a canned-shape path (see
  /// PathTemplateLabels()) directly to human _index's cmd_path, the same
  /// way sendWaypoint() sends a single point -- no subprocess, no ROS
  /// bridge involved (unlike scripts/path_template.py, the standalone CLI
  /// version of the same shapes, meant for headless/scripted use outside
  /// the GUI). _centerX/_centerY are world coordinates; _size is the
  /// radius for the circle template or the side length for the square
  /// one; _numWaypoints only applies to the circle (square is always its
  /// 4 corners).
  public: Q_INVOKABLE void sendPathTemplate(
      int _index, int _templateIndex, double _centerX, double _centerY,
      double _size, int _numWaypoints, bool _clockwise);

  /// \brief Point the GUI camera at a spawned human. _viewIndex matches
  /// ViewpointLabels()'s order: 0 = free camera, then chase/front/side/
  /// top/diagonal offsets scaled by _distance meters. Mirrors
  /// GuiderRobotManager::setViewpoint()'s view set.
  public: Q_INVOKABLE void setViewpoint(int _index, int _viewIndex, double _distance);

  /// \brief Release any follow/track target and put the camera back at the
  /// pose it had when the world first loaded (MinimalScene's own
  /// <camera_pose> from gui.config), i.e. the initial full-scene overview.
  /// A no-op until ApplyViewpoint() has captured that starting pose at
  /// least once.
  public: Q_INVOKABLE void resetToInitialView();

  protected: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Watches gz::gui::events::Render to apply viewpoint commands on
  /// the render thread (the only thread allowed to touch the Ogre2 scene).
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

  private: struct Human
  {
    std::string name;
    std::string model;
    QProcess *process{nullptr};
    gz::transport::Node::Publisher velocityPublisher;
    gz::transport::Node::Publisher pathPublisher;
    // Actors can't be removed through /world/<w>/remove (gz-sim's
    // UserCommands system only accepts MODEL/LIGHT there); this instead
    // tells the actor's own ActorCommandPlugin to remove itself via the
    // ECM directly. Only valid for actor-backed humans, same as the two
    // publishers above.
    gz::transport::Node::Publisher removePublisher;
    // Jump requests (see teleopJump()) -- only advertised for actor-backed
    // humans, same as velocityPublisher/pathPublisher above.
    gz::transport::Node::Publisher jumpPublisher;
    // Runtime follow_mode changes (setFollowMode()) -- separate from the
    // spawn-time follow_mode:= launch argument, which only sets the
    // initial SDF value. followModeIndex mirrors viewIndex below: index
    // into FollowModeLabels()/kFollowModeValues, kept in sync so the
    // global combo shows this human's actual current mode when switched
    // to, rather than always resetting to "テレオペ".
    gz::transport::Node::Publisher followModePublisher;
    int followModeIndex{0};
    // Named-pose intent (pose_topic) -- only advertised for pose-capable
    // actor humans (see IsPoseCapableIndex()), same as jumpPublisher above.
    // lockedPose is the L key's registered pose: non-empty means this human
    // holds that pose on its own, with no key held down, until it's
    // unregistered -- see UpdatePoseIntent()/togglePoseLock() in the .cc.
    gz::transport::Node::Publisher posePublisher;
    std::string lockedPose;
    // Bumped by every teleopDirection()/teleopStop() call for this human.
    // "X" schedules a delayed second Twist (see teleopDirection()); that
    // callback only fires if this still matches the value it captured,
    // so a later key press/release in the meantime cancels it instead of
    // stomping on whatever the user asked for next.
    int teleopGeneration{0};
    // Last viewpoint setViewpoint() applied to this human (0 = free/never
    // set). Lets the global viewpoint combo (see ActiveViewIndex()) show
    // the right selection when switching which human is active, instead
    // of always resetting to "自由視点".
    int viewIndex{0};
    double viewDistance{2.0};

    // Collision-body debug visualization (see human_collision_body/
    // model.sdf's translucent capsule) -- only meaningful for actor-backed
    // humans, same as velocityPublisher above (checked the same way:
    // velocityPublisher.Valid()). showCollision is the desired on/off
    // state (setShowCollision()/setShowCollisionAll()); the other two are
    // ApplyCollisionVisibility()'s own render-thread bookkeeping.
    //
    // Defaults to hidden: the capsule is a debug aid (see
    // human_collision_body/model.sdf's own comment), not something a user
    // driving/watching a human normally wants cluttering the view, so it
    // starts off and has to be opted into per row (or via "all") from the
    // panel. appliedShowCollision still starts at the tri-state "nothing
    // applied yet" below, so this hidden state is explicitly pushed to the
    // freshly-spawned collision body on frame one rather than assumed.
    //
    // appliedShowCollision is deliberately a tri-state int (-1 = nothing
    // applied yet) rather than a bool, so a freshly (re)spawned
    // collision body always gets the current state pushed to it even when
    // that state happens to equal the previous one -- see
    // applyCollisionSize(), which resets it.
    bool showCollision{false};
    int appliedShowCollision{-1};
    int collisionVisualRetries{0};
    // Current capsule dimensions, updated by applyCollisionSize() -- kept
    // here so the QML size sliders can read back what's actually applied
    // (collisionRadiusAt()/collisionLengthAt()) when switching which human
    // is active, same idea as followModeIndex/viewIndex above. Defaults
    // match models/human_collision_body/model.sdf's own spawn-time values.
    double collisionRadius{0.25};
    double collisionLength{1.2};
  };

  /// \brief Pending camera command, written on the Qt thread by
  /// setViewpoint() and consumed on the render thread by ApplyViewpoint().
  /// `engage` false means "release the camera back to free view".
  private: struct ViewCommand
  {
    bool pending{false};
    bool engage{false};
    std::string target;
    gz::math::Vector3d followOffset{0.0, 0.0, 0.0};
    gz::math::Vector3d trackOffset{0.0, 0.0, 0.6};
    // True (the default, set for every view except kViewFirstPerson) means
    // followOffset/trackOffset are fixed WORLD-frame vectors, so the
    // camera's position/look-at direction don't rotate when the human's
    // body turns -- only the human's own position (which the offsets are
    // still added to every frame) moves the camera, keeping the
    // background visually stable while the body turns in place (a chase
    // view that rotated with the body would swing the whole background
    // around on every turn, which reads as disorienting rather than as a
    // 3D-game-style chase camera). kViewFirstPerson sets this false
    // instead, since a head/eye view SHOULD turn with the body.
    bool worldFrame{true};
    QString label;
    int retries{0};
    // Only meaningful together with engage=false: also snap the camera
    // back to initialCameraPose instead of just releasing follow/track
    // and leaving it wherever it drifted to (see resetToInitialView()).
    bool resetPose{false};
  };

  private: void DiscoverWorld();
  private: void SetStatus(const QString &_status);
  private: QProcess *StartLaunchProcess(const QStringList &_arguments);
  private: void TerminateProcessGroup(QProcess *_process);

  /// \brief Request /world/<w>/remove (Entity::MODEL) for _name. Only valid
  /// for MODEL-backed humans (person_standing, custom_human) -- gz-sim's
  /// UserCommands system rejects ACTOR entities outright ("Entity [n] is
  /// not a model or a light, so it can't be removed."), no matter what
  /// Entity::Type is set on the request. Actor-backed humans are removed
  /// via their own remove-topic instead (see removeHuman()).
  private: void RequestEntityRemoval(const std::string &_name);

  /// \brief Fire-and-forget /world/<w>/create request for a MODEL entity
  /// (gz::msgs::EntityFactory: _sdf as the raw SDF text, _name as the
  /// entity name, position only -- capsules are rotationally symmetric
  /// about Z so no orientation is needed). Same request shape
  /// ProbeSafeSpawnPosition() already builds inline for its throwaway
  /// probes; pulled out here since applyCollisionSize() needs the same
  /// call for a real (non-probe) respawn.
  private: void RequestEntityCreation(const std::string &_sdf,
      const std::string &_name, double _x, double _y, double _z);

  /// \brief Synchronous, short-timeout query of /world/<w>/scene/info for
  /// whether a model named _name currently exists. Used instead of trusting
  /// spawn/remove service acks (gz-sim's create/remove services both return
  /// success at the request-acceptance level even when the entity was never
  /// actually inserted/found — the real truth only shows up in the scene
  /// graph, or as a [Err] line in the server's own log).
  private: bool QueryEntityExists(const std::string &_name);

  /// \brief spawnHuman()'s original body (build launch arguments, start
  /// the process, poll for confirmation) -- now only reached once _x/_y
  /// are already known-safe: called directly for non-actor (static)
  /// models, and via ProbeSafeSpawnPosition()/CheckProbeSettle() for
  /// actor models, which may have shifted _x/_y away from what the user/
  /// GUI originally asked for (see those two for why).
  private: void StartRealSpawn(
      int _modelIndex, QString _name, QString _posePreset, QString _followMode,
      double _x, double _y, double _z, double _yaw);

  /// \brief Spawns a throwaway probe (the same physics collision body
  /// every actor gets paired with, briefly visible as a translucent
  /// capsule -- see models/human_collision_body/model.sdf and
  /// collisionBodyTemplate) at (_x, _y) and, once it's had a moment to
  /// settle, checks whether
  /// gz-sim's own contact/penetration resolution pushed it away from
  /// where it was dropped -- the same signal that made a bare (0,0)
  /// spawn end up embedded in rcjo2025_arena's center wall. _attempt
  /// indexes into SpawnSpiralOffset(), walked outward in a nearest-first
  /// spiral from the original (_x, _y) until CheckProbeSettle() finds one
  /// that's clear, or gives up after kSpawnSafetyMaxAttempts and spawns at
  /// the original position anyway (better than refusing to spawn at all).
  private: void ProbeSafeSpawnPosition(
      int _modelIndex, QString _name, QString _posePreset, QString _followMode,
      double _x, double _y, double _z, double _yaw, int _attempt);

  /// \brief _attempt-th (dx, dy) offset (metres, in kSpawnGridSpacing
  /// steps) for ProbeSafeSpawnPosition() to add to the user-requested
  /// (_x, _y), ordered by ascending distance from (0, 0) -- attempt 0 is
  /// always (0, 0) itself (the exact requested point), then the 8
  /// neighbours at one grid step out, then the next ring, and so on. This
  /// replaces the old one-quadrant row-major grid ((attempt % columns,
  /// attempt / columns), which only ever walked +X/+Y from the requested
  /// point) with a search that tries every direction, nearest candidates
  /// first -- important now that (_x, _y) is meant to be exactly where
  /// the user asked for (e.g. a future mouse-click spawn), so the first
  /// clear spot found should also be the closest one to it.
  private: std::pair<double, double> SpawnSpiralOffset(int _attempt) const;

  /// \brief ProbeSafeSpawnPosition()'s follow-up, kSpawnSafetySettleMs
  /// later: reads the probe's settled pose from the poses cache (see
  /// OnPoseInfo()), removes the probe either way, and either proceeds to
  /// StartRealSpawn() at (_candidateX, _candidateY) if it travelled the
  /// full approach distance from (_approachStartX, _approachStartY), or
  /// retries ProbeSafeSpawnPosition() at the next spiral slot if it got
  /// pushed away (still overlapping something).
  private: void CheckProbeSettle(
      int _modelIndex, QString _name, QString _posePreset, QString _followMode,
      double _x, double _y, double _z, double _yaw, int _attempt,
      QString _probeName, double _candidateX, double _candidateY,
      double _approachStartX, double _approachStartY);

  /// \brief CheckProbeSettle()'s follow-up when the candidate was judged
  /// safe: polls QueryEntityExists() for the just-removed probe until it's
  /// actually gone (or gives up after kProbeRemovalMaxAttempts and spawns
  /// anyway -- a rare residual overlap is better than blocking forever on
  /// a removal that's stuck), then calls StartRealSpawn() at
  /// (_candidateX, _candidateY). A fixed delay here previously just
  /// guessed at how long removal takes (the same class of race
  /// applyCollisionSize() already has to guard against, see its own
  /// comment) -- confirming it explicitly removes the guess: spawning the
  /// real collision-body companion while the probe (same mass, same
  /// collision shape) is still physically present at essentially the same
  /// spot is what was launching the actor sideways right after spawn (the
  /// actor, perfectly slaved to its companion now, faithfully following
  /// that companion's post-collision physics).
  private: void PollProbeRemoval(
      QString _probeName, int _modelIndex, QString _name, QString _posePreset,
      QString _followMode, double _candidateX, double _candidateY, double _z, double _yaw,
      int _pollAttempt);

  /// \brief Poll QueryEntityExists() every ~500ms (up to ~10s) for a
  /// just-requested spawn. Finalizes the Human entry (list + publishers +
  /// diagonal-up viewpoint) only once the entity is confirmed to exist;
  /// gives up and reports an error (killing the leftover process) otherwise.
  private: void PollSpawnConfirmation(
      QString _name, QString _model, QString _followMode, QPointer<QProcess> _process,
      double _x, double _y, double _z, double _yaw, int _attempt);

  /// \brief Poll QueryEntityExists() to confirm a removal actually took
  /// (rather than trusting the /remove service's ack, which is true even
  /// for a nonexistent entity).
  private: void PollRemovalConfirmation(QString _name, int _attempt);

  /// \brief Render-thread only: apply the pending ViewCommand to the GUI
  /// user camera (retrying while the target model is still spawning).
  private: void ApplyViewpoint();

  /// \brief Render-thread only, called from the same eventFilter() Render
  /// branch as ApplyViewpoint(): pushes each actor-backed human's
  /// Human::showCollision to its collision-body capsule in the render
  /// scene, retrying while the companion model is still spawning (same
  /// idea as ApplyViewpoint()'s target search). Does nothing on a human
  /// whose state is already applied, so the per-frame cost is a couple of
  /// int comparisons in the steady state.
  ///
  /// Writing components::Transparency on the ECM side was considered and
  /// rejected -- confirmed (by reading gz-sim's own RenderUtil.cc/
  /// SceneManager.cc) that it's only ever read once, at the moment a
  /// Visual entity is first created, not watched for changes afterward.
  private: void ApplyCollisionVisibility();

  /// \brief Shows/hides the named collision-body model in the render
  /// scene, returning false if the scene has no visual for it (yet).
  ///
  /// Toggles Visual::SetVisible() rather than dimming a cloned material's
  /// transparency, which is what this used to do and what never actually
  /// worked: it required the model's capsule Visual to carry a non-null
  /// Material() of its own, and gz-sim's SceneManager hangs the material
  /// off the Geometry instead, so the search always came up empty and the
  /// button silently did nothing. Visibility needs no material at all, and
  /// "hidden" means genuinely gone rather than very faint.
  ///
  /// Applies to every visual belonging to the model -- its own top-level
  /// node plus each descendant -- instead of betting on one particular
  /// level of gz-sim's "<model>::<link>::<visual>" scoped naming being the
  /// one that matters. Setting it on all of them is idempotent, and means
  /// this keeps working whichever level the geometry actually hangs from.
  private: bool SetCollisionBodyVisible(const gz::rendering::ScenePtr &_scene,
      const std::string &_modelName, bool _visible) const;

  /// \brief Key-down handler for W/A/D/X: adds _direction to
  /// heldDirectionKeys (capped at 2 -- a 3rd simultaneous press is
  /// ignored). Exactly 1 key held routes through teleopDirection(...,
  /// true) (turn to face, then walk) unless J is held (the "strafe mode"
  /// modifier), in which case it falls through to ApplyHeldDirectionKeys()
  /// for the original no-turn strafe. 2 keys held always goes through
  /// ApplyHeldDirectionKeys() for the curving combo, strafe-mode or not.
  private: void PressDirectionKey(const std::string &_direction);

  /// \brief Key-up handler for W/A/D/X: drops _direction from
  /// heldDirectionKeys. Empty -> teleopStop(). Back down to exactly 1 key
  /// -> same turn-to-face-then-walk restart as PressDirectionKey()'s
  /// solo-key branch (unless J strafe mode, same override). Otherwise
  /// (still 2, i.e. this doesn't currently happen) falls through to
  /// ApplyHeldDirectionKeys().
  private: void ReleaseDirectionKey(const std::string &_direction);

  /// \brief Publishes the Twist for the current heldDirectionKeys onto
  /// the active human. 2 keys (and J not held) is the curving combo:
  /// PublishTurnToFace() toward the SECOND (most recently pressed) key's
  /// absolute heading, same mechanism a solo turn-to-face key uses, so
  /// the body curves in and then walks straight once it reaches that
  /// heading rather than turning forever. Everything else (1 key, or J
  /// strafe mode with either 1 or 2 keys) is a plain body-relative Twist
  /// with angular always 0 -- only reached from PressDirectionKey()/
  /// ReleaseDirectionKey() in J strafe mode, since the normal solo-key
  /// case now goes through teleopDirection() instead (see those two for
  /// why).
  private: void ApplyHeldDirectionKeys();

  /// \brief Publishes an absolute-heading turn-to-face Twist (angular.x
  /// flag set, angular.z = _targetHeadingRad) directly, bypassing
  /// teleopMove()'s plain-Twist shape -- see ActorCommandPlugin::
  /// VelocityCallback() for how the two shapes are told apart, and
  /// teleopDirection() for the only caller.
  private: void PublishTurnToFace(int _index, double _speed, double _targetHeadingRad);

  /// \brief Re-evaluates and republishes whatever heldDirectionKeys is
  /// currently doing, using the just-changed Ctrl/Shift/J state -- called
  /// right after shiftHeldState/ctrlHeldState/jHeldState update in
  /// eventFilter() so e.g. holding W and THEN pressing Shift starts
  /// running immediately, releasing Shift goes back to walking, all
  /// without needing to let go of W. Safe to just re-call
  /// ApplyHeldDirectionKeys() (2-key combo or solo J-strafe) or
  /// teleopDirection(..., true) (solo turn-to-face) unconditionally in
  /// either case now -- both are idempotent regardless of current state,
  /// since turn-to-face targets an absolute heading rather than turning
  /// relative to wherever the actor currently happens to be facing (see
  /// teleopDirection()). Diagonal (Q/E/Z/C) keys aren't tracked in
  /// heldDirectionKeys, so this intentionally doesn't cover them -- they
  /// still pick up a new Ctrl/Shift/J state on their next press.
  private: void RefreshHeldMovementSpeed();

  /// \brief Recomputes which pose _index should be holding and publishes its
  /// name to that human's posePublisher. The effective pose is the
  /// registered one (Human::lockedPose) if there is one, else whichever pose
  /// key is currently held -- and a held key only steers the ACTIVE human,
  /// same as every other keyboard shortcut in this panel.
  ///
  /// Called from eventFilter() on every pose-key press/release and from
  /// togglePoseLock(). No-op for non-pose-capable or non-actor humans
  /// (posePublisher not advertised for those).
  private: void UpdatePoseIntent(int _index);

  /// \brief The pose _index should be in right now, per UpdatePoseIntent()'s
  /// "registered wins over held" rule. Split out because togglePoseLock()
  /// needs the same answer in order to know what to register.
  private: std::string EffectivePose(int _index) const;

  /// \brief this->speedMultiplierState (the QML slider's baseline) scaled
  /// by kSlowFactor if Ctrl is held, kRunFactor if Shift is held, or 1.0 if
  /// neither -- Ctrl and Shift aren't meant to combine, so Ctrl wins if
  /// somehow both are down. Read fresh every time a Twist is (re)published
  /// (teleopDirection(), ApplyHeldDirectionKeys(), teleopRotate(), and
  /// RefreshHeldMovementSpeed() when a held key's speed needs to react to
  /// a live Ctrl/Shift change).
  private: double EffectiveSpeedMultiplier() const;

  /// \brief Handler for /world/<w>/dynamic_pose/info -- caches every
  /// entity's live (x, y, z, yaw) in poses, keyed by name. Runs on a
  /// transport thread; CheckProbeSettle() (Qt thread) reads it under
  /// poseMutex. Same technique guide_robot's GuiderRobotManager already
  /// uses for its own pose cache.
  private: void OnPoseInfo(const gz::msgs::Pose_V &_message);

  private: gz::transport::Node node;
  private: std::vector<Human> humans;
  private: std::string worldName;
  private: QString statusText{"ワールドを検出中…"};
  private: QStringList posePresetList;
  private: int activeHumanIndex{-1};
  private: bool shiftHeldState{false};
  private: bool ctrlHeldState{false};
  private: bool sHeldState{false};
  private: bool jHeldState{false};
  /// \brief Name of the pose whose hold key is down right now (see
  /// kPoseShortcuts in the .cc), or empty for none. One string rather than a
  /// bool per pose so that adding poses needs no new state here.
  private: std::string heldPoseState;
  private: double jumpHeightState{1.0};
  private: double speedMultiplierState{1.0};

  /// \brief W/A/D/X keys currently held (S not held), in press order,
  /// capped at 2 entries -- see PressDirectionKey()/ApplyHeldDirectionKeys().
  private: std::vector<std::string> heldDirectionKeys;

  private: gz::rendering::CameraPtr userCamera;
  private: std::mutex viewMutex;
  private: ViewCommand viewCommand;
  private: std::string viewpointTarget;
  // Captured once, the first time ApplyViewpoint() finds the user camera
  // (i.e. still at whatever gui.config's MinimalScene <camera_pose>
  // placed it at) -- see resetToInitialView().
  private: gz::math::Pose3d initialCameraPose;
  private: bool initialCameraPoseCaptured{false};

  private: struct CachedPose
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    bool valid{false};
  };
  /// \brief Live entity poses keyed by name, from OnPoseInfo() -- written
  /// on a transport thread, read (under poseMutex) on the Qt thread by
  /// CheckProbeSettle().
  private: std::mutex poseMutex;
  private: std::map<std::string, CachedPose> poses;
  private: bool poseSubscribed{false};

  private: struct ActiveProbe
  {
    double startX{0.0};
    double startY{0.0};
    std::shared_ptr<gz::transport::Node::Publisher> publisher;
    bool stopped{false};
  };
  /// \brief In-flight spawn-safety probes (see ProbeSafeSpawnPosition()),
  /// keyed by probe name, guarded by poseMutex (same lock as poses --
  /// OnPoseInfo() updates both together on the transport thread). Nothing
  /// ever told a probe to stop walking once it reached its candidate
  /// point -- VelocityControl just kept integrating the last Twist it
  /// got -- so by the time CheckProbeSettle()'s kSpawnSafetySettleMs timer
  /// fired, an unobstructed probe had overshot the candidate by however
  /// far the remaining settle time let it walk. CheckProbeSettle() itself
  /// still judges/spawns using the original candidate coordinates (never
  /// the probe's overshot position), so the real human always landed in
  /// the right place -- but the probe itself (the same translucent orange
  /// capsule the 当たり判定表示 toggle controls elsewhere) was visibly still
  /// walking past that point when it got deleted, reading as "the tested
  /// spot and the actual spawn spot don't match". OnPoseInfo() now stops
  /// each probe here as soon as it has travelled the full approach
  /// distance, so its last visible position is the candidate itself.
  private: std::map<std::string, ActiveProbe> activeProbes;

  /// \brief Raw text of models/human_collision_body/model.sdf, read once
  /// in LoadConfig() and reused by every ProbeSafeSpawnPosition() call
  /// (each spawn attempt just substitutes a fresh throwaway model name).
  /// Empty if the file couldn't be found, in which case the safety probe
  /// is skipped entirely and spawning proceeds unchecked, same as before
  /// this feature existed.
  private: std::string collisionBodyTemplate;

  signals: void humansChanged();
  signals: void StatusChanged();
  signals: void activeHumanChanged();
  signals: void activeViewIndexChanged();
  signals: void activeFollowModeChanged();
  signals: void shiftHeldChanged();
  signals: void ctrlHeldChanged();
  signals: void sHeldChanged();
  signals: void jHeldChanged();
  signals: void heldPoseChanged();
  signals: void activeLockedPoseChanged();
  signals: void jumpHeightChanged();
  signals: void speedMultiplierChanged();
};
}  // namespace gz_human_sim
#endif
