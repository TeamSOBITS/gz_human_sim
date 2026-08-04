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
#include <gz/msgs/param.pb.h>
#include <gz/transport/Node.hh>

// サーバー側と共有する状態の定義（構想書 §3）。
#include "gz_human_sim/CharacterState.hh"
#include "CameraController.hh"
#include "HumanRegistry.hh"

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
  // Labels for the global "経路テンプレート" combo (see generatePathTemplate()).
  // A maintained list here, same pattern as humanModels/viewpointLabels/
  // followModeLabels above -- add one label + one case in
  // generatePathTemplate()'s switch whenever scripts/path_template.py grows a
  // new --shape, rather than trying to auto-discover shapes from the
  // filesystem at runtime.
  Q_PROPERTY(QStringList pathTemplateLabels READ PathTemplateLabels CONSTANT)
  Q_PROPERTY(QStringList viewpointLabels READ ViewpointLabels CONSTANT)
  Q_PROPERTY(QStringList humanList READ HumanList NOTIFY humansChanged)
  Q_PROPERTY(QString status READ Status NOTIFY StatusChanged)
  Q_PROPERTY(int activeHumanIndex READ ActiveHumanIndex NOTIFY activeHumanChanged)
  // Held state of Shift -- now the "run" modifier (see kRunFactor in the
  // .cc); no longer means turn-to-face (that moved to sHeld/Key_S below).
  // Held state of Ctrl -- the "slow walk" modifier (kSlowFactor).
  // Held state of S -- now a pure modifier key (no longer "stop", see
  // Key_N for that): S+direction turns the actor to face that direction
  // first, then walks forward, exactly what Shift+direction used to do.
  // Held state of J -- the "strafe mode" modifier: J+direction reproduces
  // the original body-relative strafe (no turning, body stays facing
  // forward) instead of the default turn-to-face-then-walk. See
  // PressDirectionKey()/the eventFilter() diagonal-key branch.
  // Which named pose (see kPoseShortcuts in the .cc) the operator is holding
  // a key down for right now, or "" for none. Currently K = "sit"; the point
  // of carrying a NAME rather than one bool per pose is that adding a second
  // pose later is one row in that table plus its clips in the model SDF,
  // with no new property, signal, or QML binding.
  // The pose the active human has REGISTERED (L key / the panel button):
  // the pose it holds on its own, without a key being held down, until it's
  // unregistered. "" means nothing registered. Mirrors
  // activeFollowModeIndex's pattern: per-human state (Human::lockedPose),
  // reflected here only for whichever human is "対象" right now, for the QML
  // button/legend.
  // Jump peak height (meters) used by teleopJump()/the Enter-key shortcut --
  // a global setting like shiftHeld above, not per-human. Adjustable live
  // from the QML slider (setJumpHeight()).
  // Baseline multiplier on kTeleopSpeed/kTeleopDiagonal for every teleop
  // Twist this panel publishes, adjustable from the QML slider
  // (setSpeedMultiplier()). Ctrl/Shift apply a further kSlowFactor/
  // kRunFactor on top of this at the moment a direction key is pressed --
  // see EffectiveSpeedMultiplier() in the .cc.
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
  // Crowd/SFM feature set (see SfmCrowdSystem in src/sfm_crowd_system.cpp):
  // batch-spawning a crowd, recording a route by clicking in the 3D view,
  // and choosing per-human whether that route is driven by the Social
  // Force Model (avoids other humans/robots/walls, registered with
  // SfmCrowdSystem over gz-transport -- see EnsureSfmPublishers()) or by
  // the existing plain path-follow (cmd_path, no avoidance -- the same
  // mechanism sendWaypoint() already uses).
  //
  // routeRecording and spawnPicking below are the two click modes. Exactly
  // one of them can be on at a time (turning either on turns the other
  // off), and while BOTH are off a left click in the 3D view does nothing
  // at all -- which is the point of having modes: ordinary clicking,
  // selecting and camera work in the viewport must stay usable, and a panel
  // that silently turned every stray click into a spawn point made that
  // impossible.
  Q_PROPERTY(bool routeRecording READ RouteRecording NOTIFY routeRecordingChanged)
  // "x, y" strings for the not-yet-confirmed route, in click order -- the
  // QML list showing what's been placed so far. The route is now GLOBAL
  // (one route, applied to every human ticked as a route target) rather
  // than a separate list per human: a route drawn by clicking is nearly
  // always meant for "these N background people walk this corridor", and
  // making each human own a private copy meant re-clicking the same
  // corridor once per person. See routeTargetAt()/setRouteTarget().
  Q_PROPERTY(QStringList pendingRoutePoints READ PendingRoutePoints NOTIFY pendingRouteChanged)
  // How the confirmed route is driven, also global now for the same reason
  // as pendingRoutePoints above (they only take effect at confirmRoute()
  // time, so per-human copies bought nothing).
  Q_PROPERTY(bool useSfm READ UseSfm NOTIFY routeSettingsChanged)
  Q_PROPERTY(bool cyclicRoute READ CyclicRoute NOTIFY routeSettingsChanged)
  // Whether confirmRoute() runs the route through NavGridSystem's global
  // planner first (see src/nav_grid_system.cpp), turning the straight legs
  // between clicked points into legs that go AROUND walls and furniture.
  // Without it the route is sent exactly as drawn, and any leg crossing an
  // obstacle walks the human into it -- ActorCommandPlugin's waypoint
  // follower steers straight at the next point and has no notion of what is
  // in between. avoidObstaclesAvailable mirrors sfmAvailable: the planner
  // lives in a world plugin, so a world that doesn't load it can't offer it.
  Q_PROPERTY(bool avoidObstacles READ AvoidObstacles NOTIFY routeSettingsChanged)
  Q_PROPERTY(bool avoidObstaclesAvailable READ AvoidObstaclesAvailable NOTIFY routeSettingsChanged)
  // Per-human runtime switch (not a route setting): whether the active
  // human's SfmCrowdSystem commands are currently allowed to drive it.
  Q_PROPERTY(bool activeSfmEnabled READ ActiveSfmEnabled NOTIFY sfmModeChanged)
  // Whether the world actually has an SfmCrowdSystem loaded (detected by
  // looking for a subscriber on the register topic -- see
  // SfmSystemAvailable()). Worlds that don't load it, which is most of them
  // (only gz_human_sim's own sfm_crowd_demo.world does), silently swallowed
  // every SFM route registration: the panel published, nothing subscribed,
  // and the human just stood there. The QML uses this to warn instead.
  Q_PROPERTY(bool sfmAvailable READ SfmAvailable NOTIFY routeSettingsChanged)
  // How many spawned humans are currently ticked as route targets -- drives
  // the QML's "N人に適用" button label/enabled state.
  Q_PROPERTY(int routeTargetCount READ RouteTargetCount NOTIFY humansChanged)
  // Spawn-point picking: same 3D-view-click mechanism as routeRecording
  // above, but for choosing WHERE the next humans get created. Points
  // ACCUMULATE (one per click) so a single Spawn press can create a whole
  // group, each person at their own clicked spot -- see spawnHumans().
  Q_PROPERTY(bool spawnPicking READ SpawnPicking NOTIFY spawnPickingChanged)
  Q_PROPERTY(QStringList pendingSpawnPoints READ PendingSpawnPoints NOTIFY pendingSpawnPointsChanged)

  /// \brief Whether the DualSense/gamepad teleop mode is active. While on,
  /// the left stick drives activeHumanIndex (via the same teleopMove()
  /// analog hook the keyboard's J-strafe mode uses) and the right stick
  /// orbits the GUI camera around it every rendered frame -- see
  /// PollDualsense(). Independent of and layered on top of keyboard
  /// teleop, not exclusive with it. Mirrors guide_robot's
  /// GuiderRobotManager::dualsenseModeEnabled.

  /// \brief Human-readable controller connection state for the QML label
  /// (e.g. "DualSense Wireless Controller 接続中" / "コントローラが見つかりません").

  /// \brief 選択中の人物の状態（サーバーが publish した値の表示用）。
  ///        このパネルが推測した値ではありません -- 構想書 §3。
  Q_PROPERTY(QString activeCharacterState READ ActiveCharacterState
      NOTIFY characterStateChanged)

  /// \brief Right-stick vertical direction for the orbit camera. Off (the
  /// default) is the third-person convention every console game ships
  /// with: push the stick up and the camera swings down so you look up.
  /// On restores the flight-sim sense -- push up, look down -- which is
  /// what this panel did unconditionally before the flag existed. Kept
  /// identical to GuiderRobotManager's flag of the same name so the two
  /// panels never disagree about which way the stick goes.

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
  public: int ActiveViewIndex() const;
  public: double ActiveViewDistance() const;
  public: int ActiveFollowModeIndex() const;
  public: bool RouteRecording() const;
  public: QStringList PendingRoutePoints() const;
  public: bool ActiveSfmEnabled() const;
  public: bool UseSfm() const;
  public: bool CyclicRoute() const;
  public: bool SfmAvailable() const;
  public: bool AvoidObstacles() const;
  public: bool AvoidObstaclesAvailable() const;
  public: int RouteTargetCount() const;
  public: bool SpawnPicking() const;
  public: QStringList PendingSpawnPoints() const;

  /// \brief 選択中の人物の状態を日本語ラベルで返す。未受信なら "—"。
  public: QString ActiveCharacterState() const;

  /// \brief _index の人物の状態ラベル。人物一覧の行に出す用。
  public: Q_INVOKABLE QString characterStateAt(int _index) const;

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

  /// \brief Spawns exactly one human, at exactly (_x, _y). The single-person
  /// primitive every spawnHumans() layout ends up calling, kept public
  /// because it's also the natural entry point for anything scripted.
  ///
  /// _followMode is only meaningful when isActorModel(_modelIndex) is true;
  /// ignored (may be empty) for static models. Raw ActorCommandPlugin value
  /// from followModeValue(), typically "auto" or "path" -- see
  /// FollowModeLabels() for the QML-facing labels in the same order.
  public: Q_INVOKABLE void spawnHuman(
      int _modelIndex, const QString &_name, const QString &_posePreset,
      const QString &_followMode, double _x, double _y, double _z, double _yaw);

  public: Q_INVOKABLE void removeHuman(int _index);

  /// \brief Which spawned human (index into humanList) the keyboard
  /// shortcuts (movement keys, 1-9 to switch target) currently drive.
  /// Sits alongside, not instead of, the per-row GUI pad, which always
  /// targets its own row regardless of this selection.
  public: Q_INVOKABLE void setActiveHuman(int _index);

  public: Q_INVOKABLE void sendWaypoint(int _index, double _x, double _y);

  /// \brief Toggles route-recording mode (routeRecording). While on, every
  /// LeftClickToScene event (see eventFilter()) appends a point to the
  /// pending route; while off, the same click picks a spawn point instead
  /// (see the pendingSpawnPoints Q_PROPERTY).
  public: Q_INVOKABLE void setRouteRecording(bool _enabled);

  /// \brief Turns spawn-point picking on/off. While on, every
  /// LeftClickToScene appends a spawn point; turning it on turns route
  /// recording off (see the routeRecording Q_PROPERTY for why the two click
  /// modes are exclusive, and why "neither" has to remain a valid state).
  public: Q_INVOKABLE void setSpawnPicking(bool _enabled);

  /// \brief Removes the last clicked spawn point / discards all of them.
  /// Same pair of edits undoLastRoutePoint()/clearPendingRoute() offer for
  /// the route, since both lists are built the same way (by clicking).
  public: Q_INVOKABLE void undoLastSpawnPoint();
  public: Q_INVOKABLE void clearSpawnPoints();

  /// \brief The one way to create humans, whether that's one person or a
  /// crowd, at typed coordinates or at clicked ones -- what used to be
  /// spawnHuman()/spawnCrowd()/spawnAtPickedPoints() as three separate
  /// entry points with three separate forms in the panel.
  ///
  /// Where each person goes:
  ///  - picked spawn points present -> one person per point, exactly where
  ///    it was clicked, and _count/_x/_y are ignored (the clicks already
  ///    said both how many and where). The points are consumed.
  ///  - otherwise -> _count people laid out on the kSpawnGridSpacing grid
  ///    starting at (_x, _y), pre-separated so their individual spawn-safety
  ///    probes don't have to push each other apart.
  ///
  /// Naming is always _baseName + a running number continuing past whoever
  /// already exists (human1, human2, ...), so a second batch never collides
  /// with the first. Every person still goes through the same single-spawn
  /// path (name-collision check, spawn-safety probe), so this only decides
  /// how many and where.
  public: Q_INVOKABLE void spawnHumans(
      int _modelIndex, const QString &_baseName, int _count,
      const QString &_posePreset, const QString &_followMode,
      double _x, double _y, double _z, double _yaw);

  /// \brief Show/hide the spawn marker of one human, or of all of them.
  /// The marker is a flat coloured disc drawn at the position that human
  /// was originally spawned at -- purely a render-scene visual (see
  /// ApplySpawnMarkers()), never a simulation entity, so it has no
  /// collision, no mass, and never shows up in the entity tree or in a
  /// spawn-safety probe's way. Each human gets its own colour so a group of
  /// them can be told apart at a glance.
  public: Q_INVOKABLE void setShowSpawnMarker(int _index, bool _value);
  public: Q_INVOKABLE void setShowSpawnMarkerAll(bool _value);
  public: Q_INVOKABLE bool showSpawnMarkerAt(int _index) const;

  /// \brief "#rrggbb" of _index's spawn-marker colour, so the QML row can
  /// show the same colour swatch the 3D view draws.
  public: Q_INVOKABLE QString spawnMarkerColorAt(int _index) const;

  /// \brief Discards the not-yet-confirmed route (start over) / removes just
  /// its last clicked point (undo a misplaced click). Global now -- there is
  /// one route shared by every route target, see the pendingRoutePoints
  /// Q_PROPERTY.
  public: Q_INVOKABLE void clearPendingRoute();
  public: Q_INVOKABLE void undoLastRoutePoint();

  /// \brief Whether human _index is one of the route targets: the set of
  /// humans confirmRoute() sends the pending route to. Newly spawned humans
  /// start ticked, so the common "spawn a few, draw one route, press go"
  /// flow needs no per-human ticking at all; untick the ones that should
  /// keep doing something else.
  public: Q_INVOKABLE bool routeTargetAt(int _index) const;
  public: Q_INVOKABLE void setRouteTarget(int _index, bool _value);
  public: Q_INVOKABLE void setAllRouteTargets(bool _value);

  /// \brief true = drive the confirmed route through SfmCrowdSystem (avoids
  /// other humans/robots/walls, but only works in a world that actually
  /// loads that system -- see sfmAvailable); false = through the plain
  /// cmd_path follow (no avoidance, works in every world). Only affects what
  /// the NEXT confirmRoute() does; switching it afterwards does not
  /// retroactively convert an already-confirmed route -- confirm again.
  public: Q_INVOKABLE void setUseSfm(bool _value);

  /// \brief Whether the route loops back to its first point (SfmCrowdSystem's
  /// cyclicGoals / a true patrol) or stops once the last point is reached.
  /// Applies to the NEXT confirmRoute() call, same as setUseSfm() above.
  public: Q_INVOKABLE void setCyclicRoute(bool _value);

  /// \brief Turn collision-aware planning on/off for the next confirmRoute().
  public: Q_INVOKABLE void setAvoidObstacles(bool _value);

  /// \brief Only meaningful once _index has a route confirmed under SFM
  /// mode: publishes Boolean _value on that human's sfm_enable_topic (see
  /// SfmCrowdSystem), the same switch that lets a human be handed back to
  /// manual teleop without SfmCrowdSystem's own commands fighting it.
  public: Q_INVOKABLE void setSfmEnabled(int _index, bool _value);

  /// \brief setSfmEnabled() for every current route target at once.
  public: Q_INVOKABLE void setSfmEnabledForTargets(bool _value);

  /// \brief Replaces the pending route with a canned shape (see
  /// PathTemplateLabels()) instead of publishing it straight to one human.
  /// Feeding the templates into the same pending route the 3D-view clicks
  /// build means both ways of making a route end at the same one "walk it"
  /// button, for the same multi-human target set -- previously the template
  /// button published immediately, to exactly one human, through a
  /// completely separate path that ignored the SFM/cyclic settings.
  /// _centerX/_centerY are world coordinates; _size is the radius for the
  /// circle template or the side length for the square one; _numWaypoints
  /// only applies to the circle (the square is always its 4 corners).
  public: Q_INVOKABLE void generatePathTemplate(
      int _templateIndex, double _centerX, double _centerY,
      double _size, int _numWaypoints, bool _clockwise);

  /// \brief Sends the pending route to every route target, in whichever mode
  /// useSfm currently selects: registered with SfmCrowdSystem
  /// (register_topic) or published as a plain cmd_path waypoint sequence.
  /// Does not clear the pending route afterward -- confirm again after
  /// adding/undoing points to update it in place.
  ///
  /// Also makes sure each target is actually *able* to walk it: a human left
  /// in "経路専用" follow mode with SFM off, or one whose sfm_enable was
  /// switched off earlier, would otherwise accept the route and then just
  /// stand there, which is what "経路を確定しても歩かない" was.
  public: Q_INVOKABLE void confirmRoute();

  /// \brief Point the GUI camera at a spawned human. _viewIndex matches
  /// ViewpointLabels()'s order: 0 = free camera, then chase/front/side/
  /// top/diagonal offsets scaled by _distance meters. Mirrors
  /// GuiderRobotManager::setViewpoint()'s view set.
  public: Q_INVOKABLE void setViewpoint(int _index, int _viewIndex, double _distance);

  /// \brief Release any follow/track target and put the camera back at the
  /// pose it had when the world first loaded (MinimalScene's own
  /// <camera_pose> from gui.config), i.e. the initial full-scene overview.
  /// A no-op until CameraController has captured that starting pose at
  /// least once.
  public: Q_INVOKABLE void resetToInitialView();

  protected: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Watches gz::gui::events::Render to apply viewpoint commands on
  /// the render thread (the only thread allowed to touch the Ogre2 scene).
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;


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


  /// \brief Render-thread only, called from the same eventFilter() Render
  /// branch as CameraController::ApplyPending(): pushes each actor-backed human's
  /// Human::showCollision to its collision-body capsule in the render
  /// scene, retrying while the companion model is still spawning (same
  /// idea as CameraController's target search). Does nothing on a human
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

  /// \brief Handler for /world/<w>/dynamic_pose/info -- caches every
  /// entity's live (x, y, z, yaw) in poses, keyed by name. Runs on a
  /// transport thread; CheckProbeSettle() (Qt thread) reads it under
  /// poseMutex. Same technique guide_robot's GuiderRobotManager already
  /// uses for its own pose cache.
  private: void OnPoseInfo(const gz::msgs::Pose_V &_message);

  private: gz::transport::Node node;

  /// \brief Announces this panel's humans so GuiderPadController (a
  /// plugin in the separate guide_robot package, which may not be loaded
  /// at all) can drive them with the same pad as the robots.
  ///
  /// The topic name and the '|'-separated line format are specified by
  /// guide_robot's `src/GuiderTargetRoster.hh`. It is duplicated here
  /// rather than included because gz_human_sim must not gain a build
  /// dependency on guide_robot -- so any change to that format has to be
  /// mirrored in PublishRoster() below.
  private: gz::transport::Node::Publisher rosterPublisher;

  private: void PublishRoster();

  /// \brief state_topic のハンドラ。transport スレッドで走る。
  ///        _name はどの人物のものかを閉じ込めた値（トピックは人物ごと）。
  private: void OnCharacterState(const gz::msgs::Param &_message,
      const std::string &_name);

  /// \brief _index の人物の state トピックを購読する。spawn の確定時に呼ぶ。
  private: void SubscribeCharacterState(int _index);

  /// \brief 人物ごとの状態を守る。OnCharacterState() が transport スレッドで
  ///        書き、Qt スレッドが読む（poseMutex と同じ扱い）。
  private: mutable std::mutex stateMutex;

  private: QTimer *rosterTimer{nullptr};
  /// \brief スポーン済みの人物一覧。struct Human ごと
  ///        HumanRegistry.hh へ移した（構想書 §13 段階 2）。
  private: HumanRegistry humans;
  private: std::string worldName;
  private: QString statusText{"ワールドを検出中…"};
  private: QStringList posePresetList;
  private: int activeHumanIndex{-1};


  /// \brief GUI カメラの視点制御。人物のことは何も知らないクラスなので、
  /// 「何番の人物か」の解決は HumanControlPanelCamera.cc 側で行う。
  /// 将来 guide_robot と共通の GUI 基盤パッケージへ出す予定（構想書 §12）。
  private: CameraController cameraController;

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

  /// \brief Absolute path to media/spawn_marker.png (the magic-circle
  /// texture), resolved once in LoadConfig() the same way
  /// collisionBodyTemplate's path is. Empty if it could not be found, in
  /// which case markers fall back to a plain tinted plane.
  private: std::string spawnMarkerTexturePath;

  /// \brief Time the spin angle of every marker is derived from, so they
  /// rotate together at a rate independent of the GUI's frame rate.
  private: std::chrono::steady_clock::time_point spawnMarkerEpoch;
  private: bool spawnMarkerEpochValid{false};

  /// \brief Advertises sfmRegisterPublisher/sfmUnregisterPublisher on first
  /// use (not in the constructor: worldName isn't known yet there, and
  /// these topics are fixed/global rather than per-world, but deferring
  /// keeps every other transport setup in this class the same "lazy,
  /// first-use" shape). See SfmCrowdSystem's own register_topic/
  /// unregister_topic SDF defaults, which these must match.
  private: void EnsureSfmPublishers();

  /// \brief Builds and publishes the register_human payload sending the
  /// pending route to human _index -- see SfmCrowdSystem::ParseRegistration()
  /// for the exact wire format this must match.
  private: void SendSfmRegistration(int _index);

  /// \brief Publishes the pending route to _index as a plain cmd_path
  /// waypoint sequence -- no SfmCrowdSystem involvement, so
  /// ActorCommandPlugin's ordinary (non-avoiding) path-follow drives it.
  private: void SendSimplePath(int _index);

  /// \brief Whether anything is subscribed to the SFM register topic, i.e.
  /// whether this world actually loaded an SfmCrowdSystem. Queried through
  /// gz-transport's own discovery (Node::TopicInfo()'s subscriber list)
  /// rather than assumed: most worlds don't load that system, and in those
  /// an SFM registration is published into the void, which is exactly why a
  /// confirmed route used to leave everyone standing still.
  private: bool SfmSystemAvailable();

  /// \brief Whether NavGridSystem's planning service is being offered by this
  /// world, cached into avoidObstaclesAvailableState.
  private: bool NavPlannerAvailable();

  /// \brief Runs _route through NavGridSystem's planner, returning the
  /// collision-free version. Falls back to _route unchanged (returning false)
  /// whenever the planner isn't there or can't answer, so a route is always
  /// sent -- an unplanned route that walks into a table is still better than
  /// a button that silently does nothing.
  private: bool PlanAroundObstacles(
      const std::vector<std::pair<double, double>> &_route,
      double _bodyRadius,
      std::vector<std::pair<double, double>> &_planned);

  /// \brief The humans confirmRoute() should act on: every one ticked as a
  /// route target, or -- if none are -- just the active human, so the
  /// button still does the obvious thing before anyone has touched a
  /// tickbox.
  private: std::vector<int> RouteTargetIndices() const;

  /// \brief Render-thread only, called from the same eventFilter() Render
  /// branch as CameraController::ApplyPending(): creates each human's spawn-marker visual
  /// on first sight and pushes its desired visibility, plus a marker for
  /// each not-yet-spawned picked spawn point. Purely render-scene objects
  /// (Visual + Material, no ECM entity), so they can never collide with
  /// anything, be picked up by a spawn-safety probe, or appear in the
  /// entity tree -- see setShowSpawnMarker().
  private: void ApplySpawnMarkers();

  /// \brief Creates one flat magic-circle marker visual named _name at
  /// (_x, _y, _z), or nullptr if the scene won't build it. A single
  /// texture-mapped plane rather than a pile of primitives, so a world
  /// full of markers stays cheap; the texture is drawn white and tinted
  /// here (translucent white while a point is only pending, the human's
  /// own identity colour once it exists).
  private: gz::rendering::VisualPtr CreateMarkerVisual(
      const gz::rendering::ScenePtr &_scene, const std::string &_name,
      double _x, double _y, double _z, int _colorIndex, bool _pending) const;


  private: gz::transport::Node::Publisher sfmRegisterPublisher;
  private: gz::transport::Node::Publisher sfmUnregisterPublisher;
  /// \brief Global route-recording toggle -- see the routeRecording
  /// Q_PROPERTY's comment.
  private: bool routeRecordingState{false};
  /// \brief The other click mode -- see the routeRecording Q_PROPERTY.
  private: bool spawnPickingState{false};

  /// \brief The one shared route every route target walks, and how it is
  /// driven -- see the pendingRoutePoints/useSfm Q_PROPERTY comments.
  private: std::vector<std::pair<double, double>> pendingRoute;
  private: bool useSfmState{false};
  private: bool cyclicRouteState{true};

  /// \brief Cached result of SfmSystemAvailable(), refreshed on demand
  /// rather than every frame (gz-transport discovery is a network query).
  private: bool sfmAvailableState{false};
  private: bool avoidObstaclesState{true};
  private: bool avoidObstaclesAvailableState{false};
  /// \brief The route actually sent by the last confirmRoute() -- the planned
  /// one when obstacle avoidance is on, so the QML can show how many points
  /// the planner produced from the handful that were clicked.
  private: std::vector<std::pair<double, double>> lastSentRoute;

  /// \brief Spawn points picked by clicking the 3D view, awaiting a Spawn
  /// press -- see the pendingSpawnPoints Q_PROPERTY's comment.
  ///
  /// Z is carried alongside X/Y because the click that places a point
  /// reports the height of whatever it landed on, and in a multi-storey
  /// world that is the whole point: clicking the 4F slab has to spawn the
  /// human on 4F. These used to be (x, y) pairs, so every picked point
  /// silently fell back to the spawn form's own z and everyone ended up
  /// on the ground floor. See spawnHumans() for how it combines with the
  /// form's per-model ground offset.
  private: struct PickedPoint
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };
  private: std::vector<PickedPoint> pendingSpawnPoints;

  /// \brief Render-scene visuals for pendingSpawnPoints (rebuilt whenever
  /// the list length changes) and the names of markers whose human has been
  /// removed, queued here on the Qt thread for ApplySpawnMarkers() to
  /// actually destroy on the render thread.
  private: std::vector<gz::rendering::VisualPtr> pendingSpawnMarkerVisuals;
  /// \brief Set on every pendingSpawnPoints edit; ApplySpawnMarkers()
  /// rebuilds the pending markers and clears it. See its own comment for
  /// why a size comparison isn't enough.
  private: bool pendingSpawnMarkersDirty{false};
  private: std::mutex markerMutex;
  private: std::vector<std::string> markerRemovalQueue;





  signals: void humansChanged();
  signals: void StatusChanged();
  signals: void activeHumanChanged();
  signals: void activeViewIndexChanged();
  signals: void activeFollowModeChanged();
  signals: void routeRecordingChanged();
  signals: void pendingRouteChanged();
  signals: void pendingSpawnPointsChanged();
  signals: void spawnPickingChanged();
  signals: void routeSettingsChanged();
  signals: void sfmModeChanged();
  signals: void characterStateChanged();
};
}  // namespace gz_human_sim
#endif
