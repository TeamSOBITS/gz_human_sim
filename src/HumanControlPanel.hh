#ifndef GZ_HUMAN_SIM_HUMAN_CONTROL_PANEL_HH_
#define GZ_HUMAN_SIM_HUMAN_CONTROL_PANEL_HH_

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <QObject>
#include <QPointer>
#include <QProcess>
#include <QString>
#include <QStringList>

#include <gz/gui/Plugin.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
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
  Q_PROPERTY(bool shiftHeld READ ShiftHeld NOTIFY shiftHeldChanged)
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

  /// \brief Move human _index one step of the QWEASDZXC 9-key layout
  /// (Q/W/E/A/S/D/Z/X/C, matching the on-screen pad and the keyboard
  /// shortcuts handled in eventFilter()). "S" stops. Plain presses are pure
  /// body-relative strafes with no turning (X included -- straight-back
  /// strafe, i.e. moonwalking). _turnToFace (Shift+key on the keyboard)
  /// instead spins the actor in place to face that direction first, then
  /// walks forward once turned -- constant linear+angular together would
  /// just trace a circle (unicycle-model kinematics), not a turn-then-walk
  /// motion, so this is done as two sequential phases.
  public: Q_INVOKABLE void teleopDirection(
      int _index, const QString &_direction, bool _turnToFace = false);

  /// \brief Which spawned human (index into humanList) the keyboard
  /// shortcuts (QWEASDZXC to move, 1-9 to switch target) currently drive.
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
    // Runtime follow_mode changes (setFollowMode()) -- separate from the
    // spawn-time follow_mode:= launch argument, which only sets the
    // initial SDF value. followModeIndex mirrors viewIndex below: index
    // into FollowModeLabels()/kFollowModeValues, kept in sync so the
    // global combo shows this human's actual current mode when switched
    // to, rather than always resetting to "テレオペ".
    gz::transport::Node::Publisher followModePublisher;
    int followModeIndex{0};
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

  /// \brief Synchronous, short-timeout query of /world/<w>/scene/info for
  /// whether a model named _name currently exists. Used instead of trusting
  /// spawn/remove service acks (gz-sim's create/remove services both return
  /// success at the request-acceptance level even when the entity was never
  /// actually inserted/found — the real truth only shows up in the scene
  /// graph, or as a [Err] line in the server's own log).
  private: bool QueryEntityExists(const std::string &_name);

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

  private: gz::transport::Node node;
  private: std::vector<Human> humans;
  private: std::string worldName;
  private: QString statusText{"ワールドを検出中…"};
  private: QStringList posePresetList;
  private: int activeHumanIndex{-1};
  private: bool shiftHeldState{false};

  private: gz::rendering::CameraPtr userCamera;
  private: std::mutex viewMutex;
  private: ViewCommand viewCommand;
  private: std::string viewpointTarget;
  // Captured once, the first time ApplyViewpoint() finds the user camera
  // (i.e. still at whatever gui.config's MinimalScene <camera_pose>
  // placed it at) -- see resetToInitialView().
  private: gz::math::Pose3d initialCameraPose;
  private: bool initialCameraPoseCaptured{false};

  signals: void humansChanged();
  signals: void StatusChanged();
  signals: void activeHumanChanged();
  signals: void activeViewIndexChanged();
  signals: void activeFollowModeChanged();
  signals: void shiftHeldChanged();
};
}  // namespace gz_human_sim
#endif
