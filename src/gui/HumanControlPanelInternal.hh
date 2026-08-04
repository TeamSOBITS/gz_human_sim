#ifndef GZ_HUMAN_SIM_GUI_HUMANCONTROLPANELINTERNAL_HH_
#define GZ_HUMAN_SIM_GUI_HUMANCONTROLPANELINTERNAL_HH_

// Shared file-scope tables, tuning constants and small pure helpers for the
// HumanControlPanel translation units (HumanControlPanel*.cc). These all used
// to be `static` at the top of the single 3700-line HumanControlPanel.cc;
// splitting that file by responsibility means several .cc files need the same
// ones, so they live here instead.
//
// Everything is `inline` rather than `static`: an inline helper that reads a
// `static` table would bind to a different copy in every .cc that included it,
// which is an ODR violation. Values and behaviour are unchanged from the
// single-file version.

#include <algorithm>
#include <cmath>
#include <string>

#include <QGuiApplication>
#include <QKeyEvent>
#include <QString>

namespace gz_human_sim
{
// Index order backs both the QML ComboBox and defaultName()/isCustomHuman().
// person_walking (formerly here) was dropped: it's the same "Mingfei"
// generic-actor mesh as walking_actor (compare model.config authorship),
// just fetched at spawn time from a remote Fuel URL instead of the copy
// walking_actor already vendors locally, and with no ActorCommandPlugin of
// its own -- a strictly worse duplicate of walking_actor, not a second
// distinct avatar.
inline const char *const kHumanModels[] = {
  "walking_actor", "DoctorFemaleWalk",
  "person_standing", "custom_human",
};
inline const char *const kHumanModelDescriptions[] = {
  "汎用の歩行アクター。矢印パッド/キーボードで移動可",
  "女性医師の歩行スキン。矢印パッド/キーボードで移動可",
  "静止した立ち姿。移動不可",
  "ポーズ指定できる静止人物。移動不可",
};
// Suggested spawn Z per model, shown as the QML spawn form's default when
// that model is selected. DoctorFemaleWalk's own model.sdf already poses
// the actor at z=0 (its mesh origin sits at ground level, unlike
// walking_actor's, which needs the +0.86 the other three models default
// to) -- keep in sync with kHumanModels above.
inline const double kHumanModelDefaultZ[] = {
  1.0, 0.0, 1.0, 1.0,
};
inline constexpr int kHumanModelCount =
    static_cast<int>(sizeof(kHumanModels) / sizeof(kHumanModels[0]));
inline constexpr int kCustomHumanIndex = 3;

// Raw ActorCommandPlugin follow_mode values the GUI exposes, index-matched
// with FollowModeLabels()'s QML-facing Japanese labels. "velocity" (ignore
// any cmd_path, teleop-only) is deliberately left out here -- it's nearly
// indistinguishable from "auto" for anyone who never sends this human a
// path, so it only added a confusing third choice. It's still available at
// the launch-argument level (`ros2 launch gz_human_sim spawn_human.launch.py
// follow_mode:=velocity ...`) for the rare case that actually wants it.
inline const char *const kFollowModeValues[] = {"auto", "path"};
inline constexpr int kFollowModeCount =
    static_cast<int>(sizeof(kFollowModeValues) / sizeof(kFollowModeValues[0]));

// Path templates generatePathTemplate() can generate, index-matched with
// PathTemplateLabels(); keep this in sync with scripts/path_template.py's
// --shape choices (that script is the standalone-CLI version of the same
// shapes; this one publishes straight to gz-transport for the GUI button
// instead of going through ROS + a subprocess).
enum PathTemplateIndex
{
  kPathTemplateCircle = 0,
  kPathTemplateSquare,
  kPathTemplateCount,
};
inline const char *const kPathTemplateLabels[kPathTemplateCount] = {"円", "四角"};

// Spawn position auto-offset: fans consecutive default-position spawns out
// in a grid instead of stacking them on top of each other at (0, 0).
inline constexpr double kSpawnGridSpacing = 1.2;
inline constexpr int kSpawnGridColumns = 4;

// ProbeSafeSpawnPosition()/CheckProbeSettle(): how many grid slots to try
// before giving up and spawning at the original position regardless; how
// fast the probe walks in from kSpawnGridSpacing south of the candidate
// (same order of magnitude as normal teleop walking speed); how long it
// gets to complete that walk-in before its pose is trusted (must clear
// kSpawnGridSpacing / kSpawnProbeSpeed = 1.2 s at the values below, plus
// margin for acceleration and the discovery-race republishes just before
// it); and how much of that kSpawnGridSpacing walk-in it's allowed to have
// come up short by (measured from where it STARTED, not the candidate --
// see CheckProbeSettle()'s comment for why) and still count as "made it",
// vs. "got stopped partway by something".
inline constexpr int kSpawnSafetyMaxAttempts = 8;
inline constexpr double kSpawnProbeSpeed = 1.0;
inline constexpr int kSpawnSafetySettleMs = 1400;
inline constexpr double kSpawnSafetyDisplacementMeters = 0.25;

// How far the probe walks in toward the candidate before its position is
// judged. This used to be a full kSpawnGridSpacing (1.2 m), which meant the
// test really asked "is the 1.2 m corridor leading to this point clear?" --
// so a clicked spot with a table 1 m to one side got rejected and the human
// was quietly moved somewhere else, reading as "it never spawns where I
// clicked". Keeping the walk-in short makes the test local to the point
// itself: a clear point is accepted on the very first attempt (no
// displacement at all), and only genuine overlap pushes the search outward.
// It still has to be a walk-in rather than a bare drop -- a capsule simply
// dropped into a wall interpenetrates symmetrically and just sits there
// instead of being pushed out, so standing still proves nothing.
inline constexpr double kSpawnProbeApproach = 0.45;

// PollProbeRemoval(): how often to re-check whether a just-removed probe
// is actually gone (QueryEntityExists()), and how many times to check
// before giving up and spawning the real collision-body companion anyway.
// 150ms * 20 = 3s, comfortably more than removal should ever realistically
// take -- this is a safety net for a stuck removal, not the expected path.
inline constexpr int kProbeRemovalPollIntervalMs = 150;
inline constexpr int kProbeRemovalMaxAttempts = 20;

// Entity-existence polling: gz-sim's create/remove services ack the
// request, not the outcome, so spawn/removal are confirmed by polling
// /world/<w>/state instead of trusting the ack. 20 * 500ms = 10s.
inline constexpr int kEntityPollIntervalMs = 500;
inline constexpr int kEntityPollMaxAttempts = 20;
inline constexpr unsigned int kStateQueryTimeoutMs = 800u;

// Same idea as kSpawnSafetyMaxAttempts's 8-attempt cap, but per-frame
// instead of per-spawn-attempt: gives up searching the render scene for a
// human's collision-body visual after this many failed Render-event
// ticks (the companion model may take a few frames to actually appear
// after being spawned/respawned).
inline constexpr int kCollisionVisualMaxRetries = 600;

// NavGridSystem's planning service (see src/nav_grid_system.cpp's
// plan_service SDF default, which this must match) and how long to wait for
// it. Planning is a one-shot A* over an already-built grid, so it answers in
// milliseconds; the generous timeout is only for the first call after world
// load, when the grid may still be being rasterised.
inline const char *const kNavPlanService = "/gz_human_sim/nav/plan_path";
inline constexpr unsigned int kNavPlanTimeoutMs = 3000u;

// Spawn-marker palette (see setShowSpawnMarker()/ApplySpawnMarkers()):
// one colour per human, assigned in spawn order and wrapping around, so a
// group of people spawned together can be told apart by their markers.
// Deliberately saturated and well separated in hue rather than a smooth
// ramp -- these are identity labels, not a scale.
inline const double kMarkerColors[][3] = {
  {0.95, 0.26, 0.21},  // red
  {0.13, 0.59, 0.95},  // blue
  {0.30, 0.69, 0.31},  // green
  {1.00, 0.76, 0.03},  // amber
  {0.61, 0.15, 0.69},  // purple
  {0.00, 0.74, 0.83},  // cyan
  {1.00, 0.44, 0.00},  // orange
  {0.91, 0.12, 0.39},  // pink
};
inline constexpr int kMarkerColorCount =
    static_cast<int>(sizeof(kMarkerColors) / sizeof(kMarkerColors[0]));

// Spawn-marker disc geometry: a cylinder squashed flat and laid just above
// the floor, wide enough to read as "a person starts here" from the default
// overview camera without hiding the person standing on it.
// Half-width of the magic-circle marker. Deliberately much bigger than
// the 0.12 the old flat disc used: this panel is normally driven in a
// whole building, and at a camera distance that shows a floor plan a
// person-sized dot on the ground is simply not visible. 1.2 m across
// reads from across a room without burying the spot it marks.
inline constexpr double kMarkerRadius = 0.6;
inline constexpr double kMarkerZ = 0.02;

// Radians per second the circles turn. Slow on purpose: an idle animation
// to catch the eye, not something competing with the humans for attention.
inline constexpr double kMarkerSpinRate = 0.35;
// Not-yet-spawned picked points are drawn in the same shape but neutral
// white and more transparent -- they aren't anybody's marker yet.
// Near-solid: at the camera distances these panels are actually used from,
// anything much lower washes the circle out against a light floor. The
// pending marker stays a little softer than a placed one purely so the two
// are still tellable apart at a glance, not to make it subtle.
inline constexpr double kMarkerAlpha = 0.97;
inline constexpr double kPendingMarkerAlpha = 0.85;

// kHumanModelDefaultZ[] for a model NAME rather than an index -- used by
// setViewpoint()'s first-person case, which only has Human::model (a
// string) to work from. 1.0 (the walking_actor/person_standing/
// custom_human default) if the name isn't recognised, same fallback
// defaultZ() itself uses.
inline double DefaultZForModelName(const std::string &_model)
{
  const auto it = std::find(std::begin(kHumanModels), std::end(kHumanModels), _model);
  if (it == std::end(kHumanModels))
    return 1.0;
  return kHumanModelDefaultZ[it - std::begin(kHumanModels)];
}

inline bool IsActorIndex(int _index)
{
  // walking_actor, DoctorFemaleWalk -- keep in sync with kHumanModels above.
  return _index >= 0 && _index < 2;
}

// Only walking_actor ships the extra pose clips (sit_down/sitting/stand_up,
// see models/walking_actor/meshes/) -- DoctorFemaleWalk has a single
// walk-only mesh, so the named-pose feature is narrower than IsActorIndex()
// above.
inline bool IsPoseCapableIndex(int _index)
{
  return _index == 0;
}

// Poses that can be held down on a key, mirroring ActorCommandPlugin's own
// kPoseClips table (the `pose` strings here are exactly what it matches on).
//
// This is the one place a new pose gets wired to the keyboard: add a row
// here, a row in the plugin's kPoseClips, and the <animation> entries in the
// model SDF. Everything else -- the held-pose property, the L-key
// registration, the status lines, the QML button -- is written against the
// table rather than against "sit", so none of it needs touching.
struct PoseShortcut
{
  int key;              // Qt::Key_*
  const char *pose;     // published to the actor's pose_topic
  const char *label;    // shown in the panel/status line
};
inline const PoseShortcut kPoseShortcuts[] = {
  {Qt::Key_K, "sit", "着席"},
};
inline constexpr int kPoseShortcutCount =
    static_cast<int>(sizeof(kPoseShortcuts) / sizeof(kPoseShortcuts[0]));

// Label shown for "no pose held" -- i.e. the actor's ordinary standing/
// walking behaviour, which the L key can register just like any other pose.
inline const char *const kNoPoseLabel = "立ち";

inline const PoseShortcut *PoseShortcutForKey(int _key)
{
  for (int i = 0; i < kPoseShortcutCount; ++i)
  {
    if (kPoseShortcuts[i].key == _key)
      return &kPoseShortcuts[i];
  }
  return nullptr;
}

// Movement layout (Q W E / A _ D / Z X C, center vacated -- S and N are
// held modifier / stop keys now, not directions), shared by the on-screen
// pad and the keyboard shortcuts in eventFilter(). Diagonal components are
// scaled by 1/sqrt(2) so diagonal moves aren't faster than straight ones.
// X is the odd one out: rather than a pure backward strafe (which would
// moonwalk the actor away without it ever turning around), it combines
// backward motion with a turn rate so the actor visibly spins to face the
// direction it's retreating toward -- but only when _turnToFace is
// requested (S held); a plain key press is always a pure strafe, including
// X (straight-back strafe, i.e. moonwalking), same as the other 7
// directions.
inline constexpr double kTeleopSpeed = 1.0;
inline constexpr double kTeleopDiagonal = kTeleopSpeed * 0.70710678;
inline constexpr double kTeleopTurnRate = 2.5;

// Jump height (meters) range the QML slider / setJumpHeight() clamp to.
inline constexpr double kJumpHeightMin = 0.05;
inline constexpr double kJumpHeightMax = 1.5;

// Baseline speed-multiplier range setSpeedMultiplier() (the QML slider)
// clamps to.
inline constexpr double kSpeedMultiplierMin = 0.1;
inline constexpr double kSpeedMultiplierMax = 4.0;

// Extra factor EffectiveSpeedMultiplier() applies on top of the baseline
// speedMultiplierState while Ctrl (slow walk) or Shift (run) is held --
// see eventFilter()'s Key_Control/Key_Shift tracking and teleopDirection()/
// ApplyHeldDirectionKeys(), which are the only two places that call it.
inline constexpr double kSlowFactor = 0.5;
inline constexpr double kRunFactor = 2.0;

// Every entry's (linear, lateral) magnitude is kTeleopSpeed by construction
// (cardinal directions put it all on one axis; diagonals split it
// kTeleopDiagonal/kTeleopDiagonal, which is kTeleopSpeed/sqrt(2) per axis,
// i.e. kTeleopSpeed once recombined) -- teleopDirection()'s _turnToFace
// path relies on that to reuse this table for "how far to turn, then walk
// forward at this same speed" instead of a separate direction-angle table.
inline bool DirectionToTwist(
    const std::string &_direction, double &_linear, double &_lateral,
    double &_angular)
{
  _linear = 0.0;
  _lateral = 0.0;
  _angular = 0.0;
  if (_direction == "W") { _linear = kTeleopSpeed; }
  else if (_direction == "A") { _lateral = kTeleopSpeed; }
  else if (_direction == "D") { _lateral = -kTeleopSpeed; }
  else if (_direction == "X") { _linear = -kTeleopSpeed; }
  else if (_direction == "Q") { _linear = kTeleopDiagonal; _lateral = kTeleopDiagonal; }
  else if (_direction == "E") { _linear = kTeleopDiagonal; _lateral = -kTeleopDiagonal; }
  else if (_direction == "Z") { _linear = -kTeleopDiagonal; _lateral = kTeleopDiagonal; }
  else if (_direction == "C") { _linear = -kTeleopDiagonal; _lateral = -kTeleopDiagonal; }
  else { return false; }
  return true;
}

// The 4 cardinal keys eligible for steering-combo tracking (see
// PressDirectionKey()/ApplyHeldDirectionKeys()). Diagonals (Q/E/Z/C) stay
// on the old single-shot immediate path.
inline bool IsSteerableDirection(const std::string &_direction)
{
  return _direction == "W" || _direction == "A" ||
      _direction == "D" || _direction == "X";
}

// atan2(lateral, linear) for a direction's DirectionToTwist() vector --
// same convention teleopDirection()'s turnToFace path already uses for
// headingOffset, reused here so the turn direction sign logic in
// ApplyHeldDirectionKeys() matches it exactly.
inline double DirectionHeadingAngle(const std::string &_direction)
{
  double linear = 0.0, lateral = 0.0, angular = 0.0;
  DirectionToTwist(_direction, linear, lateral, angular);
  return std::atan2(lateral, linear);
}

// Keyboard shortcuts only fire when focus isn't on a text-editable QML item
// (name field, x/y/z/yaw fields, ...) -- otherwise typing "human1" into the
// name box would also drive the active human around.
inline bool IsTextEditFocused()
{
  auto *focusObject = qGuiApp ? qGuiApp->focusObject() : nullptr;
  if (!focusObject)
    return false;
  const QString className = focusObject->metaObject()->className();
  return className.contains("TextInput") || className.contains("TextEdit");
}

// Keep in sync with setViewpoint()'s switch below and the QML ComboBox.
enum ViewIndex
{
  kViewFree = 0,
  kViewFirstPerson,
  kViewBehind,
  kViewFront,
  kViewRight,
  kViewLeft,
  kViewTop,
  kViewFrontRightUp,
  kViewFrontLeftUp,
  kViewCount,
};
inline const char *const kViewpointLabels[kViewCount] = {
  "自由視点", "一人称（本人視点）", "後方追従", "前方から", "右横から", "左横から",
  "俯瞰（真上）", "右奥上から（斜め上）", "左奥上から（斜め上）",
};
// ABSOLUTE eye height above the ground for kViewFirstPerson (roughly
// standing human eye level), unlike guide_robot's robots, so this differs
// from GuiderRobotManager::setViewpoint()'s equivalent value.
//
// This is not simply added to the actor's own world Z as a followOffset:
// spawn_human.launch.py's -z places each model's ENTITY ORIGIN at
// defaultZ(modelIndex) (see kHumanModelDefaultZ), which for walking_actor/
// person_standing/custom_human is 1.0 -- already close to head height, not
// ground level -- because those models' own mesh origin sits partway up the
// body. DoctorFemaleWalk's origin is at 0.0 (ground level) instead. Naively
// adding a flat 1.6 m on top of an origin already at 1.0 m put the first-
// person camera at 2.6 m, floating well above the character's actual head.
// setViewpoint() now subtracts that model's own defaultZ from this before
// using it, so the offset means "how much higher than THIS model's origin
// the eyes are", landing at the same ~1.6 m absolute height for every model
// regardless of where each one's origin happens to sit.
inline constexpr double kEyeHeight = 1.6;

// Floor for the computed offset above -- even if a future model's own
// defaultZ were placed above eye height (making the raw subtraction negative
// or implausibly small), the first-person camera should still sit a
// sensible distance above whatever entity Z it's following.
inline constexpr double kMinEyeOffset = 0.2;

// How eagerly the chase camera (SetFollowTarget/SetTrackTarget's pgain)
// catches up to the human's current world position each frame. The
// background-swings-when-the-body-turns problem this used to be tuned for
// is now fixed properly via ViewCommand::worldFrame instead (see the .hh
// and ApplyViewpoint()) -- the human's own rotation no longer moves the
// camera at all, so this only smooths the camera's translation as the
// human actually walks somewhere. Kept a bit below the original 0.35 for
// a gentle trailing feel without being sluggish to snap onto a
// freshly-selected human.
inline constexpr double kChasePGain = 0.25;

// dladdr anchor: resolves to the shared library this code was loaded
// from, so LoadConfig() can find human_pose_presets.yaml under this
// package's own share/ directory without hardcoding an install prefix.
inline void ThisLibraryAnchor()
{
}
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_HUMANCONTROLPANELINTERNAL_HH_
