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

// roster（PublishRoster()）が「この人物が出してよい速度」として広告する値。
// 実際に速度指令を出すのは unified_entity_control 側で、その上限として
// これを読む。かつてこのパネル自身のテレオペ速度だったもの。
inline constexpr double kAdvertisedMaxLinear = 1.0;
inline constexpr double kAdvertisedMaxAngular = 2.5;

// 名前付きポーズの表示名。ActorCommandPlugin の kPoseClips と対になる
// （`pose` の文字列はあちらが照合するものと同じ）。
//
// ポーズを操作する手段はこのパッケージから外へ出た（構想書 §11）ので、
// ここに残っているのは「サーバーが state で返してきたポーズ名を、画面に
// 出す日本語へ直す」ためだけ。キー割当は持たない。
struct PoseShortcut
{
  const char *pose;     // actor の pose_topic に流れる名前
  const char *label;    // パネルの表示名
};
inline const PoseShortcut kPoseShortcuts[] = {
  {"sit", "着席"},
};
inline constexpr int kPoseShortcutCount =
    static_cast<int>(sizeof(kPoseShortcuts) / sizeof(kPoseShortcuts[0]));

// Label shown for "no pose held" -- i.e. the actor's ordinary standing/
// walking behaviour, which the L key can register just like any other pose.
inline const char *const kNoPoseLabel = "立ち";

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

// dladdr anchor: resolves to the shared library this code was loaded
// from, so LoadConfig() can find human_pose_presets.yaml under this
// package's own share/ directory without hardcoding an install prefix.
inline void ThisLibraryAnchor()
{
}
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_HUMANCONTROLPANELINTERNAL_HH_
