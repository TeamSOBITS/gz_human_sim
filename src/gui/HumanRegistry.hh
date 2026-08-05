#ifndef GZ_HUMAN_SIM_HUMANREGISTRY_HH_
#define GZ_HUMAN_SIM_HUMANREGISTRY_HH_

// ============================================================================
// HumanRegistry -- このパネルが把握している人物の一覧
// ============================================================================
//
// 【このファイルがやること】
//   spawn 済みの人物を1か所で持つ。**誰がいるか**と**その人物の通信チャンネル
//   は何か**という「同一性」の管理だけ。
//
// 【このファイルがやらないこと】
//   人物に対して何かを「する」こと。spawn・視点・マーカー・当たり判定・経路・
//   状態表示はいずれも HumanControlPanel*.cc の仕事で、ここは置き場を貸すだけ。
//
// 【いまはまだ vector の薄い包み】
//   既存の呼び出し（`this->humans.at(i)` など多数）をそのまま通すため、
//   at() / size() / empty() / begin() / end() を vector と同じ名前で出している。
//   **これは移行のための足場であって到達点ではない**（構想書 §6・§13 段階4）。
//
// 【添字で指すことの危険】
//   人物を削除すると添字がずれる。NPC が自律的に spawn/despawn し始めると
//   事故になる（削除中に別人へ指令が飛ぶ）。IndexOf()/Find() があるので、
//   **新しく書くコードは名前で引くこと。** 非同期に届くコールバック
//   （サーバーからの state 通知など）は特に、添字を握って待ってはいけない。
// ============================================================================

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include <QProcess>
#include <QString>

#include <gz/math/Vector3.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/transport/Node.hh>

#include "gz_human_sim/CharacterState.hh"

namespace gz_human_sim
{
  struct Human
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
    // サーバーが publish している「いま何をしているか」（構想書 §3・§13 段階3）。
    // **これはサーバーからの受信値で、GUI が推測したものではない。**
    // 下の lockedPose などは GUI 側が「自分が送った指令」から持っている
    // 従来の状態で、まだ残っている -- 消すのは着席機能/NPC がこちらへ
    // 乗り換えてから（それまで二重に見えるが、優劣は明確: こちらが真）。
    CharacterState serverState{CharacterState::Unknown};
    std::string serverPose;
    double serverSpeed{0.0};
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

    // Crowd/SFM feature set -- see the Q_PROPERTY block's comment at the
    // top of this class. The route itself and the useSfm/cyclicRoute
    // settings are global now (one route applied to every ticked target);
    // what stays per-human is whether this human is one of those targets,
    // and sfmEnabled, which mirrors whatever this panel last published on
    // sfmEnablePublisher -- kept here (rather than re-deriving it) purely
    // so ActiveSfmEnabled() can show the right toggle state without an
    // extra round trip.
    bool routeTarget{true};
    bool sfmEnabled{true};
    gz::transport::Node::Publisher sfmEnablePublisher;

    // Spawn marker (see setShowSpawnMarker()/ApplySpawnMarkers()): where
    // this human was originally spawned, the colour its marker is drawn in,
    // and the same "desired vs. applied" pair ApplyCollisionVisibility()
    // uses -- appliedShowSpawnMarker is a tri-state int (-1 = nothing
    // applied yet) for the same reason appliedShowCollision is.
    //
    // markerVisual is the render-scene visual itself, created lazily on the
    // render thread and owned here so removeHuman() can queue exactly that
    // one for destruction. Shown by default: the whole point of the marker
    // is to see where people started without having to ask for it.
    double spawnX{0.0};
    double spawnY{0.0};
    double spawnZ{0.0};
    int markerColorIndex{0};
    bool showSpawnMarker{true};
    int appliedShowSpawnMarker{-1};
    gz::rendering::VisualPtr markerVisual;
  };

/// \brief 人物の一覧。いまは std::vector の薄い包み（上のコメント参照）。
class HumanRegistry
{
  /// \name vector 互換の入口
  /// 既存の呼び出しをそのまま通すためのもの。**新しいコードでは
  /// 名前で引く Find()/IndexOf() を使うこと。**
  /// @{
  public: std::size_t size() const { return this->items.size(); }
  public: bool empty() const { return this->items.empty(); }
  public: Human &at(std::size_t _i) { return this->items.at(_i); }
  public: const Human &at(std::size_t _i) const { return this->items.at(_i); }
  public: Human &operator[](std::size_t _i) { return this->items[_i]; }
  public: const Human &operator[](std::size_t _i) const { return this->items[_i]; }
  public: auto begin() { return this->items.begin(); }
  public: auto end() { return this->items.end(); }
  public: auto begin() const { return this->items.begin(); }
  public: auto end() const { return this->items.end(); }
  public: Human &back() { return this->items.back(); }
  public: void push_back(Human _human) { this->items.push_back(std::move(_human)); }
  public: template <typename It> auto erase(It _it) { return this->items.erase(_it); }
  public: void clear() { this->items.clear(); }
  /// @}

  /// \brief 名前から添字。居なければ -1。
  ///
  /// 添字を**保持しない**こと。引いたらその場で使い、次に必要になったら
  /// また引く。保持している間に削除が入ると別人を指す。
  public: int IndexOf(const std::string &_name) const
  {
    const auto found = std::find_if(this->items.begin(), this->items.end(),
        [&_name](const Human &_human) { return _human.name == _name; });
    return found == this->items.end()
        ? -1 : static_cast<int>(found - this->items.begin());
  }

  /// \brief 名前から人物。居なければ nullptr。
  public: Human *Find(const std::string &_name)
  {
    const int index = this->IndexOf(_name);
    return index < 0 ? nullptr : &this->items[static_cast<std::size_t>(index)];
  }
  public: const Human *Find(const std::string &_name) const
  {
    const int index = this->IndexOf(_name);
    return index < 0 ? nullptr : &this->items[static_cast<std::size_t>(index)];
  }

  /// \brief 添字が現在の一覧の範囲内か。
  public: bool Valid(int _index) const
  {
    return _index >= 0 && _index < static_cast<int>(this->items.size());
  }

  private: std::vector<Human> items;
};
}  // namespace gz_human_sim

#endif  // GZ_HUMAN_SIM_HUMANREGISTRY_HH_
