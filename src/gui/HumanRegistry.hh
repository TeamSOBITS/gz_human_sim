#ifndef GZ_HUMAN_SIM_GUI_HUMANREGISTRY_HH_
#define GZ_HUMAN_SIM_GUI_HUMANREGISTRY_HH_

// ============================================================================
// HumanRegistry -- このパネルが把握している人物の一覧
// ============================================================================
//
// 【このファイルがやること】
//   スポーン済みの人物を 1 か所で持つ。誰がいるか、その人物の通信チャンネルは
//   何か、という「同一性」の管理だけを担当する。
//
// 【このファイルがやらないこと】
//   人物に対して何かを「する」こと。スポーン、視点、マーカー、当たり判定、
//   経路、状態表示 -- いずれも HumanControlPanel* の各ファイルの仕事で、
//   ここはその置き場を貸しているだけ。
//
// 【なぜ切り出したか】
//   struct Human は HumanControlPanel.hh の中で 114 行あり、パネルの全責務が
//   直接触っていた。テレオペが外へ出た（構想書 §11）ことで残ったメンバが
//   責務ごとにきれいに分かれたので、まず入れ物を独立させる。
//   構想書 §13 の段階 2 にあたる。
//
// 【いまはまだ vector の薄い包み】
//   既存の呼び出し（this->humans.at(i) など 91 箇所）をそのまま通すため、
//   at() / size() / begin() / end() を vector と同じ名前で出している。
//   **これは移行のための足場であって、到達点ではない。**
//   構想書 §13 の段階 3（index → 安定 ID）と段階 4（責務ごとのクラス分離）で、
//   ここは名前で引く API へ寄せていく。
//
// 【添字で指すことの危険】
//   人物を削除すると添字がずれる。NPC が自律的にスポーン／デスポーンし始めると
//   事故になる（削除中に別の人物へ指令が飛ぶ）。IndexOf() を用意してあるので、
//   新しく書くコードは名前で引くこと。構想書 §6 を参照。
// ============================================================================

#include <string>
#include <vector>

#include <QProcess>
#include <QString>

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
  /// rief 視点。担当は HumanControlPanelCamera.cc。
  ///
  /// 「対象」を切り替えたとき、視点コンボがその人物の実際の設定を表示できる
  /// よう人物ごとに覚えておく（毎回「自由視点」へ戻らないように）。
  struct CameraState
  {
    int index{0};           ///< 0 = 自由視点／未設定
    double distance{2.0};
  };
  CameraState camera;

  /// rief 当たり判定カプセルの表示。担当は HumanControlPanelCollision.cc。
  ///
  /// applied が bool ではなく tri-state の int なのは、再スポーンした
  /// カプセルへ「いまと同じ値」でも必ず一度押し込むため（-1 = 未適用）。
  /// 既定は非表示。デバッグ用の補助であって、通常は視界の邪魔になる。
  struct CollisionState
  {
    bool show{false};
    int applied{-1};        ///< -1 = まだ一度も適用していない
    int retries{0};
    double radius{0.25};    ///< models/human_collision_body/model.sdf と一致
    double length{1.2};
  };
  CollisionState collision;

  /// rief 経路と群衆制御。担当は HumanControlPanelRoute.cc。
  ///
  /// 経路そのものと useSfm/cyclicRoute はパネル全体で 1 つ。人物ごとに残るのは
  /// 「この人物が経路の対象か」と、最後に publish した sfm の有効/無効。
  /// 後者を持っておくのは、対象を切り替えたときに往復通信なしで
  /// トグルの状態を表示するため。
  struct RouteState
  {
    bool isTarget{true};
    bool sfmEnabled{true};
    gz::transport::Node::Publisher sfmEnablePublisher;
  };
  RouteState route;

  /// rief スポーン地点のマーカー。担当は HumanControlPanelOverlay.cc。
  ///
  /// visual はレンダースレッドで遅延生成し、ここが所有する。removeHuman() が
  /// 「この人物のぶんだけ」破棄を予約できるようにするため。
  /// applied が tri-state なのは CollisionState と同じ理由。既定は表示。
  struct MarkerState
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    int colorIndex{0};
    bool show{true};
    int applied{-1};
    gz::rendering::VisualPtr visual;
  };
  MarkerState marker;

  /// rief サーバーが state_topic へ流してくる状態。
  ///        担当は HumanControlPanelState.cc。
  ///
  /// 構想書 §3 のとおり真実はサーバー側にあり、ここはその写し。
  /// **このパネルが指令から推測した値ではない。**
  /// 購読は transport スレッドで走るので、読み書きは stateMutex の下で行うこと。
  struct ServerState
  {
    CharacterState state{CharacterState::Unknown};
    std::string pose;       ///< 保持中の名前付きポーズ。無ければ空
    double speed{0.0};      ///< 実測の並進速度 [m/s]。指令値ではない
    bool received{false};   ///< 一度でも受信したか（未受信と Unknown の区別）
    std::string topic;      ///< 削除時に Unsubscribe するため保持
  };
  ServerState server;
};

/// \brief 人物の一覧。いまは std::vector の薄い包みで、既存の添字アクセスを
/// そのまま通す（上のコメント参照）。
class HumanRegistry
{
  public: using iterator = std::vector<Human>::iterator;
  public: using const_iterator = std::vector<Human>::const_iterator;

  public: std::size_t size() const { return this->entries.size(); }
  public: bool empty() const { return this->entries.empty(); }
  public: Human &at(std::size_t _i) { return this->entries.at(_i); }
  public: const Human &at(std::size_t _i) const { return this->entries.at(_i); }
  public: iterator begin() { return this->entries.begin(); }
  public: iterator end() { return this->entries.end(); }
  public: const_iterator begin() const { return this->entries.begin(); }
  public: const_iterator end() const { return this->entries.end(); }
  public: void push_back(Human &&_human)
  { this->entries.push_back(std::move(_human)); }
  public: void erase(std::size_t _i)
  { this->entries.erase(this->entries.begin() + static_cast<long>(_i)); }

  /// \brief 添字が範囲内か。呼び出し側で毎回書いていた判定をここへ寄せる。
  public: bool valid(int _index) const
  { return _index >= 0 && _index < static_cast<int>(this->entries.size()); }

  /// \brief 名前から添字を引く。見つからなければ -1。
  ///        添字で指すのをやめていくための入口（構想書 §6・§13 段階 3）。
  public: int IndexOf(const std::string &_name) const
  {
    for (std::size_t i = 0; i < this->entries.size(); ++i)
    {
      if (this->entries[i].name == _name)
        return static_cast<int>(i);
    }
    return -1;
  }

  /// \brief 名前で引く。見つからなければ nullptr。
  public: Human *Find(const std::string &_name)
  {
    const int index = this->IndexOf(_name);
    return (index < 0) ? nullptr : &this->entries[static_cast<std::size_t>(index)];
  }

  private: std::vector<Human> entries;
};
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_HUMANREGISTRY_HH_
