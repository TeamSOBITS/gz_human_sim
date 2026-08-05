#ifndef GZ_HUMAN_SIM_CHARACTERSTATE_HH_
#define GZ_HUMAN_SIM_CHARACTERSTATE_HH_

// ============================================================================
// CharacterState -- 人物がいま何をしているか
// ============================================================================
//
// 【このファイルがやること】
//   サーバー側（gz-sim の System プラグイン）と、それを見る側（GUI パネル、
//   将来の NPC の行動決定）が **同じ定義** を使うための共有ヘッダ。
//   状態そのものの enum と、トピック名の決め方だけを持つ。
//
// 【なぜ共有ヘッダなのか】
//   状態はサーバー側が持ち、外へ publish する（構想書 §3）。これまで GUI は
//   「自分が送った指令」から状態を **推測** していた。テレオペだけなら成立するが、
//   NPC・自動行動・着席状態管理を足した瞬間に破綻する。GUI を閉じても NPC は
//   動き続けなければならないので、真実がクライアント側にあってはいけない。
//
//   文字列を両側で手打ちすると、構想書 §0 の roster と同じ壊れ方（片側だけ直して
//   無言で不一致）をする。なのでここに 1 か所だけ定義を置く。
//
// 【このファイルがやらないこと】
//   - 状態遷移を決めること      -> サーバー側（ActorCommandPlugin）
//   - 状態を表示すること        -> GUI（HumanControlPanel）
//   Gazebo にも Qt にも依存しない。両側から include できるようにするため。
//
// 【メッセージ形式】
//   トピック: <名前空間>/state        型: gz.msgs.Param
//
//   `gz.msgs.Param` の params は string -> Any の map なので、**キーを足しても
//   既存の読み手が壊れない**。roster（構想書 §0）が '|' 区切りの固定 18 フィールドで、
//   数が合わないと無言で降格して位置情報が消えるのとは対照的。状態に項目が
//   増えていくのは目に見えているので、こちらは map を選ぶ。
//
//   現在のキー:
//     state     文字列。下の kStateNames のいずれか
//     pose      文字列。保持中の名前付きポーズ（無ければ空）
//     speed     double。いまの並進速度 [m/s]
//
//   将来足す予定のキー（着席機能、構想書 §5）:
//     seat_id   文字列。座っている椅子のエンティティ名
// ============================================================================

#include <cstddef>
#include <cstring>
#include <string>

namespace gz_human_sim
{
/// \brief 人物の状態。ひとつだけ成立する（bool の組み合わせにしない）。
///
/// bool を並べると「座っているのに歩いている」「立ち上がり中なのにまた座る」
/// といった矛盾が表現できてしまい、どちらが真かを呼び出し側が決める羽目になる。
enum class CharacterState
{
  /// \brief まだ状態を受け取っていない。GUI 側の初期値。
  Unknown = 0,
  /// \brief 立ったまま静止。
  Standing,
  /// \brief 速度指令で移動中。
  Moving,
  /// \brief 経路（cmd_path）を追従中。
  Following,
  /// \brief ジャンプの弧を描いている最中。
  Jumping,
  /// \brief 名前付きポーズへ移行中（例: sit_down を再生中）。
  PoseEntering,
  /// \brief 名前付きポーズを保持中（例: sitting）。
  PoseHolding,
  /// \brief 名前付きポーズから復帰中（例: stand_up を再生中）。
  PoseExiting,
  /// \brief 削除要求を受け取り、消える直前。
  Removing,
  /// \brief 番兵。状態として使わないこと。
  Count,
};

/// \brief publish される文字列。**enum の並びと 1 対 1 で対応させること。**
/// 増やすときは enum と両方に足す（下の static_assert が守っている）。
inline const char *const kStateNames[] = {
  "Unknown",
  "Standing",
  "Moving",
  "Following",
  "Jumping",
  "PoseEntering",
  "PoseHolding",
  "PoseExiting",
  "Removing",
};
static_assert(
    sizeof(kStateNames) / sizeof(kStateNames[0]) ==
        static_cast<std::size_t>(CharacterState::Count),
    "kStateNames と CharacterState::Count がずれている");

/// \brief 状態 -> 文字列。範囲外は "Unknown"。
inline const char *ToString(CharacterState _state)
{
  const auto index = static_cast<std::size_t>(_state);
  if (index >= static_cast<std::size_t>(CharacterState::Count))
    return kStateNames[0];
  return kStateNames[index];
}

/// \brief 文字列 -> 状態。知らない文字列は Unknown。
///
/// **知らない文字列で落ちたり例外を投げたりしないこと。** publish 側が先に
/// 更新されて、まだ知らない状態を送ってくる状況は普通に起きる。
inline CharacterState StateFromString(const char *_name)
{
  if (_name == nullptr)
    return CharacterState::Unknown;
  for (std::size_t i = 0;
       i < static_cast<std::size_t>(CharacterState::Count); ++i)
  {
    if (std::strcmp(kStateNames[i], _name) == 0)
      return static_cast<CharacterState>(i);
  }
  return CharacterState::Unknown;
}

/// \brief 状態トピックの既定のサフィックス。
///
/// 実際のトピックは「人物名を名前空間にしたもの」＋これ、つまり
/// `/<人物名>/state`。cmd_vel などと同じ組み立て方で、`spawn_human.launch.py`
/// が SDF に流し込む。ActorCommandPlugin は SDF の `<state_topic>` で
/// 上書きできる。
inline const char *const kStateTopicSuffix = "/state";

/// \brief 人物名から状態トピックを組み立てる。両側で同じ式を使うため。
inline std::string StateTopicFor(const std::string &_humanName)
{
  return "/" + _humanName + kStateTopicSuffix;
}

/// \brief 状態を publish する間隔の上限 [ms]。
///
/// 変化した瞬間にも送るが、**変化が無くても最低これだけの間隔で送る**。
/// GUI が後から起動しても現在値を掴めるようにするため（roster が
/// 2秒ごとに出ているのと同じ理由）。
inline constexpr int kStateHeartbeatMs = 500;
}  // namespace gz_human_sim

#endif  // GZ_HUMAN_SIM_CHARACTERSTATE_HH_
