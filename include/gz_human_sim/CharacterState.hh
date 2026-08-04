#ifndef GZ_HUMAN_SIM_CHARACTERSTATE_HH_
#define GZ_HUMAN_SIM_CHARACTERSTATE_HH_

// ============================================================================
// CharacterState -- 人物がいま何をしているか
// ============================================================================
//
// 【このファイルがやること】
//   サーバー側（gz-sim の System プラグイン）と、それを見る側（GUI パネル、
//   将来の unified_entity_control、NPC の行動決定）が **同じ定義** を使うための
//   共有ヘッダ。状態そのものの enum と、トピック名の決め方だけを持つ。
//
// 【なぜ共有ヘッダなのか】
//   状態はサーバー側が持ち、外へ publish します（構想書 §3）。
//   これまで GUI は「自分が送った指令」から状態を推測していました。
//   テレオペが別パッケージへ移ると（構想書 §11）、その推測は成り立ちません。
//   外のパッケージは GUI パネルの内部変数を見られないからです。
//
//   文字列を両側で手打ちすると、§0 の roster と同じ壊れ方（片側だけ直して
//   無言で不一致）をします。なのでここに 1 か所だけ定義を置きます。
//
// 【このファイルがやらないこと】
//   - 状態遷移を決めること      -> サーバー側（ActorCommandPlugin）
//   - 状態を表示すること        -> GUI
//   Gazebo にも Qt にも依存しません。両側から include できるようにするためです。
//
// 【メッセージ形式】
//   トピック: <namespace>/state       型: gz.msgs.Param
//
//   gz.msgs.Param の params は string -> Any の map なので、キーを足しても
//   既存の読み手が壊れません。roster（§0）が '|' 区切りの固定 14 フィールドで、
//   数が合わないと無言で対象が消えるのとは対照的です。
//   状態に項目が増えていくのは目に見えているので、こちらは map を選びます。
//
//   現在のキー:
//     state     文字列。下の kStateNames のいずれか
//     pose      文字列。保持中の名前付きポーズ（無ければ空）
//     speed     double。いまの並進速度 [m/s]
//
//   将来足す予定のキー（着席機能、構想書 §5）:
//     seat_id   文字列。座っている椅子のエンティティ名
// ============================================================================

#include <cstring>

namespace gz_human_sim
{
/// \brief 人物の状態。ひとつだけ成立する（bool の組み合わせにしない）。
///
/// bool を並べると「座っているのに歩いている」「立ち上がり中なのにまた座る」
/// といった矛盾が表現できてしまいます。構想書 §7（有限状態機械）を参照。
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
/// 増やすときは enum と両方に足す（下の static_assert が守っています）。
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
/// 知らない文字列で落ちたり例外を投げたりしないこと。publish 側が先に
/// 更新されて新しい状態を送ってくる状況は普通に起きます。
inline CharacterState StateFromString(const char *_name)
{
  if (_name == nullptr)
    return CharacterState::Unknown;
  for (std::size_t i = 0; i < static_cast<std::size_t>(CharacterState::Count); ++i)
  {
    if (std::strcmp(kStateNames[i], _name) == 0)
      return static_cast<CharacterState>(i);
  }
  return CharacterState::Unknown;
}

/// \brief 状態トピックの既定名。ActorCommandPlugin の SDF <state_topic> と
///        spawn_human.launch.py が namespace を前置します。
inline const char *const kDefaultStateTopic = "/state";

/// \brief gz.msgs.Param に入れるキー。両側で手打ちしないための定数。
inline const char *const kStateKeyState = "state";
inline const char *const kStateKeyPose  = "pose";
inline const char *const kStateKeySpeed = "speed";
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_CHARACTERSTATE_HH_
