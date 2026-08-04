#ifndef GZ_HUMAN_SIM_GUI_WORLDENTITYSERVICE_HH_
#define GZ_HUMAN_SIM_GUI_WORLDENTITYSERVICE_HH_

#include <string>

#include <gz/transport/Node.hh>

// gz-sim のワールドサービス（/world/<w>/create, /remove, /state）の呼び出し。
//
// ---------------------------------------------------------------------------
// なぜ HumanControlPanel から分けたのか（構想書 §13 段階4）
// ---------------------------------------------------------------------------
// **人物とは何の関係もありません。** ワールド名とエンティティ名で話が
// 完結します。guide_robot も同じサービスを叩いているので、共通の GUI 基盤
// パッケージ（§12）へそのまま持っていける形にしてあります。
//
// **状態を持たないので、クラスではなく自由関数にしています。**
// 空のクラスを作ると「状態がありそう」という嘘の見た目になるため
// （PathTemplates と同じ判断）。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
namespace world_entity
{
/// \brief /world/<_world>/state の応答待ち上限。
///
/// 短いのは意図的で、この問い合わせはスポーン確認のポーリングから
/// 繰り返し呼ばれる。長く待つと GUI が固まる。
inline constexpr unsigned int kStateQueryTimeoutMs = 800u;

/// \brief MODEL 型エンティティ _name の削除を要求する（投げっぱなし）。
///
/// \note **ACTOR には効きません。** gz-sim の UserCommands は
/// 「Entity [n] is not a model or a light, so it can't be removed.」で
/// 弾きます。Entity::Type に何を入れても同じです。アクター由来の人物は
/// それぞれの remove トピック経由で消してください。
void Remove(gz::transport::Node &_node, const std::string &_world,
    const std::string &_name);

/// \brief _sdf のエンティティを (_x, _y, _z) に作る（投げっぱなし）。
void Create(gz::transport::Node &_node, const std::string &_world,
    const std::string &_sdf, const std::string &_name,
    double _x, double _y, double _z);

/// \brief _name のエンティティがワールドに居るか。
///
/// /world/<w>/scene/info や `gz model` CLI は **MODEL しか列挙しません。**
/// このパネルが spawn する human_model は ECS 上アクターなので、そこには
/// 一切出てきません（実測: 生きていて操作もできるアクターに対して
/// `gz model -m <name> -p` が "No model named <name> was found" を返す）。
///
/// アクターも含まれるのは /world/<w>/state だけ。その応答は ECS の
/// コンポーネント集合を serialize した生バイト列なので、gz-sim の
/// コンポーネント型 ID を手で辿る代わりに、Name コンポーネントの文字列を
/// 部分一致で探しています。存在確認にはこれで十分です — 本当に削除された
/// 後は、その名前に言及するコンポーネント（Name、プラグインの
/// vel_topic / path_topic 文字列など）が ECS から丸ごと消えるためです。
bool Exists(gz::transport::Node &_node, const std::string &_world,
    const std::string &_name);
}  // namespace world_entity
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_WORLDENTITYSERVICE_HH_
