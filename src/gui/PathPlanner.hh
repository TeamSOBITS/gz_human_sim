#ifndef GZ_HUMAN_SIM_GUI_PATHPLANNER_HH_
#define GZ_HUMAN_SIM_GUI_PATHPLANNER_HH_

#include <utility>
#include <vector>

#include <gz/transport/Node.hh>

// 障害物を避ける経路計画。NavGridSystem（サーバー側のワールド単位システム）
// へのサービス呼び出しをまとめたもの。
//
// ---------------------------------------------------------------------------
// なぜ HumanControlPanel から分けたのか（構想書 §13 段階4）
// ---------------------------------------------------------------------------
// **このクラスは人物を知りません。** 点列と body 半径を受け取り、点列を返す
// だけです。誰の経路なのかは呼び出し側の関心事。
//
// guide_robot のロボットにも同じ計画が要るはずで、そのときは人物固有の型が
// 混ざっていないこのクラスをそのまま使えます。
//
// ---------------------------------------------------------------------------
// 使えるかどうかの判定
// ---------------------------------------------------------------------------
// NavGridSystem はワールドの SDF に <plugin> が書かれていなければ存在しません。
// SFM はトピックで判定していますが、こちらは**サービス**なので
// ServiceList() を見ます。どちらも「ワールドプラグインが読み込まれて
// いなければ、そもそも提供されていない」という同じ考え方です。
//
// 判定結果は Available() が内部に控え、CachedAvailable() が返します。
// QML の bind 先（Q_PROPERTY の getter）は const なので、そこから毎回
// サービス一覧を引き直さないための作りです（gz-transport の discovery は
// ネットワーク問い合わせ）。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
/// \brief NavGridSystem::OnPlanPath() が待ち受けるサービス名。
inline const char *const kNavPlanService = "/gz_human_sim/nav/plan_path";
inline constexpr unsigned int kNavPlanTimeoutMs = 3000u;

class PathPlanner
{
  /// \brief プランナがこのワールドに居るか問い合わせ、結果を控える。
  public: bool Available(gz::transport::Node &_node);

  /// \brief 最後に Available() が返した値。問い合わせは行わない。
  public: bool CachedAvailable() const;

  /// \brief _route を、半径 _bodyRadius の体が通れる経路に引き直す。
  ///
  /// \param[out] _planned 成功時は計画結果、失敗時は _route のまま。
  ///             **失敗しても _planned は必ず埋まる**ので、呼び出し側は
  ///             戻り値を見ずにそのまま使っても経路を失わない。
  /// \return 計画できたら true。点が 2 個未満、プランナ不在、
  ///         サービス失敗、返答が 2 点未満のいずれでも false。
  public: bool Plan(gz::transport::Node &_node,
              const std::vector<std::pair<double, double>> &_route,
              double _bodyRadius,
              std::vector<std::pair<double, double>> &_planned);

  private: bool available{false};
};
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_PATHPLANNER_HH_
