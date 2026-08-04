#ifndef GZ_HUMAN_SIM_GUI_PATHTEMPLATES_HH_
#define GZ_HUMAN_SIM_GUI_PATHTEMPLATES_HH_

#include <utility>
#include <vector>

// 定型の巡回経路（円・四角）の点列を作る。
//
// ---------------------------------------------------------------------------
// なぜ分けたのか（構想書 §13 段階4）
// ---------------------------------------------------------------------------
// **純粋な幾何計算**で、人物にもトピックにもシーンにも依存しません。
// クラスにする理由が無いので、状態を持たない自由関数のままにしてあります
// （空のクラスを作ると「状態がありそう」という嘘の見た目になる）。
//
// ---------------------------------------------------------------------------
// scripts/path_template.py との関係
// ---------------------------------------------------------------------------
// 同じ計算が Python 側（headless / CLI 用）の _circle_waypoints() /
// _square_waypoints() にもあります。**手で同期させています。**
// 片方だけ直すと、GUI ボタンと CLI で違う経路が出ます。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
/// \brief 経路テンプレートの種類。QML の ComboBox の並び順、
/// kPathTemplateLabels、TemplateWaypoints() の分岐が同じ順序であること。
enum PathTemplateIndex
{
  kPathTemplateCircle = 0,
  kPathTemplateSquare,
  kPathTemplateCount,
};

inline const char *const kPathTemplateLabels[kPathTemplateCount] = {"円", "四角"};

/// \brief (_centerX, _centerY) を中心とする定型経路の点列を返す。
///
/// \param[in] _templateIndex 範囲外なら空を返す。
/// \param[in] _size 円なら半径、四角なら一辺。0.1 未満は 0.1 に切り上げ。
/// \param[in] _numWaypoints 円のみ有効。3〜200 に収める。四角は常に 4 点。
/// \param[in] _clockwise 進行方向。
///
/// \note 向き（yaw）は返しません。呼び出し側（confirmRoute()）が点の順序
/// から derive し直すためです。
std::vector<std::pair<double, double>> TemplateWaypoints(
    int _templateIndex, double _centerX, double _centerY,
    double _size, int _numWaypoints, bool _clockwise);
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_PATHTEMPLATES_HH_
