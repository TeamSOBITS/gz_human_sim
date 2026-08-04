#ifndef GZ_HUMAN_SIM_GUI_LAUNCHPROCESS_HH_
#define GZ_HUMAN_SIM_GUI_LAUNCHPROCESS_HH_

#include <QStringList>

class QObject;
class QProcess;

// `ros2 launch` を子プロセスとして起動し、後片付けする。
//
// ---------------------------------------------------------------------------
// なぜ HumanControlPanel から分けたのか（構想書 §13 段階4）
// ---------------------------------------------------------------------------
// **人物とは何の関係もありません。** guide_robot も同じことをしています
// （こちらの実装がそもそも GuiderRobotManager 由来）。共通の GUI 基盤
// パッケージ（§12）へ持っていける形にしてあります。
//
// 状態を持たないので、クラスではなく自由関数（PathTemplates と同じ判断）。
//
// ---------------------------------------------------------------------------
// なぜ setsid して、プロセス「グループ」を殺すのか
// ---------------------------------------------------------------------------
// `ros2 launch` は自分の下にノードを何個も起こします。親だけ殺すと、
// bridge や spawn のノードが**孤児として残り続けます。** setsid で子を
// 独立したプロセスグループにしておけば、グループごとまとめて signal を
// 送れます。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
namespace launch_process
{
/// \brief `setsid ros2 <_arguments...>` を起動する。
///
/// 標準出力・標準エラーは捨てる（Gazebo のログに ros2 launch の出力が
/// 混ざると読めなくなるため）。
///
/// \param[in] _parent QProcess の親。ふつうはパネル自身。
/// \return 3 秒以内に起動しなければ nullptr。
QProcess *Start(QObject *_parent, const QStringList &_arguments);

/// \brief _process のプロセスグループを段階的に終了させる。
///
/// SIGINT → 4 秒後にまだ生きていれば SIGTERM → 8 秒後に SIGKILL、
/// 9 秒後に QProcess を deleteLater()。いきなり SIGKILL しないのは、
/// ノードに後始末（トピックの unadvertise など）をさせるため。
///
/// \param[in] _context タイマーの寿命を縛る QObject。これが先に死んだら
///            残りの段階は実行されない。
void TerminateGroup(QObject *_context, QProcess *_process);
}  // namespace launch_process
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_LAUNCHPROCESS_HH_
