#include "LaunchProcess.hh"

#include <csignal>
#include <sys/types.h>

#include <QObject>
#include <QPointer>
#include <QProcess>
#include <QTimer>

// 中身は HumanControlPanelSpawn.cc からそのまま移したもので、
// 振る舞いは変えていない。

namespace gz_human_sim
{
namespace launch_process
{
QProcess *Start(QObject *_parent, const QStringList &_arguments)
{
  auto *process = new QProcess(_parent);
  // setsid makes the child (ros2 launch, plus every node it spawns) its
  // own process group, so TerminateGroup() can signal all of them
  // at once on removal instead of leaving orphaned bridge/spawn nodes
  // behind. Same pattern as guide_robot's GuiderRobotManager.
  process->setProgram("setsid");
  process->setArguments(QStringList{"ros2"} + _arguments);
  process->setStandardOutputFile(QProcess::nullDevice());
  process->setStandardErrorFile(QProcess::nullDevice());
  process->start();
  if (!process->waitForStarted(3000))
  {
    process->deleteLater();
    return nullptr;
  }
  return process;
}

void TerminateGroup(QObject *_context, QProcess *_process)
{
  if (!_process)
    return;
  const qint64 pid = _process->processId();
  if (pid > 0)
  {
    ::kill(static_cast<pid_t>(-pid), SIGINT);
    QTimer::singleShot(4000, _context, [pid]()
    {
      if (::kill(static_cast<pid_t>(-pid), 0) == 0)
        ::kill(static_cast<pid_t>(-pid), SIGTERM);
    });
    QTimer::singleShot(8000, _context, [pid]()
    {
      if (::kill(static_cast<pid_t>(-pid), 0) == 0)
        ::kill(static_cast<pid_t>(-pid), SIGKILL);
    });
  }
  QTimer::singleShot(9000, _context, [guard = QPointer<QProcess>(_process)]()
  {
    if (guard)
      guard->deleteLater();
  });
}
}  // namespace launch_process
}  // namespace gz_human_sim
