// HumanControlPanelState.cc -- 人物の状態の読み書き。修飾キーの保持状態、姿勢の
// ロック/解除、follow mode。**真実はいずれサーバー側へ移る**（構想書 §3）。
//
// 3808行あった単一の HumanControlPanel.cc を責務ごとに割ったもの
// （構想書/これからやるやつ/gz_human_sim再設計構想.md §13 段階2）。
// **中身は移動しただけで、振る舞いは変えていない。**
// クラス宣言は HumanControlPanel.hh、ファイル間で共有する表と定数は
// HumanControlPanelInternal.hh にある。

#include "HumanControlPanel.hh"
#include "HumanControlPanelInternal.hh"

// include は分割前の単一ファイルと同じものを揃えてある。責務ごとに
// 削るのは「動くこと」を確認してからでよい（段階2は移動だけ）。
#include "GuiderDeleteRecoverEvent.hh"
#include "GuiderViewpointRequestEvent.hh"

#include <algorithm>
#include <map>
#include <array>
#include <cmath>
#include <csignal>
#include <dlfcn.h>
#include <fstream>
#include <functional>
#include <memory>
#include <regex>
#include <sstream>
#include <utility>
#include <vector>

#include <QGuiApplication>
#include <QKeyEvent>
#include <QPointer>
#include <QTimer>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/twist.pb.h>
// gz/plugin/Register.hh はここでは include しない。**翻訳単位ごとに
// GzPluginHook を定義する**ので、複数の .cc から include すると
// リンクが multiple definition で落ちる。GZ_ADD_PLUGIN を持つ
// HumanControlPanel.cc の1本だけが include する。
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

// DualSense/gamepad polling only -- SDL_INIT_GAMECONTROLLER (never
// SDL_INIT_VIDEO), so this never touches windowing/GL and cannot conflict
// with the already-running Ogre2/Qt scene. See PollDualsense(). Mirrors
// guide_robot's GuiderRobotManager, which this mode is modeled on.
#include <SDL2/SDL.h>

namespace gz_human_sim
{
bool HumanControlPanel::ShiftHeld() const
{
  return this->shiftHeldState;
}

bool HumanControlPanel::CtrlHeld() const
{
  return this->ctrlHeldState;
}

bool HumanControlPanel::SHeld() const
{
  return this->sHeldState;
}

bool HumanControlPanel::JHeld() const
{
  return this->jHeldState;
}

QString HumanControlPanel::HeldPose() const
{
  return QString::fromStdString(this->heldPoseState);
}

QString HumanControlPanel::ActiveLockedPose() const
{
  if (this->activeHumanIndex < 0 ||
      this->activeHumanIndex >= static_cast<int>(this->humans.size()))
    return {};
  return QString::fromStdString(this->humans.at(this->activeHumanIndex).lockedPose);
}

QString HumanControlPanel::poseLabel(const QString &_pose) const
{
  const std::string pose = _pose.toStdString();
  for (int i = 0; i < kPoseShortcutCount; ++i)
  {
    if (pose == kPoseShortcuts[i].pose)
      return kPoseShortcuts[i].label;
  }
  return kNoPoseLabel;
}

int HumanControlPanel::ActiveFollowModeIndex() const
{
  if (this->activeHumanIndex < 0 ||
      this->activeHumanIndex >= static_cast<int>(this->humans.size()))
    return 0;
  return this->humans.at(this->activeHumanIndex).followModeIndex;
}

void HumanControlPanel::setFollowMode(int _index, int _followModeIndex)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  if (_followModeIndex < 0 || _followModeIndex >= kFollowModeCount)
    return;
  auto &human = this->humans.at(_index);
  if (!human.followModePublisher.Valid())
    return;

  gz::msgs::StringMsg message;
  message.set_data(kFollowModeValues[_followModeIndex]);
  human.followModePublisher.Publish(message);

  human.followModeIndex = _followModeIndex;
  if (_index == this->activeHumanIndex)
    this->activeFollowModeChanged();
  this->SetStatus(QString::fromStdString(human.name) + " の動作モードを「" +
      this->FollowModeLabels().value(_followModeIndex) + "」に変更しました");
}

void HumanControlPanel::togglePoseLock(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.posePublisher.Valid())
    return;

  if (!human.lockedPose.empty())
  {
    // Already registered -- unregister, dropping back to whatever a held key
    // is asking for right now (nothing, usually, so the human stands up).
    const QString previous = this->poseLabel(
        QString::fromStdString(human.lockedPose));
    human.lockedPose.clear();
    this->SetStatus(QString::fromStdString(human.name) + " の姿勢登録（" +
        previous + "）を解除しました");
  }
  else
  {
    // Register whatever pose is happening right now. Registering the
    // no-pose (standing) state is allowed and meaningful: it pins the human
    // upright, so a pose key held afterwards is ignored until it's
    // unregistered.
    human.lockedPose = this->EffectivePose(_index);
    this->SetStatus(QString::fromStdString(human.name) + " の現在の姿勢（" +
        this->poseLabel(QString::fromStdString(human.lockedPose)) +
        "）を登録しました");
  }
  if (_index == this->activeHumanIndex)
    this->activeLockedPoseChanged();
  this->UpdatePoseIntent(_index);
}

std::string HumanControlPanel::EffectivePose(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return {};
  const auto &human = this->humans.at(_index);
  // Registered wins over held: once a pose is registered with L, this human
  // holds it regardless of what key the operator is leaning on -- including
  // a registered "standing", which is how you deliberately stop a human from
  // responding to K at all.
  if (!human.lockedPose.empty())
    return human.lockedPose;
  // A held key only ever steers the ACTIVE human, same as every other
  // keyboard shortcut in this panel.
  if (_index == this->activeHumanIndex)
    return this->heldPoseState;
  return {};
}

void HumanControlPanel::UpdatePoseIntent(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.posePublisher.Valid())
    return;

  gz::msgs::StringMsg message;
  message.set_data(this->EffectivePose(_index));
  human.posePublisher.Publish(message);
}

bool HumanControlPanel::isHumanActorAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(_index).velocityPublisher.Valid();
}

/// \brief サーバーからの状態通知（構想書 §3・§13 段階3）。
///
/// **transport のスレッドから呼ばれる。** Qt のオブジェクトを直接触らず、
/// 値を書いてから humansChanged() をキュー経由で出す。
void HumanControlPanel::OnCharacterState(
    const std::string &_name, const gz::msgs::Param &_message)
{
  const auto &params = _message.params();

  // 名前で引く。添字は削除でずれるので、非同期のコールバックからは使わない
  // （構想書 §6）。HumanRegistry::Find() がその入口。
  Human *found = this->humans.Find(_name);
  if (found == nullptr)
    return;

  // 知らないキーは黙って無視し、知っているキーだけ読む。これが Param を
  // 選んだ理由そのもので、送り側が先に新しいキーを足しても壊れない
  // （'|' 区切り固定長の roster とは対照的 -- CLAUDE.md 罠4）。
  const auto stateIt = params.find("state");
  const CharacterState state = stateIt != params.end()
      ? StateFromString(stateIt->second.string_value().c_str())
      : CharacterState::Unknown;
  const auto poseIt = params.find("pose");
  const std::string pose =
      poseIt != params.end() ? poseIt->second.string_value() : std::string();
  const auto speedIt = params.find("speed");
  const double speed =
      speedIt != params.end() ? speedIt->second.double_value() : 0.0;

  if (found->serverState == state && found->serverPose == pose &&
      std::abs(found->serverSpeed - speed) < 1e-6)
  {
    return;   // heartbeat。変わっていないなら QML を起こさない
  }
  found->serverState = state;
  found->serverPose = pose;
  found->serverSpeed = speed;

  // 表示の更新は GUI スレッドで。ここから直接 emit すると Qt の規約違反。
  QMetaObject::invokeMethod(this, [this]() { emit this->humansChanged(); },
      Qt::QueuedConnection);
}

/// \brief サーバーが言っている状態のラベル。GUI の推測ではない。
QString HumanControlPanel::humanStateLabel(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return QString();
  const Human &human = this->humans.at(static_cast<std::size_t>(_index));
  if (human.serverState == CharacterState::Unknown)
    return QString("-");

  // 表示用の日本語。enum と文字列の対応は CharacterState.hh が正本で、
  // ここはあくまで見せ方。知らない値は英語名のまま出す（黙って隠さない）。
  static const std::map<CharacterState, const char *> kLabels = {
    {CharacterState::Standing,     "立っている"},
    {CharacterState::Moving,       "移動中"},
    {CharacterState::Following,    "経路を追従中"},
    {CharacterState::Jumping,      "ジャンプ中"},
    {CharacterState::PoseEntering, "姿勢へ移行中"},
    {CharacterState::PoseHolding,  "姿勢を保持中"},
    {CharacterState::PoseExiting,  "姿勢から復帰中"},
    {CharacterState::Removing,     "削除中"},
  };
  const auto label = kLabels.find(human.serverState);
  QString text = label != kLabels.end()
      ? QString::fromUtf8(label->second)
      : QString::fromUtf8(ToString(human.serverState));
  if (!human.serverPose.empty())
    text += QString(" (%1)").arg(QString::fromStdString(human.serverPose));
  return text;
}
}  // namespace gz_human_sim
