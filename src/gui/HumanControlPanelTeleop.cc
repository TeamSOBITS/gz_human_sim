#include "HumanControlPanel.hh"

#include <algorithm>
#include <string>

#include <QString>

#include "HumanControlPanelInternal.hh"

#include <unified_entity_control/CommandDispatcher.hh>
#include <unified_entity_control/DirectionKeys.hh>

// 人物を動かす操作の QML 窓口。
//
// ---------------------------------------------------------------------------
// 中身はここに無い（構想書 §11）
// ---------------------------------------------------------------------------
// **速度の作り方も、送り方も、押しっぱなしの扱いも unified_entity_control
// にあります。** このファイルがやるのは翻訳だけ:
//
//   「何番の人物か」  ->  EntityEntry（名前・トピック・動ける範囲）
//
// EntityEntry は素の構造体で、gz_human_sim の型は一切含みません。だから
// あちらはこちらを知らずに済み、guide_robot のロボットも同じ道を通れます。
// **依存は一方通行。逆向きの参照を足さないこと。**
//
// ---------------------------------------------------------------------------
// ここに残っているもの
// ---------------------------------------------------------------------------
// 姿勢の「登録」（L キー / 「現在の姿勢を登録」ボタン）だけは人物ごとの
// 帳簿なので Human::lockedPose に残しています。登録済みの姿勢は、キーを
// 離しても保持され、押しても上書きされません。この優先順位は人物固有の
// 仕様で、汎用のテレオペ層に持たせるものではありません。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
namespace
{
using unified_entity_control::EntityEntry;

/// \brief 「gztopic:」前置き。CommandDispatcher がこの形で受ける。
std::string Channel(const std::string &_topic)
{
  return "gztopic:" + _topic;
}
}  // namespace

EntityEntry HumanControlPanel::EntryFor(int _index) const
{
  EntityEntry entry;
  if (!this->humans.valid(_index))
    return entry;
  const auto &human = this->humans.at(_index);

  entry.kind = "human";
  entry.name = human.name;
  entry.label = human.name;
  // spawn_human.launch.py が組み立てるのと同じ規則。人物名がそのまま
  // namespace になる（PublishRoster() の cmdVelTopic と同じ）。
  entry.cmdVelTopic = "/" + human.name + "/cmd_vel";
  // 人物は真横に歩ける。
  entry.holonomic = true;
  // 人物のヨーは反転不要（sobit_pro だけが -1.0）。
  entry.yawSign = 1.0;
  entry.maxLinear = kAdvertisedMaxLinear;
  entry.maxAngular = kAdvertisedMaxAngular;
  // ActorCommandPlugin は「絶対方位を向きながら歩く」指令を受け付ける。
  entry.turnToFace = true;

  if (human.jumpPublisher.Valid())
    entry.jumpChannel = Channel("/" + human.name + "/cmd_jump");
  if (human.posePublisher.Valid())
    entry.poseChannel = Channel("/" + human.name + "/cmd_pose");
  return entry;
}

// ---------------------------------------------------------------------------
// 移動パラメーター（QML のスライダー）
// ---------------------------------------------------------------------------
double HumanControlPanel::JumpHeight() const
{
  return this->jumpHeightState;
}

void HumanControlPanel::setJumpHeight(double _value)
{
  const double clamped = std::clamp(_value,
      unified_entity_control::kJumpHeightMin,
      unified_entity_control::kJumpHeightMax);
  if (this->jumpHeightState == clamped)
    return;
  this->jumpHeightState = clamped;
  this->jumpHeightChanged();
}

double HumanControlPanel::SpeedMultiplier() const
{
  return this->directionKeys.SpeedMultiplier();
}

void HumanControlPanel::setSpeedMultiplier(double _value)
{
  const double before = this->directionKeys.SpeedMultiplier();
  this->directionKeys.SetSpeedMultiplier(_value);
  if (this->directionKeys.SpeedMultiplier() == before)
    return;
  this->speedMultiplierChanged();
  // 移動中に倍率を変えたら、その場で効かせる。押し直しを要求しない。
  this->RefreshHeldMovement();
}

// ---------------------------------------------------------------------------
// 移動パッド / キーボードから呼ばれる操作
// ---------------------------------------------------------------------------
void HumanControlPanel::teleopMove(
    int _index, double _linear, double _lateral, double _angular)
{
  if (!this->humans.valid(_index))
    return;
  const auto entry = this->EntryFor(_index);
  if (entry.cmdVelTopic.empty())
    return;

  // 生の速度をそのまま送る。倍率も上限もここまでで決まっているので、
  // Dispatcher 側で二重に掛からないよう maxLinear/maxAngular で割り戻す
  // 形の正規化値を渡す。
  unified_entity_control::CommandIntent intent;
  intent.linear = entry.maxLinear > 0.0 ? _linear / entry.maxLinear : 0.0;
  intent.lateral = entry.maxLinear > 0.0 ? _lateral / entry.maxLinear : 0.0;
  intent.angular = entry.maxAngular > 0.0 ? _angular / entry.maxAngular : 0.0;
  this->dispatcher.Send(entry, intent);
}

void HumanControlPanel::teleopStop(int _index)
{
  if (!this->humans.valid(_index))
    return;
  this->dispatcher.SendStop(this->EntryFor(_index));
}

void HumanControlPanel::teleopJump(int _index)
{
  if (!this->humans.valid(_index))
    return;
  this->dispatcher.SendJump(this->EntryFor(_index), this->jumpHeightState);
}

void HumanControlPanel::teleopRotate(int _index, bool _counterClockwise)
{
  if (!this->humans.valid(_index))
    return;
  // Pure-angular -- no linear/lateral -- so the actor spins on the spot
  // rather than walking. Sign matches DirectionToTwist()'s A/D convention
  // (A: +lateral -> +yaw -> counterclockwise; D: -lateral -> -yaw).
  const double angular =
      (_counterClockwise ? unified_entity_control::kTurnRate
                         : -unified_entity_control::kTurnRate) *
      this->directionKeys.EffectiveMultiplier();
  this->teleopMove(_index, 0.0, 0.0, angular);
}

void HumanControlPanel::teleopDirection(
    int _index, const QString &_direction, bool _turnToFace)
{
  if (!this->humans.valid(_index))
    return;

  const std::string direction = _direction.toStdString();
  double linear = 0.0, lateral = 0.0, angular = 0.0;
  if (!unified_entity_control::DirectionToTwist(direction, linear, lateral, angular))
    return;
  const double speed = this->directionKeys.EffectiveMultiplier();

  if (_turnToFace)
  {
    // 押した方向の**絶対方位**へ向きながら歩き続ける。絶対なので、同じ
    // キーを押し直しても 90 度ずつ積み上がらない。
    this->dispatcher.SendTurnToFace(this->EntryFor(_index),
        unified_entity_control::kWalkSpeed * speed,
        unified_entity_control::DirectionHeadingAngle(direction));
    return;
  }

  this->teleopMove(_index, linear * speed, lateral * speed, angular);
}

// ---------------------------------------------------------------------------
// 押しっぱなしの反映。状態機械は unified_entity_control 側。
// ---------------------------------------------------------------------------
void HumanControlPanel::ApplyMotion(
    const unified_entity_control::DirectionKeys::Motion &_motion)
{
  const int index = this->activeHumanIndex;
  if (!this->humans.valid(index))
    return;
  if (_motion.none)
    return;
  if (_motion.stop)
  {
    this->teleopStop(index);
    return;
  }
  if (_motion.turnToFace)
  {
    this->dispatcher.SendTurnToFace(
        this->EntryFor(index), _motion.speed, _motion.heading);
    return;
  }
  this->teleopMove(index, _motion.linear, _motion.lateral, _motion.angular);
}

void HumanControlPanel::PressDirectionKey(const std::string &_direction)
{
  this->ApplyMotion(this->directionKeys.Press(_direction));
}

void HumanControlPanel::ReleaseDirectionKey(const std::string &_direction)
{
  this->ApplyMotion(this->directionKeys.Release(_direction));
}

void HumanControlPanel::RefreshHeldMovement()
{
  this->ApplyMotion(this->directionKeys.Refresh());
}

// ---------------------------------------------------------------------------
// 修飾キーの保持状態（QML のインジケータが読む）
// ---------------------------------------------------------------------------
bool HumanControlPanel::ShiftHeld() const { return this->directionKeys.Run(); }
bool HumanControlPanel::CtrlHeld() const { return this->directionKeys.Slow(); }
bool HumanControlPanel::JHeld() const { return this->directionKeys.Strafe(); }
bool HumanControlPanel::SHeld() const { return this->sHeldState; }

// ---------------------------------------------------------------------------
// 姿勢。登録の帳簿だけがこちら側の仕事。
// ---------------------------------------------------------------------------
QString HumanControlPanel::HeldPose() const
{
  return QString::fromStdString(this->heldPoseState);
}

QString HumanControlPanel::ActiveLockedPose() const
{
  if (!this->humans.valid(this->activeHumanIndex))
    return {};
  return QString::fromStdString(
      this->humans.at(this->activeHumanIndex).lockedPose);
}

std::string HumanControlPanel::EffectivePose(int _index) const
{
  if (!this->humans.valid(_index))
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
  if (!this->humans.valid(_index))
    return;
  if (!this->humans.at(_index).posePublisher.Valid())
    return;
  this->dispatcher.SendPose(this->EntryFor(_index), this->EffectivePose(_index));
}

void HumanControlPanel::togglePoseLock(int _index)
{
  if (!this->humans.valid(_index))
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

// ---------------------------------------------------------------------------
// DualSense（PS5）モード
// ---------------------------------------------------------------------------
// **パッドを読むのも、その値を速度にするのも unified_entity_control 側。**
// ここにあるのは QML 用の表示と有効化トグルだけ。旧版はこのパネル自身が
// SDL2 を開いていたが、そうするとロボットを操作したいときに同じコードを
// guide_robot にも書く羽目になる。だから入力デバイスは向こうに一本化した。
//
// 「有効化」は、このパネルが対象にしている人物を、あちらの操作対象として
// 差し出すという意味になる。
// ---------------------------------------------------------------------------
bool HumanControlPanel::DualsenseModeEnabled() const
{
  return this->dualsenseModeState;
}

void HumanControlPanel::SetDualsenseModeEnabled(bool _enabled)
{
  if (this->dualsenseModeState == _enabled)
    return;
  this->dualsenseModeState = _enabled;
  // 無効化したら、押しっぱなしの扱いを残さない。有効時に押していた方向が
  // 残っていると、無効化後も歩き続けているように見える。
  if (!_enabled)
  {
    this->directionKeys.Clear();
    this->teleopStop(this->activeHumanIndex);
  }
  this->dualsenseModeChanged();
  this->dualsenseStatusChanged();
}

QString HumanControlPanel::DualsenseStatusText() const
{
  if (!this->dualsenseModeState)
    return "無効";
  // roster を購読しているのは unified_entity_control 側なので、こちらは
  // 接続状態を直接は知らない。**知っているふりをしないこと。**
  return "有効（コントローラの状態は「統合操作」パネルを参照）";
}

bool HumanControlPanel::InvertCameraY() const
{
  return this->invertCameraYState;
}

void HumanControlPanel::SetInvertCameraY(bool _enabled)
{
  if (this->invertCameraYState == _enabled)
    return;
  this->invertCameraYState = _enabled;
  this->invertCameraYChanged();
}
}  // namespace gz_human_sim
