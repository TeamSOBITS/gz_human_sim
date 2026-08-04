#include "HumanControlPanel.hh"

#include <mutex>
#include <string>

#include <QString>

#include <gz/msgs/param.pb.h>

#include "HumanControlPanelInternal.hh"

// サーバーから流れてくる人物の状態を受け取り、QML へ渡す。
//
// ---------------------------------------------------------------------------
// なぜこのファイルがあるのか（構想書 §3）
// ---------------------------------------------------------------------------
// このパネルは長らく「自分が送った指令」から人物の状態を推測していました。
// テレオペしか操作手段が無いうちはそれで成立していましたが、
//
//   - テレオペが別パッケージへ移る（構想書 §11）
//   - NPC が自分で動き出す（構想書 §5）
//   - 着席のような、指令と状態が 1 対 1 でない振る舞いが増える
//
// と、推測は成り立ちません。パネルが送っていない指令で人物が動くからです。
//
// そこで真実はサーバー側（ActorCommandPlugin）が持ち、state_topic へ
// publish します。このファイルはそれを受け取って表示するだけです。
// **ここで状態を計算しないこと。** 計算したらまた二重管理に戻ります。
//
// ---------------------------------------------------------------------------
// スレッド
// ---------------------------------------------------------------------------
// OnCharacterState() は gz-transport のスレッドで走ります。Qt スレッドが
// 読む値なので、書き込みも読み出しも stateMutex の下で行います
// （OnPoseInfo()/poseMutex と同じ扱い）。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
namespace
{
/// \brief 状態の日本語ラベル。CharacterState の並びと 1 対 1。
///
/// 英語の enum 名をそのまま出さないのは、このパネルの他の表示（"後方追従"、
/// "着席"、"テレオペ"）が日本語で揃っているためです。
/// 増やすときは CharacterState.hh の kStateNames と両方に足すこと。
const char *const kStateLabels[] = {
  "—",            // Unknown（未受信）
  "静止",         // Standing
  "移動中",       // Moving
  "経路追従中",   // Following
  "ジャンプ中",   // Jumping
  "姿勢へ移行中", // PoseEntering
  "姿勢を保持中", // PoseHolding
  "姿勢から復帰中",  // PoseExiting
  "削除中",       // Removing
};
static_assert(
    sizeof(kStateLabels) / sizeof(kStateLabels[0]) ==
        static_cast<std::size_t>(CharacterState::Count),
    "kStateLabels と CharacterState::Count がずれている");

QString LabelFor(CharacterState _state)
{
  const auto index = static_cast<std::size_t>(_state);
  if (index >= static_cast<std::size_t>(CharacterState::Count))
    return QString::fromUtf8(kStateLabels[0]);
  return QString::fromUtf8(kStateLabels[index]);
}
}  // namespace

void HumanControlPanel::SubscribeCharacterState(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.server.topic.empty())
    return;

  // spawn_human.launch.py が namespace を前置するのと同じ組み立て方。
  // 人物名がそのまま namespace になっている（PublishRoster() の
  // cmdVelTopic と同じ規則）。
  const std::string topic = "/" + human.name + kDefaultStateTopic;

  // 名前をコピーで束ねる。Human の参照を持つと、humans が再確保された
  // ときに宙を指す（削除・追加のたびに vector が動く）。
  const std::string name = human.name;
  if (!this->node.Subscribe<gz::msgs::Param>(topic,
        [this, name](const gz::msgs::Param &_message)
        {
          this->OnCharacterState(_message, name);
        }))
  {
    return;
  }
  human.server.topic = topic;
}

void HumanControlPanel::OnCharacterState(
    const gz::msgs::Param &_message, const std::string &_name)
{
  CharacterState state = CharacterState::Unknown;
  std::string pose;
  double speed = 0.0;

  // gz.msgs.Param は string -> Any の map なので、知らないキーが増えても
  // ここは壊れません。逆に、期待したキーが無くても落ちないこと
  // （publish 側が古い可能性がある）。
  const auto &params = _message.params();
  const auto stateIt = params.find(kStateKeyState);
  if (stateIt != params.end())
    state = StateFromString(stateIt->second.string_value().c_str());
  const auto poseIt = params.find(kStateKeyPose);
  if (poseIt != params.end())
    pose = poseIt->second.string_value();
  const auto speedIt = params.find(kStateKeySpeed);
  if (speedIt != params.end())
    speed = speedIt->second.double_value();

  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(this->stateMutex);
    for (auto &human : this->humans)
    {
      if (human.name != _name)
        continue;
      changed = !human.server.received || human.server.state != state ||
          human.server.pose != pose;
      human.server.state = state;
      human.server.pose = pose;
      human.server.speed = speed;
      human.server.received = true;
      break;
    }
  }

  // 速度だけが変わったときに毎回シグナルを出すと、移動中は毎フレーム
  // QML を起こすことになるので、状態かポーズが変わったときだけ通知する。
  if (changed)
    this->characterStateChanged();
}

QString HumanControlPanel::ActiveCharacterState() const
{
  return this->characterStateAt(this->activeHumanIndex);
}

QString HumanControlPanel::characterStateAt(int _index) const
{
  std::lock_guard<std::mutex> lock(this->stateMutex);
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return QString::fromUtf8(kStateLabels[0]);
  const auto &human = this->humans.at(_index);
  if (!human.server.received)
    return QString::fromUtf8(kStateLabels[0]);

  // 保持中のポーズがあれば併記する。"姿勢を保持中" だけでは、どの姿勢なのか
  // 分からないため（poseLabel() が持っている日本語名を使う）。
  const QString label = LabelFor(human.server.state);
  if (human.server.pose.empty())
    return label;
  return label + "（" +
      this->poseLabel(QString::fromStdString(human.server.pose)) + "）";
}
}  // namespace gz_human_sim
