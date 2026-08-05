import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// PoseSection -- 姿勢の登録・解除（walking_actor 系のみ）
//
// HumanControlPanel.qml から切り出したもの。親の ColumnLayout に
// そのまま並ぶよう、ここも ColumnLayout で同じ spacing を持つ。
// **親からは絶対 qrc URL で import される**（相対 import と暗黙の
// 同一ディレクトリ解決は gz-gui のロード経路では効かない。qmldir と
// .qrc の3点セットで初めて成立する -- CLAUDE.md 罠1）。
ColumnLayout {
  Layout.fillWidth: true
  spacing: 10

  // ── 姿勢（walking_actorのみ） ──────────────────────────
  // Lキーのマウス版。押している間だけのポーズキー（K＝着席）とは別に、
  // 「いまの姿勢を登録」して押しっぱなしをやめても保持させるボタン。
  // 現在の姿勢そのものを登録する方式なので、今後ポーズが増えても
  // このボタンとLキーは変更なしでそのまま使える
  // -- HumanControlPanel::togglePoseLock()を参照。
  Rectangle {
    Layout.fillWidth: true; height: 1; color: "#c7d8d4"
    visible: HumanControlPanel.activeHumanIndex >= 0 &&
        HumanControlPanel.isPoseCapableHumanAt(HumanControlPanel.activeHumanIndex)
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    visible: HumanControlPanel.activeHumanIndex >= 0 &&
        HumanControlPanel.isPoseCapableHumanAt(HumanControlPanel.activeHumanIndex)
    Label { text: "姿勢"; color: "#183b37"; font.bold: true }
    Label {
      // 「いま何をしているか」＝登録済みならそれ、なければ押されている
      // ポーズキーのもの。登録の有無はボタン側のラベルが示す。
      text: HumanControlPanel.poseLabel(
          HumanControlPanel.activeLockedPose !== ""
              ? HumanControlPanel.activeLockedPose : HumanControlPanel.heldPose)
      color: "#536b67"
    }
    Button {
      id: poseLockButton
      Layout.fillWidth: true
      text: HumanControlPanel.activeLockedPose !== ""
          ? "姿勢の登録を解除" : "現在の姿勢を登録"
      onClicked: HumanControlPanel.togglePoseLock(HumanControlPanel.activeHumanIndex)
    }
  }
}
