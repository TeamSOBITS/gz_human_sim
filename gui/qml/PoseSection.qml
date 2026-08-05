/*
 * PoseSection -- 姿勢の表示と、現在の姿勢の登録・解除。
 *
 * HumanControlPanel.qml から切り出したもの（構想書 §13）。**中身は
 * gz_human_sim_old と同一。** 変えたのは以下の 2 点だけ:
 *
 *   - 親の ColumnLayout の子として並んでいた要素を ColumnLayout で包んだ
 *   - `id: root` とこのセクションが使うプロパティをここに置いた
 *
 * 2 点目は必須。**QML の id とプロパティはファイル単位のスコープ**なので、
 * 分割元ファイルの `root` やプロパティはここからは見えない。付け忘れると
 * バインディングが静かに切れる（実際に一度やった）。
 *
 * ロジックはここに書かないこと。C++ 側（HumanControlPanel）の
 * Q_PROPERTY / Q_INVOKABLE を呼ぶだけにする。
 */
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

ColumnLayout {
  id: root
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
