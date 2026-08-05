/*
 * FollowModeSection -- 動作モードの実行時変更。
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

  // ── Follow mode (runtime) ──────────────────────────────
  // Changes an already-spawned human's follow_mode live, via
  // ActorCommandPlugin's follow_mode_topic -- separate from the spawn
  // form's "動作モード" combo above, which only sets the initial value at
  // spawn time and has no effect on a human that's already walking
  // around. Same "対象" (activeHumanIndex) as viewpoint/path templates.
  Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
  Label { text: "動作モード（実行中に変更）"; color: "#183b37"; font.bold: true }
  RowLayout {
    Layout.fillWidth: true
    spacing: 4
    Label { text: "モード"; color: "#536b67" }
    ComboBox {
      id: runtimeFollowModeCombo
      Layout.fillWidth: true
      enabled: HumanControlPanel.activeHumanIndex >= 0 &&
          HumanControlPanel.isHumanActorAt(HumanControlPanel.activeHumanIndex)
      model: HumanControlPanel.followModeLabels
      Component.onCompleted: currentIndex = HumanControlPanel.activeFollowModeIndex
      onActivated: HumanControlPanel.setFollowMode(
          HumanControlPanel.activeHumanIndex, currentIndex)
    }
  }
  Connections {
    target: HumanControlPanel
    function onActiveFollowModeChanged() {
      runtimeFollowModeCombo.currentIndex = HumanControlPanel.activeFollowModeIndex
    }
  }
}
