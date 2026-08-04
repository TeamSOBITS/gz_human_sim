/*
 * FollowModeSection -- 動作モード（テレオペ／経路専用）の実行時変更。
 *
 * HumanControlPanel.qml から切り出したもの（構想書 §13）。中身は変えていない。
 * 親の ColumnLayout の子として並んでいた要素をそのまま包んだので、
 * ルートは ColumnLayout のまま。Layout.fillWidth を親から引き継ぐ。
 *
 * ロジックはここに書かないこと。C++ 側（HumanControlPanel）の
 * Q_PROPERTY / Q_INVOKABLE を呼ぶだけにする。
 */
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

ColumnLayout {
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
