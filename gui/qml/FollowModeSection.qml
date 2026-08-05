import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// FollowModeSection -- 実行中の follow mode（テレオペ／経路専用）の切替
//
// HumanControlPanel.qml から切り出したもの。親の ColumnLayout に
// そのまま並ぶよう、ここも ColumnLayout で同じ spacing を持つ。
// **親からは絶対 qrc URL で import される**（相対 import と暗黙の
// 同一ディレクトリ解決は gz-gui のロード経路では効かない。qmldir と
// .qrc の3点セットで初めて成立する -- CLAUDE.md 罠1）。
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
