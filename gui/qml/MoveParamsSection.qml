import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// MoveParamsSection -- ジャンプ高さと基準移動速度（キー操作対象にかかるグローバル設定）
//
// HumanControlPanel.qml から切り出したもの。親の ColumnLayout に
// そのまま並ぶよう、ここも ColumnLayout で同じ spacing を持つ。
// **親からは絶対 qrc URL で import される**（相対 import と暗黙の
// 同一ディレクトリ解決は gz-gui のロード経路では効かない。qmldir と
// .qrc の3点セットで初めて成立する -- CLAUDE.md 罠1）。
ColumnLayout {
  Layout.fillWidth: true
  spacing: 10

  // ── 移動パラメーター（ジャンプ・速度）─────────────────
  // ジャンプ高さ・基準移動速度はキー操作対象(activeHumanIndex)にかかる
  // グローバル設定 -- shiftHeld/ctrlHeld/sHeld/jHeld と同じ扱いで、対象人物を
  // 切り替えても値は維持される。基準速度はCtrl（ゆっくり歩く）/Shift
  // （走る）を押していない、通常の移動時の速度。Ctrl/Shiftを押すと
  // この基準速度に対してさらに倍率がかかる。
  Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
  Label { text: "移動パラメーター"; color: "#183b37"; font.bold: true }
  RowLayout {
    Layout.fillWidth: true
    spacing: 4
    Label { text: "ジャンプの高さ[m]"; color: "#536b67" }
    Slider {
      id: jumpHeightSlider
      Layout.fillWidth: true
      from: 0.05
      to: 1.5
      Component.onCompleted: value = HumanControlPanel.jumpHeight
      onMoved: HumanControlPanel.setJumpHeight(value)
    }
    Label {
      text: HumanControlPanel.jumpHeight.toFixed(2)
      color: "#536b67"
      Layout.preferredWidth: 34
    }
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    Label { text: "基準移動速度"; color: "#536b67" }
    Slider {
      id: speedMultiplierSlider
      Layout.fillWidth: true
      from: 0.1
      to: 4.0
      Component.onCompleted: value = HumanControlPanel.speedMultiplier
      onMoved: HumanControlPanel.setSpeedMultiplier(value)
    }
    Label {
      text: (HumanControlPanel.ctrlHeld ? "遅い" :
          (HumanControlPanel.shiftHeld ? "走る" : "歩く")) +
          "（x" + HumanControlPanel.speedMultiplier.toFixed(2) + "）"
      color: "#536b67"
      Layout.preferredWidth: 92
    }
    Button {
      text: "⤴ ジャンプ"
      enabled: HumanControlPanel.activeHumanIndex >= 0 &&
          HumanControlPanel.isHumanActorAt(HumanControlPanel.activeHumanIndex)
      onClicked: HumanControlPanel.teleopJump(HumanControlPanel.activeHumanIndex)
    }
  }
  Connections {
    target: HumanControlPanel
    function onJumpHeightChanged() {
      jumpHeightSlider.value = HumanControlPanel.jumpHeight
    }
    function onSpeedMultiplierChanged() {
      speedMultiplierSlider.value = HumanControlPanel.speedMultiplier
    }
  }
}
