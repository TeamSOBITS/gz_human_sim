/*
 * MoveParamsSection -- 移動パラメーター。ジャンプの高さ・基準移動速度と、その場ジャンプ。
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
