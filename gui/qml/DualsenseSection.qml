import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// DualsenseSection -- DualSenseモードの有効化と、その説明
//
// HumanControlPanel.qml から切り出したもの。親の ColumnLayout に
// そのまま並ぶよう、ここも ColumnLayout で同じ spacing を持つ。
// **親からは絶対 qrc URL で import される**（相対 import と暗黙の
// 同一ディレクトリ解決は gz-gui のロード経路では効かない。qmldir と
// .qrc の3点セットで初めて成立する -- CLAUDE.md 罠1）。
ColumnLayout {
  Layout.fillWidth: true
  spacing: 10

  // ── DualSense mode ───────────────────────────────────
  // Left stick drives activeHumanIndex (full 360°移動、上の「基準移動速度」
  // スライダーがそのまま適用される)、右スティックはカメラをその人物の
  // 周りで旋回させる -- キーボード/矢印パッドの代替であり、併用も可能
  // （どちらもHumanControlPanel.teleopMove()を呼ぶだけ）。
  // guide_robotのGuiderRobotManagerと同じ仕組み。
  Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
  Label { text: "DualSenseモード"; color: "#183b37"; font.bold: true }
  RowLayout {
    Layout.fillWidth: true
    CheckBox {
      text: "有効化"
      checked: HumanControlPanel.dualsenseModeEnabled
      onToggled: HumanControlPanel.dualsenseModeEnabled = checked
    }
    Label {
      Layout.fillWidth: true
      wrapMode: Text.Wrap
      text: HumanControlPanel.dualsenseStatusText
      color: HumanControlPanel.dualsenseModeEnabled ? "#126e68" : "#8aa19c"
    }
  }
  // 既定は「上に倒すと見上げる」(据置ゲーム機の標準)。以前は逆
  // (フライトシム式)で固定だったので、その挙動が好みならここをON。
  // GuiderRobotManager 側の同名設定と挙動を揃えてある。
  CheckBox {
    visible: HumanControlPanel.dualsenseModeEnabled
    text: "視点の上下を反転（上に倒すと見下ろす）"
    font.pixelSize: 12
    checked: HumanControlPanel.invertCameraY
    onToggled: HumanControlPanel.invertCameraY = checked
  }
  Label {
    Layout.fillWidth: true
    visible: HumanControlPanel.dualsenseModeEnabled
    wrapMode: Text.Wrap
    font.pixelSize: 10
    color: "#8aa19c"
    text: "左スティック=全方向移動、L2/R2=旋回、右スティック=視点。" +
        "上の「基準移動速度」スライダーとShift/Ctrlがそのまま適用されます。"
  }
}
