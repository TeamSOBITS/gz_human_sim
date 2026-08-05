/*
 * DualsenseSection -- DualSense（PS5 コントローラ）モードの有効化と状態表示。
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
