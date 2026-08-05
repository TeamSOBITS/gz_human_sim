/*
 * MarkerSection -- 初期スポーン地点のマーカー表示。
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

  // ── 初期スポーンマーカー ────────────────────────────────
  // 各人物が最初にspawnされた地点に置かれる、その人物専用の色付きの円。
  // シミュレーション上のオブジェクトではなく描画だけのビジュアルなので
  // （HumanControlPanel::ApplySpawnMarkers()）、当たり判定も質量も持たず、
  // 誰かの通行を邪魔したりスポーン判定に引っかかったりすることはない。
  // 個別の表示切替は人物リストの各行のボタン、ここは全員まとめて。
  Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    Label { text: "初期スポーンマーカー"; color: "#183b37"; font.bold: true }
    Item { Layout.fillWidth: true }
    // 「全員表示にする / 全員非表示にする」の1ボタン式は当たり判定の一括
    // ボタンと同じ方式（状態が揃っていないときは、まず揃う方を出す）。
    Button {
      id: markerAllToggleButton
      property bool allShown: false
      function resync() {
        var any = false
        var all = true
        for (var i = 0; i < HumanControlPanel.humanList.length; ++i) {
          any = true
          if (!HumanControlPanel.showSpawnMarkerAt(i))
            all = false
        }
        allShown = any && all
      }
      Component.onCompleted: resync()
      enabled: HumanControlPanel.humanList.length > 0
      text: allShown ? "全員非表示にする" : "全員表示にする"
      onClicked: HumanControlPanel.setShowSpawnMarkerAll(!allShown)
      Connections {
        target: HumanControlPanel
        function onHumansChanged() { markerAllToggleButton.resync() }
      }
    }
  }
}
