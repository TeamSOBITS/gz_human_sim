import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// MarkerSection -- 初期スポーンマーカー（各人物の初期地点に置く色付きの円）の一括表示切替
//
// HumanControlPanel.qml から切り出したもの。親の ColumnLayout に
// そのまま並ぶよう、ここも ColumnLayout で同じ spacing を持つ。
// **親からは絶対 qrc URL で import される**（相対 import と暗黙の
// 同一ディレクトリ解決は gz-gui のロード経路では効かない。qmldir と
// .qrc の3点セットで初めて成立する -- CLAUDE.md 罠1）。
ColumnLayout {
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
