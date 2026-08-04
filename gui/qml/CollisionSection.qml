/*
 * CollisionSection -- 当たり判定カプセルの表示とサイズ。
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

  // ── 当たり判定（衝突カプセルの表示・サイズ） ─────────────
  // 表示/非表示そのものは人物リストの各行のボタン（上）で個別に切り替える。
  // ここは「全員まとめて」操作と、対象人物（activeHumanIndex）のカプセル
  // サイズ変更 -- gz-simにはその場で形状を連続変形する機能がないため、
  // スライダーは値を保持するだけで、「サイズを適用」を押した瞬間だけ
  // HumanControlPanel::applyCollisionSize()が呼ばれ、削除→新サイズで再生成
  // される（ドラッグ中に何度も再生成されないように）。
  Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
  Label { text: "当たり判定"; color: "#183b37"; font.bold: true }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    Label { text: "表示"; color: "#536b67" }
    // 表示/非表示を1つのボタンで切り替える（個別行と同じ「ラベルが現在の
    // 状態を示すトグルボタン」方式）。ここは全員分をまとめて操作するので、
    // 状態が揃っていないとき（誰かだけ非表示）は「全員表示にする」を出す
    // ―― まず全員を揃える方が、押した結果が読みやすい。
    Button {
      id: collisionAllToggleButton
      property bool allShown: false
      function resync() {
        var any = false
        var all = true
        for (var i = 0; i < HumanControlPanel.humanList.length; ++i) {
          if (!HumanControlPanel.isHumanActorAt(i))
            continue
          any = true
          if (!HumanControlPanel.showCollisionAt(i))
            all = false
        }
        allShown = any && all
      }
      Component.onCompleted: resync()
      text: allShown ? "全員非表示にする" : "全員表示にする"
      onClicked: HumanControlPanel.setShowCollisionAll(!allShown)
      Connections {
        target: HumanControlPanel
        function onHumansChanged() { collisionAllToggleButton.resync() }
      }
    }
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 4
    visible: HumanControlPanel.activeHumanIndex >= 0 &&
        HumanControlPanel.isHumanActorAt(HumanControlPanel.activeHumanIndex)
    Label { text: "半径[m]"; color: "#536b67" }
    Slider {
      id: collisionRadiusSlider
      Layout.fillWidth: true
      from: 0.15
      to: 0.5
      Component.onCompleted: {
        value = HumanControlPanel.activeHumanIndex >= 0
            ? HumanControlPanel.collisionRadiusAt(HumanControlPanel.activeHumanIndex) : 0.25
      }
    }
    Label {
      text: collisionRadiusSlider.value.toFixed(2)
      color: "#536b67"
      Layout.preferredWidth: 34
    }
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 4
    visible: HumanControlPanel.activeHumanIndex >= 0 &&
        HumanControlPanel.isHumanActorAt(HumanControlPanel.activeHumanIndex)
    Label { text: "長さ[m]"; color: "#536b67" }
    Slider {
      id: collisionLengthSlider
      Layout.fillWidth: true
      from: 0.5
      to: 2.0
      Component.onCompleted: {
        value = HumanControlPanel.activeHumanIndex >= 0
            ? HumanControlPanel.collisionLengthAt(HumanControlPanel.activeHumanIndex) : 1.2
      }
    }
    Label {
      text: collisionLengthSlider.value.toFixed(2)
      color: "#536b67"
      Layout.preferredWidth: 34
    }
    Button {
      // 半径/長さスライダーの値を確定してカプセルへ反映するボタン --
      // スライダーをドラッグしている間は何もせず、押した瞬間だけ
      // HumanControlPanel::applyCollisionSize()が呼ばれる。「決定」だと
      // 何を確定するのか伝わらないため、対象を明示したラベルにしている。
      text: "このサイズを当たり判定に適用"
      onClicked: HumanControlPanel.applyCollisionSize(
          HumanControlPanel.activeHumanIndex,
          collisionRadiusSlider.value, collisionLengthSlider.value)
    }
  }
  Connections {
    target: HumanControlPanel
    function onActiveHumanChanged() {
      if (HumanControlPanel.activeHumanIndex < 0)
        return
      collisionRadiusSlider.value =
          HumanControlPanel.collisionRadiusAt(HumanControlPanel.activeHumanIndex)
      collisionLengthSlider.value =
          HumanControlPanel.collisionLengthAt(HumanControlPanel.activeHumanIndex)
    }
  }

}
