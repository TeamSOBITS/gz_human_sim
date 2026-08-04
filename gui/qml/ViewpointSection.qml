/*
 * ViewpointSection -- カメラ視点の選択と距離。
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

  // ── Viewpoint ──────────────────────────────────────────
  // One global control block for whichever human is "対象" (activeHumanIndex),
  // not one per row -- mirrors guide_robot's GuiderRobotManager, which
  // drives a single viewpointCombo off whichever robot robotCombo
  // currently selects rather than giving every robot its own combo.
  // currentIndex/text below are refreshed imperatively (not bound live)
  // so they don't fight with the user's own edits while this human stays
  // selected, same pattern as the x/y spawn fields above.
  Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
  RowLayout {
    Layout.fillWidth: true
    Label { text: "視点操作"; color: "#183b37"; font.bold: true; Layout.fillWidth: true }
    // Not tied to any one human -- releases follow/track (like "自由視点"
    // for whichever human is "対象") and also snaps the camera back to
    // the pose gui.config's MinimalScene started it at, instead of just
    // leaving it wherever it last drifted to.
    Button {
      text: "🏠 全体俯瞰"
      onClicked: HumanControlPanel.resetToInitialView()
    }
  }
  Label {
    Layout.fillWidth: true
    text: HumanControlPanel.activeHumanIndex >= 0
        ? "対象: ⌨ " + HumanControlPanel.humanList[HumanControlPanel.activeHumanIndex]
        : "対象の人物が未選択です（各行の「対象にする」、または1〜9キーで選択）"
    color: "#7b928d"
    font.pixelSize: 11
    wrapMode: Text.Wrap
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 4
    Label { text: "視点"; color: "#536b67" }
    ComboBox {
      id: viewCombo
      Layout.fillWidth: true
      enabled: HumanControlPanel.activeHumanIndex >= 0
      model: HumanControlPanel.viewpointLabels
      Component.onCompleted: currentIndex = HumanControlPanel.activeViewIndex
      onActivated: HumanControlPanel.setViewpoint(
          HumanControlPanel.activeHumanIndex, currentIndex,
          parseFloat(viewDistanceField.text) || 2.0)
    }
    Label { text: "距離[m]"; color: "#536b67" }
    TextField {
      id: viewDistanceField
      Layout.preferredWidth: 56
      text: HumanControlPanel.activeViewDistance.toFixed(2)
      // Index 0 (自由視点) and 1 (一人称) don't use a distance.
      enabled: HumanControlPanel.activeHumanIndex >= 0 && viewCombo.currentIndex > 1
      onEditingFinished: {
        if (viewCombo.currentIndex > 1)
          HumanControlPanel.setViewpoint(
              HumanControlPanel.activeHumanIndex, viewCombo.currentIndex,
              parseFloat(text) || 2.0)
      }
    }
  }
  // Refreshes the combo/distance whenever the active human or its stored
  // viewpoint changes (switching "対象", a spawn's auto-focus, a manual
  // selection made here, resetToInitialView(), ...).
  Connections {
    target: HumanControlPanel
    function onActiveViewIndexChanged() {
      viewCombo.currentIndex = HumanControlPanel.activeViewIndex
      viewDistanceField.text = HumanControlPanel.activeViewDistance.toFixed(2)
    }
  }

}
