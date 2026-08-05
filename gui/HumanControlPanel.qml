import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// セクションは gui/qml/ 配下。**絶対 qrc URL で import すること。**
// gz-gui はこの QML をリソース上の位置を基準 URL にせずに読み込むため、
// 相対 import（import "qml"）も同一ディレクトリの暗黙解決も効かない。
// 型の宣言は gui/qml/qmldir にある。3点セットで初めて成立する。
// 検証は 構想書/これからやるやつ/gz_human_sim再設計構想.md §7、
// 注意点は CLAUDE.md 罠1。
import "qrc:/HumanControlPanel/qml"

Rectangle {
  id: root
  // gz-gui sizes the right sidebar split from the root item's
  // Layout.minimum* values; without them the sidebar collapses to zero
  // width and hides behind the window's right edge. Content has grown a
  // lot since 560 was first picked here (follow-mode combos, path
  // templates, the viewpoint block, ...) and previously just kept getting
  // squeezed -- the fixed-height stuff would silently eat the spawned-
  // humans ListView's space (Layout.fillHeight: true) down to a sliver,
  // taking the per-row 削除/teleop pad with it even though they were still
  // there in the tree. Now everything below lives inside a ScrollView
  // instead, so this only needs to be a *reasonable starting* height, not
  // a tally of every field added since -- anything that doesn't fit just
  // scrolls.
  Layout.minimumWidth: 380
  Layout.minimumHeight: 640
  anchors.fill: parent
  color: "#eef4f2"


  ScrollView {
    anchors.fill: parent
    clip: true
    ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

    ColumnLayout {
    // ScrollView's content item is a plain Flickable, not a Layout, so
    // anchors.margins (used everywhere else in this file) doesn't apply
    // here -- x/y/width stand in for the same 12px margin instead.
    x: 12
    y: 12
    width: root.width - 24
    spacing: 10

    RowLayout {
      Layout.fillWidth: true
      spacing: 8
      Rectangle { width: 5; height: 36; radius: 2; color: "#16847c" }
      ColumnLayout {
        spacing: 0
        Label { text: "HUMAN"; color: "#126e68"; font.bold: true; font.pixelSize: 12 }
        Label { text: "人物操作"; color: "#183b37"; font.bold: true; font.pixelSize: 19 }
      }
      Item { Layout.fillWidth: true }
    }

    Rectangle {
      Layout.fillWidth: true
      height: 32
      radius: 6
      color: "#dcefe9"
      Label {
        anchors.fill: parent
        anchors.leftMargin: 10
        anchors.rightMargin: 10
        text: HumanControlPanel.status
        color: "#126e68"
        font.bold: true
        elide: Text.ElideRight
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
      }
    }


    // 中身は gui/qml/ の各セクションへ。並び順が画面の並び順。
    // セクションを足したら gui/qml/qmldir と
    // gui/HumanControlPanel.qrc の両方に足すこと（どちらか片方だと
    // 起動時に "is not a type" でパネルごと出なくなる）。
    SpawnSection { }
    MarkerSection { }
    ViewpointSection { }
    MoveParamsSection { }
    DualsenseSection { }
    FollowModeSection { }
    PoseSection { }
    CollisionSection { }
    RouteSection { }
    HelpSection { }
    }
  }
}
