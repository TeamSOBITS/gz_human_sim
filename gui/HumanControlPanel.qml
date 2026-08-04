import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// セクションは qml/ 配下。**絶対 qrc URL で import すること。**
// gz-gui はこの QML を、リソース上の位置を基準 URL にせずに読み込むため、
// 相対 import（import "qml"）も同一ディレクトリの暗黙解決も効かない。
// 型の宣言は qml/qmldir にある。
import "qrc:/HumanControlPanel/qml"

Rectangle {
  id: root
  // gz-gui sizes the right sidebar split from the root item's
  // Layout.minimum* values; without them the sidebar collapses to zero
  // width and hides behind the window's right edge. Content has grown a
  // lot since 560 was first picked here (follow-mode combos, path
  // templates, the viewpoint block, ...) and previously just kept getting
  // squeezed -- the fixed-height stuff would silently eat the spawned-
  // humans ListView's space (Layout.fillHeight: true) down to a sliver. Now everything below lives inside a ScrollView
  // instead, so this only needs to be a *reasonable starting* height, not
  // a tally of every field added since -- anything that doesn't fit just
  // scrolls.
  Layout.minimumWidth: 380
  Layout.minimumHeight: 640
  anchors.fill: parent
  color: "#eef4f2"

  property int selectedModelIndex: 0
  property int selectedPoseIndex: 0
  property int selectedFollowModeIndex: 0
  property int selectedPathTemplateIndex: 0

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

    // 選択中の人物がいま何をしているか。
    //
    // これはパネルが指令から推測した値ではなく、サーバー
    // （ActorCommandPlugin）が state トピックへ流してくる値です
    // （構想書 §3）。テレオペ以外の理由で人物が動いたとき -- 経路追従、
    // 将来の NPC や着席 -- も、ここには正しい状態が出ます。
    Rectangle {
      Layout.fillWidth: true
      height: 28
      radius: 6
      color: "#eef2f1"
      visible: HumanControlPanel.activeHumanIndex >= 0
      Label {
        anchors.fill: parent
        anchors.leftMargin: 10
        anchors.rightMargin: 10
        text: "状態: " + HumanControlPanel.activeCharacterState
        color: "#3d5450"
        elide: Text.ElideRight
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
      }
    }

    SpawnSection { Layout.fillWidth: true }
    MarkerSection { Layout.fillWidth: true }
    ViewpointSection { Layout.fillWidth: true }
    FollowModeSection { Layout.fillWidth: true }
    CollisionSection { Layout.fillWidth: true }
    RouteSection { Layout.fillWidth: true }
    }
  }
}
