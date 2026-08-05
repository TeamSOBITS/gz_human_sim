import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// RouteSection -- 経路の作成（3Dビュークリック／テンプレート）と、対象人物への適用
//
// HumanControlPanel.qml から切り出したもの。親の ColumnLayout に
// そのまま並ぶよう、ここも ColumnLayout で同じ spacing を持つ。
// **親からは絶対 qrc URL で import される**（相対 import と暗黙の
// 同一ディレクトリ解決は gz-gui のロード経路では効かない。qmldir と
// .qrc の3点セットで初めて成立する -- CLAUDE.md 罠1）。
ColumnLayout {
  Layout.fillWidth: true
  spacing: 10

  // 経路テンプレートの選択状態。移した理由は SpawnSection.qml と同じ。
  property int selectedPathTemplateIndex: 0

  // ── 経路（複数人にまとめて歩かせる）─────────────────────
  // 経路の「作り方」が2通り（3Dビュークリック／テンプレート図形）あるだけで、
  // できあがる経由点リストは1つに統合されている。どちらで作っても同じ一覧に
  // 並び、同じ「1点取り消し/クリア」で直せて、同じボタンで同じ対象人物へ
  // 適用される ―― 以前はテンプレートだけが別経路で、1人へ即送信され、
  // SFM/往復の設定も無視されていた。
  //
  // 対象は人物リスト各行の「経路対象」チェック（spawn直後はON）。複数人に
  // チェックが入っていれば、同じ経路をその全員へ一度に適用する。
  Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
  RowLayout {
    Layout.fillWidth: true
    Label { text: "経路"; color: "#183b37"; font.bold: true; Layout.fillWidth: true }
    Label {
      text: "対象 " + HumanControlPanel.routeTargetCount + "人"
      color: "#536b67"; font.pixelSize: 11
    }
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    Button {
      Layout.fillWidth: true
      text: "全員を対象にする"
      enabled: HumanControlPanel.humanList.length > 0
      onClicked: HumanControlPanel.setAllRouteTargets(true)
    }
    Button {
      Layout.fillWidth: true
      text: "対象を全解除"
      enabled: HumanControlPanel.humanList.length > 0
      onClicked: HumanControlPanel.setAllRouteTargets(false)
    }
  }

  // ① 3Dビュークリックで作る
  Label { text: "① 3Dビューをクリックして作る"; color: "#536b67"; font.pixelSize: 11 }
  Button {
    Layout.fillWidth: true
    text: HumanControlPanel.routeRecording
        ? "経路登録モード: ON（クリックが経由点になります）"
        : "経路登録モードを開始（クリックが経由点になります）"
    checkable: true
    checked: HumanControlPanel.routeRecording
    contentItem: Text {
      text: parent.text
      color: "white"
      horizontalAlignment: Text.AlignHCenter
      verticalAlignment: Text.AlignVCenter
      wrapMode: Text.Wrap
    }
    background: Rectangle {
      radius: 6
      color: HumanControlPanel.routeRecording ? "#e65100" : "#78909c"
    }
    onClicked: HumanControlPanel.setRouteRecording(!HumanControlPanel.routeRecording)
  }

  // ② テンプレート図形で作る
  Label { text: "② テンプレート図形で作る"; color: "#536b67"; font.pixelSize: 11 }
  RowLayout {
    Layout.fillWidth: true
    spacing: 4
    Label { text: "形状"; color: "#536b67" }
    ComboBox {
      id: pathTemplateCombo
      Layout.fillWidth: true
      model: HumanControlPanel.pathTemplateLabels
      currentIndex: selectedPathTemplateIndex
      onActivated: selectedPathTemplateIndex = currentIndex
    }
  }
  GridLayout {
    Layout.fillWidth: true
    columns: 4
    columnSpacing: 6
    Label { text: "中心x"; color: "#536b67" }
    TextField { id: pathCenterXField; text: "0.0"; Layout.fillWidth: true }
    Label { text: "中心y"; color: "#536b67" }
    TextField { id: pathCenterYField; text: "0.0"; Layout.fillWidth: true }
    Label {
      text: pathTemplateCombo.currentIndex === 1 ? "一辺[m]" : "半径[m]"
      color: "#536b67"
    }
    TextField { id: pathSizeField; text: "2.0"; Layout.fillWidth: true }
    Label {
      text: "分割数"; color: "#536b67"
      visible: pathTemplateCombo.currentIndex !== 1
    }
    TextField {
      id: pathWaypointsField
      text: "12"
      Layout.fillWidth: true
      visible: pathTemplateCombo.currentIndex !== 1
    }
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    CheckBox {
      id: pathClockwiseCheck
      text: "時計回り"
    }
    Button {
      Layout.fillWidth: true
      text: "この図形で経由点を生成"
      onClicked: HumanControlPanel.generatePathTemplate(
          pathTemplateCombo.currentIndex,
          parseFloat(pathCenterXField.text) || 0.0,
          parseFloat(pathCenterYField.text) || 0.0,
          parseFloat(pathSizeField.text) || 2.0,
          parseInt(pathWaypointsField.text) || 12,
          pathClockwiseCheck.checked)
    }
  }

  // できあがった経由点（①②共通）
  Label {
    text: "経由点（" + HumanControlPanel.pendingRoutePoints.length + "点）"
    color: "#536b67"; font.pixelSize: 11
  }
  ListView {
    Layout.fillWidth: true
    Layout.preferredHeight: Math.min(Math.max(contentHeight, 20), 100)
    clip: true
    model: HumanControlPanel.pendingRoutePoints
    delegate: Label {
      text: (index + 1) + ": (" + modelData + ")"
      color: "#536b67"
      font.pixelSize: 11
    }
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    visible: HumanControlPanel.pendingRoutePoints.length > 0
    Button {
      Layout.fillWidth: true
      text: "1点取り消し"
      onClicked: HumanControlPanel.undoLastRoutePoint()
    }
    Button {
      Layout.fillWidth: true
      text: "クリア"
      onClicked: HumanControlPanel.clearPendingRoute()
    }
  }

  // 歩かせ方
  RowLayout {
    Layout.fillWidth: true
    spacing: 4
    Label { text: "歩き方"; color: "#536b67" }
    ComboBox {
      id: sfmModeCombo
      Layout.fillWidth: true
      model: ["単純パス追従（回避なし・既定）", "SFM（自動回避）"]
      Component.onCompleted: currentIndex = HumanControlPanel.useSfm ? 1 : 0
      onActivated: HumanControlPanel.setUseSfm(currentIndex === 1)
    }
  }
  // SFMはワールド側にSfmCrowdSystemが読み込まれている必要がある。
  // 入っていないワールドで選ぶと登録が誰にも届かず、確定しても人が
  // 動かない（これが「経路を確定しても歩かない」の原因だった）ので、
  // 選んだ時点で警告する。
  Label {
    Layout.fillWidth: true
    visible: sfmModeCombo.currentIndex === 1 && !HumanControlPanel.sfmAvailable
    text: "⚠ このワールドにはSFM（自動回避）システムが読み込まれていません。" +
        "このままでは経路を確定しても動きません。「単純パス追従」を選んでください" +
        "（SFMを使う場合はワールド側にSfmCrowdSystemプラグインが必要です）。"
    color: "#e65100"
    font.pixelSize: 11
    wrapMode: Text.Wrap
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    CheckBox {
      id: cyclicRouteCheck
      text: "往復（終点まで来たら折り返す）"
      Component.onCompleted: checked = HumanControlPanel.cyclicRoute
      onToggled: HumanControlPanel.setCyclicRoute(checked)
    }
  }
  // 障害物回避（グローバル経路計画）。ONだと、クリックした経由点の
  // 「間」を壁や家具を避けて通るように経路が引き直されてから送信される
  // （src/nav_grid_system.cpp）。OFFだと点と点を直線で結ぶので、間に
  // 何かあれば突っ込む。人物の当たり判定の半径ぶん膨らませた地図上で
  // 探索するので、体が通れない隙間には経路が引かれない。
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    CheckBox {
      id: avoidObstaclesCheck
      text: "障害物を避ける経路にする"
      Component.onCompleted: checked = HumanControlPanel.avoidObstacles
      onToggled: HumanControlPanel.setAvoidObstacles(checked)
    }
  }
  Label {
    Layout.fillWidth: true
    visible: avoidObstaclesCheck.checked && !HumanControlPanel.avoidObstaclesAvailable
    text: "⚠ このワールドには経路プランナ（NavGridSystem）が読み込まれていません。" +
        "このままだと経由点を直線で結ぶだけになります。ワールドのSDFに " +
        "gz_human_nav_grid_system プラグインを追加してください。"
    color: "#e65100"
    font.pixelSize: 11
    wrapMode: Text.Wrap
  }
  Button {
    Layout.fillWidth: true
    text: "この経路で歩かせる（対象 " + HumanControlPanel.routeTargetCount + "人）"
    enabled: HumanControlPanel.pendingRoutePoints.length > 0 &&
        HumanControlPanel.humanList.length > 0
    contentItem: Text {
      text: parent.text
      color: "white"
      font.bold: true
      horizontalAlignment: Text.AlignHCenter
      verticalAlignment: Text.AlignVCenter
    }
    background: Rectangle {
      radius: 6
      color: parent.enabled ? (parent.pressed ? "#2e7d32" : "#43a047") : "#b0bec5"
    }
    onClicked: HumanControlPanel.confirmRoute()
  }
  // SFMで走らせた人物を手動操作へ戻すためのスイッチ。経路そのものは
  // 消えないので、もう一度有効にすれば続きから巡回を再開する。
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    visible: sfmModeCombo.currentIndex === 1
    Button {
      Layout.fillWidth: true
      text: "対象のSFMを有効化"
      onClicked: HumanControlPanel.setSfmEnabledForTargets(true)
    }
    Button {
      Layout.fillWidth: true
      text: "対象を手動操作へ戻す"
      onClicked: HumanControlPanel.setSfmEnabledForTargets(false)
    }
  }
  Connections {
    target: HumanControlPanel
    function onRouteSettingsChanged() {
      sfmModeCombo.currentIndex = HumanControlPanel.useSfm ? 1 : 0
      cyclicRouteCheck.checked = HumanControlPanel.cyclicRoute
      avoidObstaclesCheck.checked = HumanControlPanel.avoidObstacles
    }
  }
  Label {
    Layout.fillWidth: true
    text: "単純パス追従は回避なしで経由点を直進します（軽い・どのワールドでも動く）。" +
        "SFM（自動回避）は他の人物・ロボット・壁を避けながら巡回します" +
        "（自然だが重く、ワールド側にSfmCrowdSystemが必要）。" +
        "経路を歩けるのは walking_actor / DoctorFemaleWalk だけです。"
    color: "#7b928d"
    font.pixelSize: 11
    wrapMode: Text.Wrap
  }
}
