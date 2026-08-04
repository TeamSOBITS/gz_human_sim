/*
 * SpawnSection -- 人物のスポーン。モデル・動作モード・名前・配置方法・人数・座標と、
// スポーン済み一覧（各行の対象切替と削除）。
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

  // ── 人物をspawn ────────────────────────────────────────
  // 1人でも複数人でも、座標指定でもクリック指定でも、ここ1か所で完結する
  // （以前は「人物をspawn」と「まとめてspawn」で別フォーム・別ボタンだった）。
  // 違いは「どこに置くか」だけなので、モデル/ポーズ/動作モード/名前/z/yaw の
  // 指定は共通のまま、配置方法だけを下で選ぶ形にしている。
  Label { text: "人物をspawn"; color: "#183b37"; font.bold: true }

  Label { text: "モデル"; color: "#536b67" }
  ComboBox {
    id: modelCombo
    Layout.fillWidth: true
    model: HumanControlPanel.humanModels
    currentIndex: selectedModelIndex
    delegate: ItemDelegate {
      width: modelCombo.width
      highlighted: modelCombo.highlightedIndex === index
      contentItem: Text {
        text: modelData +
            " <span style='font-size:10px;color:#7b928d'>（" +
            HumanControlPanel.modelDescription(index) + "）</span>"
        textFormat: Text.RichText
        verticalAlignment: Text.AlignVCenter
      }
    }
    onActivated: {
      selectedModelIndex = currentIndex
      zField.text = HumanControlPanel.defaultZ(currentIndex).toFixed(2)
    }
  }
  Label {
    Layout.fillWidth: true
    text: "（" + HumanControlPanel.modelDescription(selectedModelIndex) + "）"
    color: "#7b928d"
    font.pixelSize: 10
    wrapMode: Text.Wrap
  }

  Label {
    visible: HumanControlPanel.isCustomHuman(selectedModelIndex)
    text: "ポーズ"; color: "#536b67"
  }
  ComboBox {
    id: poseCombo
    visible: HumanControlPanel.isCustomHuman(selectedModelIndex)
    Layout.fillWidth: true
    model: HumanControlPanel.posePresets
    currentIndex: selectedPoseIndex
    onActivated: selectedPoseIndex = currentIndex
  }

  // Actor models (walking_actor, DoctorFemaleWalk) only -- static humans
  // have no ActorCommandPlugin, so follow_mode means nothing for them.
  // "テレオペ" (auto) is right for almost everyone; "経路専用" is for a
  // background character that should keep walking its route no matter
  // what, ignoring a stray velocity command aimed at it by mistake。
  // 操作そのものは unified_entity_control が担当する（構想書 §11）。
  Label {
    visible: HumanControlPanel.isActorModel(selectedModelIndex)
    text: "動作モード"; color: "#536b67"
  }
  ComboBox {
    id: followModeCombo
    visible: HumanControlPanel.isActorModel(selectedModelIndex)
    Layout.fillWidth: true
    model: HumanControlPanel.followModeLabels
    currentIndex: selectedFollowModeIndex
    onActivated: selectedFollowModeIndex = currentIndex
  }

  // 名前は常に「ベース名＋通し番号」（human1, human2, ...）。1人だけの
  // ときも同じ規則なので、あとから人数を増やしても命名がぶれない。
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    Label { text: "名前"; color: "#536b67" }
    TextField {
      id: nameField
      Layout.fillWidth: true
      text: "human"
      placeholderText: "human"
    }
    Label { text: "→ " + nameField.text + "1, 2, …"; color: "#7b928d"; font.pixelSize: 10 }
  }

  // ── 配置方法 ───────────────────────────────────────────
  // ①座標指定（人数ぶんを自動でグリッド配置）と ②3Dビュークリック
  // （クリックした数だけ、その場所ぴったりに配置）の2択。②で地点を
  // 選んでいる間は人数はクリック数で決まるので、人数欄は隠す。
  //
  // 2つのRadioButtonの「checked」を別々の式で計算していると、両方が真に
  // なりうる瞬間ができてしまい（座標指定側は常にtrue固定、クリック側は
  // 地点を1つでも選ぶとtrueになるため）両方選択済みに見えるボタンに
  // なっていた。single source of truth（このplacingByClick 1つ）から
  // 両方のcheckedを導くことで、常にどちらか一方だけがcheckedになる。
  property bool placingByClick: false
  Label { text: "配置方法"; color: "#536b67" }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    RadioButton {
      id: placeByCoordRadio
      text: "座標指定"
      checked: !root.placingByClick
      onToggled: if (checked) {
        root.placingByClick = false
        HumanControlPanel.setSpawnPicking(false)
        HumanControlPanel.clearSpawnPoints()
      }
    }
    RadioButton {
      id: placeByClickRadio
      text: "3Dビューでクリック"
      checked: root.placingByClick
      onToggled: if (checked) {
        root.placingByClick = true
        HumanControlPanel.setSpawnPicking(true)
      }
    }
  }
  // 地点を1つでもクリックしたら、座標指定に戻さなくてもクリック側の
  // 表示に自動で切り替える（従来の「両方checked」を避けつつ、クリックで
  // 選び始めた操作の流れは維持する）。
  Connections {
    target: HumanControlPanel
    function onPendingSpawnPointsChanged() {
      if (HumanControlPanel.pendingSpawnPoints.length > 0)
        root.placingByClick = true
    }
    function onSpawnPickingChanged() {
      root.placingByClick = HumanControlPanel.spawnPicking ||
          HumanControlPanel.pendingSpawnPoints.length > 0
    }
  }

  // ① 座標指定
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    visible: !placeByClickRadio.checked
    Label { text: "人数"; color: "#536b67" }
    SpinBox { id: countSpin; from: 1; to: 50; value: 1; editable: true }
    Label {
      text: countSpin.value > 1 ? "（" + countSpin.value + "人を自動整列）" : ""
      color: "#7b928d"; font.pixelSize: 10
    }
  }
  // x/yはspawnされるたびnextSpawnX/Y()で自動更新し、既定値のまま連続spawn
  // しても人物同士が同じ座標に重ならないようにする(z/yawは固定のまま)。
  Connections {
    target: HumanControlPanel
    function onHumansChanged() {
      xField.text = HumanControlPanel.nextSpawnX().toFixed(2)
      yField.text = HumanControlPanel.nextSpawnY().toFixed(2)
    }
  }
  GridLayout {
    Layout.fillWidth: true
    visible: !placeByClickRadio.checked
    columns: 4
    columnSpacing: 6
    Label { text: "x"; color: "#536b67" }
    TextField {
      id: xField
      text: HumanControlPanel.nextSpawnX().toFixed(2)
      Layout.fillWidth: true
    }
    Label { text: "y"; color: "#536b67" }
    TextField {
      id: yField
      text: HumanControlPanel.nextSpawnY().toFixed(2)
      Layout.fillWidth: true
    }
  }

  // ② 3Dビュークリック
  Button {
    Layout.fillWidth: true
    visible: placeByClickRadio.checked
    text: HumanControlPanel.spawnPicking
        ? "スポーン地点選択モード: ON（クリックした数だけ配置）"
        : "スポーン地点選択モードを開始"
    checkable: true
    checked: HumanControlPanel.spawnPicking
    contentItem: Text {
      text: parent.text
      color: "white"
      horizontalAlignment: Text.AlignHCenter
      verticalAlignment: Text.AlignVCenter
      wrapMode: Text.Wrap
    }
    background: Rectangle {
      radius: 6
      color: HumanControlPanel.spawnPicking ? "#e65100" : "#78909c"
    }
    onClicked: HumanControlPanel.setSpawnPicking(!HumanControlPanel.spawnPicking)
  }
  Label {
    Layout.fillWidth: true
    visible: placeByClickRadio.checked && HumanControlPanel.spawnPicking
    wrapMode: Text.Wrap
    text: "クリックした地点に魔法陣が出ます．上の階の床をクリックすれば，" +
          "その階の高さに配置されます（下のリストの3つ目の数字が高さ）．"
    color: "#8aa19c"
    font.pixelSize: 11
  }
  Label {
    Layout.fillWidth: true
    visible: placeByClickRadio.checked
    text: "選択中の地点: " + HumanControlPanel.pendingSpawnPoints.length + "地点" +
        (HumanControlPanel.pendingSpawnPoints.length > 0
            ? "（この数だけ人物が配置されます）" : "")
    color: "#536b67"
    font.pixelSize: 11
  }
  ListView {
    Layout.fillWidth: true
    visible: placeByClickRadio.checked &&
        HumanControlPanel.pendingSpawnPoints.length > 0
    Layout.preferredHeight: Math.min(Math.max(contentHeight, 20), 90)
    clip: true
    model: HumanControlPanel.pendingSpawnPoints
    delegate: Label {
      text: (index + 1) + ": (" + modelData + ")"
      color: "#536b67"
      font.pixelSize: 11
    }
  }
  RowLayout {
    Layout.fillWidth: true
    spacing: 8
    visible: placeByClickRadio.checked &&
        HumanControlPanel.pendingSpawnPoints.length > 0
    Button {
      Layout.fillWidth: true
      text: "1点取り消し"
      onClicked: HumanControlPanel.undoLastSpawnPoint()
    }
    Button {
      Layout.fillWidth: true
      text: "クリア"
      onClicked: HumanControlPanel.clearSpawnPoints()
    }
  }

  GridLayout {
    Layout.fillWidth: true
    columns: 4
    columnSpacing: 6
    Label { text: "z"; color: "#536b67" }
    TextField {
      id: zField
      text: HumanControlPanel.defaultZ(selectedModelIndex).toFixed(2)
      Layout.fillWidth: true
    }
    Label { text: "yaw"; color: "#536b67" }
    TextField { id: yawField; text: "0.0"; Layout.fillWidth: true }
  }

  Button {
    Layout.fillWidth: true
    text: placeByClickRadio.checked
        ? (HumanControlPanel.pendingSpawnPoints.length > 0
            ? "選択した" + HumanControlPanel.pendingSpawnPoints.length + "地点にSpawn"
            : "Spawn（先に地点をクリックしてください）")
        : (countSpin.value > 1 ? "Spawn（" + countSpin.value + "人）" : "Spawn")
    enabled: !placeByClickRadio.checked ||
        HumanControlPanel.pendingSpawnPoints.length > 0
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
    // 地点が選ばれていればそちらが優先され、人数・x/yは無視される
    // （HumanControlPanel::spawnHumans()）。QML側で分岐する必要はない。
    onClicked: HumanControlPanel.spawnHumans(
        selectedModelIndex, nameField.text, countSpin.value,
        HumanControlPanel.isCustomHuman(selectedModelIndex)
            ? HumanControlPanel.posePresets[selectedPoseIndex] : "",
        HumanControlPanel.isActorModel(selectedModelIndex)
            ? HumanControlPanel.followModeValue(selectedFollowModeIndex) : "",
        parseFloat(xField.text) || 0.0, parseFloat(yField.text) || 0.0,
        parseFloat(zField.text) || 0.0, parseFloat(yawField.text) || 0.0)
  }

  Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }

  Label { text: "スポーン済み"; color: "#183b37"; font.bold: true }

  ListView {
    Layout.fillWidth: true
    // Layout.fillHeight doesn't make sense now that the whole panel is
    // inside a ScrollView (the outer content has no bounded height to
    // fill against) -- size to the spawned humans instead, with a floor
    // so an empty list doesn't collapse to nothing, and a cap so a
    // large list scrolls inside its own viewport rather than pushing
    // the panel to an unbounded height. Both this list's own scrolling
    // and the outer ScrollView work together fine since this one only
    // engages once contentHeight exceeds the cap.
    Layout.minimumHeight: 120
    Layout.preferredHeight: Math.min(contentHeight, 320)
    clip: true
    model: HumanControlPanel.humanList
    spacing: 6
    delegate: Rectangle {
      id: humanRow
      // Captured once here because nested signal handlers below re-use
      // the name "index" for their own parameter (ComboBox.activated(int
      // index), Repeater's own delegate-local index, ...), which shadows
      // this ListView delegate's "index" (this human's row) inside those
      // handlers. Referencing humanRow.humanIndex there instead avoids
      // silently reading the wrong index (this bit the viewpoint
      // ComboBox below: HumanControlPanel.setViewpoint(index, ...) was
      // actually passing the combo's own selected-item index as the
      // human index).
      property int humanIndex: index
      width: ListView.view.width
      height: rowContent.implicitHeight + 16
      radius: 6
      color: "#ffffff"
      border.color: HumanControlPanel.activeHumanIndex === index ? "#2e7d32" : "#d7e0e6"
      border.width: HumanControlPanel.activeHumanIndex === index ? 2 : 1

      ColumnLayout {
        id: rowContent
        x: 8; y: 8
        width: parent.width - 16
        spacing: 6

        RowLayout {
          Layout.fillWidth: true
          // その人物のスポーンマーカーと同じ色の四角。3Dビュー上のどの円が
          // この行の人物のものかを、色だけで対応付けられるようにする。
          Rectangle {
            width: 12; height: 12; radius: 3
            color: HumanControlPanel.spawnMarkerColorAt(index)
            border.color: "#7b928d"
            border.width: 1
          }
          Label {
            text: (HumanControlPanel.activeHumanIndex === index ? "⌨ " : "") + modelData
            Layout.fillWidth: true
            elide: Text.ElideRight
            font.bold: true
            color: "#183b37"
          }
          Button {
            text: "対象にする"
            // Any human can be the viewpoint target; only actor-backed
            // ones actually respond to velocity commands, but selecting a
            // static one here still drives the global viewpoint panel
            // below.
            visible: HumanControlPanel.activeHumanIndex !== index
            onClicked: HumanControlPanel.setActiveHuman(index)
          }
          Button {
            text: "削除"
            onClicked: HumanControlPanel.removeHuman(index)
          }
        }

        // 当たり判定カプセルの表示切替え（人物ごと）。実際の表示/非表示は
        // HumanControlPanel::ApplyCollisionVisibility()がRenderイベントで
        // 行う -- humansChanged()発火（setShowCollision()/
        // setShowCollisionAll()の両方が発火させる）のたびにここで再同期する。
        // チェックボックスではなくボタン: クリックのたびに表示⇔非表示を
        // 切り替え、ボタンラベル自体が現在の状態を示す。
        //
        // shownはC++側(showCollisionAt())の写しであって独立した状態では
        // ない。クリック時に自前でトグルせずsetShowCollision()を呼んで
        // からhumansChanged()経由で読み直すのは、「全員表示にする」など
        // 他の経路でC++側が変わったときにボタンの表示がずれないように
        // するため（表示中なのにラベルが「表示」のまま、を防ぐ）。
        RowLayout {
          Layout.fillWidth: true
          spacing: 8
          Button {
            id: collisionToggleButton
            property bool shown: HumanControlPanel.showCollisionAt(humanRow.humanIndex)
            visible: HumanControlPanel.isHumanActorAt(humanRow.humanIndex)
            text: shown ? "当たり判定を非表示" : "当たり判定を表示"
            onClicked: HumanControlPanel.setShowCollision(humanRow.humanIndex, !shown)
            Connections {
              target: HumanControlPanel
              function onHumansChanged() {
                collisionToggleButton.shown =
                    HumanControlPanel.showCollisionAt(humanRow.humanIndex)
              }
            }
          }
          // この人物ひとりぶんのスポーンマーカー表示切替。全員分の一括切替は
          // 下の「初期スポーンマーカー」セクション。collisionToggleButtonと
          // 同じく、押した状態を自前で持たずC++側を読み直すことで、一括操作で
          // 変わったときもラベルがずれないようにしている。
          Button {
            id: markerToggleButton
            property bool shown: HumanControlPanel.showSpawnMarkerAt(humanRow.humanIndex)
            text: shown ? "マーカー非表示" : "マーカー表示"
            onClicked: HumanControlPanel.setShowSpawnMarker(humanRow.humanIndex, !shown)
            Connections {
              target: HumanControlPanel
              function onHumansChanged() {
                markerToggleButton.shown =
                    HumanControlPanel.showSpawnMarkerAt(humanRow.humanIndex)
              }
            }
          }
        }

        // 経路（下の「経路」セクション）を適用する対象かどうか。spawn直後は
        // ONなので、「何人かspawnして経路を描いて歩かせる」だけならここを
        // 触る必要はない。特定の人物だけ別の動きをさせたいときに外す。
        CheckBox {
          id: routeTargetCheck
          checked: HumanControlPanel.routeTargetAt(humanRow.humanIndex)
          text: "経路対象"
          onToggled: HumanControlPanel.setRouteTarget(humanRow.humanIndex, checked)
          Connections {
            target: HumanControlPanel
            function onHumansChanged() {
              routeTargetCheck.checked =
                  HumanControlPanel.routeTargetAt(humanRow.humanIndex)
            }
          }
        }

      }
    }
  }

}
