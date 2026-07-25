import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

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

  property int selectedModelIndex: 0
  property string spawnName: HumanControlPanel.defaultName(0)
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

    // ── Spawn ──────────────────────────────────────────────
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
        spawnName = HumanControlPanel.defaultName(currentIndex)
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
    // what, ignoring a stray teleop press aimed at it by mistake.
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

    Row {
      Layout.fillWidth: true
      spacing: 8
      Label { text: "名前"; width: 40; anchors.verticalCenter: parent.verticalCenter; color: "#536b67" }
      TextField {
        id: nameField
        width: parent.width - 48
        text: spawnName
        onTextEdited: spawnName = text
        placeholderText: "human1"
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
      text: "Spawn"
      contentItem: Text {
        text: parent.text
        color: "white"
        font.bold: true
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
      }
      background: Rectangle {
        radius: 6
        color: parent.pressed ? "#2e7d32" : "#43a047"
      }
      onClicked: HumanControlPanel.spawnHuman(
          selectedModelIndex, nameField.text,
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
              // ones actually respond to QWEASDZXC teleop, but selecting a
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

          // Teleop pad: press-and-hold buttons publish Twist while pressed
          // and stop on release. Layout mirrors the QWEASDZXC keyboard
          // shortcuts in HumanControlPanel::eventFilter() (Q W E / A S D /
          // Z X C, S = stop) so mouse and keyboard drive this human the
          // same way; the keyboard shortcuts always target whichever human
          // is マークed "操作対象" (⌨) above, not necessarily this row.
          GridLayout {
            id: teleopPad
            // The ListView delegate's own `index` (this human's row) would
            // otherwise be shadowed by the Repeater below's delegate-local
            // `index` (0-8, the direction-button's own position) -- capture
            // it under a distinct name before entering that inner scope.
            property int humanIndex: index
            visible: HumanControlPanel.isHumanActorAt(humanIndex)
            columns: 3
            rowSpacing: 2
            columnSpacing: 2

            // Repeater over the QWEASDZXC layout instead of 9 near-identical
            // Button blocks. Each Button gets its own contentItem (plain,
            // non-eliding, wrapping Text) rather than the QQC2 default
            // style's single-line elided Text -- that default silently
            // collapsed our two-line "Q\n↖" labels down to just "…" at this
            // button size.
            Repeater {
              model: [
                {key: "Q", glyph: "↖"}, {key: "W", glyph: "↑"}, {key: "E", glyph: "↗"},
                {key: "A", glyph: "←"}, {key: "N", glyph: "■"}, {key: "D", glyph: "→"},
                {key: "Z", glyph: "↙"}, {key: "X", glyph: "↓"}, {key: "C", glyph: "↘"},
              ]
              delegate: Button {
                Layout.preferredWidth: 40
                Layout.preferredHeight: 40
                contentItem: Text {
                  text: modelData.key + "\n" + modelData.glyph
                  horizontalAlignment: Text.AlignHCenter
                  verticalAlignment: Text.AlignVCenter
                  wrapMode: Text.WordWrap
                  font.pixelSize: 12
                }
                // Movement keys turn to face where they're walking by
                // default now (see teleopDirection()'s _turnToFace),
                // matching the keyboard's diagonal-key branch: J held
                // switches back to the old no-turn strafe. S-held-click on
                // A/D instead spins in place (teleopRotate()), same as
                // S+A/S+D from the keyboard.
                onPressed: {
                    if (modelData.key === "N") return
                    if (HumanControlPanel.sHeld && (modelData.key === "A" || modelData.key === "D"))
                        HumanControlPanel.teleopRotate(teleopPad.humanIndex, modelData.key === "A")
                    else
                        HumanControlPanel.teleopDirection(teleopPad.humanIndex, modelData.key,
                            !HumanControlPanel.jHeld)
                }
                onReleased: HumanControlPanel.teleopStop(teleopPad.humanIndex)
                onClicked: if (modelData.key === "N")
                    HumanControlPanel.teleopStop(teleopPad.humanIndex)
              }
            }
          }
        }
      }
    }

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

    // ── 移動パラメーター（ジャンプ・速度）─────────────────
    // ジャンプ高さ・基準移動速度はキー操作対象(activeHumanIndex)にかかる
    // グローバル設定 -- shiftHeld/ctrlHeld/sHeld/jHeld と同じ扱いで、対象人物を
    // 切り替えても値は維持される。基準速度はCtrl（ゆっくり歩く）/Shift
    // （走る）を押していない、通常の移動時の速度。Ctrl/Shiftを押すと
    // この基準速度に対してさらに倍率がかかる。
    Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
    Label { text: "移動パラメーター"; color: "#183b37"; font.bold: true }
    RowLayout {
      Layout.fillWidth: true
      spacing: 4
      Label { text: "ジャンプの高さ[m]"; color: "#536b67" }
      Slider {
        id: jumpHeightSlider
        Layout.fillWidth: true
        from: 0.05
        to: 1.5
        Component.onCompleted: value = HumanControlPanel.jumpHeight
        onMoved: HumanControlPanel.setJumpHeight(value)
      }
      Label {
        text: HumanControlPanel.jumpHeight.toFixed(2)
        color: "#536b67"
        Layout.preferredWidth: 34
      }
    }
    RowLayout {
      Layout.fillWidth: true
      spacing: 8
      Label { text: "基準移動速度"; color: "#536b67" }
      Slider {
        id: speedMultiplierSlider
        Layout.fillWidth: true
        from: 0.1
        to: 4.0
        Component.onCompleted: value = HumanControlPanel.speedMultiplier
        onMoved: HumanControlPanel.setSpeedMultiplier(value)
      }
      Label {
        text: (HumanControlPanel.ctrlHeld ? "遅い" :
            (HumanControlPanel.shiftHeld ? "走る" : "歩く")) +
            "（x" + HumanControlPanel.speedMultiplier.toFixed(2) + "）"
        color: "#536b67"
        Layout.preferredWidth: 92
      }
      Button {
        text: "⤴ ジャンプ"
        enabled: HumanControlPanel.activeHumanIndex >= 0 &&
            HumanControlPanel.isHumanActorAt(HumanControlPanel.activeHumanIndex)
        onClicked: HumanControlPanel.teleopJump(HumanControlPanel.activeHumanIndex)
      }
    }
    Connections {
      target: HumanControlPanel
      function onJumpHeightChanged() {
        jumpHeightSlider.value = HumanControlPanel.jumpHeight
      }
      function onSpeedMultiplierChanged() {
        speedMultiplierSlider.value = HumanControlPanel.speedMultiplier
      }
    }

    // ── Follow mode (runtime) ──────────────────────────────
    // Changes an already-spawned human's follow_mode live, via
    // ActorCommandPlugin's follow_mode_topic -- separate from the spawn
    // form's "動作モード" combo above, which only sets the initial value at
    // spawn time and has no effect on a human that's already walking
    // around. Same "対象" (activeHumanIndex) as viewpoint/path templates.
    Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
    Label { text: "動作モード（実行中に変更）"; color: "#183b37"; font.bold: true }
    RowLayout {
      Layout.fillWidth: true
      spacing: 4
      Label { text: "モード"; color: "#536b67" }
      ComboBox {
        id: runtimeFollowModeCombo
        Layout.fillWidth: true
        enabled: HumanControlPanel.activeHumanIndex >= 0 &&
            HumanControlPanel.isHumanActorAt(HumanControlPanel.activeHumanIndex)
        model: HumanControlPanel.followModeLabels
        Component.onCompleted: currentIndex = HumanControlPanel.activeFollowModeIndex
        onActivated: HumanControlPanel.setFollowMode(
            HumanControlPanel.activeHumanIndex, currentIndex)
      }
    }
    Connections {
      target: HumanControlPanel
      function onActiveFollowModeChanged() {
        runtimeFollowModeCombo.currentIndex = HumanControlPanel.activeFollowModeIndex
      }
    }

    // ── 姿勢（walking_actorのみ） ──────────────────────────
    // Lキーのマウス版。押している間だけのポーズキー（K＝着席）とは別に、
    // 「いまの姿勢を登録」して押しっぱなしをやめても保持させるボタン。
    // 現在の姿勢そのものを登録する方式なので、今後ポーズが増えても
    // このボタンとLキーは変更なしでそのまま使える
    // -- HumanControlPanel::togglePoseLock()を参照。
    Rectangle {
      Layout.fillWidth: true; height: 1; color: "#c7d8d4"
      visible: HumanControlPanel.activeHumanIndex >= 0 &&
          HumanControlPanel.isPoseCapableHumanAt(HumanControlPanel.activeHumanIndex)
    }
    RowLayout {
      Layout.fillWidth: true
      spacing: 8
      visible: HumanControlPanel.activeHumanIndex >= 0 &&
          HumanControlPanel.isPoseCapableHumanAt(HumanControlPanel.activeHumanIndex)
      Label { text: "姿勢"; color: "#183b37"; font.bold: true }
      Label {
        // 「いま何をしているか」＝登録済みならそれ、なければ押されている
        // ポーズキーのもの。登録の有無はボタン側のラベルが示す。
        text: HumanControlPanel.poseLabel(
            HumanControlPanel.activeLockedPose !== ""
                ? HumanControlPanel.activeLockedPose : HumanControlPanel.heldPose)
        color: "#536b67"
      }
      Button {
        id: poseLockButton
        Layout.fillWidth: true
        text: HumanControlPanel.activeLockedPose !== ""
            ? "姿勢の登録を解除" : "現在の姿勢を登録"
        onClicked: HumanControlPanel.togglePoseLock(HumanControlPanel.activeHumanIndex)
      }
    }

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

    // ── Path templates ─────────────────────────────────────
    // Same "対象" (activeHumanIndex) as the viewpoint block above -- one
    // global control, not one per row. New shapes only ever need a new
    // entry in HumanControlPanel.pathTemplateLabels/sendPathTemplate(), so
    // this combo grows automatically as gz_human_sim gains more of them;
    // nothing here needs to change.
    Rectangle { Layout.fillWidth: true; height: 1; color: "#c7d8d4" }
    Label { text: "経路テンプレート"; color: "#183b37"; font.bold: true }
    RowLayout {
      Layout.fillWidth: true
      spacing: 4
      Label { text: "形状"; color: "#536b67" }
      ComboBox {
        id: pathTemplateCombo
        Layout.fillWidth: true
        enabled: HumanControlPanel.activeHumanIndex >= 0
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
        text: "この経路で歩かせる"
        enabled: HumanControlPanel.activeHumanIndex >= 0
        onClicked: HumanControlPanel.sendPathTemplate(
            HumanControlPanel.activeHumanIndex, pathTemplateCombo.currentIndex,
            parseFloat(pathCenterXField.text) || 0.0,
            parseFloat(pathCenterYField.text) || 0.0,
            parseFloat(pathSizeField.text) || 2.0,
            parseInt(pathWaypointsField.text) || 12,
            pathClockwiseCheck.checked)
      }
    }

    Label {
      Layout.fillWidth: true
      text: "walking_actor / DoctorFemaleWalk は移動パッドで操作できます" +
          "（/<名前>/cmd_vel, /<名前>/cmd_path, /<名前>/cmd_jump をgz-transportで" +
          "直接publish）。"
      color: "#7b928d"
      font.pixelSize: 11
      wrapMode: Text.Wrap
    }
    Label {
      Layout.fillWidth: true
      text: "キーボードでも操作可: Q W E / A D / Z X C の8方向キー、Nで停止。" +
          "移動すると体もその方向を向きます。Ctrlを押しながら移動でゆっくり歩き、" +
          "Shiftを押しながら移動で走ります（上のスライダーの基準速度に対して" +
          "倍率がかかります。移動中に後からCtrl/Shiftを押しても即座に反映され" +
          "ます）。Jを押しながら移動すると、体を正面に向けたまま横や後ろへ進む" +
          "従来のストレイフ移動になります。1〜9キーで⌨操作対象を切替。"
      color: "#7b928d"
      font.pixelSize: 11
      wrapMode: Text.Wrap
    }
    Label {
      Layout.fillWidth: true
      text: "Sを押しながらA/Dでその場回転します（Aで左回り、Dで右回り）。" +
          "Ctrl/Shiftを押しながらだと回転速度も変わります。"
      color: "#7b928d"
      font.pixelSize: 11
      wrapMode: Text.Wrap
    }
    Label {
      Layout.fillWidth: true
      text: "Enterキーでその場ジャンプ（移動キーを押しながらだとその方向に進みつつ" +
          "ジャンプ）。空中でもう一度Enterを押すと二段ジャンプできます。" +
          "ジャンプの高さは上のスライダーで調整できます。"
      color: "#7b928d"
      font.pixelSize: 11
      wrapMode: Text.Wrap
    }
    Label {
      Layout.fillWidth: true
      visible: HumanControlPanel.activeHumanIndex >= 0 &&
          HumanControlPanel.isPoseCapableHumanAt(HumanControlPanel.activeHumanIndex)
      text: "Kを押している間だけ座ります（離すと立ちます）。Lを押すと" +
          "そのときの姿勢が登録され、Kを離しても保持されます。" +
          "もう一度Lを押すと登録を解除します（上の「現在の姿勢を登録」" +
          "ボタンでも同じ操作ができます）。立ったままLを押せば直立で" +
          "固定され、Kを押しても座らなくなります。"
      color: "#7b928d"
      font.pixelSize: 11
      wrapMode: Text.Wrap
    }
    }
  }
}
