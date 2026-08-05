/*
 * HelpSection -- キーボード操作の説明。表示専用。
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
