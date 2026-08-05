#include "HumanControlPanel.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <dlfcn.h>
#include <fstream>
#include <functional>
#include <memory>
#include <regex>
#include <sstream>
#include <utility>
#include <vector>

#include <QGuiApplication>
#include <QKeyEvent>
#include <QPointer>
#include <QTimer>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>


#include "HumanControlPanelInternal.hh"

// Keyboard and mouse-click routing (the gz-gui event filter).
//
// Split out of the single-file HumanControlPanel.cc; the code below is
// unchanged from that file.
//
// ---------------------------------------------------------------------------
// キーボード操作はこのパッケージから削除済みです
// ---------------------------------------------------------------------------
// W/A/D/X、Q/E/Z/C、S/J/N/L/K、Shift/Ctrl、数字キーによる対象切替、
// Enter によるジャンプ -- いずれも削除しました。人物の操作は
// unified_entity_control（ゲームパッド）と、これから作るキーボード用
// パッケージが担当します。経緯は 構想書/gz_human_sim再設計構想.md §11。
//
// ここに残っているのは LeftClickToScene の分岐だけです。経路の waypoint
// 打ちとスポーン位置のクリックは「操作」ではなく「世界の編集」なので、
// このパッケージの仕事です。
//
// キーボード操作を新パッケージへ実装するときの注意（ここにあった知見）:
//   - IsTextEditFocused() 相当の除外を必ず入れること。無いと名前欄に
//     "human1" と打つだけで人物が走り出します。
//   - isAutoRepeat() の除外も同様。
//   - 押しっぱなし（Held）だけでなく Pressed / Released も区別すること。
//     トグル操作（例: ○で着席・起立）は Held だけでは書けません。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
bool HumanControlPanel::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->cameraController.ApplyPending();
    this->ApplyCollisionVisibility();
    this->ApplySpawnMarkers();
  }
  else if (_event->type() == gz::gui::events::LeftClickToScene::kType)
  {
    // Route-recording mode (setRouteRecording()): every left click resolved
    // to a 3D scene point becomes one waypoint of whichever human is
    // currently activeHumanIndex -- same "targets whatever is 対象 right
    // now" convention the movement keys already use, not a separate
    // per-row selection of its own. Silently does nothing when
    // routeRecordingState is off, so this never steals an ordinary
    // camera-orbit click.
    auto *clickEvent = static_cast<gz::gui::events::LeftClickToScene *>(_event);
    const auto point = clickEvent->Point();
    if (this->routeRecordingState)
    {
      // Route recording appends to the one shared route (see the
      // pendingRoutePoints Q_PROPERTY) -- no longer tied to whichever human
      // happens to be 対象, since the route is applied to a whole set of
      // them at confirmRoute() time.
      this->pendingRoute.emplace_back(point.X(), point.Y());
      this->pendingRouteChanged();
      this->SetStatus(QString("経由点を追加しました: (%1, %2)")
          .arg(point.X(), 0, 'f', 2).arg(point.Y(), 0, 'f', 2));
    }
    else if (this->spawnPickingState)
    {
      // Points accumulate so one Spawn press can create a whole group, each
      // person where they were clicked. Z is kept as well as X/Y: the click
      // reports the height of the surface it hit, which is what lets a
      // click on an upper floor actually put someone on that floor.
      this->pendingSpawnPoints.push_back(
          PickedPoint{point.X(), point.Y(), point.Z()});
      this->spawnMarkers.InvalidatePending();
      this->pendingSpawnPointsChanged();
      this->SetStatus(QString("スポーン地点%1: (%2, %3, %4)")
          .arg(this->pendingSpawnPoints.size())
          .arg(point.X(), 0, 'f', 2).arg(point.Y(), 0, 'f', 2)
          .arg(point.Z(), 0, 'f', 2));
    }
    // Neither mode on: deliberately does nothing, leaving ordinary clicking
    // in the viewport (selection, camera work) alone -- see the
    // routeRecording Q_PROPERTY's comment.
  }
  else if (_event->type() == QEvent::KeyPress || _event->type() == QEvent::KeyRelease)
  {
    auto *keyEvent = static_cast<QKeyEvent *>(_event);
    const bool pressed = _event->type() == QEvent::KeyPress;

    // Shift/Ctrl/S/J は「押されているか」をそのまま持つ。modifiers() を
    // 方向キーが飛んだ瞬間に読む方式にしないのは、QML のパッドが
    // shiftHeld/ctrlHeld/sHeld/jHeld を表示に使うためと、S が Qt の
    // modifier ではないため。IsTextEditFocused() で塞がないのも意図的で、
    // 名前欄にフォーカスがあっても表示だけは正しくしておく。
    if (keyEvent->key() == Qt::Key_Shift && !keyEvent->isAutoRepeat() &&
        pressed != this->directionKeys.Run())
    {
      this->directionKeys.SetRun(pressed);
      this->shiftHeldChanged();
      // 押しっぱなしの途中で Shift を足したらその場で走り出す。W を離して
      // 押し直す必要はない。
      this->RefreshHeldMovement();
    }
    else if (keyEvent->key() == Qt::Key_Control && !keyEvent->isAutoRepeat() &&
        pressed != this->directionKeys.Slow())
    {
      this->directionKeys.SetSlow(pressed);
      this->ctrlHeldChanged();
      this->RefreshHeldMovement();
    }
    else if (keyEvent->key() == Qt::Key_S && !keyEvent->isAutoRepeat() &&
        pressed != this->sHeldState)
    {
      this->sHeldState = pressed;
      this->sHeldChanged();
    }
    else if (keyEvent->key() == Qt::Key_J && !keyEvent->isAutoRepeat() &&
        pressed != this->directionKeys.Strafe())
    {
      this->directionKeys.SetStrafe(pressed);
      this->jHeldChanged();
      // J はストレイフの切替。押しっぱなしの最中でも即座に切り替わる。
      this->RefreshHeldMovement();
    }
    else if (!keyEvent->isAutoRepeat() && PoseShortcutForKey(keyEvent->key()))
    {
      // 押している間だけその姿勢（K = 着席）。ただし L で登録済みの姿勢が
      // ある人物では登録側が勝つ（UpdatePoseIntent() 参照）。
      const std::string pose =
          pressed ? PoseShortcutForKey(keyEvent->key())->pose : std::string();
      if (pose != this->heldPoseState)
      {
        this->heldPoseState = pose;
        this->heldPoseChanged();
        this->UpdatePoseIntent(this->activeHumanIndex);
      }
    }

    if (!keyEvent->isAutoRepeat() && !IsTextEditFocused())
    {
      if (pressed && keyEvent->key() == Qt::Key_L)
      {
        // L は「いまの姿勢を登録」。もう一度押すと解除。QML の
        // 「現在の姿勢を登録」ボタンと同じ Q_INVOKABLE を呼ぶ。
        this->togglePoseLock(this->activeHumanIndex);
      }
      else if (pressed && keyEvent->key() >= Qt::Key_1 && keyEvent->key() <= Qt::Key_9)
      {
        this->setActiveHuman(keyEvent->key() - Qt::Key_1);
      }
      else if (pressed &&
          (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter))
      {
        // その場ジャンプ。方向キーを押していれば、その方向へ進みながら跳ぶ
        // （teleopJump() は Z 方向を足すだけで、水平の指令はそのまま）。
        this->teleopJump(this->activeHumanIndex);
      }
      else if (pressed && keyEvent->key() == Qt::Key_N)
      {
        // 即停止。これは以前 S の役目だったが、S はその場回転の
        // モディファイアになった。
        this->directionKeys.Clear();
        this->teleopStop(this->activeHumanIndex);
      }
      else
      {
        std::string direction;
        switch (keyEvent->key())
        {
          case Qt::Key_Q: direction = "Q"; break;
          case Qt::Key_W: direction = "W"; break;
          case Qt::Key_E: direction = "E"; break;
          case Qt::Key_A: direction = "A"; break;
          case Qt::Key_D: direction = "D"; break;
          case Qt::Key_Z: direction = "Z"; break;
          case Qt::Key_X: direction = "X"; break;
          case Qt::Key_C: direction = "C"; break;
          default: break;
        }
        if (!direction.empty())
        {
          if (this->sHeldState && (direction == "A" || direction == "D"))
          {
            // S+A / S+D はその場回転（A が左回り、D が右回り）。
            this->directionKeys.Clear();
            if (pressed)
              this->teleopRotate(this->activeHumanIndex, direction == "A");
            else
              this->teleopStop(this->activeHumanIndex);
          }
          else if (IsSteerableDirection(direction))
          {
            // W/A/D/X は押しっぱなしの状態機械へ。単独なら向きを変えて
            // 歩き、2 つ目が加われば曲がる。J 中はストレイフ。
            if (pressed)
              this->PressDirectionKey(direction);
            else
              this->ReleaseDirectionKey(direction);
          }
          else if (pressed)
          {
            // 斜め（Q/E/Z/C）は単発。既定は向きを変えて歩く、J 中は
            // 従来のストレイフ。
            this->directionKeys.Clear();
            this->teleopDirection(this->activeHumanIndex,
                QString::fromStdString(direction), !this->directionKeys.Strafe());
          }
          else
          {
            this->directionKeys.Clear();
            this->teleopStop(this->activeHumanIndex);
          }
        }
      }
    }
  }

  return QObject::eventFilter(_obj, _event);
}
}  // namespace gz_human_sim
