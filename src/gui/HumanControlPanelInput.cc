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

// DualSense/gamepad polling only -- SDL_INIT_GAMECONTROLLER (never
// SDL_INIT_VIDEO), so this never touches windowing/GL and cannot conflict
// with the already-running Ogre2/Qt scene. See PollDualsense(). Mirrors
// guide_robot's GuiderRobotManager, which this mode is modeled on.
#include <SDL2/SDL.h>

#include "HumanControlPanelInternal.hh"

// Keyboard and mouse-click routing (the gz-gui event filter).
//
// Split out of the single-file HumanControlPanel.cc; the code below is
// unchanged from that file.
//
// ---------------------------------------------------------------------------
// このファイルのキーボード部分は「キーボード操作用の新パッケージ」へ移管予定です
// ---------------------------------------------------------------------------
// 移管先: 未作成（unified_entity_control とは別のパッケージ）
//   eventFilter() のキー押下/離上の分岐一式（W/A/D/X、Q/E/Z/C、S/J/N/L/K、
//   Shift/Ctrl、数字キーによる対象切替、Enter によるジャンプ）
//
// 移管しないもの（gz_human_sim に残る）:
//   LeftClickToScene の分岐 -- 経路の waypoint 打ちとスポーン位置のクリックは
//   「操作」ではなく「世界の編集」なので、このパッケージに残ります。
//
// 移管時の注意:
//   - IsTextEditFocused() の除外は必ず引き継ぐこと。これが無いと名前欄に
//     "human1" と打つだけで人物が走り出します。
//   - isAutoRepeat() の除外も同様。
//   - 現状は押しっぱなし（Held）しか見ていません。移管先では
//     Pressed / Held / Released を区別してください。トグル操作
//     （例: ○で着席・起立）は現状の作りでは書けません。
//
// それまでは、このファイルはこのまま動き続けます。先に消さないでください。
// 経緯は 構想書/gz_human_sim再設計構想.md §11 を参照。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
bool HumanControlPanel::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->ApplyViewpoint();
    this->ApplyCollisionVisibility();
    this->ApplySpawnMarkers();
    this->PollDualsense();
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
      this->pendingSpawnMarkersDirty = true;
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
    // Shift/Ctrl/S are tracked as plain held-state (not read off
    // modifiers()/a per-key switch at the moment a direction key fires) so
    // QML buttons can also read shiftHeld/ctrlHeld/sHeld/jHeld live for
    // mouse-driven clicks, and so S -- not a real Qt modifier -- can be
    // tracked the same way as Shift/Ctrl. Not gated on IsTextEditFocused()
    // (unlike the direction dispatch below) so these stay accurate even
    // while a name/x/y/z field has focus elsewhere in the panel.
    if (keyEvent->key() == Qt::Key_Shift && !keyEvent->isAutoRepeat() &&
        pressed != this->shiftHeldState)
    {
      this->shiftHeldState = pressed;
      this->shiftHeldChanged();
      // Live speed/mode react to Shift toggling mid-hold -- e.g. holding W
      // and THEN pressing Shift starts running immediately, no need to
      // release and re-press W. See RefreshHeldMovementSpeed().
      this->RefreshHeldMovementSpeed();
    }
    else if (keyEvent->key() == Qt::Key_Control && !keyEvent->isAutoRepeat() &&
        pressed != this->ctrlHeldState)
    {
      this->ctrlHeldState = pressed;
      this->ctrlHeldChanged();
      this->RefreshHeldMovementSpeed();
    }
    else if (keyEvent->key() == Qt::Key_S && !keyEvent->isAutoRepeat() &&
        pressed != this->sHeldState)
    {
      this->sHeldState = pressed;
      this->sHeldChanged();
    }
    else if (keyEvent->key() == Qt::Key_J && !keyEvent->isAutoRepeat() &&
        pressed != this->jHeldState)
    {
      this->jHeldState = pressed;
      this->jHeldChanged();
      // J is the strafe-mode modifier -- live-switch a currently-held key
      // between turn-to-face and strafe the same way Shift/Ctrl live-swap
      // speed above.
      this->RefreshHeldMovementSpeed();
    }
    else if (!keyEvent->isAutoRepeat() && PoseShortcutForKey(keyEvent->key()))
    {
      // Hold-to-pose (K = sit, see kPoseShortcuts): pressing publishes that
      // pose's name, releasing publishes "no pose" again -- unless the
      // active human has a pose REGISTERED, in which case the registered one
      // keeps winning (see UpdatePoseIntent()). Tracked as plain held-state,
      // same as Shift/Ctrl/S/J above, so QML can read heldPose too.
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
        // L registers the active human's CURRENT pose, so it keeps holding
        // it once the pose key is released; pressing L again unregisters.
        // Same Q_INVOKABLE the QML button uses.
        //
        // Note this is "register whatever pose is happening now", not "sit"
        // -- which is why it's a separate key from K rather than a modifier
        // on it, and why it keeps working unchanged as more poses are added.
        this->togglePoseLock(this->activeHumanIndex);
      }
      else if (pressed && keyEvent->key() >= Qt::Key_1 && keyEvent->key() <= Qt::Key_9)
      {
        this->setActiveHuman(keyEvent->key() - Qt::Key_1);
      }
      else if (pressed &&
          (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter))
      {
        // Jump in place, or -- if a direction key is still held -- while
        // continuing to move that way (teleopJump() only adds a Z arc, the
        // held key(s)' horizontal Twist keeps applying unchanged).
        this->teleopJump(this->activeHumanIndex);
      }
      else if (pressed && keyEvent->key() == Qt::Key_N)
      {
        // Immediate stop -- this is S's old job; S is now the
        // turn-to-face modifier (see sHeldState above/teleopDirection()).
        this->heldDirectionKeys.clear();
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
            // S+A / S+D: spin in place (A = counterclockwise, D =
            // clockwise) instead of walking -- see teleopRotate(). This
            // is S's whole remaining job now that plain movement below
            // always turns to face where it's going anyway.
            this->heldDirectionKeys.clear();
            if (pressed)
              this->teleopRotate(this->activeHumanIndex, direction == "A");
            else
              this->teleopStop(this->activeHumanIndex);
          }
          else if (IsSteerableDirection(direction))
          {
            // W/A/D/X go through the held-key tracker: PressDirectionKey()/
            // ReleaseDirectionKey() decide there whether a solo key turns
            // to face and walks, or (J strafe mode, or a 2nd key joining
            // for the curving combo) falls through to the old strafe/curve
            // Twist via ApplyHeldDirectionKeys().
            if (pressed)
              this->PressDirectionKey(direction);
            else
              this->ReleaseDirectionKey(direction);
          }
          else if (pressed)
          {
            // Diagonals (Q/E/Z/C): always single-shot immediate. Turn to
            // face and walk by default, same as W/A/D/X; J held switches
            // to the original no-turn strafe (see PressDirectionKey()'s
            // matching override).
            this->heldDirectionKeys.clear();
            const bool turnToFace = !this->jHeldState;
            this->teleopDirection(
                this->activeHumanIndex, QString::fromStdString(direction), turnToFace);
          }
          else
          {
            this->heldDirectionKeys.clear();
            this->teleopStop(this->activeHumanIndex);
          }
        }
      }
    }
  }
  return QObject::eventFilter(_obj, _event);
}
}  // namespace gz_human_sim
