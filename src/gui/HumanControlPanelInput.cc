// HumanControlPanelInput.cc -- キーボード入力の受け口（eventFilter）。押下/離しを
// HumanControlPanelTeleop.cc 側の Press/ReleaseDirectionKey へ振り分ける。
//
// 3808行あった単一の HumanControlPanel.cc を責務ごとに割ったもの
// （構想書/これからやるやつ/gz_human_sim再設計構想.md §13 段階2）。
// **中身は移動しただけで、振る舞いは変えていない。**
// クラス宣言は HumanControlPanel.hh、ファイル間で共有する表と定数は
// HumanControlPanelInternal.hh にある。

#include "HumanControlPanel.hh"
#include "HumanControlPanelInternal.hh"

// include は分割前の単一ファイルと同じものを揃えてある。責務ごとに
// 削るのは「動くこと」を確認してからでよい（段階2は移動だけ）。
#include "GuiderDeleteRecoverEvent.hh"
#include "GuiderHumanCommandEvent.hh"
#include "GuiderViewpointRequestEvent.hh"

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
// gz/plugin/Register.hh はここでは include しない。**翻訳単位ごとに
// GzPluginHook を定義する**ので、複数の .cc から include すると
// リンクが multiple definition で落ちる。GZ_ADD_PLUGIN を持つ
// HumanControlPanel.cc の1本だけが include する。
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
  else if (_event->type() == guider::events::ViewpointRequest::kType)
  {
    // guide_robot's GuiderPadController (OPTIONS wheel, → 視点). Only
    // "human"-kind requests are ours; GuiderRobotManager's own
    // eventFilter() picks up the "robot" ones from the same broadcast.
    auto *request =
        static_cast<guider::events::ViewpointRequest *>(_event);
    if (request->Kind() == "human")
    {
      const std::string name = request->TargetName().toStdString();
      int index = -1;
      for (std::size_t i = 0; i < this->humans.size(); ++i)
      {
        if (this->humans[i].name == name)
        {
          index = static_cast<int>(i);
          break;
        }
      }
      if (index >= 0)
        this->setViewpoint(index, request->ViewIndex(), 2.0);
    }
  }
  else if (_event->type() == guider::events::DeleteRecoverRequest::kType)
  {
    // OPTIONS wheel's ↙ 削除・復帰. Only "human"-kind requests are ours.
    auto *request =
        static_cast<guider::events::DeleteRecoverRequest *>(_event);
    if (request->Kind() == "human")
    {
      const std::string name = request->TargetName().toStdString();
      int index = -1;
      for (std::size_t i = 0; i < this->humans.size(); ++i)
      {
        if (this->humans[i].name == name)
        {
          index = static_cast<int>(i);
          break;
        }
      }
      if (index < 0)
      {
        // Nothing to log through: name may already be gone (the pad's
        // roster snapshot is up to 2s stale). Silent no-op, same as
        // every other index-miss in this file.
      }
      else if (request->Recover())
      {
        // Confirmed no fall-recovery mechanism exists for humans at all
        // (actors are kinematic, not rigid-body-tumbling -- there is
        // nothing to stand back up). Said out loud rather than silently
        // ignored, so a stray "復帰" on a human reads as "not supported"
        // instead of as a bug in the pad.
        this->SetStatus(
            QString::fromStdString(this->humans.at(index).name) +
            "：人物には「復帰」がありません（転倒しないため）");
      }
      else
      {
        this->removeHuman(index);
      }
    }
  }
  else if (_event->type() == guider::events::HumanCommandRequest::kType)
  {
    this->HandleHumanCommand(
        static_cast<guider::events::HumanCommandRequest *>(_event));
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

void HumanControlPanel::HandleHumanCommand(
    guider::events::HumanCommandRequest *_request)
{
  namespace ev = guider::events;
  const QString command = _request->Command();
  const QVariantMap &args = _request->Args();
  QVariantMap &reply = _request->Reply();

  // 名前 -> 添字は**その場で引く**（HumanRegistry.hh の注意書き。
  // 添字を持ち回ると削除でずれる）。対象を取らない命令では -1 のまま。
  const int index = _request->Target().isEmpty()
      ? -1 : this->humans.IndexOf(_request->Target().toStdString());

  if (command == ev::kCmdModels)
  {
    // spawn できるモデルの一覧は**ここが正本**。パッド側に同じ表を
    // 置くと roster と同じ「片方だけ直して黙って壊れる」に必ずなる
    // （罠4）ので、同期イベントで訊きに来てもらう。
    QStringList labels;
    QVariantList defaultZ;
    QVariantList movable;
    for (int i = 0; i < kHumanModelCount; ++i)
    {
      labels << QString::fromUtf8(kHumanModels[i]);
      defaultZ << kHumanModelDefaultZ[i];
      // 「動かせる」＝アクター実体があること。静止モデル
      // (person_standing/custom_human) は cmd_vel を持たないので、
      // spawn はできても roster には出ない（PublishRoster() が
      // velocityPublisher の無い人物を落とす）。パッドはこれを見て
      // 「この機種は動かせません」と先に言える。
      movable << IsActorIndex(i);
    }
    reply["labels"] = labels;
    reply["defaultZ"] = defaultZ;
    reply["movable"] = movable;
    _request->SetHandled(true);
    return;
  }

  if (command == ev::kCmdSpawn)
  {
    const int model = args.value("model", 0).toInt();
    if (model < 0 || model >= kHumanModelCount)
      return;   // Handled() は立てない ＝ パッドが「不明なモデル」と言える

    // 名前はこちらが決める。defaultName() は humans.size()+1 を返すだけ
    // なので、削除で穴が空いた一覧では既存名とぶつかる。ぶつかったら
    // 空くまで送るのはこちらの仕事で、パッドに知らせる筋のものではない
    // （spawnHuman() は重複を**拒否**する）。
    QString name;
    for (int suffix = static_cast<int>(this->humans.size()) + 1;
         suffix < 10000; ++suffix)
    {
      const QString candidate = QString("human%1").arg(suffix);
      if (this->humans.IndexOf(candidate.toStdString()) < 0)
      {
        name = candidate;
        break;
      }
    }
    if (name.isEmpty())
      return;

    // z はパッドが送ってきた**床の高さ**。モデルごとの接地オフセットは
    // こちら（kHumanModelDefaultZ）しか知らないので、ここで足す。
    const double z = args.value("z", 0.0).toDouble() + kHumanModelDefaultZ[model];
    this->spawnHuman(model, name, QString(),
        QString::fromUtf8(kFollowModeValues[0]),
        args.value("x", 0.0).toDouble(), args.value("y", 0.0).toDouble(),
        z, args.value("yaw", 0.0).toDouble());
    reply["name"] = name;
    _request->SetHandled(true);
    return;
  }

  if (command == ev::kCmdSpeed)
  {
    this->setSpeedMultiplier(args.value("value", 1.0).toDouble());
    reply["value"] = this->speedMultiplierState;
    _request->SetHandled(true);
    return;
  }

  if (command == ev::kCmdSpeedQuery)
  {
    reply["value"] = this->speedMultiplierState;
    _request->SetHandled(true);
    return;
  }

  // ここから下は対象が要る。名前が引けなければ**未処理のまま返す**
  // （パッドの roster は最大2秒古いので、消えた人物への指令は普通に来る）。
  if (index < 0)
    return;

  if (command == ev::kCmdJump)
  {
    this->teleopJump(index);
    _request->SetHandled(true);
    return;
  }

  if (command == ev::kCmdFollowMode || command == ev::kCmdFollowModeQuery)
  {
    if (command == ev::kCmdFollowMode)
    {
      const std::string mode = args.value("mode").toString().toStdString();
      for (int i = 0; i < kFollowModeCount; ++i)
      {
        if (mode == kFollowModeValues[i])
        {
          this->setFollowMode(index, i);
          break;
        }
      }
    }
    reply["mode"] = QString::fromUtf8(
        kFollowModeValues[this->humans.at(index).followModeIndex]);
    _request->SetHandled(true);
    return;
  }

  if (command == ev::kCmdWaypoint)
  {
    // 経路は cmd_path 経由。follow_mode が "auto" でも "path" でも
    // ActorCommandPlugin は経路を歩く（"velocity" だけが無視する）ので、
    // ここでモードを勝手に変えない。
    this->sendWaypoint(index, args.value("x", 0.0).toDouble(),
        args.value("y", 0.0).toDouble());
    _request->SetHandled(true);
    return;
  }

  if (command == ev::kCmdShowCollision || command == ev::kCmdShowCollisionQuery)
  {
    if (command == ev::kCmdShowCollision)
      this->setShowCollision(index, args.value("on", false).toBool());
    reply["on"] = this->showCollisionAt(index);
    _request->SetHandled(true);
    return;
  }

  if (command == ev::kCmdStateQuery)
  {
    // **サーバーが言っている値**（構想書 §3・罠14）。GUI の推測ではない。
    reply["text"] = this->humanStateLabel(index);
    _request->SetHandled(true);
    return;
  }
}
}  // namespace gz_human_sim
