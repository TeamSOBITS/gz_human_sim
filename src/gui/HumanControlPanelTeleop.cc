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

// Teleoperation: velocity/turn commands, the held-direction-key steering
// state machine, and named-pose intent.
//
// Split out of the single-file HumanControlPanel.cc; the code below is
// unchanged from that file.

namespace gz_human_sim
{
double HumanControlPanel::EffectiveSpeedMultiplier() const
{
  const double factor = this->ctrlHeldState
      ? kSlowFactor : (this->shiftHeldState ? kRunFactor : 1.0);
  return this->speedMultiplierState * factor;
}

void HumanControlPanel::teleopJump(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.jumpPublisher.Valid())
    return;

  gz::msgs::Double message;
  message.set_data(this->jumpHeightState);
  human.jumpPublisher.Publish(message);
}

void HumanControlPanel::teleopRotate(int _index, bool _counterClockwise)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  // Pure-angular Twist -- no linear/lateral -- so the actor spins on the
  // spot rather than walking. Sign matches DirectionToTwist()'s A/D
  // convention (A: +lateral -> +yaw -> counterclockwise; D: -lateral ->
  // -yaw -> clockwise), same as ApplyHeldDirectionKeys()'s steering sign.
  const double angular =
      (_counterClockwise ? kTeleopTurnRate : -kTeleopTurnRate) *
      this->EffectiveSpeedMultiplier();
  ++this->humans.at(_index).teleopGeneration;
  this->teleopMove(_index, 0.0, 0.0, angular);
}

void HumanControlPanel::togglePoseLock(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.posePublisher.Valid())
    return;

  if (!human.lockedPose.empty())
  {
    // Already registered -- unregister, dropping back to whatever a held key
    // is asking for right now (nothing, usually, so the human stands up).
    const QString previous = this->poseLabel(
        QString::fromStdString(human.lockedPose));
    human.lockedPose.clear();
    this->SetStatus(QString::fromStdString(human.name) + " の姿勢登録（" +
        previous + "）を解除しました");
  }
  else
  {
    // Register whatever pose is happening right now. Registering the
    // no-pose (standing) state is allowed and meaningful: it pins the human
    // upright, so a pose key held afterwards is ignored until it's
    // unregistered.
    human.lockedPose = this->EffectivePose(_index);
    this->SetStatus(QString::fromStdString(human.name) + " の現在の姿勢（" +
        this->poseLabel(QString::fromStdString(human.lockedPose)) +
        "）を登録しました");
  }
  if (_index == this->activeHumanIndex)
    this->activeLockedPoseChanged();
  this->UpdatePoseIntent(_index);
}

std::string HumanControlPanel::EffectivePose(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return {};
  const auto &human = this->humans.at(_index);
  // Registered wins over held: once a pose is registered with L, this human
  // holds it regardless of what key the operator is leaning on -- including
  // a registered "standing", which is how you deliberately stop a human from
  // responding to K at all.
  if (!human.lockedPose.empty())
    return human.lockedPose;
  // A held key only ever steers the ACTIVE human, same as every other
  // keyboard shortcut in this panel.
  if (_index == this->activeHumanIndex)
    return this->heldPoseState;
  return {};
}

void HumanControlPanel::UpdatePoseIntent(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.posePublisher.Valid())
    return;

  gz::msgs::StringMsg message;
  message.set_data(this->EffectivePose(_index));
  human.posePublisher.Publish(message);
}

void HumanControlPanel::teleopMove(
    int _index, double _linear, double _lateral, double _angular)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;

  gz::msgs::Twist message;
  message.mutable_linear()->set_x(_linear);
  message.mutable_linear()->set_y(_lateral);
  message.mutable_angular()->set_z(_angular);
  human.velocityPublisher.Publish(message);
}

void HumanControlPanel::teleopStop(int _index)
{
  if (_index >= 0 && _index < static_cast<int>(this->humans.size()))
    ++this->humans.at(_index).teleopGeneration;
  this->teleopMove(_index, 0.0, 0.0, 0.0);
}

void HumanControlPanel::PublishTurnToFace(int _index, double _speed, double _targetHeadingRad)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;

  // angular.x doubles as the turn-to-face flag ActorCommandPlugin::
  // VelocityCallback() checks -- see the comment there. angular.z here is
  // an ABSOLUTE world heading (radians), not a turn rate.
  gz::msgs::Twist message;
  message.mutable_linear()->set_x(_speed);
  message.mutable_angular()->set_x(1.0);
  message.mutable_angular()->set_z(_targetHeadingRad);
  human.velocityPublisher.Publish(message);
}

void HumanControlPanel::teleopDirection(
    int _index, const QString &_direction, bool _turnToFace)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;

  const std::string direction = _direction.toStdString();
  double linear = 0.0;
  double lateral = 0.0;
  double angular = 0.0;
  if (!DirectionToTwist(direction, linear, lateral, angular))
    return;
  const double speed = this->EffectiveSpeedMultiplier();

  if (_turnToFace)
  {
    // Steer toward this direction's ABSOLUTE world heading (fixed per-key
    // angle table -- see DirectionHeadingAngle()) while continuously
    // walking forward, instead of the old stop-in-place-then-walk two
    // phases: ActorCommandPlugin::PreUpdate() now does the smooth
    // in-motion arc itself, every tick, for as long as this Twist keeps
    // being the latest one received. Because the target is an ABSOLUTE
    // heading rather than a turn relative to wherever the actor currently
    // happens to be facing, pressing the same direction key again (or a
    // live speed change re-publishing this, see RefreshHeldMovementSpeed())
    // is always idempotent -- it never stacks another 90 degrees on top,
    // unlike the old relative-angle design. magnitude is kTeleopSpeed for
    // every entry in DirectionToTwist()'s table by construction (see its
    // comment), so this is exactly the walking speed regardless of which
    // key it was.
    ++this->humans.at(_index).teleopGeneration;
    this->PublishTurnToFace(
        _index, kTeleopSpeed * speed, DirectionHeadingAngle(direction));
    return;
  }

  linear *= speed;
  lateral *= speed;
  ++this->humans.at(_index).teleopGeneration;
  this->teleopMove(_index, linear, lateral, angular);
}

void HumanControlPanel::PressDirectionKey(const std::string &_direction)
{
  auto &keys = this->heldDirectionKeys;
  if (std::find(keys.begin(), keys.end(), _direction) == keys.end())
  {
    // A 3rd simultaneous direction key has no defined combo meaning here;
    // ignore it rather than guess (the first two keep driving the human).
    if (keys.size() >= 2)
      return;
    keys.push_back(_direction);
  }
  // Solo key, not in J strafe-mode: turn the body to face this direction,
  // then walk -- teleopDirection()'s turnToFace path already handles
  // "already facing that way" (W) as an instant walk, no visible turn
  // needed. This bypasses ApplyHeldDirectionKeys()'s pure-strafe Twist
  // entirely for the solo case; that function is only still used below
  // for the 2-key curving combo, or here in strafe mode.
  if (keys.size() == 1 && !this->jHeldState)
  {
    this->teleopDirection(
        this->activeHumanIndex, QString::fromStdString(_direction), true);
    return;
  }
  this->ApplyHeldDirectionKeys();
}

void HumanControlPanel::ReleaseDirectionKey(const std::string &_direction)
{
  auto &keys = this->heldDirectionKeys;
  keys.erase(std::remove(keys.begin(), keys.end(), _direction), keys.end());
  if (keys.empty())
  {
    this->teleopStop(this->activeHumanIndex);
  }
  else if (keys.size() == 1 && !this->jHeldState)
  {
    // Back down to a single held key -- same turn-to-face-then-walk
    // restart as PressDirectionKey()'s solo-key branch (the 2-key combo
    // we may have just been doing can leave the body facing a heading
    // that doesn't match this key's own).
    this->teleopDirection(
        this->activeHumanIndex, QString::fromStdString(keys.front()), true);
  }
  else
  {
    this->ApplyHeldDirectionKeys();
  }
}

void HumanControlPanel::ApplyHeldDirectionKeys()
{
  const auto &keys = this->heldDirectionKeys;
  if (keys.empty())
    return;

  const int index = this->activeHumanIndex;
  if (index < 0 || index >= static_cast<int>(this->humans.size()))
    return;

  if (keys.size() == 2 && !this->jHeldState)
  {
    // Curving combo: steer toward the SECOND (most recently pressed)
    // key's absolute heading via the same turn-to-face mechanism
    // teleopDirection() uses for a solo key, instead of publishing a
    // fixed turn RATE that never got zeroed back out -- that old version
    // kept rotating for as long as both keys stayed held, so holding e.g.
    // W then D never settled into moving right, it just spiralled
    // forever. This version curves in and then walks straight the moment
    // it reaches the second key's heading, exactly like a solo
    // turn-to-face key. Releasing either key still falls through to
    // ReleaseDirectionKey()'s solo-key retarget, unchanged.
    ++this->humans.at(index).teleopGeneration;
    this->PublishTurnToFace(index, kTeleopSpeed * this->EffectiveSpeedMultiplier(),
        DirectionHeadingAngle(keys.back()));
    return;
  }

  // J strafe mode (1 or 2 keys -- a 2nd key's direction is ignored here,
  // strafe mode never turns): the original body-relative Twist, angular
  // always 0.
  double linear = 0.0, lateral = 0.0, angular = 0.0;
  DirectionToTwist(keys.front(), linear, lateral, angular);
  const double speed = this->EffectiveSpeedMultiplier();
  linear *= speed;
  lateral *= speed;
  ++this->humans.at(index).teleopGeneration;
  this->teleopMove(index, linear, lateral, angular);
}

void HumanControlPanel::RefreshHeldMovementSpeed()
{
  const int index = this->activeHumanIndex;
  if (index < 0 || index >= static_cast<int>(this->humans.size()))
    return;
  auto &keys = this->heldDirectionKeys;
  if (keys.empty())
    return;

  if (keys.size() == 2 || this->jHeldState)
  {
    // 2-key curve combo, or solo J strafe -- both are the
    // ApplyHeldDirectionKeys() Twist, always safe/idempotent to just
    // recompute and republish, including switching INTO strafe mode
    // mid-hold (J newly held while a solo key was already in turn-to-face
    // mode below).
    this->ApplyHeldDirectionKeys();
    return;
  }

  // Solo key, turn-to-face mode: always safe/idempotent to just reissue
  // at the current speed, whether or not the actor has finished turning
  // yet -- teleopDirection()'s target is an absolute world heading, not a
  // turn relative to wherever the actor currently is facing, so this
  // never adds extra rotation, it just updates the walking speed (and,
  // if J was just released, switches back into turn-to-face mode from
  // strafe mode).
  this->teleopDirection(index, QString::fromStdString(keys.front()), true);
}
}  // namespace gz_human_sim
