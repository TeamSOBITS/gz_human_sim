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
#include "SfmBridge.hh"

// Route editing, path templates, SFM registration and the obstacle-aware
// path planner hookup.
//
// Split out of the single-file HumanControlPanel.cc; the code below is
// unchanged from that file.

namespace gz_human_sim
{
bool HumanControlPanel::ActiveSfmEnabled() const
{
  if (this->activeHumanIndex < 0 ||
      this->activeHumanIndex >= static_cast<int>(this->humans.size()))
    return true;
  return this->humans.at(this->activeHumanIndex).route.sfmEnabled;
}

bool HumanControlPanel::UseSfm() const
{
  return this->useSfmState;
}

bool HumanControlPanel::CyclicRoute() const
{
  return this->cyclicRouteState;
}

bool HumanControlPanel::SfmAvailable() const
{
  return this->sfm.CachedAvailable();
}

bool HumanControlPanel::AvoidObstacles() const
{
  return this->avoidObstaclesState;
}

bool HumanControlPanel::AvoidObstaclesAvailable() const
{
  return this->pathPlanner.CachedAvailable();
}

void HumanControlPanel::setAvoidObstacles(bool _value)
{
  if (this->avoidObstaclesState == _value)
    return;
  this->avoidObstaclesState = _value;
  if (_value)
    this->NavPlannerAvailable();
  this->routeSettingsChanged();
}

bool HumanControlPanel::NavPlannerAvailable()
{
  // 判定そのものは PathPlanner が持つ。ここは QML 向けの窓口。
  return this->pathPlanner.Available(this->node);
}

bool HumanControlPanel::PlanAroundObstacles(
    const std::vector<std::pair<double, double>> &_route, double _bodyRadius,
    std::vector<std::pair<double, double>> &_planned)
{
  return this->pathPlanner.Plan(this->node, _route, _bodyRadius, _planned);
}

void HumanControlPanel::sendWaypoint(int _index, double _x, double _y)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.pathPublisher.Valid())
    return;

  gz::msgs::Pose_V message;
  auto *pose = message.add_pose();
  pose->mutable_position()->set_x(_x);
  pose->mutable_position()->set_y(_y);
  pose->mutable_orientation()->set_w(1.0);
  human.pathPublisher.Publish(message);
}

void HumanControlPanel::generatePathTemplate(
    int _templateIndex, double _centerX, double _centerY,
    double _size, int _numWaypoints, bool _clockwise)
{
  if (_templateIndex < 0 || _templateIndex >= kPathTemplateCount)
    return;

  // Replaces the pending route rather than publishing straight to one
  // human: from here on a template-made route and a clicked one are the
  // same thing, edited with the same undo/clear buttons and applied to the
  // same target set by the same confirmRoute() button.
  this->pendingRoute = TemplateWaypoints(_templateIndex, _centerX, _centerY,
      _size, _numWaypoints, _clockwise);
  this->pendingRouteChanged();
  this->SetStatus(QString(kPathTemplateLabels[_templateIndex]) + "の経路（" +
      QString::number(this->pendingRoute.size()) +
      "点）を生成しました。「この経路で歩かせる」で適用してください");
}

void HumanControlPanel::setRouteRecording(bool _enabled)
{
  if (this->routeRecordingState == _enabled)
    return;
  this->routeRecordingState = _enabled;
  // Only one click mode at a time (see the routeRecording Q_PROPERTY): a
  // click can't mean two things, and silently letting both be on would make
  // whichever branch eventFilter() tests first quietly win.
  if (_enabled && this->spawnPickingState)
  {
    this->spawnPickingState = false;
    this->spawnPickingChanged();
  }
  this->routeRecordingChanged();
  this->SetStatus(_enabled
      ? "経路登録モード: ON（3Dビューのクリックが経由点になります）"
      : "経路登録モード: OFF");
}

void HumanControlPanel::setSpawnPicking(bool _enabled)
{
  if (this->spawnPickingState == _enabled)
    return;
  this->spawnPickingState = _enabled;
  if (_enabled && this->routeRecordingState)
  {
    this->routeRecordingState = false;
    this->routeRecordingChanged();
  }
  this->spawnPickingChanged();
  this->SetStatus(_enabled
      ? "スポーン地点選択モード: ON（3Dビューをクリックした数だけ人物を配置できます）"
      : "スポーン地点選択モード: OFF");
}

void HumanControlPanel::clearPendingRoute()
{
  if (this->pendingRoute.empty())
    return;
  this->pendingRoute.clear();
  this->pendingRouteChanged();
}

void HumanControlPanel::undoLastRoutePoint()
{
  if (this->pendingRoute.empty())
    return;
  this->pendingRoute.pop_back();
  this->pendingRouteChanged();
}

void HumanControlPanel::undoLastSpawnPoint()
{
  if (this->pendingSpawnPoints.empty())
    return;
  this->pendingSpawnPoints.pop_back();
  this->spawnMarkers.InvalidatePending();
  this->pendingSpawnPointsChanged();
}

void HumanControlPanel::clearSpawnPoints()
{
  if (this->pendingSpawnPoints.empty())
    return;
  this->pendingSpawnPoints.clear();
  this->spawnMarkers.InvalidatePending();
  this->pendingSpawnPointsChanged();
}

bool HumanControlPanel::routeTargetAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(_index).route.isTarget;
}

void HumanControlPanel::setRouteTarget(int _index, bool _value)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  this->humans.at(_index).route.isTarget = _value;
  this->humansChanged();
}

void HumanControlPanel::setAllRouteTargets(bool _value)
{
  for (auto &human : this->humans)
    human.route.isTarget = _value;
  this->humansChanged();
}

std::vector<int> HumanControlPanel::RouteTargetIndices() const
{
  std::vector<int> indices;
  for (std::size_t i = 0; i < this->humans.size(); ++i)
  {
    if (this->humans.at(i).route.isTarget)
      indices.push_back(static_cast<int>(i));
  }
  // Nothing ticked: fall back to the active human so the button still does
  // the obvious thing for someone who never touched a tickbox.
  if (indices.empty() && this->activeHumanIndex >= 0 &&
      this->activeHumanIndex < static_cast<int>(this->humans.size()))
  {
    indices.push_back(this->activeHumanIndex);
  }
  return indices;
}

void HumanControlPanel::setUseSfm(bool _value)
{
  if (this->useSfmState == _value)
    return;
  this->useSfmState = _value;
  // Re-check availability at the moment someone actually asks for SFM, so
  // the warning in the QML reflects this world rather than a stale probe.
  if (_value)
    this->sfm.Available(this->node);
  this->routeSettingsChanged();
}

void HumanControlPanel::setCyclicRoute(bool _value)
{
  if (this->cyclicRouteState == _value)
    return;
  this->cyclicRouteState = _value;
  this->routeSettingsChanged();
}

void HumanControlPanel::setSfmEnabled(int _index, bool _value)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.route.sfmEnablePublisher.Valid())
    return;
  gz::msgs::Boolean message;
  message.set_data(_value);
  human.route.sfmEnablePublisher.Publish(message);
  human.route.sfmEnabled = _value;
  if (_index == this->activeHumanIndex)
    this->sfmModeChanged();
  this->SetStatus(QString::fromStdString(human.name) + " のSFM（自動回避）を" +
      (_value ? QString("有効") : QString("無効")) + "にしました");
}

void HumanControlPanel::setSfmEnabledForTargets(bool _value)
{
  const auto targets = this->RouteTargetIndices();
  for (const int index : targets)
    this->setSfmEnabled(index, _value);
  this->SetStatus(QString("%1人のSFM（自動回避）を%2にしました")
      .arg(targets.size()).arg(_value ? "有効" : "無効"));
}

void HumanControlPanel::SendSfmRegistration(int _index)
{
  auto &human = this->humans.at(_index);

  // lastSentRoute, not pendingRoute: when obstacle avoidance is on this is
  // the planned route that goes around things, and when it's off the two are
  // identical (confirmRoute() sets it either way).
  this->sfm.Register(this->node, human.name, this->cyclicRouteState,
      this->lastSentRoute);

  // A registered route only actually moves the human once sfm_enable is
  // also true. Force it back on rather than merely republishing whatever it
  // happened to be: a human that had been handed back to manual teleop
  // (sfm_enable false) would otherwise accept the registration and keep
  // standing still, which is exactly the "経路を確定しても歩かない" case.
  if (human.route.sfmEnablePublisher.Valid())
  {
    gz::msgs::Boolean enableMessage;
    enableMessage.set_data(true);
    human.route.sfmEnablePublisher.Publish(enableMessage);
    human.route.sfmEnabled = true;
  }
}

void HumanControlPanel::SendSimplePath(int _index)
{
  auto &human = this->humans.at(_index);
  if (!human.pathPublisher.Valid())
    return;

  // Orientation toward the NEXT point so ActorCommandPlugin's path-follow
  // arrives facing onward instead of at whatever heading it happened to
  // have, except the last point (nothing to face, so identity orientation
  // -- ApplyPathCommand() only ever uses a waypoint's position to steer, so
  // this is purely cosmetic, not load-bearing).
  gz::msgs::Pose_V message;
  const auto &route = this->lastSentRoute;
  for (std::size_t i = 0; i < route.size(); ++i)
  {
    auto *pose = message.add_pose();
    pose->mutable_position()->set_x(route[i].first);
    pose->mutable_position()->set_y(route[i].second);
    if (i + 1 < route.size())
    {
      const double yaw = std::atan2(route[i + 1].second - route[i].second,
          route[i + 1].first - route[i].first);
      pose->mutable_orientation()->set_z(std::sin(yaw * 0.5));
      pose->mutable_orientation()->set_w(std::cos(yaw * 0.5));
    }
    else
    {
      pose->mutable_orientation()->set_w(1.0);
    }
  }

  // SfmCrowdSystem, if this human was previously registered with it, keeps
  // publishing its own cmd_vel every tick and would immediately overwrite
  // the path-follow motion. Hand the human back FIRST, so there is no
  // window where both are steering it.
  if (human.route.sfmEnablePublisher.Valid() && human.route.sfmEnabled)
  {
    gz::msgs::Boolean enableMessage;
    enableMessage.set_data(false);
    human.route.sfmEnablePublisher.Publish(enableMessage);
    human.route.sfmEnabled = false;
  }

  human.pathPublisher.Publish(message);
}

void HumanControlPanel::confirmRoute()
{
  if (this->pendingRoute.empty())
  {
    this->SetStatus("経由点がありません。「経路登録モード」をONにして3Dビューを"
        "クリックするか、経路テンプレートで生成してください");
    return;
  }

  const auto targets = this->RouteTargetIndices();
  if (targets.empty())
  {
    this->SetStatus("経路の対象人物がいません（人物をspawnして「経路対象」に"
        "チェックを入れてください）");
    return;
  }

  if (this->useSfmState && !this->sfm.Available(this->node))
  {
    // Publishing an SFM registration into a world with no SfmCrowdSystem is
    // a silent no-op -- refuse instead of letting the operator watch nobody
    // move and have no idea why.
    this->SetStatus("このワールドにはSFM（自動回避）システムが読み込まれていません。"
        "「単純パス追従」に切り替えてください");
    this->routeSettingsChanged();
    return;
  }

  // Turn the drawn route into one that actually goes around walls and
  // furniture, BEFORE it's sent to anybody. Everything downstream
  // (SendSfmRegistration()/SendSimplePath()) reads lastSentRoute, so both
  // modes get the planned version and neither has to know planning happened.
  //
  // The body radius handed to the planner is the target's own collision
  // capsule (the same value the 当たり判定 slider sets), so the path is only
  // routed through gaps this person actually fits through.
  bool planned = false;
  if (this->avoidObstaclesState)
  {
    double bodyRadius = 0.25;
    if (!targets.empty())
      bodyRadius = this->humans.at(targets.front()).collision.radius;
    planned = this->PlanAroundObstacles(
        this->pendingRoute, bodyRadius, this->lastSentRoute);
    if (!planned && !this->NavPlannerAvailable())
    {
      this->SetStatus("このワールドには経路プランナ（NavGridSystem）が"
          "読み込まれていないため、障害物を避けない直線経路で送信します");
    }
  }
  else
  {
    this->lastSentRoute = this->pendingRoute;
  }
  if (this->lastSentRoute.empty())
    this->lastSentRoute = this->pendingRoute;

  int sent = 0;
  int skipped = 0;
  for (const int index : targets)
  {
    auto &human = this->humans.at(index);
    // Only actor-backed humans can walk anything at all (static models have
    // no ActorCommandPlugin) -- count them out loud rather than silently.
    if (!human.pathPublisher.Valid())
    {
      ++skipped;
      continue;
    }
    if (this->useSfmState)
    {
      this->SendSfmRegistration(index);
    }
    else
    {
      // "経路専用"/"テレオペ" both follow a path; make sure the human is in
      // one of them. followModeIndex 0 = auto, 1 = path (see
      // kFollowModeValues) -- both fine, so nothing to change here, but a
      // human whose SDF was spawned with follow_mode "velocity" would
      // ignore cmd_path, so push "auto" to be certain.
      if (human.followModePublisher.Valid())
      {
        gz::msgs::StringMsg modeMessage;
        modeMessage.set_data(kFollowModeValues[human.followModeIndex]);
        human.followModePublisher.Publish(modeMessage);
      }
      this->SendSimplePath(index);
    }
    ++sent;
  }

  this->sfmModeChanged();
  if (sent == 0)
  {
    this->SetStatus("経路を歩ける人物が対象にいません"
        "（walking_actor / DoctorFemaleWalk のみ経路に対応しています）");
    return;
  }
  QString status = QString("%1人に経路（%2点・%3）を適用しました")
      .arg(sent).arg(this->lastSentRoute.size())
      .arg(this->useSfmState ? "SFM自動回避" : "単純パス追従");
  if (planned)
  {
    status += QString("／障害物を回避（経由点%1→%2点）")
        .arg(this->pendingRoute.size()).arg(this->lastSentRoute.size());
  }
  if (skipped > 0)
    status += QString("／%1人は経路非対応のため除外").arg(skipped);
  this->SetStatus(status);
}
}  // namespace gz_human_sim
