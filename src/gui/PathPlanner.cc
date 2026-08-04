#include "PathPlanner.hh"

#include <algorithm>
#include <cstddef>
#include <sstream>
#include <string>

#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>

// 中身は HumanControlPanelRoute.cc からそのまま移したもので、
// 振る舞いは変えていない。

namespace gz_human_sim
{
bool PathPlanner::Available(gz::transport::Node &_node)
{
  // A service, unlike SFM's topic, so this asks the service list rather than
  // the subscriber list -- same idea either way: the feature lives in a world
  // plugin, and a world that didn't load it simply won't be offering this.
  std::vector<std::string> services;
  _node.ServiceList(services);
  this->available =
      std::find(services.begin(), services.end(), kNavPlanService) != services.end();
  return this->available;
}

bool PathPlanner::CachedAvailable() const
{
  return this->available;
}

bool PathPlanner::Plan(gz::transport::Node &_node,
    const std::vector<std::pair<double, double>> &_route, double _bodyRadius,
    std::vector<std::pair<double, double>> &_planned)
{
  _planned = _route;
  if (_route.size() < 2)
    return false;
  if (!this->Available(_node))
    return false;

  // Wire format must match NavGridSystem::OnPlanPath():
  // "inflationRadius|x1,y1;x2,y2;..."
  std::ostringstream payload;
  payload << _bodyRadius << '|';
  for (std::size_t i = 0; i < _route.size(); ++i)
  {
    if (i > 0)
      payload << ';';
    payload << _route[i].first << ',' << _route[i].second;
  }

  gz::msgs::StringMsg request;
  request.set_data(payload.str());
  gz::msgs::Pose_V response;
  bool result = false;
  const bool executed = _node.Request(
      kNavPlanService, request, kNavPlanTimeoutMs, response, result);
  if (!executed || !result || response.pose_size() < 2)
    return false;

  _planned.clear();
  for (int i = 0; i < response.pose_size(); ++i)
  {
    _planned.emplace_back(
        response.pose(i).position().x(), response.pose(i).position().y());
  }
  return true;
}
}  // namespace gz_human_sim
