#include "WorldEntityService.hh"

#include <functional>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/serialized_map.pb.h>

// 中身は HumanControlPanelSpawn.cc からそのまま移したもので、
// 振る舞いは変えていない。

namespace gz_human_sim
{
namespace world_entity
{
void Remove(gz::transport::Node &_node, const std::string &_world,
    const std::string &_name)
{
  gz::msgs::Entity request;
  request.set_name(_name);
  request.set_type(gz::msgs::Entity::MODEL);

  const std::string service = "/world/" + _world + "/remove";
  std::function<void(const gz::msgs::Boolean &, const bool)> callback =
      [](const gz::msgs::Boolean &, const bool) { /* fire and forget */ };
  _node.Request(service, request, callback);
}

void Create(gz::transport::Node &_node, const std::string &_world,
    const std::string &_sdf, const std::string &_name,
    double _x, double _y, double _z)
{
  gz::msgs::EntityFactory request;
  request.set_sdf(_sdf);
  request.set_name(_name);
  request.mutable_pose()->mutable_position()->set_x(_x);
  request.mutable_pose()->mutable_position()->set_y(_y);
  request.mutable_pose()->mutable_position()->set_z(_z);

  const std::string service = "/world/" + _world + "/create";
  // Fire and forget, same as Remove() above -- if this fails,
  // ActorCommandPlugin simply keeps not finding a collision entity by name
  // (same as before the human_collision_body companion first spawns)
  // rather than anything crashing, so there's nothing useful to do with a
  // failure callback here.
  std::function<void(const gz::msgs::Boolean &, const bool)> callback =
      [](const gz::msgs::Boolean &, const bool) { /* fire and forget */ };
  _node.Request(service, request, callback);
}

bool Exists(gz::transport::Node &_node, const std::string &_world,
    const std::string &_name)
{
  if (_world.empty())
    return false;
  gz::msgs::Empty request;
  gz::msgs::SerializedStepMap response;
  bool result = false;
  const bool executed = _node.Request(
      "/world/" + _world + "/state", request,
      kStateQueryTimeoutMs, response, result);
  if (!executed || !result)
    return false;
  const std::string serialized = response.SerializeAsString();
  return serialized.find(_name) != std::string::npos;
}
}  // namespace world_entity
}  // namespace gz_human_sim
