#include "SfmBridge.hh"

#include <cstddef>
#include <sstream>

#include <gz/msgs/stringmsg.pb.h>

// 中身は HumanControlPanelRoute.cc / HumanControlPanelSpawn.cc から
// そのまま移したもので、振る舞いは変えていない。

namespace gz_human_sim
{
namespace
{
// Topic strings must match SfmCrowdSystem's own register_topic/
// unregister_topic SDF defaults (src/server/sfm_crowd_system.cpp).
const char *const kRegisterTopic = "/gz_human_sim/sfm/register_human";
const char *const kUnregisterTopic = "/gz_human_sim/sfm/unregister_human";
}  // namespace

void SfmBridge::EnsurePublishers(gz::transport::Node &_node)
{
  if (!this->registerPublisher.Valid())
    this->registerPublisher = _node.Advertise<gz::msgs::StringMsg>(kRegisterTopic);
  if (!this->unregisterPublisher.Valid())
    this->unregisterPublisher = _node.Advertise<gz::msgs::StringMsg>(kUnregisterTopic);
}

bool SfmBridge::Available(gz::transport::Node &_node)
{
  // SfmCrowdSystem subscribes to the register topic when it loads; nothing
  // else does. gz-transport's own discovery can therefore answer "is that
  // system in this world?" without either side having to advertise a
  // dedicated heartbeat.
  std::vector<gz::transport::MessagePublisher> publishers;
  std::vector<gz::transport::MessagePublisher> subscribers;
  const bool queried = _node.TopicInfo(kRegisterTopic, publishers, subscribers);
  this->available = queried && !subscribers.empty();
  return this->available;
}

bool SfmBridge::CachedAvailable() const
{
  return this->available;
}

void SfmBridge::Register(gz::transport::Node &_node, const std::string &_name,
    bool _cyclic, const std::vector<std::pair<double, double>> &_route)
{
  this->EnsurePublishers(_node);

  // Wire format: name|cyclicGoals(0|1)|desiredVelocity|radius|x1,y1;x2,y2;...
  // -1|-1 for desiredVelocity/radius means "use SfmCrowdSystem's own
  // default" -- see SfmCrowdSystem::ParseRegistration()/RegistrationRequest,
  // which this must match exactly.
  std::ostringstream payload;
  payload << _name << '|' << (_cyclic ? '1' : '0') << "|-1|-1|";
  for (std::size_t i = 0; i < _route.size(); ++i)
  {
    if (i > 0)
      payload << ';';
    payload << _route[i].first << ',' << _route[i].second;
  }
  gz::msgs::StringMsg message;
  message.set_data(payload.str());
  this->registerPublisher.Publish(message);
}

void SfmBridge::Unregister(gz::transport::Node &_node, const std::string &_name)
{
  this->EnsurePublishers(_node);
  gz::msgs::StringMsg message;
  message.set_data(_name);
  this->unregisterPublisher.Publish(message);
}
}  // namespace gz_human_sim
