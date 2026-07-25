#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <lightsfm/sfm.hpp>

namespace gz_human_sim
{
namespace
{
/// \brief Closest point on segment [_a, _b] to _p, clamped to the segment
/// (not the infinite line through it) -- the point lightsfm's
/// computeObstacleForce() pushes an agent away from.
utils::Vector2d NearestPointOnSegment(const utils::Vector2d &_p,
    const utils::Vector2d &_a, const utils::Vector2d &_b)
{
  const utils::Vector2d ab = _b - _a;
  const double lengthSquared = ab.squaredNorm();
  if (lengthSquared < 1e-9)
    return _a;
  const double t = std::clamp((_p - _a).dot(ab) / lengthSquared, 0.0, 1.0);
  return _a + ab * t;
}

/// \brief Reads a "<elem><x>..</x><y>..</y></elem>"-shaped SDF element into
/// a 2D point. Used for both <goal> (x/y are direct children) and a wall
/// segment's <p1>/<p2> (x/y are children of THAT sub-element) -- callers
/// just pass whichever ElementPtr should directly own the x/y children.
utils::Vector2d ReadPoint(const sdf::ElementConstPtr &_elem)
{
  if (_elem == nullptr)
    return utils::Vector2d(0.0, 0.0);
  return utils::Vector2d(_elem->Get<double>("x", 0.0).first,
      _elem->Get<double>("y", 0.0).first);
}
}  // namespace

/// \brief A straight-line obstacle segment, e.g. one wall of a corridor.
/// Minimal stand-in for a proper occupancy-grid sfm::Map (future work --
/// see the plugin's own top-of-class comment); fine for a handful of
/// hand-authored walls, not meant to scale to a full building footprint.
struct WallSegment
{
  utils::Vector2d a;
  utils::Vector2d b;
};

/// \brief One physics body this plugin reads a world pose from every tick
/// (a human's collision body, or a robot model) -- never written to.
/// lightsfm needs each agent's velocity, not just its position, for the
/// desired/social force formulas; since this plugin deliberately never
/// calls sfm::Agent::move()/SocialForceModel::updatePosition() (position
/// is left entirely to Gazebo's physics, per this feature's own
/// constraint), that velocity has nowhere else to come from and is
/// estimated here as a plain finite difference between this tick's and
/// last tick's measured position.
struct TrackedBody
{
  std::string modelName;
  gz::sim::Entity entity{gz::sim::kNullEntity};
  bool havePosition{false};
  utils::Vector2d position;
  utils::Vector2d velocity;
  double yaw{0.0};
};

/// \brief One SFM-controlled human: the physics body it reads pose from,
/// the persistent sfm::Agent identity (radius/desiredVelocity/params and
/// crucially `goals`, which must survive across ticks -- see PreUpdate()'s
/// comment on AdvanceGoalIfReached() for why this plugin cannot rely on
/// lightsfm's own updatePosition() to advance it), and the gz-transport
/// wiring back to this human's own ActorCommandPlugin.
struct HumanAgent
{
  // Stable identity key for runtime re-registration/unregistration and for
  // the sfm_enable_topic callback to find its own entry back by (see
  // SfmCrowdSystem::SubscribeEnableTopic()) -- NOT necessarily the same
  // string as body.modelName. SDF-configured <human> entries (Configure())
  // set this to their collision_model, since that is already required to
  // be unique per human there; runtime-registered ones (ApplyRegistration())
  // set it to the plain name HumanControlPanel registered, and derive
  // body.modelName/cmdVelTopic/sfmEnableTopic from it following the same
  // "<name>_collision" / "/<name>/cmd_vel" / "/<name>/sfm_enable"
  // convention human_model_utils.py's spawn already uses.
  std::string name;
  TrackedBody body;
  sfm::Agent agent;
  std::string cmdVelTopic;
  std::string sfmEnableTopic;
  gz::transport::Node::Publisher publisher;
  bool enabled{true};
  // The velocity COMMAND's own persistent state -- separate from
  // body.velocity (a measured, physical, read-only quantity). The
  // collision body is velocity-controlled (gz-sim-velocity-control-system
  // sets whatever it's told, achieving it almost immediately -- confirmed
  // directly against this world), not force-controlled, so there is
  // nothing else already integrating the SFM force into a velocity over
  // time the way a Newtonian body would. This is that integrator: it
  // plays the exact role sfm::Agent::velocity plays inside lightsfm's own
  // (unused here) updatePosition() -- force accumulates into it every
  // tick, clamped to desiredVelocity -- except this plugin owns it
  // directly instead of going through updatePosition(), since that
  // function would also try to integrate `position`, which this plugin
  // must not do (see this class's top comment).
  utils::Vector2d commandedVelocity;
};

/// \brief One "please track/drive this human" request received on
/// SfmCrowdSystem's register_topic (see ParseRegistration()), e.g. from
/// HumanControlPanel's batch-spawn or click-to-route GUI features. Kept as
/// a plain data struct separate from HumanAgent itself so parsing (which
/// happens on the gz-transport callback thread) never touches `humans`
/// directly -- PreUpdate() drains a queue of these under crowdMutex and
/// applies them on the simulation thread instead (see ApplyRegistration()).
struct RegistrationRequest
{
  std::string name;
  bool cyclicGoals{true};
  // <= 0 means "use SfmCrowdSystem's own default" (defaultDesiredVelocity/
  // defaultAgentRadius) -- not every caller wants to specify these.
  double desiredVelocity{-1.0};
  double radius{-1.0};
  std::vector<std::pair<double, double>> goals;
};

/// \brief World System plugin: drives a crowd of gz_human_sim humans with
/// the Social Force Model (lightsfm), publishing gz::msgs::Twist on each
/// human's existing ActorCommandPlugin vel_topic -- exactly the same
/// message an operator's teleop would send, so ActorCommandPlugin itself,
/// the human_collision_body physics, and the walk animation are all
/// reused completely unmodified.
///
/// Deliberately NOT calling sfm::Agent::move() / SocialForceModel::
/// updatePosition(): this plugin only ever reads positions/velocities back
/// from Gazebo's own physics (the collision bodies) and only ever writes
/// velocity COMMANDS forward, the same one-way flow ActorCommandPlugin
/// already uses for teleop. Position integration happens exactly once,
/// inside Gazebo's physics engine, so there is never a double update to
/// reconcile between this plugin and the collision body / actor.
///
/// The "force -> velocity" arithmetic in PublishCommand() below reproduces
/// only the velocity half of what SocialForceModel::updatePosition() does
/// internally (accumulate the computed force into a persistent velocity
/// every tick, clamp to desiredVelocity) -- lightsfm has no standalone
/// function for that half alone, and it is a handful of lines, so it is
/// reimplemented here rather than pulled in via updatePosition()'s
/// position-writing half. The accumulator is HumanAgent::commandedVelocity,
/// not sfm::Agent::velocity (which this plugin instead overwrites every
/// tick with the collision body's own measured physical velocity, for the
/// force formulas themselves to react to reality) -- the two play
/// different roles and must not be conflated: gz-sim-velocity-control-
/// system is a velocity-controlled (not force/Newtonian-controlled) body
/// that achieves almost exactly whatever it is told nearly immediately
/// (confirmed directly against this world), so nothing else is already
/// integrating this force into a velocity over time the way lightsfm's own
/// updatePosition() assumes a Newtonian agent would. lightsfm's own files
/// (vendored under lightsfm/include/lightsfm/) are not modified anywhere
/// for this.
class SfmCrowdSystem
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    this->defaultDesiredVelocity = _sdf->Get<double>("desired_velocity", 1.0).first;
    this->defaultAgentRadius = _sdf->Get<double>("agent_radius", 0.35).first;
    this->obstacleRange = std::max(0.0, _sdf->Get<double>("obstacle_range", 6.0).first);
    this->registerTopic = _sdf->Get<std::string>(
        "register_topic", "/gz_human_sim/sfm/register_human").first;
    this->unregisterTopic = _sdf->Get<std::string>(
        "unregister_topic", "/gz_human_sim/sfm/unregister_human").first;

    for (sdf::ElementConstPtr wallElem = _sdf->FindElement("wall_segment"); wallElem;
        wallElem = wallElem->GetNextElement("wall_segment"))
    {
      WallSegment segment;
      segment.a = ReadPoint(wallElem->FindElement("p1"));
      segment.b = ReadPoint(wallElem->FindElement("p2"));
      this->walls.push_back(segment);
    }

    for (sdf::ElementConstPtr robotElem = _sdf->FindElement("robot_model"); robotElem;
        robotElem = robotElem->GetNextElement("robot_model"))
    {
      TrackedBody robot;
      robot.modelName = robotElem->Get<std::string>();
      if (!robot.modelName.empty())
        this->robots.push_back(robot);
    }

    for (sdf::ElementConstPtr humanElem = _sdf->FindElement("human"); humanElem;
        humanElem = humanElem->GetNextElement("human"))
    {
      HumanAgent human;
      human.body.modelName = humanElem->Get<std::string>("collision_model", "").first;
      // Reused as this human's stable identity key -- see HumanAgent::name's
      // comment. SDF-configured humans have no separate "name" concept of
      // their own, and collision_model is already required to be unique.
      human.name = human.body.modelName;
      human.cmdVelTopic = humanElem->Get<std::string>("cmd_vel_topic", "").first;
      human.sfmEnableTopic = humanElem->Get<std::string>("sfm_enable_topic", "").first;
      if (human.body.modelName.empty() || human.cmdVelTopic.empty())
      {
        gzerr << "SfmCrowdSystem: <human> requires collision_model and cmd_vel_topic, "
              << "skipping this entry." << std::endl;
        continue;
      }
      human.agent.radius = humanElem->Get<double>("radius", this->defaultAgentRadius).first;
      human.agent.desiredVelocity =
          humanElem->Get<double>("desired_velocity", this->defaultDesiredVelocity).first;
      human.agent.cyclicGoals = humanElem->Get<bool>("cyclic_goals", true).first;
      human.agent.teleoperated = false;
      human.agent.groupId = -1;
      for (sdf::ElementConstPtr goalElem = humanElem->FindElement("goal"); goalElem;
          goalElem = goalElem->GetNextElement("goal"))
      {
        sfm::Goal goal;
        goal.center = ReadPoint(goalElem);
        goal.radius = goalElem->Get<double>("radius", 0.3).first;
        human.agent.goals.push_back(goal);
      }
      if (human.agent.goals.empty())
      {
        gzerr << "SfmCrowdSystem: human with collision_model '" << human.body.modelName
              << "' has no <goal>, skipping this entry." << std::endl;
        continue;
      }
      human.publisher = this->transportNode.Advertise<gz::msgs::Twist>(human.cmdVelTopic);
      this->humans.push_back(std::move(human));
    }

    // Bound by name rather than by index: unlike at Configure() time (where
    // `humans` is filled once and never resized), ApplyRegistration()/
    // unregistration can add or remove entries at runtime after this, which
    // would silently invalidate a captured index (it would go on pointing
    // at whatever entry happens to occupy that slot later, not the human it
    // was set up for). See SubscribeEnableTopic().
    for (auto &human : this->humans)
    {
      if (!human.sfmEnableTopic.empty())
        this->SubscribeEnableTopic(human.name, human.sfmEnableTopic);
    }

    // Runtime registration: lets e.g. HumanControlPanel's batch-spawn/
    // click-to-route GUI features add or replace a tracked human (or drop
    // one) while the simulation is already running, without needing a
    // second SfmCrowdSystem instance or a world restart. See
    // RegistrationRequest/ApplyRegistration()/ParseRegistration() for the
    // wire format. Both payloads are parsed/queued here on the transport
    // thread; PreUpdate() drains the queue and does the actual `humans`
    // mutation on the simulation thread (see crowdMutex's comment).
    this->transportNode.Subscribe<gz::msgs::StringMsg>(this->registerTopic,
        std::function<void(const gz::msgs::StringMsg &)>(
            [this](const gz::msgs::StringMsg &_msg)
            {
              RegistrationRequest request;
              if (this->ParseRegistration(_msg.data(), request))
              {
                std::lock_guard<std::mutex> lock(this->crowdMutex);
                this->pendingRegistrations.push_back(std::move(request));
              }
            }));
    this->transportNode.Subscribe<gz::msgs::StringMsg>(this->unregisterTopic,
        std::function<void(const gz::msgs::StringMsg &)>(
            [this](const gz::msgs::StringMsg &_msg)
            {
              std::lock_guard<std::mutex> lock(this->crowdMutex);
              this->pendingUnregistrations.push_back(_msg.data());
            }));

    gzmsg << "SfmCrowdSystem: managing " << this->humans.size() << " human(s), "
          << this->robots.size() << " robot(s), " << this->walls.size()
          << " wall segment(s). Listening for runtime registrations on '"
          << this->registerTopic << "' / '" << this->unregisterTopic << "'." << std::endl;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
      return;

    // Apply any registrations/unregistrations queued since last tick --
    // see crowdMutex's comment for why this has to happen here (on the
    // simulation thread) rather than directly inside the transport
    // callbacks that queued them.
    {
      std::lock_guard<std::mutex> lock(this->crowdMutex);
      for (const auto &name : this->pendingUnregistrations)
      {
        this->humans.erase(std::remove_if(this->humans.begin(), this->humans.end(),
            [&name](const HumanAgent &_h) { return _h.name == name; }), this->humans.end());
      }
      this->pendingUnregistrations.clear();
      for (const auto &request : this->pendingRegistrations)
        this->ApplyRegistration(request);
      this->pendingRegistrations.clear();
    }

    for (auto &robot : this->robots)
      this->ReadBodyPose(_ecm, robot, dt);

    // Every human and robot with a known pose this tick goes into one
    // shared roster, so lightsfm's vector computeForces() has everyone
    // avoid everyone else in a single pass (humans avoid robots and each
    // other; a robot's own force is computed too but simply never read
    // back below -- this plugin never publishes to a robot). `activeHumans`
    // mirrors agents[0..activeHumans.size()) one-to-one so the write-back
    // loop can find each computed force's owning HumanAgent.
    std::vector<sfm::Agent> agents;
    std::vector<HumanAgent *> activeHumans;
    agents.reserve(this->humans.size() + this->robots.size());
    activeHumans.reserve(this->humans.size());

    for (auto &human : this->humans)
    {
      this->ReadBodyPose(_ecm, human.body, dt);
      if (!human.body.havePosition)
        continue;
      human.agent.position = human.body.position;
      human.agent.velocity = human.body.velocity;
      human.agent.yaw = utils::Angle::fromRadian(human.body.yaw);
      // See this class's top comment: lightsfm only advances `goals` (pop
      // the reached one, cycle it back in if cyclicGoals) from inside
      // updatePosition(), which this plugin never calls, so the same
      // check is reproduced here instead.
      this->AdvanceGoalIfReached(human.agent);
      this->FillObstacles(human.agent);
      agents.push_back(human.agent);
      activeHumans.push_back(&human);
    }
    for (auto &robot : this->robots)
    {
      if (!robot.havePosition)
        continue;
      sfm::Agent robotAgent;
      robotAgent.position = robot.position;
      robotAgent.velocity = robot.velocity;
      robotAgent.yaw = utils::Angle::fromRadian(robot.yaw);
      robotAgent.radius = this->defaultAgentRadius;
      robotAgent.groupId = -1;
      // No goals: computeDesiredForce() falls into its antimove branch for
      // this agent, which is fine -- a robot's own computed force is
      // discarded below, it is only ever present so the humans' own
      // computeSocialForce() reacts to it.
      agents.push_back(robotAgent);
    }

    sfm::SFM.computeForces(agents, nullptr);

    for (std::size_t i = 0; i < activeHumans.size(); ++i)
    {
      bool enabled = true;
      {
        std::lock_guard<std::mutex> lock(this->crowdMutex);
        enabled = activeHumans[i]->enabled;
      }
      if (!enabled)
        continue;
      this->PublishCommand(*activeHumans[i], agents[i].forces.globalForce, dt);
    }
  }

  private: void ReadBodyPose(gz::sim::EntityComponentManager &_ecm, TrackedBody &_body,
      double _dt)
  {
    if (_body.entity == gz::sim::kNullEntity)
    {
      _body.entity = _ecm.EntityByComponents(gz::sim::components::Name(_body.modelName));
      if (_body.entity == gz::sim::kNullEntity)
        return;  // Not spawned yet -- spawn_human.launch.py runs after the
                 // world (and this plugin) is already up.
    }
    auto poseComponent = _ecm.Component<gz::sim::components::Pose>(_body.entity);
    if (poseComponent == nullptr)
    {
      // The entity we had is gone (e.g. removed via HumanControlPanel) --
      // forget it and try to re-resolve the name fresh next tick, rather
      // than keep reading a component pointer that no longer exists.
      _body.entity = gz::sim::kNullEntity;
      _body.havePosition = false;
      return;
    }
    const gz::math::Pose3d pose = poseComponent->Data();
    const utils::Vector2d newPosition(pose.Pos().X(), pose.Pos().Y());
    _body.velocity = _body.havePosition ? (newPosition - _body.position) / _dt
                                        : utils::Vector2d(0.0, 0.0);
    _body.position = newPosition;
    _body.yaw = pose.Rot().Yaw();
    _body.havePosition = true;
  }

  private: void AdvanceGoalIfReached(sfm::Agent &_agent) const
  {
    if (_agent.goals.empty())
      return;
    if ((_agent.goals.front().center - _agent.position).norm() <=
        _agent.goals.front().radius)
    {
      const sfm::Goal reached = _agent.goals.front();
      _agent.goals.pop_front();
      if (_agent.cyclicGoals)
        _agent.goals.push_back(reached);
    }
  }

  private: void FillObstacles(sfm::Agent &_agent) const
  {
    _agent.obstacles1.clear();
    for (const auto &wall : this->walls)
    {
      const utils::Vector2d nearest =
          NearestPointOnSegment(_agent.position, wall.a, wall.b);
      if ((nearest - _agent.position).norm() <= this->obstacleRange)
        _agent.obstacles1.push_back(nearest);
    }
  }

  /// \brief Turns one tick's computed SFM force into a velocity command and
  /// publishes it as a turn-to-face Twist -- see this class's top comment
  /// for why only this half of updatePosition()'s math is reproduced here.
  /// angular.x/angular.z encode ActorCommandPlugin's turn-to-face mode
  /// (see its VelocityCallback()): _human's actor turns smoothly toward an
  /// absolute world heading while walking, instead of this plugin having
  /// to convert to body-frame linear/lateral/angular itself.
  private: void PublishCommand(HumanAgent &_human, const utils::Vector2d &_globalForce,
      double _dt)
  {
    _human.commandedVelocity += _globalForce * _dt;
    if (_human.commandedVelocity.norm() > _human.agent.desiredVelocity)
    {
      _human.commandedVelocity.normalize();
      _human.commandedVelocity *= _human.agent.desiredVelocity;
    }
    // Below this speed the direction is noise, so keep facing the actor's
    // current heading instead of snapping to atan2(0, 0).
    const double heading = _human.commandedVelocity.norm() > 1e-3
        ? _human.commandedVelocity.angle().toRadian() : _human.body.yaw;

    gz::msgs::Twist command;
    command.mutable_linear()->set_x(std::max(0.0, _human.commandedVelocity.norm()));
    command.mutable_angular()->set_x(1.0);
    command.mutable_angular()->set_z(heading);
    _human.publisher.Publish(command);
  }

  /// \brief Subscribes _topic's sfm_enable Boolean for the human currently
  /// named _name, looking it up BY NAME (not a captured index/pointer)
  /// every time the callback fires -- see HumanAgent::name's comment for
  /// why an index would go stale once runtime registration can resize
  /// `humans`. Shared by Configure()'s SDF-configured humans and
  /// ApplyRegistration()'s runtime ones so both go through the exact same
  /// lookup path.
  private: void SubscribeEnableTopic(const std::string &_name, const std::string &_topic)
  {
    this->transportNode.Subscribe<gz::msgs::Boolean>(_topic,
        std::function<void(const gz::msgs::Boolean &)>(
            [this, _name](const gz::msgs::Boolean &_msg)
            {
              std::lock_guard<std::mutex> lock(this->crowdMutex);
              for (auto &human : this->humans)
              {
                if (human.name == _name)
                {
                  human.enabled = _msg.data();
                  break;
                }
              }
            }));
  }

  /// \brief Parses a register_topic payload into _out. Wire format (plain
  /// '|'-separated text over gz::msgs::StringMsg, not a dedicated message
  /// type -- this avoids needing a custom .proto/message package just for
  /// a handful of scalars and one point list):
  ///
  ///   name|cyclicGoals(0|1)|desiredVelocity|radius|x1,y1;x2,y2;...
  ///
  /// desiredVelocity/radius <= 0 means "use this plugin's own default" (see
  /// RegistrationRequest). At least one point is required. Unlike the SDF
  /// <human> block, collision_model/cmd_vel_topic/sfm_enable_topic are not
  /// part of this payload at all -- ApplyRegistration() derives all three
  /// from `name` following the same convention human_model_utils.py's
  /// spawn already uses ("<name>_collision" / "/<name>/cmd_vel" /
  /// "/<name>/sfm_enable"), so callers (e.g. HumanControlPanel) only ever
  /// need to agree on a name, not repeat every derived topic string too.
  private: bool ParseRegistration(const std::string &_data, RegistrationRequest &_out) const
  {
    std::vector<std::string> fields;
    std::stringstream fieldStream(_data);
    std::string field;
    while (std::getline(fieldStream, field, '|'))
      fields.push_back(field);
    if (fields.size() != 5)
    {
      gzerr << "SfmCrowdSystem: malformed register_human payload (expected 5 "
            << "'|'-separated fields), ignoring: '" << _data << "'" << std::endl;
      return false;
    }
    _out.name = fields[0];
    _out.cyclicGoals = fields[1] == "1";
    try
    {
      _out.desiredVelocity = std::stod(fields[2]);
      _out.radius = std::stod(fields[3]);
    }
    catch (const std::exception &)
    {
      gzerr << "SfmCrowdSystem: malformed numeric field in register_human payload, "
            << "ignoring: '" << _data << "'" << std::endl;
      return false;
    }
    std::stringstream pointStream(fields[4]);
    std::string pointToken;
    while (std::getline(pointStream, pointToken, ';'))
    {
      if (pointToken.empty())
        continue;
      const auto comma = pointToken.find(',');
      if (comma == std::string::npos)
        continue;
      try
      {
        const double x = std::stod(pointToken.substr(0, comma));
        const double y = std::stod(pointToken.substr(comma + 1));
        _out.goals.emplace_back(x, y);
      }
      catch (const std::exception &)
      {
        gzerr << "SfmCrowdSystem: malformed point '" << pointToken
              << "' in register_human payload, skipping just that point." << std::endl;
      }
    }
    if (_out.name.empty() || _out.goals.empty())
    {
      gzerr << "SfmCrowdSystem: register_human payload needs a name and at least "
            << "one point, ignoring: '" << _data << "'" << std::endl;
      return false;
    }
    return true;
  }

  /// \brief Adds (or, for an already-tracked name, replaces outright --
  /// HumanControlPanel re-registers whenever a route is edited in place
  /// rather than diffing it) one runtime-registered human. Caller (only
  /// PreUpdate(), draining pendingRegistrations) must already hold
  /// crowdMutex: this itself does not lock it, both because it mutates
  /// `humans`' structure directly and because SubscribeEnableTopic()'s
  /// callback also takes crowdMutex (not reentrant-safe otherwise).
  private: void ApplyRegistration(const RegistrationRequest &_request)
  {
    this->humans.erase(std::remove_if(this->humans.begin(), this->humans.end(),
        [&_request](const HumanAgent &_h) { return _h.name == _request.name; }),
        this->humans.end());

    HumanAgent human;
    human.name = _request.name;
    human.body.modelName = _request.name + "_collision";
    human.cmdVelTopic = "/" + _request.name + "/cmd_vel";
    human.sfmEnableTopic = "/" + _request.name + "/sfm_enable";
    human.agent.radius = _request.radius > 0.0 ? _request.radius : this->defaultAgentRadius;
    human.agent.desiredVelocity =
        _request.desiredVelocity > 0.0 ? _request.desiredVelocity : this->defaultDesiredVelocity;
    human.agent.cyclicGoals = _request.cyclicGoals;
    human.agent.teleoperated = false;
    human.agent.groupId = -1;
    for (const auto &point : _request.goals)
    {
      sfm::Goal goal;
      goal.center.set(point.first, point.second);
      goal.radius = 0.3;
      human.agent.goals.push_back(goal);
    }
    human.publisher = this->transportNode.Advertise<gz::msgs::Twist>(human.cmdVelTopic);
    this->SubscribeEnableTopic(human.name, human.sfmEnableTopic);

    gzmsg << "SfmCrowdSystem: registered '" << human.name << "' with "
          << human.agent.goals.size() << " waypoint(s), cyclic="
          << (human.agent.cyclicGoals ? "true" : "false") << "." << std::endl;
    this->humans.push_back(std::move(human));
  }

  private: gz::transport::Node transportNode;
  private: double defaultDesiredVelocity{1.0};
  private: double defaultAgentRadius{0.35};
  private: double obstacleRange{6.0};
  private: std::string registerTopic;
  private: std::string unregisterTopic;
  private: std::vector<WallSegment> walls;
  private: std::vector<TrackedBody> robots;
  private: std::vector<HumanAgent> humans;
  // Guards `humans`' structure (ApplyRegistration()'s/unregistration's
  // erase/push_back, always performed from PreUpdate() on the simulation
  // thread) together with every human's `enabled` flag and the two pending
  // queues below (both written from gz-transport's callback thread). One
  // mutex for all of it rather than a separate one per concern: none of
  // these operations are hot-path/high-frequency (registration/enable
  // toggles are user-triggered events, not a per-tick cost), so there is
  // nothing to gain from finer-grained locking here.
  private: std::mutex crowdMutex;
  private: std::vector<RegistrationRequest> pendingRegistrations;
  private: std::vector<std::string> pendingUnregistrations;
};
}  // namespace gz_human_sim

GZ_ADD_PLUGIN(gz_human_sim::SfmCrowdSystem, gz::sim::System,
    gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz_human_sim::SfmCrowdSystem, "gz_human_sim::SfmCrowdSystem")
