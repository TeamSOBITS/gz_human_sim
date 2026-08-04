#include "CollisionBodyController.hh"

#include <utility>

#include <gz/common/Console.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

// 中身は HumanControlPanelCollision.cc と HumanControlPanelSpawn.cc から
// そのまま移したもので、振る舞いは変えていない。

namespace gz_human_sim
{
namespace
{
// human_collision_body/model.sdf に書かれているリテラル。SDF 側を変えたら
// ここも変えること。scripts/human_model_utils.py がトピック差し替えで
// 使っているのと同じ、素朴な文字列置換の方針。
const char *const kNamePlaceholder = "<model name=\"human_collision_body\">";
const char *const kTopicPlaceholder =
    "<topic>/model/human_collision_body/cmd_vel</topic>";
const char *const kRadiusPlaceholder = "<radius>0.25</radius>";
const char *const kLengthPlaceholder = "<length>1.2</length>";
}  // namespace

void CollisionBodyController::SetTemplate(std::string _sdf)
{
  this->sdfTemplate = std::move(_sdf);
}

bool CollisionBodyController::HasTemplate() const
{
  return !this->sdfTemplate.empty();
}

std::string CollisionBodyController::BuildSdf(const std::string &_modelName,
    double _radius, double _length) const
{
  std::string sdf = this->sdfTemplate;

  const std::string namePlaceholder = kNamePlaceholder;
  const auto namePos = sdf.find(namePlaceholder);
  if (namePos != std::string::npos)
  {
    sdf.replace(namePos, namePlaceholder.size(),
        "<model name=\"" + _modelName + "\">");
  }

  // Give this model its OWN VelocityControl topic -- reusing the
  // template's unmodified placeholder topic would make every simultaneous
  // probe (and any real, already-spawned collision body still using the
  // template's literal default) fight over the same one.
  const std::string topicPlaceholder = kTopicPlaceholder;
  const auto topicPos = sdf.find(topicPlaceholder);
  if (topicPos != std::string::npos)
  {
    sdf.replace(topicPos, topicPlaceholder.size(),
        "<topic>/model/" + _modelName + "/cmd_vel</topic>");
  }

  if (_radius < 0.0 && _length < 0.0)
    return sdf;

  // <radius>/<length> each appear twice in the template (the <collision>
  // and its matching debug <visual>, see human_collision_body/model.sdf)
  // -- replace every occurrence, same pragmatic literal-value
  // substitution scripts/human_model_utils.py already relies on for topics.
  if (_radius >= 0.0)
  {
    const std::string placeholder = kRadiusPlaceholder;
    const std::string replacement =
        "<radius>" + std::to_string(_radius) + "</radius>";
    for (auto pos = sdf.find(placeholder); pos != std::string::npos;
        pos = sdf.find(placeholder, pos + replacement.size()))
      sdf.replace(pos, placeholder.size(), replacement);
  }
  if (_length >= 0.0)
  {
    const std::string placeholder = kLengthPlaceholder;
    const std::string replacement =
        "<length>" + std::to_string(_length) + "</length>";
    for (auto pos = sdf.find(placeholder); pos != std::string::npos;
        pos = sdf.find(placeholder, pos + replacement.size()))
      sdf.replace(pos, placeholder.size(), replacement);
  }
  return sdf;
}

bool CollisionBodyController::SetVisible(
    const gz::rendering::ScenePtr &_scene, const std::string &_modelName,
    bool _visible) const
{
  // gz-sim's SceneManager names a model's visuals with "::"-scoped paths
  // ("<model>", "<model>::<link>", "<model>::<link>::<visual>", sometimes
  // with a further scope prefix in front). Match all three shapes rather
  // than any one of them, and apply to every hit -- picking a single
  // "the" visual is what broke this before.
  const std::string prefix = _modelName + "::";
  const std::string suffix = "::" + _modelName;
  bool found = false;
  for (unsigned int i = 0; i < _scene->VisualCount(); ++i)
  {
    auto visual = _scene->VisualByIndex(i);
    if (!visual)
      continue;
    const std::string &name = visual->Name();
    const bool exact = name == _modelName;
    const bool scopedChild = name.compare(0, prefix.size(), prefix) == 0;
    const bool scopedSelf = name.size() > suffix.size() &&
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
    // A scoped parent ("world::human1_collision") also has scoped children
    // ("world::human1_collision::body"), which neither of the two checks
    // above catches -- hence the plain containment test for that one case.
    const bool scopedDescendant =
        name.find(suffix + "::") != std::string::npos;
    if (!exact && !scopedChild && !scopedSelf && !scopedDescendant)
      continue;
    visual->SetVisible(_visible);
    found = true;
  }
  return found;
}

void CollisionBodyController::LogMissing(
    const gz::rendering::ScenePtr &_scene, const std::string &_modelName) const
{
  // The companion model normally just needs a few more frames to appear,
  // but if it never does, dumping every scene visual whose name mentions
  // "collision" shows what naming scheme this gz-sim version's scene
  // actually uses.
  gzmsg << "[HumanControlPanel] collision-visual '" << _modelName
        << "' not found among " << _scene->VisualCount()
        << " scene visuals. Names containing \"collision\": ";
  unsigned int logged = 0;
  for (unsigned int i = 0; i < _scene->VisualCount() && logged < 40; ++i)
  {
    auto visual = _scene->VisualByIndex(i);
    if (visual && visual->Name().find("collision") != std::string::npos)
    {
      gzmsg << "[" << visual->Name() << "] ";
      ++logged;
    }
  }
  if (logged == 0)
    gzmsg << "(none)";
  gzmsg << std::endl;
}
}  // namespace gz_human_sim
