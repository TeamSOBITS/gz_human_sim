// HumanControlPanelCollision.cc -- 当たり判定カプセルの表示とサイズ。
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
bool HumanControlPanel::showCollisionAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(_index).showCollision;
}

void HumanControlPanel::setShowCollision(int _index, bool _value)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;
  human.showCollision = _value;
  // A previous search that ran out of retries (kCollisionVisualMaxRetries)
  // must not make this human's button dead forever -- an explicit toggle
  // is exactly the moment to start looking again.
  human.collisionVisualRetries = 0;
  // ApplyCollisionVisibility() (render thread, driven off Render events)
  // picks this up and applies it next frame -- see its own comment for
  // why this can't just be done synchronously here. humansChanged() lets
  // every row's QML button (not just this one, e.g. after
  // setShowCollisionAll()) resync its displayed state.
  this->humansChanged();
}

void HumanControlPanel::setShowCollisionAll(bool _value)
{
  for (auto &human : this->humans)
  {
    if (human.velocityPublisher.Valid())
    {
      human.showCollision = _value;
      human.collisionVisualRetries = 0;
    }
  }
  this->humansChanged();
}

double HumanControlPanel::collisionRadiusAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return 0.25;
  return this->humans.at(_index).collisionRadius;
}

double HumanControlPanel::collisionLengthAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return 1.2;
  return this->humans.at(_index).collisionLength;
}

void HumanControlPanel::applyCollisionSize(int _index, double _radius, double _length)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;  // Static (non-actor) humans have no collision-body companion.
  if (this->worldName.empty() || this->collisionBodyTemplate.empty())
    return;

  // Same naming convention spawn_human.launch.py's _spawn_human_cmd()
  // uses (collision_model_name = f'{model_name}_collision', topic =
  // f'/model/{collision_model_name}/cmd_vel') -- derived here rather than
  // stored on Human, since it's fully determined by the human's own name.
  const std::string collisionModelName = human.name + "_collision";
  const std::string collisionCmdVelTopic = "/model/" + collisionModelName + "/cmd_vel";

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->poseMutex);
    const auto it = this->poses.find(collisionModelName);
    if (it != this->poses.end() && it->second.valid)
    {
      x = it->second.x;
      y = it->second.y;
      z = it->second.z;
    }
  }

  // Same template ProbeSafeSpawnPosition() reuses for its throwaway
  // probes, but with the real per-human name/topic substituted (not a
  // probe placeholder) plus the new capsule dimensions.
  std::string sdf = this->collisionBodyTemplate;
  const std::string namePlaceholder = "<model name=\"human_collision_body\">";
  const auto namePos = sdf.find(namePlaceholder);
  if (namePos != std::string::npos)
  {
    sdf.replace(namePos, namePlaceholder.size(),
        "<model name=\"" + collisionModelName + "\">");
  }
  const std::string topicPlaceholder =
      "<topic>/model/human_collision_body/cmd_vel</topic>";
  const auto topicPos = sdf.find(topicPlaceholder);
  if (topicPos != std::string::npos)
  {
    sdf.replace(topicPos, topicPlaceholder.size(),
        "<topic>" + collisionCmdVelTopic + "</topic>");
  }
  // <radius>/<length> each appear twice in the template (the <collision>
  // and its matching debug <visual>, see human_collision_body/model.sdf)
  // -- replace every occurrence, same pragmatic literal-value
  // substitution scripts/human_model_utils.py already relies on for topics.
  const std::string radiusStr = std::to_string(_radius);
  const std::string radiusPlaceholder = "<radius>0.25</radius>";
  const std::string newRadius = "<radius>" + radiusStr + "</radius>";
  for (auto pos = sdf.find(radiusPlaceholder); pos != std::string::npos;
      pos = sdf.find(radiusPlaceholder, pos + newRadius.size()))
    sdf.replace(pos, radiusPlaceholder.size(), newRadius);
  const std::string lengthStr = std::to_string(_length);
  const std::string lengthPlaceholder = "<length>1.2</length>";
  const std::string newLength = "<length>" + lengthStr + "</length>";
  for (auto pos = sdf.find(lengthPlaceholder); pos != std::string::npos;
      pos = sdf.find(lengthPlaceholder, pos + newLength.size()))
    sdf.replace(pos, lengthPlaceholder.size(), newLength);

  this->RequestEntityRemoval(collisionModelName);
  // The replacement model brings brand-new scene visuals with it, which
  // start out visible regardless of what this human's toggle says -- reset
  // to "nothing applied yet" so ApplyCollisionVisibility() pushes the
  // current state onto them once they appear.
  human.appliedShowCollision = -1;
  human.collisionVisualRetries = 0;
  human.collisionRadius = _radius;
  human.collisionLength = _length;

  // A create request for this name landing before the remove request has
  // actually been processed server-side would collide with the entity
  // still being torn down -- give it a moment, same "async operations
  // need explicit settling time" approach kSpawnSafetySettleMs already
  // uses elsewhere in this file.
  QTimer::singleShot(500, this,
      [this, sdf, collisionModelName, x, y, z]()
      {
        this->RequestEntityCreation(sdf, collisionModelName, x, y, z);
      });
  this->SetStatus(QString::fromStdString(human.name) +
      " の当たり判定サイズを変更しました（半径" + QString::number(_radius, 'f', 2) +
      "m・長さ" + QString::number(_length, 'f', 2) + "m）");
}

bool HumanControlPanel::SetCollisionBodyVisible(
    const gz::rendering::ScenePtr &_scene, const std::string &_modelName,
    bool _visible) const
{
  // gz-sim's SceneManager names a model's visuals with "::"-scoped paths
  // ("<model>", "<model>::<link>", "<model>::<link>::<visual>", sometimes
  // with a further scope prefix in front). Match all three shapes rather
  // than any one of them, and apply to every hit -- see the header for why
  // picking a single "the" visual is what broke this before.
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

void HumanControlPanel::ApplyCollisionVisibility()
{
  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!scene)
    return;

  for (auto &human : this->humans)
  {
    // Only actor-backed humans get a human_collision_body companion at
    // all (see spawn_human.launch.py) -- same check isHumanActorAt() uses.
    if (!human.velocityPublisher.Valid())
      continue;

    const int desired = human.showCollision ? 1 : 0;
    if (human.appliedShowCollision == desired)
      continue;
    if (human.collisionVisualRetries > kCollisionVisualMaxRetries)
      continue;

    const std::string collisionModelName = human.name + "_collision";
    if (!this->SetCollisionBodyVisible(scene, collisionModelName, human.showCollision))
    {
      // Diagnostic (fires once per search that starts from scratch, not
      // every retried frame): the companion model normally just needs a
      // few more frames to appear, but if it never does, dumping every
      // scene visual whose name mentions "collision" shows what naming
      // scheme this gz-sim version's scene actually uses.
      if (human.collisionVisualRetries == 0)
      {
        gzmsg << "[HumanControlPanel] collision-visual '" << collisionModelName
              << "' not found among " << scene->VisualCount()
              << " scene visuals. Names containing \"collision\": ";
        unsigned int logged = 0;
        for (unsigned int i = 0; i < scene->VisualCount() && logged < 40; ++i)
        {
          auto visual = scene->VisualByIndex(i);
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
      ++human.collisionVisualRetries;
      continue;
    }
    human.appliedShowCollision = desired;
    human.collisionVisualRetries = 0;
  }
}
}  // namespace gz_human_sim
