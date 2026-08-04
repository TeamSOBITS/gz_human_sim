#include "HumanControlPanel.hh"

#include <mutex>
#include <string>

#include <QString>
#include <QTimer>

#include <gz/rendering/RenderingIface.hh>

#include "HumanControlPanelInternal.hh"

// 当たり判定カプセルの QML 窓口と、人物ごとの表示状態の管理。
//
// SDF の組み立てとシーン走査は CollisionBodyController にある。ここに
// 残っているのは **「どの人物に当たり判定モデルが付いているか」**という、
// 人物を知っている側にしか判断できない部分だけ（分けた理由は
// CollisionBodyController.hh の頭を参照）。
//
// アクター由来の人物にしか当たり判定モデルは付かない。その判定に
// velocityPublisher.Valid() を使っているのは isHumanActorAt() と同じ。

namespace gz_human_sim
{
bool HumanControlPanel::showCollisionAt(int _index) const
{
  if (!this->humans.valid(_index))
    return false;
  return this->humans.at(_index).collision.show;
}

void HumanControlPanel::setShowCollision(int _index, bool _value)
{
  if (!this->humans.valid(_index))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;
  human.collision.show = _value;
  // A previous search that ran out of retries (kCollisionVisualMaxRetries)
  // must not make this human's button dead forever -- an explicit toggle
  // is exactly the moment to start looking again.
  human.collision.retries = 0;
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
      human.collision.show = _value;
      human.collision.retries = 0;
    }
  }
  this->humansChanged();
}

double HumanControlPanel::collisionRadiusAt(int _index) const
{
  if (!this->humans.valid(_index))
    return 0.25;
  return this->humans.at(_index).collision.radius;
}

double HumanControlPanel::collisionLengthAt(int _index) const
{
  if (!this->humans.valid(_index))
    return 1.2;
  return this->humans.at(_index).collision.length;
}

void HumanControlPanel::applyCollisionSize(int _index, double _radius, double _length)
{
  if (!this->humans.valid(_index))
    return;
  auto &human = this->humans.at(_index);
  if (!human.velocityPublisher.Valid())
    return;  // Static (non-actor) humans have no collision-body companion.
  if (this->worldName.empty() || !this->collisionBody.HasTemplate())
    return;

  // Same naming convention spawn_human.launch.py's _spawn_human_cmd()
  // uses (collision_model_name = f'{model_name}_collision') -- derived
  // here rather than stored on Human, since it's fully determined by the
  // human's own name. cmd_vel トピックは BuildSdf() が同じ規則で作る。
  const std::string collisionModelName = human.name + "_collision";

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

  const std::string sdf =
      this->collisionBody.BuildSdf(collisionModelName, _radius, _length);

  this->RequestEntityRemoval(collisionModelName);
  // The replacement model brings brand-new scene visuals with it, which
  // start out visible regardless of what this human's toggle says -- reset
  // to "nothing applied yet" so ApplyCollisionVisibility() pushes the
  // current state onto them once they appear.
  human.collision.applied = -1;
  human.collision.retries = 0;
  human.collision.radius = _radius;
  human.collision.length = _length;

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

    const int desired = human.collision.show ? 1 : 0;
    if (human.collision.applied == desired)
      continue;
    if (human.collision.retries > kCollisionVisualMaxRetries)
      continue;

    const std::string collisionModelName = human.name + "_collision";
    if (!this->collisionBody.SetVisible(
          scene, collisionModelName, human.collision.show))
    {
      // 診断は「探索を最初からやり直した回」だけ。毎フレーム出すと
      // ログが埋まる。
      if (human.collision.retries == 0)
        this->collisionBody.LogMissing(scene, collisionModelName);
      ++human.collision.retries;
      continue;
    }
    human.collision.applied = desired;
    human.collision.retries = 0;
  }
}
}  // namespace gz_human_sim
