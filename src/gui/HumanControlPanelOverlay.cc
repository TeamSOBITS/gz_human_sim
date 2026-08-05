#include "HumanControlPanel.hh"

#include <cstddef>

#include <QString>

#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Visual.hh>

#include "HumanControlPanelInternal.hh"

// スポーンマーカーの QML 窓口と、人物ごとのマーカーの割り当て。
//
// 描く仕事そのものは unified_entity_gui::SpawnMarkerRenderer にある
// （構想書 §12）。ここに残っているのは **「どの人物にどのマーカーが
// 要るか」**という、人物を知っている側にしか判断できない部分だけ。

namespace gz_human_sim
{
void HumanControlPanel::setShowSpawnMarker(int _index, bool _value)
{
  if (!this->humans.valid(_index))
    return;
  this->humans.at(_index).marker.show = _value;
  // ApplySpawnMarkers() (render thread) picks this up next frame, same
  // deferred-to-the-render-thread arrangement setShowCollision() uses.
  this->humansChanged();
}

void HumanControlPanel::setShowSpawnMarkerAll(bool _value)
{
  for (auto &human : this->humans)
    human.marker.show = _value;
  this->humansChanged();
}

bool HumanControlPanel::showSpawnMarkerAt(int _index) const
{
  if (!this->humans.valid(_index))
    return false;
  return this->humans.at(_index).marker.show;
}

QString HumanControlPanel::spawnMarkerColorAt(int _index) const
{
  int colorIndex = 0;
  if (this->humans.valid(_index))
    colorIndex = this->humans.at(_index).marker.colorIndex;
  return unified_entity_gui::SpawnMarkerRenderer::ColorHex(colorIndex);
}

void HumanControlPanel::ApplySpawnMarkers()
{
  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!scene)
    return;

  // 溜まっていた破棄を片付け、このフレームの回転角をもらう。
  const double angle = this->spawnMarkers.BeginFrame(scene);

  for (auto &human : this->humans)
  {
    if (!human.marker.visual)
    {
      human.marker.visual = this->spawnMarkers.Create(
          scene, "__spawn_marker_" + human.name,
          human.marker.x, human.marker.y, human.marker.z,
          human.marker.colorIndex, false);
      if (!human.marker.visual)
        continue;
      // A freshly created visual is visible; force the desired state to be
      // pushed below rather than assumed, same tri-state reasoning
      // appliedShowCollision uses.
      human.marker.applied = -1;
    }
    const int desired = human.marker.show ? 1 : 0;
    if (human.marker.applied == desired)
      continue;
    human.marker.visual->SetVisible(human.marker.show);
    human.marker.applied = desired;
  }

  // Picked-but-not-yet-spawned points get their own neutral markers, so
  // clicking a spawn point gives immediate feedback in the 3D view rather
  // than only a line of text in the panel. Rebuilt wholesale whenever the
  // list changes at all -- see SpawnMarkerRenderer::InvalidatePending().
  if (this->spawnMarkers.ConsumePendingDirty())
  {
    this->spawnMarkers.ClearPending(scene);
    for (std::size_t i = 0; i < this->pendingSpawnPoints.size(); ++i)
    {
      this->spawnMarkers.AddPending(scene, i,
          this->pendingSpawnPoints[i].x, this->pendingSpawnPoints[i].y,
          this->pendingSpawnPoints[i].z);
    }
  }

  for (auto &human : this->humans)
  {
    if (human.marker.visual)
      human.marker.visual->SetLocalRotation(0.0, 0.0, angle);
  }
  this->spawnMarkers.SpinPending(angle);
}
}  // namespace gz_human_sim
