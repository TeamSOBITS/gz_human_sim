#include "SpawnMarkerRenderer.hh"

#include <QChar>

#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

// 中身は HumanControlPanelOverlay.cc からそのまま移したもので、
// 振る舞いは変えていない。人物ごとのループだけがパネル側に残っている。

namespace gz_human_sim
{
void SpawnMarkerRenderer::SetTexturePath(const std::string &_path)
{
  this->texturePath = _path;
}

void SpawnMarkerRenderer::QueueRemoval(const std::string &_name)
{
  std::lock_guard<std::mutex> lock(this->removalMutex);
  this->removalQueue.push_back(_name);
}

void SpawnMarkerRenderer::InvalidatePending()
{
  this->pendingDirty = true;
}

bool SpawnMarkerRenderer::ConsumePendingDirty()
{
  if (!this->pendingDirty)
    return false;
  this->pendingDirty = false;
  return true;
}

double SpawnMarkerRenderer::BeginFrame(const gz::rendering::ScenePtr &_scene)
{
  // Markers whose human was removed on the Qt thread -- destroying a scene
  // node is render-thread-only, hence the queue.
  {
    std::lock_guard<std::mutex> lock(this->removalMutex);
    for (const auto &name : this->removalQueue)
    {
      if (auto visual = _scene->VisualByName(name))
        _scene->DestroyVisual(visual);
    }
    this->removalQueue.clear();
  }

  // Spin every marker together. Driven from elapsed wall time rather than
  // a per-frame increment so the rate does not follow the GUI's frame rate.
  if (!this->epochValid)
  {
    this->epoch = std::chrono::steady_clock::now();
    this->epochValid = true;
  }
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - this->epoch).count() * kMarkerSpinRate;
}

gz::rendering::VisualPtr SpawnMarkerRenderer::Create(
    const gz::rendering::ScenePtr &_scene, const std::string &_name,
    double _x, double _y, double _z, int _colorIndex, bool _pending) const
{
  if (!_scene)
    return nullptr;
  // A stale visual under this name (same human name respawned, a marker
  // whose removal is still queued, ...) would make CreateVisual() fail.
  if (auto existing = _scene->VisualByName(_name))
    _scene->DestroyVisual(existing);

  auto visual = _scene->CreateVisual(_name);
  if (!visual)
    return nullptr;

  auto material = _scene->CreateMaterial();
  if (material)
  {
    const double alpha = _pending ? kPendingMarkerAlpha : kMarkerAlpha;
    double r = 1.0, g = 1.0, b = 1.0;
    if (!_pending)
    {
      const auto &color = kMarkerColors[_colorIndex % kMarkerColorCount];
      r = color[0];
      g = color[1];
      b = color[2];
    }
    if (!this->texturePath.empty())
    {
      material->SetTexture(this->texturePath);
      // Without this the PNG's alpha is ignored and the marker renders as
      // an opaque square instead of a circle.
      material->SetAlphaFromTexture(true);
    }
    // The texture is white, so these tint it rather than replace it.
    material->SetAmbient(r, g, b, alpha);
    material->SetDiffuse(r, g, b, alpha);
    // Emissive as well as diffuse so the circle keeps its identity colour
    // in shadow -- it's a label, and a label that goes dark under a table
    // is no longer telling anyone which person it belongs to.
    // Emissive at full tint rather than a fraction of it: these worlds are
    // scanned building interiors that are dimly lit, and a marker relying
    // on diffuse alone reads as a grey smudge indoors. Self-lighting it
    // makes the circle look the same on a dark floor as on a bright one.
    material->SetEmissive(r, g, b);
    material->SetTransparency(1.0 - alpha);
    // Without this the transparency above is ignored by ogre2.
    material->SetDepthWriteEnabled(false);
    material->SetCastShadows(false);
  }

  // One textured plane rather than a disc built from primitives: the whole
  // design (rings, seal, ticks) lives in the texture, so a world full of
  // markers costs one quad each.
  auto geometry = _scene->CreatePlane();
  if (!geometry)
  {
    _scene->DestroyVisual(visual);
    return nullptr;
  }
  if (material)
    geometry->SetMaterial(material);
  visual->AddGeometry(geometry);
  // CreatePlane() is already a unit quad in XY, the orientation a floor
  // marker wants; only size and height need setting. kMarkerZ lifts it
  // clear of the slab so it does not z-fight with the floor.
  visual->SetLocalScale(kMarkerRadius * 2.0, kMarkerRadius * 2.0, 1.0);
  visual->SetLocalPosition(_x, _y, _z + kMarkerZ);
  // Purely decorative: never let a click in the 3D view select the marker
  // instead of what's behind it (that click is how spawn points and route
  // points get placed in the first place -- see eventFilter()).
  visual->SetUserData("gui-only", true);
  _scene->RootVisual()->AddChild(visual);
  return visual;
}

void SpawnMarkerRenderer::ClearPending(const gz::rendering::ScenePtr &_scene)
{
  for (auto &visual : this->pendingVisuals)
  {
    if (visual)
      _scene->DestroyVisual(visual);
  }
  this->pendingVisuals.clear();
}

void SpawnMarkerRenderer::AddPending(const gz::rendering::ScenePtr &_scene,
    std::size_t _index, double _x, double _y, double _z)
{
  this->pendingVisuals.push_back(this->Create(
      _scene, "__spawn_pending_" + std::to_string(_index), _x, _y, _z, 0, true));
}

void SpawnMarkerRenderer::SpinPending(double _angle)
{
  for (auto &visual : this->pendingVisuals)
  {
    if (visual)
      visual->SetLocalRotation(0.0, 0.0, _angle);
  }
}

QString SpawnMarkerRenderer::ColorHex(int _colorIndex)
{
  const auto &color = kMarkerColors[_colorIndex % kMarkerColorCount];
  return QString("#%1%2%3")
      .arg(static_cast<int>(color[0] * 255.0), 2, 16, QChar('0'))
      .arg(static_cast<int>(color[1] * 255.0), 2, 16, QChar('0'))
      .arg(static_cast<int>(color[2] * 255.0), 2, 16, QChar('0'));
}
}  // namespace gz_human_sim
