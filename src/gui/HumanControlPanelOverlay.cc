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

// DualSense/gamepad polling only -- SDL_INIT_GAMECONTROLLER (never
// SDL_INIT_VIDEO), so this never touches windowing/GL and cannot conflict
// with the already-running Ogre2/Qt scene. See PollDualsense(). Mirrors
// guide_robot's GuiderRobotManager, which this mode is modeled on.
#include <SDL2/SDL.h>

#include "HumanControlPanelInternal.hh"

// Spawn markers drawn into the render scene.
//
// Split out of the single-file HumanControlPanel.cc; the code below is
// unchanged from that file.

namespace gz_human_sim
{
void HumanControlPanel::setShowSpawnMarker(int _index, bool _value)
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return;
  this->humans.at(_index).showSpawnMarker = _value;
  // ApplySpawnMarkers() (render thread) picks this up next frame, same
  // deferred-to-the-render-thread arrangement setShowCollision() uses.
  this->humansChanged();
}

void HumanControlPanel::setShowSpawnMarkerAll(bool _value)
{
  for (auto &human : this->humans)
    human.showSpawnMarker = _value;
  this->humansChanged();
}

bool HumanControlPanel::showSpawnMarkerAt(int _index) const
{
  if (_index < 0 || _index >= static_cast<int>(this->humans.size()))
    return false;
  return this->humans.at(_index).showSpawnMarker;
}

QString HumanControlPanel::spawnMarkerColorAt(int _index) const
{
  int colorIndex = 0;
  if (_index >= 0 && _index < static_cast<int>(this->humans.size()))
    colorIndex = this->humans.at(_index).markerColorIndex;
  const auto &color = kMarkerColors[colorIndex % kMarkerColorCount];
  return QString("#%1%2%3")
      .arg(static_cast<int>(color[0] * 255.0), 2, 16, QChar('0'))
      .arg(static_cast<int>(color[1] * 255.0), 2, 16, QChar('0'))
      .arg(static_cast<int>(color[2] * 255.0), 2, 16, QChar('0'));
}

gz::rendering::VisualPtr HumanControlPanel::CreateMarkerVisual(
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
    if (!this->spawnMarkerTexturePath.empty())
    {
      material->SetTexture(this->spawnMarkerTexturePath);
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

void HumanControlPanel::ApplySpawnMarkers()
{
  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!scene)
    return;

  // Markers whose human was removed on the Qt thread -- destroying a scene
  // node is render-thread-only, hence the queue.
  {
    std::lock_guard<std::mutex> lock(this->markerMutex);
    for (const auto &name : this->markerRemovalQueue)
    {
      if (auto visual = scene->VisualByName(name))
        scene->DestroyVisual(visual);
    }
    this->markerRemovalQueue.clear();
  }

  for (auto &human : this->humans)
  {
    if (!human.markerVisual)
    {
      human.markerVisual = this->CreateMarkerVisual(
          scene, "__spawn_marker_" + human.name,
          human.spawnX, human.spawnY, human.spawnZ,
          human.markerColorIndex, false);
      if (!human.markerVisual)
        continue;
      // A freshly created visual is visible; force the desired state to be
      // pushed below rather than assumed, same tri-state reasoning
      // appliedShowCollision uses.
      human.appliedShowSpawnMarker = -1;
    }
    const int desired = human.showSpawnMarker ? 1 : 0;
    if (human.appliedShowSpawnMarker == desired)
      continue;
    human.markerVisual->SetVisible(human.showSpawnMarker);
    human.appliedShowSpawnMarker = desired;
  }

  // Picked-but-not-yet-spawned points get their own neutral markers, so
  // clicking a spawn point gives immediate feedback in the 3D view rather
  // than only a line of text in the panel. Rebuilt wholesale whenever the
  // list changes at all -- tracked by an explicit dirty flag rather than by
  // comparing sizes, since undoing one point and clicking a different one
  // leaves the count identical while every marker needs to move.
  if (this->pendingSpawnMarkersDirty)
  {
    this->pendingSpawnMarkersDirty = false;
    for (auto &visual : this->pendingSpawnMarkerVisuals)
    {
      if (visual)
        scene->DestroyVisual(visual);
    }
    this->pendingSpawnMarkerVisuals.clear();
    for (std::size_t i = 0; i < this->pendingSpawnPoints.size(); ++i)
    {
      this->pendingSpawnMarkerVisuals.push_back(this->CreateMarkerVisual(
          scene, "__spawn_pending_" + std::to_string(i),
          this->pendingSpawnPoints[i].x, this->pendingSpawnPoints[i].y,
          this->pendingSpawnPoints[i].z, 0, true));
    }
  }

  // Spin every marker together. Driven from elapsed wall time rather than
  // a per-frame increment so the rate does not follow the GUI's frame rate.
  if (!this->spawnMarkerEpochValid)
  {
    this->spawnMarkerEpoch = std::chrono::steady_clock::now();
    this->spawnMarkerEpochValid = true;
  }
  const double angle = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - this->spawnMarkerEpoch).count()
      * kMarkerSpinRate;
  for (auto &human : this->humans)
  {
    if (human.markerVisual)
      human.markerVisual->SetLocalRotation(0.0, 0.0, angle);
  }
  for (auto &visual : this->pendingSpawnMarkerVisuals)
  {
    if (visual)
      visual->SetLocalRotation(0.0, 0.0, angle);
  }
}
}  // namespace gz_human_sim
