// Collision-aware global path planning for gz_human_sim humans.
//
// Why this exists: ActorCommandPlugin::ApplyPathCommand() steers straight at
// the next waypoint. That is a *local controller* with no *global planner*
// behind it, so a route whose straight leg crosses a wall walks the human
// into (or through) that wall, and even the SFM crowd system -- which only
// pushes back reactively -- gets stuck in the concave corner behind it.
//
// The structure here mirrors the Nav2 stack this workspace already uses for
// its robots (see src/sobits_navigation_stack/sobits_nav/param/*/
// navigation_config.yaml), one layer at a time:
//
//   nav2_costmap_2d static_layer   -> BuildGrid(), rasterised from the
//                                     world's own collision geometry instead
//                                     of a pre-recorded .pgm map, so it can
//                                     never drift out of sync with the world
//                                     and needs no per-world map authoring.
//   nav2_costmap_2d inflation_layer-> InflateGrid(), growing obstacles by the
//                                     walker's own body radius so a path is
//                                     only ever planned where the body fits.
//   nav2_navfn_planner NavfnPlanner-> PlanCell(), A* over that grid.
//   nav2_smoother SimpleSmoother   -> SmoothPath(), line-of-sight shortcutting
//                                     so the result is a few long straight
//                                     legs rather than a staircase of cells.
//
// What it deliberately does NOT do is replace the local controller: the
// planned waypoints are handed straight to the existing cmd_path follow (or
// to SfmCrowdSystem), so ActorCommandPlugin needs no changes at all -- it
// simply receives waypoints that already go around things.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Sphere.hh>

namespace gz_human_sim
{
// Grid cell size, in metres. 0.05 is what sobits_nav's costmaps use; 0.10 is
// plenty here because the thing being planned for is a person walking through
// furniture rather than a robot threading a doorway at speed, and it keeps a
// whole arena's grid comfortably small (a 40x40 m world is 400x400 cells).
static constexpr double kDefaultResolution = 0.10;

// How far obstacles are grown before planning, in metres -- the human's body
// half-width plus a little clearance. Same role as sobits_nav's
// inflation_radius, and the reason a planned path never clips a table corner
// even though the planner itself only ever tests the centre point of a cell.
static constexpr double kDefaultInflationRadius = 0.35;

// Only collisions that overlap this height band block a walking person.
// Without it, the floor itself (and any low lip or threshold) would fill the
// entire grid, and a high shelf or ceiling beam nobody has to duck under
// would carve out corridors that are actually walkable.
static constexpr double kDefaultZMin = 0.15;
static constexpr double kDefaultZMax = 1.70;

// Padding around the outermost obstacle, so a goal just outside the furniture
// still lands inside the grid.
static constexpr double kGridMargin = 3.0;

// Guard against a pathological world (or a stray enormous collision) turning
// into a multi-gigabyte grid.
static constexpr int kMaxGridCells = 4000000;

/// \brief One rasterised, inflated occupancy grid plus the A* over it.
class NavGridSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventManager) override;

  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Rasterise every world collision that a walking person would hit
  /// into this->grid. Runs on the server thread with ECM access.
  private: void BuildGrid(const gz::sim::EntityComponentManager &_ecm);

  /// \brief Grow occupied cells by inflationRadius (the "inflation layer").
  /// A single multi-source BFS over the whole grid rather than stamping a
  /// disc per obstacle cell: linear in cells regardless of how much of the
  /// world is occupied.
  private: void InflateGrid();

  /// \brief Service handler: plan a collision-free path through a sequence of
  /// waypoints. Reads only the cached grid, so it never touches the ECM and
  /// is safe to answer from the transport thread.
  private: bool OnPlanPath(
      const gz::msgs::StringMsg &_request, gz::msgs::Pose_V &_response);

  /// \brief A* between two world points, returning grid cells. Empty if
  /// unreachable.
  private: std::vector<std::pair<int, int>> PlanCell(
      int _startX, int _startY, int _goalX, int _goalY) const;

  /// \brief Drop waypoints that a straight line can skip (the "smoother").
  private: std::vector<std::pair<int, int>> SmoothPath(
      const std::vector<std::pair<int, int>> &_cells) const;

  /// \brief Whether the straight segment between two cells stays clear --
  /// Bresenham, used by SmoothPath().
  private: bool LineOfSight(int _x0, int _y0, int _x1, int _y1) const;

  /// \brief Nearest free cell to (_x, _y), spiralling outward. Lets a goal
  /// clicked on top of a table still plan to the floor beside it instead of
  /// failing outright -- the same intent as Nav2's goal tolerance.
  private: bool NearestFree(int &_x, int &_y) const;

  private: bool WorldToCell(double _wx, double _wy, int &_cx, int &_cy) const;
  private: void CellToWorld(int _cx, int _cy, double &_wx, double &_wy) const;
  private: bool Occupied(int _cx, int _cy) const;

  private: gz::transport::Node node;
  private: std::string planService{"/gz_human_sim/nav/plan_path"};

  private: double resolution{kDefaultResolution};
  private: double inflationRadius{kDefaultInflationRadius};
  private: double zMin{kDefaultZMin};
  private: double zMax{kDefaultZMax};

  /// \brief Grid state, guarded because BuildGrid() writes it on the server
  /// thread while OnPlanPath() reads it on a transport thread.
  private: mutable std::mutex gridMutex;
  private: std::vector<uint8_t> grid;      // 1 = blocked (after inflation)
  private: std::vector<uint8_t> rawGrid;   // 1 = blocked (before inflation)
  private: int gridWidth{0};
  private: int gridHeight{0};
  private: double originX{0.0};
  private: double originY{0.0};
  private: bool gridReady{false};
  private: bool rebuildRequested{true};
};

void NavGridSystem::Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &)
{
  if (_sdf)
  {
    this->resolution =
        _sdf->Get<double>("resolution", this->resolution).first;
    this->inflationRadius =
        _sdf->Get<double>("inflation_radius", this->inflationRadius).first;
    this->zMin = _sdf->Get<double>("z_min", this->zMin).first;
    this->zMax = _sdf->Get<double>("z_max", this->zMax).first;
    this->planService =
        _sdf->Get<std::string>("plan_service", this->planService).first;
  }
  this->resolution = std::max(0.02, this->resolution);

  if (!this->node.Advertise(this->planService, &NavGridSystem::OnPlanPath, this))
  {
    gzerr << "NavGridSystem: failed to advertise [" << this->planService << "]"
          << std::endl;
  }
  else
  {
    gzmsg << "NavGridSystem: path planning available on [" << this->planService
          << "] (resolution " << this->resolution << " m, inflation "
          << this->inflationRadius << " m)" << std::endl;
  }

  // Lets the GUI ask for a fresh rasterisation after furniture is moved or
  // added, without restarting the world.
  this->node.Subscribe<gz::msgs::StringMsg>(
      "/gz_human_sim/nav/rebuild",
      [this](const gz::msgs::StringMsg &)
      {
        std::lock_guard<std::mutex> lock(this->gridMutex);
        this->rebuildRequested = true;
      });
}

void NavGridSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;
  {
    std::lock_guard<std::mutex> lock(this->gridMutex);
    if (!this->rebuildRequested)
      return;
    this->rebuildRequested = false;
  }
  this->BuildGrid(_ecm);
}

void NavGridSystem::BuildGrid(const gz::sim::EntityComponentManager &_ecm)
{
  // Pass 1: collect the world-frame footprint of every collision a walking
  // person could hit, so pass 2 knows how big the grid has to be. Footprints
  // are kept as (centre, half-extents, yaw) boxes or (centre, radius) circles
  // rather than being flattened to an AABB straight away: a wall at 45
  // degrees would otherwise block a square twice its real width.
  struct Footprint
  {
    double x{0.0};
    double y{0.0};
    double halfX{0.0};   // box: half length along its own X; circle: radius
    double halfY{0.0};   // box: half length along its own Y; circle: radius
    double yaw{0.0};
    bool circle{false};
  };
  std::vector<Footprint> footprints;

  double minX = std::numeric_limits<double>::max();
  double minY = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double maxY = std::numeric_limits<double>::lowest();

  _ecm.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Collision *,
          const gz::sim::components::Geometry *_geom) -> bool
      {
        if (!_geom)
          return true;
        const sdf::Geometry &geometry = _geom->Data();

        // Skip anything that belongs to a human this package spawned: the
        // people are what's being planned FOR, and their own capsule bodies
        // (plus the throwaway spawn probes) would otherwise wall themselves
        // in. Matched on the owning model's name, which is exactly what
        // HumanControlPanel/spawn_human.launch.py control.
        std::string ownerName;
        gz::sim::Entity ancestor = _entity;
        while (ancestor != gz::sim::kNullEntity)
        {
          const auto *nameComp =
              _ecm.Component<gz::sim::components::Name>(ancestor);
          if (nameComp)
            ownerName = nameComp->Data();
          const auto *parent =
              _ecm.Component<gz::sim::components::ParentEntity>(ancestor);
          if (!parent)
            break;
          ancestor = parent->Data();
        }
        if (ownerName.find("_collision") != std::string::npos ||
            ownerName.rfind("__spawn_probe_", 0) == 0)
        {
          return true;
        }

        const auto pose = gz::sim::worldPose(_entity, _ecm);
        const double centreZ = pose.Pos().Z();

        Footprint footprint;
        footprint.x = pose.Pos().X();
        footprint.y = pose.Pos().Y();
        footprint.yaw = pose.Rot().Yaw();
        double halfHeight = 0.0;

        switch (geometry.Type())
        {
          case sdf::GeometryType::BOX:
          {
            if (!geometry.BoxShape())
              return true;
            const auto size = geometry.BoxShape()->Size();
            footprint.halfX = size.X() * 0.5;
            footprint.halfY = size.Y() * 0.5;
            halfHeight = size.Z() * 0.5;
            break;
          }
          case sdf::GeometryType::CYLINDER:
          {
            if (!geometry.CylinderShape())
              return true;
            footprint.circle = true;
            footprint.halfX = geometry.CylinderShape()->Radius();
            footprint.halfY = footprint.halfX;
            halfHeight = geometry.CylinderShape()->Length() * 0.5;
            break;
          }
          case sdf::GeometryType::CAPSULE:
          {
            if (!geometry.CapsuleShape())
              return true;
            footprint.circle = true;
            footprint.halfX = geometry.CapsuleShape()->Radius();
            footprint.halfY = footprint.halfX;
            halfHeight = geometry.CapsuleShape()->Length() * 0.5 +
                geometry.CapsuleShape()->Radius();
            break;
          }
          case sdf::GeometryType::SPHERE:
          {
            if (!geometry.SphereShape())
              return true;
            footprint.circle = true;
            footprint.halfX = geometry.SphereShape()->Radius();
            footprint.halfY = footprint.halfX;
            halfHeight = footprint.halfX;
            break;
          }
          case sdf::GeometryType::MESH:
          {
            // The worlds this is built for use meshes for visuals but boxes
            // and cylinders for collision (verified across every model in
            // tmc_wrs_gz_worlds: 76 boxes, 10 cylinders, 1 mesh), so rather
            // than pull in the mesh loader for the single outlier, treat it
            // as a modest square obstacle. Wrong-but-conservative beats
            // silently ignoring it.
            footprint.halfX = 0.5;
            footprint.halfY = 0.5;
            halfHeight = 1.0;
            break;
          }
          default:
            // PLANE (the ground) and anything exotic: not an obstacle for a
            // person walking on it.
            return true;
        }

        // Height-band test: skip whatever a walker passes over or under.
        if (centreZ + halfHeight < this->zMin || centreZ - halfHeight > this->zMax)
          return true;

        const double reach = std::hypot(footprint.halfX, footprint.halfY);
        minX = std::min(minX, footprint.x - reach);
        maxX = std::max(maxX, footprint.x + reach);
        minY = std::min(minY, footprint.y - reach);
        maxY = std::max(maxY, footprint.y + reach);
        footprints.push_back(footprint);
        return true;
      });

  if (footprints.empty())
  {
    gzwarn << "NavGridSystem: no obstacles found; paths will be straight lines."
           << std::endl;
    std::lock_guard<std::mutex> lock(this->gridMutex);
    this->gridReady = false;
    return;
  }

  minX -= kGridMargin;
  minY -= kGridMargin;
  maxX += kGridMargin;
  maxY += kGridMargin;

  const int width =
      static_cast<int>(std::ceil((maxX - minX) / this->resolution)) + 1;
  const int height =
      static_cast<int>(std::ceil((maxY - minY) / this->resolution)) + 1;
  if (width <= 0 || height <= 0 ||
      static_cast<long long>(width) * height > kMaxGridCells)
  {
    gzerr << "NavGridSystem: refusing to build a " << width << "x" << height
          << " grid (too large); path planning disabled." << std::endl;
    std::lock_guard<std::mutex> lock(this->gridMutex);
    this->gridReady = false;
    return;
  }

  std::vector<uint8_t> raw(static_cast<std::size_t>(width) * height, 0);

  // Pass 2: stamp each footprint. Every cell inside the footprint's own AABB
  // is tested against the exact shape (rotated rectangle or circle), so a
  // diagonal wall blocks a diagonal band of cells rather than its bounding
  // square.
  for (const auto &footprint : footprints)
  {
    const double reach =
        std::hypot(footprint.halfX, footprint.halfY) + this->resolution;
    const int cellMinX =
        std::max(0, static_cast<int>((footprint.x - reach - minX) / this->resolution));
    const int cellMaxX = std::min(width - 1,
        static_cast<int>((footprint.x + reach - minX) / this->resolution) + 1);
    const int cellMinY =
        std::max(0, static_cast<int>((footprint.y - reach - minY) / this->resolution));
    const int cellMaxY = std::min(height - 1,
        static_cast<int>((footprint.y + reach - minY) / this->resolution) + 1);

    const double cosYaw = std::cos(-footprint.yaw);
    const double sinYaw = std::sin(-footprint.yaw);

    for (int cy = cellMinY; cy <= cellMaxY; ++cy)
    {
      for (int cx = cellMinX; cx <= cellMaxX; ++cx)
      {
        const double wx = minX + (cx + 0.5) * this->resolution;
        const double wy = minY + (cy + 0.5) * this->resolution;
        const double dx = wx - footprint.x;
        const double dy = wy - footprint.y;
        bool inside = false;
        if (footprint.circle)
        {
          inside = (dx * dx + dy * dy) <= (footprint.halfX * footprint.halfX);
        }
        else
        {
          // Into the box's own frame, then a plain extent test.
          const double localX = dx * cosYaw - dy * sinYaw;
          const double localY = dx * sinYaw + dy * cosYaw;
          inside = std::abs(localX) <= footprint.halfX &&
              std::abs(localY) <= footprint.halfY;
        }
        if (inside)
          raw[static_cast<std::size_t>(cy) * width + cx] = 1;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(this->gridMutex);
    this->rawGrid = std::move(raw);
    this->gridWidth = width;
    this->gridHeight = height;
    this->originX = minX;
    this->originY = minY;
    this->InflateGrid();
    this->gridReady = true;
  }

  gzmsg << "NavGridSystem: grid " << width << "x" << height << " @ "
        << this->resolution << " m, origin (" << minX << ", " << minY
        << "), " << footprints.size() << " collision shapes." << std::endl;
}

void NavGridSystem::InflateGrid()
{
  // Caller holds gridMutex.
  const int width = this->gridWidth;
  const int height = this->gridHeight;
  this->grid = this->rawGrid;
  if (this->inflationRadius <= 0.0)
    return;

  const int reach =
      static_cast<int>(std::ceil(this->inflationRadius / this->resolution));
  if (reach <= 0)
    return;

  // Multi-source BFS outward from every occupied cell, stopping at `reach`
  // rings. Cheaper and simpler than stamping a disc per obstacle cell, and
  // gives exactly the same result for a uniform radius.
  std::vector<int32_t> distance(
      static_cast<std::size_t>(width) * height, -1);
  std::deque<std::pair<int, int>> frontier;
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const std::size_t index = static_cast<std::size_t>(y) * width + x;
      if (this->rawGrid[index])
      {
        distance[index] = 0;
        frontier.emplace_back(x, y);
      }
    }
  }
  while (!frontier.empty())
  {
    const auto [x, y] = frontier.front();
    frontier.pop_front();
    const std::size_t index = static_cast<std::size_t>(y) * width + x;
    const int32_t next = distance[index] + 1;
    if (next > reach)
      continue;
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        if (dx == 0 && dy == 0)
          continue;
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height)
          continue;
        const std::size_t nIndex = static_cast<std::size_t>(ny) * width + nx;
        if (distance[nIndex] >= 0)
          continue;
        distance[nIndex] = next;
        this->grid[nIndex] = 1;
        frontier.emplace_back(nx, ny);
      }
    }
  }
}

bool NavGridSystem::WorldToCell(double _wx, double _wy, int &_cx, int &_cy) const
{
  _cx = static_cast<int>((_wx - this->originX) / this->resolution);
  _cy = static_cast<int>((_wy - this->originY) / this->resolution);
  return _cx >= 0 && _cy >= 0 && _cx < this->gridWidth && _cy < this->gridHeight;
}

void NavGridSystem::CellToWorld(int _cx, int _cy, double &_wx, double &_wy) const
{
  _wx = this->originX + (_cx + 0.5) * this->resolution;
  _wy = this->originY + (_cy + 0.5) * this->resolution;
}

bool NavGridSystem::Occupied(int _cx, int _cy) const
{
  if (_cx < 0 || _cy < 0 || _cx >= this->gridWidth || _cy >= this->gridHeight)
    return true;
  return this->grid[static_cast<std::size_t>(_cy) * this->gridWidth + _cx] != 0;
}

bool NavGridSystem::NearestFree(int &_x, int &_y) const
{
  if (!this->Occupied(_x, _y))
    return true;
  const int maxRing = static_cast<int>(std::ceil(3.0 / this->resolution));
  for (int ring = 1; ring <= maxRing; ++ring)
  {
    for (int dy = -ring; dy <= ring; ++dy)
    {
      for (int dx = -ring; dx <= ring; ++dx)
      {
        // Perimeter of this ring only; the interior was covered already.
        if (std::abs(dx) != ring && std::abs(dy) != ring)
          continue;
        const int nx = _x + dx;
        const int ny = _y + dy;
        if (!this->Occupied(nx, ny))
        {
          _x = nx;
          _y = ny;
          return true;
        }
      }
    }
  }
  return false;
}

std::vector<std::pair<int, int>> NavGridSystem::PlanCell(
    int _startX, int _startY, int _goalX, int _goalY) const
{
  // Caller holds gridMutex.
  const int width = this->gridWidth;
  const int height = this->gridHeight;
  const std::size_t cells = static_cast<std::size_t>(width) * height;
  const std::size_t startIndex =
      static_cast<std::size_t>(_startY) * width + _startX;
  const std::size_t goalIndex =
      static_cast<std::size_t>(_goalY) * width + _goalX;

  std::vector<float> cost(cells, std::numeric_limits<float>::max());
  std::vector<int32_t> cameFrom(cells, -1);
  std::vector<uint8_t> closed(cells, 0);

  const auto heuristic = [&](std::size_t _index)
  {
    const int x = static_cast<int>(_index % width);
    const int y = static_cast<int>(_index / width);
    // Octile distance: the exact cost-to-go on an 8-connected grid with
    // unit/diagonal step costs, so A* stays admissible (never returns a
    // longer path than necessary) while expanding far fewer cells than
    // Dijkstra would.
    const double dx = std::abs(x - _goalX);
    const double dy = std::abs(y - _goalY);
    return static_cast<float>(
        (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
  };

  using Node = std::pair<float, std::size_t>;  // (f, index)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  cost[startIndex] = 0.0f;
  open.emplace(heuristic(startIndex), startIndex);

  while (!open.empty())
  {
    const auto [f, index] = open.top();
    open.pop();
    (void)f;
    if (closed[index])
      continue;
    closed[index] = 1;
    if (index == goalIndex)
      break;

    const int x = static_cast<int>(index % width);
    const int y = static_cast<int>(index / width);
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        if (dx == 0 && dy == 0)
          continue;
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height)
          continue;
        if (this->Occupied(nx, ny))
          continue;
        // No cutting a blocked corner diagonally -- a person can't slip
        // through the join between two touching tables.
        if (dx != 0 && dy != 0 &&
            (this->Occupied(x + dx, y) || this->Occupied(x, y + dy)))
        {
          continue;
        }
        const std::size_t nIndex = static_cast<std::size_t>(ny) * width + nx;
        if (closed[nIndex])
          continue;
        const float step = (dx != 0 && dy != 0)
            ? static_cast<float>(std::sqrt(2.0)) : 1.0f;
        const float candidate = cost[index] + step;
        if (candidate < cost[nIndex])
        {
          cost[nIndex] = candidate;
          cameFrom[nIndex] = static_cast<int32_t>(index);
          open.emplace(candidate + heuristic(nIndex), nIndex);
        }
      }
    }
  }

  std::vector<std::pair<int, int>> path;
  if (!closed[goalIndex])
    return path;  // Unreachable.
  for (std::size_t index = goalIndex; ; )
  {
    path.emplace_back(static_cast<int>(index % width), static_cast<int>(index / width));
    if (index == startIndex)
      break;
    const int32_t previous = cameFrom[index];
    if (previous < 0)
      break;
    index = static_cast<std::size_t>(previous);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

bool NavGridSystem::LineOfSight(int _x0, int _y0, int _x1, int _y1) const
{
  int dx = std::abs(_x1 - _x0);
  int dy = -std::abs(_y1 - _y0);
  int sx = _x0 < _x1 ? 1 : -1;
  int sy = _y0 < _y1 ? 1 : -1;
  int err = dx + dy;
  int x = _x0;
  int y = _y0;
  while (true)
  {
    if (this->Occupied(x, y))
      return false;
    if (x == _x1 && y == _y1)
      return true;
    const int err2 = 2 * err;
    if (err2 >= dy)
    {
      err += dy;
      x += sx;
    }
    if (err2 <= dx)
    {
      err += dx;
      y += sy;
    }
  }
}

std::vector<std::pair<int, int>> NavGridSystem::SmoothPath(
    const std::vector<std::pair<int, int>> &_cells) const
{
  if (_cells.size() <= 2)
    return _cells;
  // Greedy shortcutting: from each kept point, jump to the furthest later
  // point still in line of sight. Turns A*'s cell-by-cell staircase into a
  // handful of long straight legs, which is both what a person walking looks
  // like and far less work for ActorCommandPlugin's waypoint follower.
  std::vector<std::pair<int, int>> result;
  result.push_back(_cells.front());
  std::size_t current = 0;
  while (current + 1 < _cells.size())
  {
    std::size_t best = current + 1;
    for (std::size_t candidate = _cells.size() - 1; candidate > current + 1; --candidate)
    {
      if (this->LineOfSight(_cells[current].first, _cells[current].second,
              _cells[candidate].first, _cells[candidate].second))
      {
        best = candidate;
        break;
      }
    }
    result.push_back(_cells[best]);
    current = best;
  }
  return result;
}

bool NavGridSystem::OnPlanPath(
    const gz::msgs::StringMsg &_request, gz::msgs::Pose_V &_response)
{
  // Wire format: "inflationRadius|x1,y1;x2,y2;..." -- an inflation radius of
  // -1 means "use the configured default". Same pipe-delimited shape
  // SfmCrowdSystem's own register_human topic uses, kept deliberately
  // consistent rather than introducing a second convention.
  const std::string payload = _request.data();
  const auto bar = payload.find('|');
  if (bar == std::string::npos)
    return false;

  double requestedRadius = -1.0;
  try
  {
    requestedRadius = std::stod(payload.substr(0, bar));
  }
  catch (const std::exception &)
  {
    requestedRadius = -1.0;
  }

  std::vector<std::pair<double, double>> waypoints;
  std::istringstream stream(payload.substr(bar + 1));
  std::string token;
  while (std::getline(stream, token, ';'))
  {
    const auto comma = token.find(',');
    if (comma == std::string::npos)
      continue;
    try
    {
      waypoints.emplace_back(
          std::stod(token.substr(0, comma)), std::stod(token.substr(comma + 1)));
    }
    catch (const std::exception &)
    {
      continue;
    }
  }
  if (waypoints.size() < 2)
    return false;

  std::lock_guard<std::mutex> lock(this->gridMutex);
  if (!this->gridReady)
    return false;

  // A per-request radius (the caller's own body size) re-inflates the grid.
  // Skipped when it matches what's already applied, which is the common case
  // -- every human in a crowd normally shares one capsule size.
  if (requestedRadius >= 0.0 &&
      std::abs(requestedRadius - this->inflationRadius) > 1e-3)
  {
    this->inflationRadius = requestedRadius;
    this->InflateGrid();
  }

  // Plan each leg separately and concatenate: the caller's waypoints are
  // places it wants visited in order, not just a start and an end.
  std::vector<std::pair<double, double>> planned;
  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
  {
    int startX = 0, startY = 0, goalX = 0, goalY = 0;
    if (!this->WorldToCell(waypoints[i].first, waypoints[i].second, startX, startY) ||
        !this->WorldToCell(waypoints[i + 1].first, waypoints[i + 1].second, goalX, goalY))
    {
      // Outside the mapped area: nothing known to avoid out there, so keep
      // the leg as the caller gave it rather than dropping it.
      if (planned.empty())
        planned.push_back(waypoints[i]);
      planned.push_back(waypoints[i + 1]);
      continue;
    }
    if (!this->NearestFree(startX, startY) || !this->NearestFree(goalX, goalY))
    {
      if (planned.empty())
        planned.push_back(waypoints[i]);
      planned.push_back(waypoints[i + 1]);
      continue;
    }

    const auto cells = this->SmoothPath(this->PlanCell(startX, startY, goalX, goalY));
    if (cells.empty())
    {
      // No route at all (a goal sealed behind furniture): fall back to the
      // straight leg so the human still tries, rather than silently
      // dropping part of the route.
      if (planned.empty())
        planned.push_back(waypoints[i]);
      planned.push_back(waypoints[i + 1]);
      continue;
    }
    for (std::size_t c = 0; c < cells.size(); ++c)
    {
      // Skip the first cell of every leg after the first: it's the previous
      // leg's goal, already in the list.
      if (c == 0 && !planned.empty())
        continue;
      double wx = 0.0, wy = 0.0;
      this->CellToWorld(cells[c].first, cells[c].second, wx, wy);
      planned.emplace_back(wx, wy);
    }
    // Finish each leg exactly on the point that was asked for, not on the
    // centre of the cell nearest to it.
    if (!planned.empty())
      planned.back() = waypoints[i + 1];
  }

  for (std::size_t i = 0; i < planned.size(); ++i)
  {
    auto *pose = _response.add_pose();
    pose->mutable_position()->set_x(planned[i].first);
    pose->mutable_position()->set_y(planned[i].second);
    double yaw = 0.0;
    if (i + 1 < planned.size())
    {
      yaw = std::atan2(planned[i + 1].second - planned[i].second,
          planned[i + 1].first - planned[i].first);
    }
    pose->mutable_orientation()->set_z(std::sin(yaw * 0.5));
    pose->mutable_orientation()->set_w(std::cos(yaw * 0.5));
  }
  return true;
}
}  // namespace gz_human_sim

GZ_ADD_PLUGIN(
  gz_human_sim::NavGridSystem,
  gz::sim::System,
  gz_human_sim::NavGridSystem::ISystemConfigure,
  gz_human_sim::NavGridSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(gz_human_sim::NavGridSystem, "gz_human_sim::NavGridSystem")
