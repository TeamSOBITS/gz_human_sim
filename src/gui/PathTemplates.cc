#include "PathTemplates.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

// 中身は HumanControlPanelRoute.cc の generatePathTemplate() から
// そのまま移したもので、振る舞いは変えていない。yaw を捨てるのも元どおり
// （元は waypoints に (x, y, yaw) を積んでから yaw を (void) で捨てていた）。

namespace gz_human_sim
{
std::vector<std::pair<double, double>> TemplateWaypoints(
    int _templateIndex, double _centerX, double _centerY,
    double _size, int _numWaypoints, bool _clockwise)
{
  std::vector<std::pair<double, double>> result;
  if (_templateIndex < 0 || _templateIndex >= kPathTemplateCount)
    return result;

  // (x, y, yaw) tuples, same math as scripts/path_template.py's
  // _circle_waypoints()/_square_waypoints() -- kept in sync by hand since
  // one is Python (for headless/CLI use) and this is C++ (for the GUI
  // button), not sharable code between the two.
  std::vector<std::array<double, 3>> waypoints;
  const double direction = _clockwise ? -1.0 : 1.0;

  if (_templateIndex == kPathTemplateCircle)
  {
    const double radius = std::max(0.1, _size);
    const int count = std::clamp(_numWaypoints, 3, 200);
    for (int i = 0; i < count; ++i)
    {
      const double angle = direction * i * (2.0 * M_PI / count);
      const double tangent = angle + direction * (M_PI / 2.0);
      waypoints.push_back({_centerX + radius * std::cos(angle),
          _centerY + radius * std::sin(angle), tangent});
    }
  }
  else
  {
    const double half = std::max(0.1, _size) / 2.0;
    std::vector<std::pair<double, double>> corners = {
      {_centerX + half, _centerY + half}, {_centerX - half, _centerY + half},
      {_centerX - half, _centerY - half}, {_centerX + half, _centerY - half}};
    if (_clockwise)
      std::reverse(corners.begin() + 1, corners.end());
    for (std::size_t i = 0; i < corners.size(); ++i)
    {
      const auto &[x, y] = corners[i];
      const auto &[nextX, nextY] = corners[(i + 1) % corners.size()];
      waypoints.push_back({x, y, std::atan2(nextY - y, nextX - x)});
    }
  }

  for (const auto &[x, y, yaw] : waypoints)
  {
    (void)yaw;  // 呼び出し側が点の順序から derive し直す。
    result.emplace_back(x, y);
  }
  return result;
}
}  // namespace gz_human_sim
