#include "drake/automotive/maliput/rndf/directed_waypoint.h"

#include <algorithm>
#include <limits>

namespace drake {
namespace maliput {
namespace rndf {

std::pair<ignition::math::Vector3d, ignition::math::Vector3d>
DirectedWaypoint::CalculateBoundingBox(
    const std::vector<DirectedWaypoint>& directed_waypoints) {
  std::vector<double> x_coordinates, y_coordinates;
  if (directed_waypoints.size() == 0) {
    return std::make_pair(ignition::math::Vector3d::Zero,
                          ignition::math::Vector3d::Zero);
  }
  double x_min{std::numeric_limits<double>::max()};
  double y_min{std::numeric_limits<double>::max()};
  double x_max{std::numeric_limits<double>::min()};
  double y_max{std::numeric_limits<double>::min()};
  for (const DirectedWaypoint& directed_waypoint : directed_waypoints) {
    x_min = std::min(x_min, directed_waypoint.position().X());
    y_min = std::min(y_min, directed_waypoint.position().Y());
    x_max = std::max(x_max, directed_waypoint.position().X());
    y_max = std::max(y_max, directed_waypoint.position().Y());
  }
  return std::make_pair(ignition::math::Vector3d(x_min, y_min, 0.0),
                        ignition::math::Vector3d(x_max, y_max, 0.0));
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
