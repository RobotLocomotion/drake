#include "drake/automotive/maliput/rndf/builder.h"

#include <algorithm>
#include <tuple>
#include <vector>

namespace drake {
namespace maliput {
namespace rndf {

std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>
DirectedWaypoint::CalculateBoundingBox(
    const std::vector<DirectedWaypoint>& directed_waypoints) {
  std::vector<double> x_coordinates, y_coordinates;
  if (directed_waypoints.size() == 0) {
    return std::make_tuple(ignition::math::Vector3d::Zero,
                           ignition::math::Vector3d::Zero);
  }
  for (const auto& dw : directed_waypoints) {
    x_coordinates.push_back(dw.position().X());
    y_coordinates.push_back(dw.position().Y());
  }
  return std::make_tuple(
      ignition::math::Vector3d(
          *std::min_element(x_coordinates.begin(), x_coordinates.end()),
          *std::min_element(y_coordinates.begin(), y_coordinates.end()), 0.0),
      ignition::math::Vector3d(
          *std::max_element(x_coordinates.begin(), x_coordinates.end()),
          *std::max_element(y_coordinates.begin(), y_coordinates.end()), 0.0));
}

void Connection::AddWaypoint(const DirectedWaypoint& waypoint, int position) {
  DRAKE_THROW_UNLESS(position >= 0);
  DRAKE_THROW_UNLESS(position <= static_cast<int>(waypoints_.size()));
  waypoints_.insert(waypoints_.begin() + position, DirectedWaypoint(waypoint));
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
