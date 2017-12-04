#include "drake/automotive/maliput/rndf/connection.h"

namespace drake {
namespace maliput {
namespace rndf {

void Connection::AddWaypoint(const DirectedWaypoint& waypoint, int position) {
  DRAKE_THROW_UNLESS(position >= 0);
  DRAKE_THROW_UNLESS(position <= static_cast<int>(waypoints_.size()));
  waypoints_.insert(waypoints_.begin() + position, DirectedWaypoint(waypoint));
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
