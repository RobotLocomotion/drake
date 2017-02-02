#include "drake/automotive/maliput/dragway/southbound_lane.h"

#include <cmath>

namespace drake {
namespace maliput {
namespace dragway {

SouthboundLane::SouthboundLane(const Segment* segment, const api::LaneId& id,
    int index, double length, double y_offset, const api::RBounds& lane_bounds,
    const api::RBounds& driveable_bounds)
  : Lane(segment, id, index, length, y_offset, lane_bounds, driveable_bounds) {
}

api::GeoPosition SouthboundLane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  return {length() - lane_pos.s, Lane::y_offset() - lane_pos.r, lane_pos.h};
}

api::Rotation SouthboundLane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  return api::Rotation(0, 0, M_PI);  // roll, pitch, yaw.
}

api::LanePosition SouthboundLane::DoToLanePosition(
    const api::GeoPosition& geo_pos) const {
  DRAKE_ABORT();  // TODO(liangfok) Implement me.
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
