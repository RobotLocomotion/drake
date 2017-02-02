#include "drake/automotive/maliput/dragway/northbound_lane.h"

namespace drake {
namespace maliput {
namespace dragway {


NorthboundLane::NorthboundLane(const Segment* segment, const api::LaneId& id,
    int index, double length, double y_offset, const api::RBounds& lane_bounds,
    const api::RBounds& driveable_bounds)
  : Lane(segment, id, index, length, y_offset, lane_bounds, driveable_bounds) {
}

api::GeoPosition NorthboundLane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  return {lane_pos.s, lane_pos.r + Lane::y_offset(), lane_pos.h};
}

api::Rotation NorthboundLane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  return api::Rotation(0, 0, 0);  // roll, pitch, yaw.
}

api::LanePosition NorthboundLane::DoToLanePosition(
    const api::GeoPosition& geo_pos) const {
  DRAKE_ABORT();  // TODO(liangfok) Implement me.
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
