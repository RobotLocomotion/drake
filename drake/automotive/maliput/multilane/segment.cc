#include "drake/automotive/maliput/multilane/segment.h"

namespace drake {
namespace maliput {
namespace multilane {

const api::Junction* Segment::do_junction() const {
  return junction_;
}

Lane* Segment::NewLane(api::LaneId id,
                       const api::RBounds& lane_bounds,
                       const api::RBounds& driveable_bounds,
                       const api::HBounds& elevation_bounds) {
  DRAKE_DEMAND(lane_.get() == nullptr);
  lane_ = std::make_unique<Lane>(id, this, lane_bounds, driveable_bounds,
                                 elevation_bounds, road_curve_.get());
  return lane_.get();
}

const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index == 0);
  return lane_.get();
}


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
