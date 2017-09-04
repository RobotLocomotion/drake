#include "drake/automotive/maliput/multilane/segment.h"

namespace drake {
namespace maliput {
namespace multilane {

const api::Junction* Segment::do_junction() const {
  return junction_;
}

Lane* Segment::NewLane(api::LaneId id, double r0) {
  DRAKE_DEMAND(lane_.get() == nullptr);
  DRAKE_DEMAND(r_min_ <= r0 && r0 <= r_max_);
  const api::RBounds lateral_bounds(r_min_ - r0, r_max_ - r0);
  lane_ = std::make_unique<Lane>(id, this, lateral_bounds, lateral_bounds,
                                 elevation_bounds_, road_curve_.get(), r0);
  return lane_.get();
}

const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index == 0);
  return lane_.get();
}


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
