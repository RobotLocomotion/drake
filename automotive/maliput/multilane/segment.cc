#include "drake/automotive/maliput/multilane/segment.h"

namespace drake {
namespace maliput {
namespace multilane {

const api::Junction* Segment::do_junction() const {
  return junction_;
}

Lane* Segment::NewLane(api::LaneId id, double r0,
                       const api::RBounds& lane_bounds) {
  const int index = lanes_.size();
  DRAKE_DEMAND(r_min_ <= r0 && r0 <= r_max_);
  if (lanes_.size() != 0) {
    DRAKE_DEMAND(r0 > lanes_.back()->r0());
  }
  const api::RBounds driveable_bounds(r_min_ - r0, r_max_ - r0);
  DRAKE_DEMAND(lane_bounds.min() >= driveable_bounds.min() &&
               lane_bounds.max() <= driveable_bounds.max());
  auto lane_ =
      std::make_unique<Lane>(id, this, index, lane_bounds, driveable_bounds,
                             elevation_bounds_, road_curve_.get(), r0);
  lanes_.push_back(std::move(lane_));
  Lane* result = lanes_.back().get();
  register_lane_(result);
  return result;
}

const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index >= 0 && index < static_cast<int>(lanes_.size()));
  return lanes_[index].get();
}


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
