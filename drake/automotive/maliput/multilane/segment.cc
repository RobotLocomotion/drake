#include "drake/automotive/maliput/multilane/segment.h"

namespace drake {
namespace maliput {
namespace multilane {

Segment::Segment(const api::SegmentId& id, api::Junction* junction,
                 std::unique_ptr<RoadCurve> road_curve, int num_lanes,
                 double r0, double r_spacing, double r_min, double r_max,
                 const api::HBounds& elevation_bounds)
    : id_(id),
      junction_(junction),
      road_curve_(std::move(road_curve)),
      num_lanes_(num_lanes),
      r0_(r0),
      r_spacing_(r_spacing),
      r_min_(r_min),
      r_max_(r_max),
      elevation_bounds_(elevation_bounds) {
  DRAKE_DEMAND(road_curve_.get() != nullptr);
  // Ensures that the Segment has at least one Lane.
  DRAKE_DEMAND(num_lanes_ > 0);
  // Ensures safe Segment extents.
  DRAKE_DEMAND(r_min_ <= r_max_);
  // Ensures a safe lateral distance for the first Lane.
  DRAKE_DEMAND(r_min_ <= r0_ && r0_ <= r_max_);
  // Ensures that r_spacing is positive or zero.
  DRAKE_DEMAND(r_spacing_ >= 0.);
  // Ensures that all lanes are set inside Segment bounds.
  const double last_lane_centerline =
      r0 + static_cast<double>(num_lanes_ - 1) * std::copysign(r_spacing_, r0_);
  DRAKE_DEMAND(last_lane_centerline <= r_max_);
  DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, elevation_bounds_));
}

const api::Junction* Segment::do_junction() const {
  return junction_;
}

Lane* Segment::NewLane(api::LaneId id, const api::RBounds& lane_bounds) {
  DRAKE_DEMAND(static_cast<int>(lanes_.size()) < num_lanes_);
  const int index = lanes_.size();
  const double r_offset =
      r0_ + static_cast<double>(index) * std::copysign(r_spacing_, r0_);
  DRAKE_DEMAND(r_min_ <= r_offset && r_offset <= r_max_);
  const api::RBounds driveable_bounds(r_min_ - r_offset, r_max_ - r_offset);
  DRAKE_DEMAND(lane_bounds.min() >= driveable_bounds.min() &&
               lane_bounds.max() <= driveable_bounds.max());
  auto lane_ =
      std::make_unique<Lane>(id, this, index, lane_bounds, driveable_bounds,
                             elevation_bounds_, road_curve_.get(), r_offset);
  lanes_.push_back(std::move(lane_));
  return lanes_.back().get();
}

const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index >= 0 && index < static_cast<int>(lanes_.size()));
  return lanes_[index].get();
}


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
