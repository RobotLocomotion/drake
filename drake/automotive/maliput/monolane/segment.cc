#include "drake/automotive/maliput/monolane/segment.h"

#include <utility>

#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

const api::Junction* Segment::do_junction() const {
  return junction_;
}


LineLane* Segment::NewLineLane(api::LaneId id,
                               const V2& xy0, const V2& dxy,
                               const api::RBounds& lane_bounds,
                               const api::RBounds& driveable_bounds,
                               const CubicPolynomial& elevation,
                               const CubicPolynomial& superelevation) {
  DRAKE_DEMAND(lane_.get() == nullptr);
  std::unique_ptr<LineLane> lane = std::make_unique<LineLane>(
      id, this, xy0, dxy,
      lane_bounds, driveable_bounds,
      elevation, superelevation);
  LineLane* result = lane.get();
  lane_ = std::move(lane);
  return result;
}


ArcLane* Segment::NewArcLane(api::LaneId id,
                             const V2& center, const double radius,
                             const double theta0, const double d_theta,
                             const api::RBounds& lane_bounds,
                             const api::RBounds& driveable_bounds,
                             const CubicPolynomial& elevation,
                             const CubicPolynomial& superelevation) {
  DRAKE_DEMAND(lane_.get() == nullptr);
  std::unique_ptr<ArcLane> lane = std::make_unique<ArcLane>(
      id, this, center, radius, theta0, d_theta,
      lane_bounds, driveable_bounds,
      elevation, superelevation);
  ArcLane* result = lane.get();
  lane_ = std::move(lane);
  return result;
}


const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index == 0);
  return lane_.get();
}


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
