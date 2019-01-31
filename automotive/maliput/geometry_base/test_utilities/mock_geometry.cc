#include "drake/automotive/maliput/geometry_base/test_utilities/mock_geometry.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace geometry_base {
namespace test {


api::RoadPosition MockRoadGeometry::DoToRoadPosition(
    const api::GeoPosition&, const api::RoadPosition*,
    api::GeoPosition*, double*) const {
  DRAKE_THROW_UNLESS(false);
  DRAKE_UNREACHABLE();
}


double MockLane::do_length() const {
  DRAKE_THROW_UNLESS(false);
  DRAKE_UNREACHABLE();
}

api::RBounds MockLane::do_lane_bounds(double) const {
  DRAKE_THROW_UNLESS(false);
  DRAKE_UNREACHABLE();
}

api::RBounds MockLane::do_driveable_bounds(double) const {
  DRAKE_THROW_UNLESS(false);
  DRAKE_UNREACHABLE();
}

api::HBounds MockLane::do_elevation_bounds(double, double) const {
  DRAKE_THROW_UNLESS(false);
  DRAKE_UNREACHABLE();
}

api::GeoPosition MockLane::DoToGeoPosition(const api::LanePosition&) const {
  DRAKE_THROW_UNLESS(false);
  DRAKE_UNREACHABLE();
}

api::Rotation MockLane::DoGetOrientation(const api::LanePosition&) const {
  DRAKE_THROW_UNLESS(false);
  DRAKE_UNREACHABLE();
}

api::LanePosition MockLane::DoEvalMotionDerivatives(
    const api::LanePosition&, const api::IsoLaneVelocity&) const {
  DRAKE_THROW_UNLESS(false);
  DRAKE_UNREACHABLE();
}

api::LanePosition MockLane::DoToLanePosition(
    const api::GeoPosition&, api::GeoPosition*, double*) const {
  DRAKE_THROW_UNLESS(false);
  DRAKE_UNREACHABLE();
}


}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
