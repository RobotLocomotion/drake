#include "drake/automotive/maliput/dragway/road_geometry.h"

#include <memory>

#include "drake/automotive/maliput/dragway/branch_point.h"
#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/common/drake_assert.h"

using std::make_unique;

namespace drake {
namespace maliput {
namespace dragway {

RoadGeometry::RoadGeometry(const api::RoadGeometryId& id,
           int num_southbound_lanes,
           int num_northbound_lanes,
           double length,
           const api::RBounds& lane_bounds,
           const api::RBounds& driveable_bounds,
           double linear_tolerance,
           double angular_tolerance)
  : id_(id),
    length_(length),
    linear_tolerance_(linear_tolerance),
    angular_tolerance_(angular_tolerance),
    junction_(this, num_southbound_lanes, num_northbound_lanes, length,
        lane_bounds, driveable_bounds) {
}

const api::Junction* RoadGeometry::do_junction(int index) const {
  DRAKE_DEMAND(index < num_junctions());
  return &junction_;
}

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  return branch_points_[index].get();
}

api::RoadPosition RoadGeometry::DoToRoadPosition(
    const api::GeoPosition&, const api::RoadPosition&) const {
  DRAKE_ABORT();  // TODO(liang.fok) Implement me.
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
