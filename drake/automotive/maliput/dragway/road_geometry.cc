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
               int num_lanes,
               double length,
               double lane_width,
               double shoulder_width,
               double linear_tolerance,
               double angular_tolerance)
  : id_(id),
    linear_tolerance_(linear_tolerance),
    angular_tolerance_(angular_tolerance),
    junction_(this, num_lanes, length, lane_width, shoulder_width) {
  DRAKE_DEMAND(length > 0);
  DRAKE_DEMAND(lane_width > 0);
  DRAKE_DEMAND(shoulder_width >= 0);
  DRAKE_DEMAND(linear_tolerance >= 0);
  DRAKE_DEMAND(angular_tolerance >= 0);
}

const api::Junction* RoadGeometry::do_junction(int index) const {
  DRAKE_DEMAND(index < num_junctions());
  return &junction_;
}

int RoadGeometry::do_num_branch_points() const {
  // There is only one BranchPoint per lane. Thus, return the number of lanes.
  return junction_.segment(0)->num_lanes();
}

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  DRAKE_DEMAND(index < num_branch_points());
  // The same BranchPoint is at the start versus end of a Lane, thus it doesn't
  // matter whether the start or finish BranchPoint is returned.
  return junction_.segment(0)->lane(index)->GetBranchPoint(
      api::LaneEnd::kStart);
}

api::RoadPosition RoadGeometry::DoToRoadPosition(
    const api::GeoPosition&, const api::RoadPosition&) const {
  DRAKE_ABORT();  // TODO(liang.fok) Implement me.
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
