#include "drake/automotive/maliput/monolane/road_geometry.h"

#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/junction.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

Junction* RoadGeometry::NewJunction(api::JunctionId id) {
  junctions_.push_back(std::make_unique<Junction>(id, this));
  return junctions_.back().get();
}


BranchPoint* RoadGeometry::NewBranchPoint(api::BranchPointId id) {
  branch_points_.push_back(std::make_unique<BranchPoint>(id, this));
  return branch_points_.back().get();
}


const api::Junction* RoadGeometry::do_junction(int index) const {
  return junctions_[index].get();
}


const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  return branch_points_[index].get();
}


api::RoadPosition RoadGeometry::DoToRoadPosition(
    const api::GeoPosition&, const api::RoadPosition&) const {
  DRAKE_ABORT();  // TODO(maddog@tri.global) Implement me.
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
