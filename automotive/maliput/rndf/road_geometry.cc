#include "drake/automotive/maliput/rndf/road_geometry.h"

#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace rndf {

Junction* RoadGeometry::NewJunction(api::JunctionId id) {
  junctions_.push_back(std::make_unique<Junction>(id, this));
  return junctions_.back().get();
}

BranchPoint* RoadGeometry::NewBranchPoint(api::BranchPointId id) {
  branch_points_.push_back(std::make_unique<BranchPoint>(id, this));
  return branch_points_.back().get();
}

const api::Junction* RoadGeometry::do_junction(int index) const {
  DRAKE_THROW_UNLESS(index >= 0 && index < static_cast<int>(junctions_.size()));
  return junctions_[index].get();
}

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  DRAKE_THROW_UNLESS(index >= 0 &&
                     index < static_cast<int>(branch_points_.size()));
  return branch_points_[index].get();
}

api::RoadPosition RoadGeometry::DoToRoadPosition(const api::GeoPosition&,
                                                 const api::RoadPosition*,
                                                 api::GeoPosition*,
                                                 double*) const {
  // TODO(@agalbachicar) We need to find a way of implement this.
  throw std::runtime_error(
      "RoadGeometry::DoToRoadPosition is not implemented");
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
