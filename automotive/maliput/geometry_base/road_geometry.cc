#include "drake/automotive/maliput/geometry_base/road_geometry.h"

#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace maliput {
namespace geometry_base {


void RoadGeometry::AddJunctionPrivate(std::unique_ptr<Junction> junction) {
  // Parameter checks
  DRAKE_THROW_UNLESS(junction.get() != nullptr);
  junctions_.emplace_back(std::move(junction));
  Junction* const raw_junction = junctions_.back().get();
  raw_junction->AttachToRoadGeometry(
      {},
      this,
      [this](auto segment) { id_index_.AddSegment(segment); },
      [this](auto lane) { id_index_.AddLane(lane); });
  id_index_.AddJunction(raw_junction);
}


void RoadGeometry::AddBranchPointPrivate(
    std::unique_ptr<BranchPoint> branch_point) {
  // Parameter checks
  DRAKE_THROW_UNLESS(branch_point.get() != nullptr);
  branch_points_.emplace_back(std::move(branch_point));
  BranchPoint* const raw_branch_point = branch_points_.back().get();
  raw_branch_point->AttachToRoadGeometry({}, this);
  id_index_.AddBranchPoint(raw_branch_point);
}


const api::Junction* RoadGeometry::do_junction(int index) const {
  return junctions_.at(index).get();
}

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  return branch_points_.at(index).get();
}


}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
