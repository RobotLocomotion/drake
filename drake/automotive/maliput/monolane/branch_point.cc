#include "drake/automotive/maliput/monolane/branch_point.h"

#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

const api::LaneEnd& LaneEndSet::do_get(int index) const {
  return ends_[index]; }


BranchPoint::BranchPoint(const api::BranchPointId& id, RoadGeometry* rg)
    : id_(id), road_geometry_(rg) {}

const api::RoadGeometry* BranchPoint::do_road_geometry() const {
  return road_geometry_;
}

const api::LaneEndSet* BranchPoint::DoGetConfluentBranches(
    const api::LaneEnd& end) const {
  return confluent_branches_.at(end);
}

const api::LaneEndSet* BranchPoint::DoGetOngoingBranches(
    const api::LaneEnd& end) const {
  return ongoing_branches_.at(end);
}

std::unique_ptr<api::LaneEnd> BranchPoint::DoGetDefaultBranch(
    const api::LaneEnd& end) const {
  return std::move(
      std::make_unique<api::LaneEnd>(defaults_.find(end)->second));
}

const api::LaneEnd& BranchPoint::AddABranch(const api::LaneEnd& lane_end) {
  DRAKE_DEMAND(
      confluent_branches_.find(lane_end) == confluent_branches_.end());
  DRAKE_DEMAND(ongoing_branches_.find(lane_end) == ongoing_branches_.end());
  a_side_.add(lane_end);
  confluent_branches_[lane_end] = &a_side_;
  ongoing_branches_[lane_end] = &b_side_;
  return lane_end;
}

const api::LaneEnd& BranchPoint::AddBBranch(const api::LaneEnd& lane_end) {
  DRAKE_DEMAND(
      confluent_branches_.find(lane_end) == confluent_branches_.end());
  DRAKE_DEMAND(ongoing_branches_.find(lane_end) == ongoing_branches_.end());
  b_side_.add(lane_end);
  confluent_branches_[lane_end] = &b_side_;
  ongoing_branches_[lane_end] = &a_side_;
  return lane_end;
}

void BranchPoint::SetDefault(const api::LaneEnd& lane_end,
                             const api::LaneEnd& default_branch) {
  DRAKE_DEMAND(ongoing_branches_.find(lane_end) != ongoing_branches_.end());
  DRAKE_DEMAND(
      ongoing_branches_.find(default_branch) != ongoing_branches_.end());
  // TODO(maddog)  assert that default_branch is actually legal for lane_end.
  defaults_[lane_end] = default_branch;
}



}  // namespace monolane
}  // namespace maliput
}  // namespace drake
