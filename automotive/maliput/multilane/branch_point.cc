#include "drake/automotive/maliput/multilane/branch_point.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace multilane {

BranchPoint::BranchPoint(const api::BranchPointId& id,
                         const api::RoadGeometry* road_geometry)
    : id_(id), road_geometry_(road_geometry) {}

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

optional<api::LaneEnd> BranchPoint::DoGetDefaultBranch(
    const api::LaneEnd& end) const {
  auto default_it = defaults_.find(end);
  if (default_it == defaults_.end()) { return nullopt; }
  return default_it->second;
}

const api::LaneEnd& BranchPoint::AddABranch(const api::LaneEnd& lane_end) {
  DRAKE_DEMAND(confluent_branches_.find(lane_end) == confluent_branches_.end());
  DRAKE_DEMAND(ongoing_branches_.find(lane_end) == ongoing_branches_.end());
  a_side_.add(lane_end);
  confluent_branches_[lane_end] = &a_side_;
  ongoing_branches_[lane_end] = &b_side_;
  return lane_end;
}

const api::LaneEnd& BranchPoint::AddBBranch(const api::LaneEnd& lane_end) {
  DRAKE_DEMAND(confluent_branches_.find(lane_end) == confluent_branches_.end());
  DRAKE_DEMAND(ongoing_branches_.find(lane_end) == ongoing_branches_.end());
  b_side_.add(lane_end);
  confluent_branches_[lane_end] = &b_side_;
  ongoing_branches_[lane_end] = &a_side_;
  return lane_end;
}

void BranchPoint::SetDefault(const api::LaneEnd& lane_end,
                             const api::LaneEnd& default_branch) {
  const auto& le_ongoing = ongoing_branches_.find(lane_end);
  const auto& db_confluent = confluent_branches_.find(default_branch);
  // Verify that lane_end belongs to this BranchPoint.
  DRAKE_DEMAND(le_ongoing != ongoing_branches_.end());
  // Verify that default_branch belongs to this BranchPoint.
  DRAKE_DEMAND(db_confluent != confluent_branches_.end());
  // Verify that default_branch is an ongoing lane for lane_end.
  DRAKE_DEMAND(db_confluent->second == le_ongoing->second);

  defaults_[lane_end] = default_branch;
}


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
