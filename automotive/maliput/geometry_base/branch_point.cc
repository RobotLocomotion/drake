#include "drake/automotive/maliput/geometry_base/branch_point.h"

#include "drake/automotive/maliput/geometry_base/lane.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace geometry_base {

BranchPoint::BranchPoint(const api::BranchPointId& id) : id_(id) {}

void BranchPoint::AttachToRoadGeometry(
    Passkey<RoadGeometry>,
    const api::RoadGeometry* road_geometry) {
  // Parameter checks
  DRAKE_THROW_UNLESS(road_geometry != nullptr);
  // Preconditions
  DRAKE_THROW_UNLESS(road_geometry_ == nullptr);
  road_geometry_ = road_geometry;
}


void BranchPoint::AddBranch(Lane* lane, api::LaneEnd::Which end,
                            LaneEndSet* side) {
  // Parameter checks
  // TODO(maddog@tri.global)  This invariant should be part of api::LaneEnd.
  DRAKE_THROW_UNLESS(lane != nullptr);
  DRAKE_THROW_UNLESS((side == &a_side_) || (side == &b_side_));
  // Preconditions
  const api::LaneEnd lane_end{lane, end};
  DRAKE_THROW_UNLESS(confluent_branches_.count(lane_end) == 0);
  DRAKE_THROW_UNLESS(ongoing_branches_.count(lane_end) == 0);
  LaneEndSet* const other_side = (side == &a_side_) ? &b_side_ : &a_side_;
  side->Add(lane_end);
  confluent_branches_[lane_end] = side;
  ongoing_branches_[lane_end] = other_side;
  switch (end) {
    case api::LaneEnd::kStart:  {
      lane->SetStartBranchPoint({}, this);
      break;
    }
    case api::LaneEnd::kFinish: {
      lane->SetFinishBranchPoint({}, this);
      break;
    }
  }
}


void BranchPoint::SetDefault(const api::LaneEnd& lane_end,
                             const api::LaneEnd& default_branch) {
  // Parameter checks
  const auto& le_ongoing = ongoing_branches_.find(lane_end);
  const auto& db_confluent = confluent_branches_.find(default_branch);
  // Verify that lane_end belongs to this BranchPoint.
  DRAKE_THROW_UNLESS(le_ongoing != ongoing_branches_.end());
  // Verify that default_branch belongs to this BranchPoint.
  DRAKE_THROW_UNLESS(db_confluent != confluent_branches_.end());
  // Verify that default_branch is an ongoing lane for lane_end.
  DRAKE_THROW_UNLESS(db_confluent->second == le_ongoing->second);

  defaults_[lane_end] = default_branch;
}


api::BranchPointId BranchPoint::do_id() const { return id_; }

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

const api::LaneEndSet* BranchPoint::DoGetASide() const { return &a_side_; }

const api::LaneEndSet* BranchPoint::DoGetBSide() const { return &b_side_; }

}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
