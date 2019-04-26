#include "drake/automotive/maliput/geometry_base/lane.h"

#include "drake/automotive/maliput/geometry_base/branch_point.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace geometry_base {

void Lane::AttachToSegment(Passkey<Segment>,
                           const api::Segment* segment, int index) {
  // Parameter checks
  DRAKE_THROW_UNLESS(segment != nullptr);
  DRAKE_THROW_UNLESS(index >= 0);
  // Preconditions
  DRAKE_THROW_UNLESS(segment_ == nullptr);
  DRAKE_THROW_UNLESS(index_ == -1);

  segment_ = segment;
  index_ = index;
}

void Lane::SetStartBranchPoint(Passkey<BranchPoint>,
                               BranchPoint* branch_point) {
  // Parameter checks
  DRAKE_THROW_UNLESS(branch_point != nullptr);
  // Preconditions
  DRAKE_THROW_UNLESS(start_branch_point_ == nullptr);

  start_branch_point_ = branch_point;
}

void Lane::SetFinishBranchPoint(Passkey<BranchPoint>,
                                BranchPoint* branch_point) {
  // Parameter checks
  DRAKE_THROW_UNLESS(branch_point != nullptr);
  // Preconditions
  DRAKE_THROW_UNLESS(finish_branch_point_ == nullptr);

  finish_branch_point_ = branch_point;
}


api::LaneId Lane::do_id() const { return id_; }

const api::Segment* Lane::do_segment() const { return segment_; }

int Lane::do_index() const { return index_; }

const api::Lane* Lane::do_to_left() const {
  if (index_ == (segment_->num_lanes() - 1)) {
    return nullptr;
  } else {
    return segment_->lane(index_ + 1);
  }
}

const api::Lane* Lane::do_to_right() const {
  if (index_ == 0) {
    return nullptr;
  } else {
    return segment_->lane(index_ - 1);
  }
}

const api::BranchPoint* Lane::DoGetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart:  { return start_branch_point_; }
    case api::LaneEnd::kFinish: { return finish_branch_point_; }
  }
  DRAKE_UNREACHABLE();
}

const api::LaneEndSet* Lane::DoGetConfluentBranches(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetOngoingBranches({this, which_end});
}

optional<api::LaneEnd> Lane::DoGetDefaultBranch(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetDefaultBranch({this, which_end});
}

}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
