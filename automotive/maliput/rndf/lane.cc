#include "drake/automotive/maliput/rndf/lane.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace rndf {

const api::Segment* Lane::do_segment() const { return segment_; }

const api::BranchPoint* Lane::DoGetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart: {
      return start_bp_;
    }
    case api::LaneEnd::kFinish: {
      return end_bp_;
    }
  }
  DRAKE_ABORT_MSG("which_end value is not supported.");
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

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
