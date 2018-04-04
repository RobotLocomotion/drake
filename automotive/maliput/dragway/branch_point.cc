#include "drake/automotive/maliput/dragway/branch_point.h"

#include "drake/automotive/maliput/dragway/lane.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace dragway {

BranchPoint::BranchPoint(const api::BranchPointId& id, const Lane* lane,
                         const api::RoadGeometry* road_geometry)
    : id_(id),
      road_geometry_(road_geometry),
      start_side_lane_end_set_(lane, api::LaneEnd::kStart),
      finish_side_lane_end_set_(lane, api::LaneEnd::kFinish) {
}

const api::RoadGeometry* BranchPoint::do_road_geometry() const {
  return road_geometry_;
}

const api::LaneEndSet* BranchPoint::DoGetConfluentBranches(
    const api::LaneEnd& end) const {
  if (end.end == api::LaneEnd::kStart) {
    return &start_side_lane_end_set_;
  } else {
    return &finish_side_lane_end_set_;
  }
}

const api::LaneEndSet* BranchPoint::DoGetOngoingBranches(
    const api::LaneEnd& end) const {
  if (end.end == api::LaneEnd::kStart) {
    return &finish_side_lane_end_set_;
  } else {
    return &start_side_lane_end_set_;
  }
}

optional<api::LaneEnd> BranchPoint::DoGetDefaultBranch(
    const api::LaneEnd& end) const {
  // The result should be an ongoing branch for the given input. Thus, a Start
  // input should yield a Finish output (since start connects to finish) and
  // vice-versa.
  //
  // Since there is only one LaneEnd per side, return it as the default.
  if (end.end == api::LaneEnd::kStart) {
    return finish_side_lane_end_set_.get(0);
  } else {
    return start_side_lane_end_set_.get(0);
  }
}

const api::LaneEndSet* BranchPoint::DoGetASide() const {
  return &start_side_lane_end_set_;
}

const api::LaneEndSet* BranchPoint::DoGetBSide() const {
  return &finish_side_lane_end_set_;
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
