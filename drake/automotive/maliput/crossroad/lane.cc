#include "drake/automotive/maliput/crossroad/lane.h"

#include <cmath>
#include <memory>
#include <string>

#include "drake/automotive/maliput/crossroad/branch_point.h"
#include "drake/automotive/maliput/crossroad/road_geometry.h"
#include "drake/automotive/maliput/crossroad/segment.h"
#include "drake/common/drake_assert.h"

using std::make_unique;

namespace drake {
namespace maliput {
namespace crossroad {

Lane::Lane(const Segment* segment, const api::LaneId& id, int index,
           double length, double r_offset, const api::RBounds& lane_bounds,
           const api::RBounds& driveable_bounds)
    : segment_(segment),
      id_(id),
      index_(index),
      length_(length),
      r_offset_(r_offset),
      lane_bounds_(lane_bounds),
      driveable_bounds_(driveable_bounds) {
  DRAKE_DEMAND(segment != nullptr);
  DRAKE_DEMAND(lane_bounds_.r_min >= driveable_bounds_.r_min);
  DRAKE_DEMAND(lane_bounds_.r_max <= driveable_bounds_.r_max);

  branch_point_ =
      make_unique<BranchPoint>(api::BranchPointId({id.id + "_Branch_Point"}),
                               this, segment->junction()->road_geometry());
}

const api::Segment* Lane::do_segment() const { return segment_; }

void Lane::set_lane_to_left(api::Lane* lane_to_left) {
  lane_to_left_ = lane_to_left;
}

void Lane::set_lane_to_right(api::Lane* lane_to_right) {
  lane_to_right_ = lane_to_right;
}

const api::BranchPoint* Lane::DoGetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  return branch_point_.get();
}

const api::LaneEndSet* Lane::DoGetConfluentBranches(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetOngoingBranches({this, which_end});
}

std::unique_ptr<api::LaneEnd> Lane::DoGetDefaultBranch(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetDefaultBranch({this, which_end});
}

api::RBounds Lane::do_lane_bounds(double) const { return lane_bounds_; }

api::RBounds Lane::do_driveable_bounds(double) const {
  return driveable_bounds_;
}

api::LanePosition Lane::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity) const {
  return api::LanePosition(velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

api::GeoPosition Lane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  api::SegmentId segment_id = this->segment()->id();
  if (strcmp(segment_id.id.c_str(), "Crossroad_Horizontal_Segment") == 0) {
    return {lane_pos.s - length_ / 2, lane_pos.r + Lane::r_offset(),
            lane_pos.h};
  } else if (strcmp(segment_id.id.c_str(), "Crossroad_Vertical_Segment") == 0) {
    return {lane_pos.r + Lane::r_offset(), lane_pos.s - length_ / 2,
            lane_pos.h};
  }
  throw std::runtime_error("Segment ID not recogonized");
}

api::Rotation Lane::DoGetOrientation(const api::LanePosition& lane_pos) const {
  return api::Rotation(0, 0, 0);  // roll, pitch, yaw.
}

api::LanePosition Lane::DoToLanePosition(const api::GeoPosition& geo_pos,
                                         api::GeoPosition* nearest_point,
                                         double* distance) const {
  DRAKE_ABORT();
  // TODO(shensquared): implement me.
}

}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
