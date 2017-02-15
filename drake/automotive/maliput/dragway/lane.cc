#include "drake/automotive/maliput/dragway/lane.h"

#include <memory>

#include "drake/automotive/maliput/dragway/branch_point.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/dragway/segment.h"
#include "drake/common/drake_assert.h"

using std::make_unique;

namespace drake {
namespace maliput {
namespace dragway {

Lane::Lane(const Segment* segment, const api::LaneId& id,  int index,
    double length, double y_offset, const api::RBounds& lane_bounds,
    const api::RBounds& driveable_bounds)
    : segment_(segment),
      id_(id),
      index_(index),
      length_(length),
      y_offset_(y_offset),
      lane_bounds_(lane_bounds),
      driveable_bounds_(driveable_bounds) {
  DRAKE_DEMAND(segment != nullptr);
  DRAKE_DEMAND(lane_bounds_.r_min >= driveable_bounds_.r_min);
  DRAKE_DEMAND(lane_bounds_.r_max <= driveable_bounds_.r_max);
  // TODO(liang.fok) Consider initializing this variable in the constructor's
  // initializer list so branch_point_ can be declared `const`.
  branch_point_ = make_unique<BranchPoint>(
      api::BranchPointId({id.id + "_Branch_Point"}), this,
      segment->junction()->road_geometry());
}

const api::Segment* Lane::do_segment() const {
  return segment_;
}

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

api::RBounds Lane::do_lane_bounds(double) const {
  return lane_bounds_;
}

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
  return {lane_pos.s, lane_pos.r + Lane::y_offset(), lane_pos.h};
}

api::Rotation Lane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  return api::Rotation(0, 0, 0);  // roll, pitch, yaw.
}

api::LanePosition Lane::DoToLanePosition(
    const api::GeoPosition& geo_pos) const {
  DRAKE_ABORT();  // TODO(liangfok) Implement me.
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
