#include "drake/automotive/maliput/multilane/lane.h"

#include <cmath>

#include "drake/automotive/maliput/multilane/branch_point.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace multilane {

const api::Segment* Lane::do_segment() const { return segment_; }

const api::BranchPoint* Lane::DoGetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart:  { return start_bp_; }
    case api::LaneEnd::kFinish: { return end_bp_; }
  }
  DRAKE_ABORT();
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


double Lane::do_length() const { return road_curve_->s_from_p(1., r0_); }

api::GeoPosition Lane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  // Recover parameter p from arc-length position s.
  const double p = road_curve_->p_from_s(lane_pos.s(), r0_);
  const Vector3<double> xyz =
      road_curve_->W_of_prh(p, lane_pos.r() + r0_, lane_pos.h());
  return {xyz.x(), xyz.y(), xyz.z()};
}


api::Rotation Lane::DoGetOrientation(const api::LanePosition& lane_pos) const {
  const double p = road_curve_->p_from_s(lane_pos.s(), r0_);
  const Rot3 rotation =
      road_curve_->Orientation(p, lane_pos.r() + r0_, lane_pos.h());
  return api::Rotation::FromRpy(rotation.roll(), rotation.pitch(),
                                rotation.yaw());
}


api::LanePosition Lane::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity) const {

  const double p = road_curve_->p_from_s(position.s(), r0_);
  const double r = position.r() + r0_;
  const double h = position.h();
  const Rot3 R = road_curve_->Rabg_of_p(p);
  // TODO(maddog@tri.global)  When s(p) is integrated properly,
  //                          fake_g_prime should be replaced by real_g_prime.
  const double real_g_prime = road_curve_->elevation().f_dot_p(p);
  const double fake_g_prime = road_curve_->elevation().fake_gprime(p);

  // The definition of path-length of a path along σ yields dσ = |∂W/∂p| dp
  // evaluated at (p, r, h).
  // Similarly, path-length s along the segment surface at r = r0 (which is
  // along the Lane's centerline) is related to p by ds = |∂W/∂p| dp evaluated
  // at (p, r0, 0).  Chaining yields ds/dσ:
  const double ds_dsigma =
      road_curve_->W_prime_of_prh(p, r0_, 0, R, fake_g_prime).norm() /
      road_curve_->W_prime_of_prh(p, r, h, R, real_g_prime).norm();

  return api::LanePosition(ds_dsigma * velocity.sigma_v,
                           velocity.rho_v,
                           velocity.eta_v);
}

api::LanePosition Lane::DoToLanePosition(
      const api::GeoPosition& geo_position,
      api::GeoPosition* nearest_position,
      double* distance) const {
  // Computes the lateral extents of the surface in terms of the definition of
  // the reference curve. It implies a translation of the driveable bounds
  // center by the lane by r0 distance.
  const double r_min = driveable_bounds_.min() + r0_;
  const double r_max = driveable_bounds_.max() + r0_;
  // Lane position is over the segment's road curve frame, so a change is
  // needed. That implies getting the path length s from p and translating the r
  // coordinate because of the offset.
  const V3 lane_position_in_segment_curve_frame = road_curve_->ToCurveFrame(
      geo_position.xyz(), r_min, r_max, elevation_bounds_);
  const double s = road_curve_->s_from_p(
      lane_position_in_segment_curve_frame[0], r0_);
  const api::LanePosition lane_position = api::LanePosition(
      s,
      lane_position_in_segment_curve_frame[1] - r0_,
      lane_position_in_segment_curve_frame[2]);

  const api::GeoPosition nearest = ToGeoPosition(lane_position);
  if (nearest_position != nullptr) {
    *nearest_position = nearest;
  }

  if (distance != nullptr) {
    *distance = (nearest.xyz() - geo_position.xyz()).norm();
  }

  return lane_position;
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
