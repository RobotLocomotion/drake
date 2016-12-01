#pragma once

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"

#include "drake/automotive/maliput/monolane/mathiness.h"

namespace drake {
namespace maliput {
namespace monolane {

class BranchPoint;
class Segment;


/// Base class for the monolane implementation of Lane.
class Lane : public api::Lane {
 public:
  /// @param id the ID
  /// @param segment the Segment to which this Lane will belong
  Lane(const api::LaneId& id, Segment* segment,
       const api::RBounds& lane_bounds,
       const api::RBounds& driveable_bounds,
       const double p_scale,
       const CubicPolynomial& elevation,
       const CubicPolynomial& superelevation)
      : id_(id), segment_(segment),
        lane_bounds_(lane_bounds),
        driveable_bounds_(driveable_bounds),
        p_scale_(p_scale),
        elevation_(elevation),
        superelevation_(superelevation) {}

  // TODO(maddog) Explain clearly how/why the elevation and superelevation
  //              functions need to be isotropically scaled by the xy-projected
  //              arc-length of the xy-primitive curve.


  // TODO(maddog) Allow lane centerline to be offset from "segment ref line",
  //              so that superelevation can have a center-of-rotation which
  //              is different from r=0.

  const CubicPolynomial& elevation() const { return elevation_; }

  const CubicPolynomial& superelevation() const { return superelevation_; }

  void SetStartBp(BranchPoint* bp) { start_bp_ = bp; }

  void SetEndBp(BranchPoint* bp) { end_bp_ = bp; }

  BranchPoint* start_bp() { return start_bp_; }

  BranchPoint* end_bp() { return end_bp_; }

  virtual ~Lane() {}

 private:
  const api::LaneId do_id() const override { return id_; }

  const api::Segment* do_segment() const override;

  int do_index() const override { return 0; }  // Only one lane per segment!

  const api::Lane* do_to_left() const override { return nullptr; }

  const api::Lane* do_to_right() const override { return nullptr; }

  const api::BranchPoint* DoGetBranchPoint(
      const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd::Which which_end) const override;

  std::unique_ptr<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd::Which which_end) const override;

  double do_length() const override;

  api::RBounds do_lane_bounds(double) const override { return lane_bounds_; }

  api::RBounds do_driveable_bounds(double) const override {
    return driveable_bounds_;
  }

  api::GeoPosition DoToGeoPosition(
      const api::LanePosition& lane_pos) const override;

  api::Rotation DoGetOrientation(
      const api::LanePosition& lane_pos) const override;

  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const override;

  virtual V2 xy_of_p(const double p) const = 0;
  virtual V2 xy_dot_of_p(const double p) const = 0;
  virtual double heading_of_p(const double p) const = 0;
  virtual double heading_dot_of_p(const double p) const = 0;

  Rot3 rot3_of_p(const double p) const;
  double p_from_s(const double s) const;
  V3 W_prime_of_prh(const double p, const double r, const double h,
                     const Rot3& gba) const;
  V3 s_hat_of_prh(const double p, const double r, const double h,
                   const Rot3& gba) const;
  V3 r_hat_of_gba(const Rot3& gba) const;

  const api::LaneId id_;
  const Segment* segment_{};
  BranchPoint* start_bp_{};
  BranchPoint* end_bp_{};

  const api::RBounds lane_bounds_;
  const api::RBounds driveable_bounds_;
  const double p_scale_{};
  // Common elevation and superelevation structures.
  const CubicPolynomial elevation_;
  const CubicPolynomial superelevation_;
};


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
