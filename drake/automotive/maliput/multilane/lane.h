#pragma once

#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace maliput {
namespace multilane {

class BranchPoint;

typedef Vector2<double> V2;
typedef Vector3<double> V3;

/// An R^3 rotation parameterized by roll, pitch, yaw.
///
/// This effects a compound rotation around space-fixed x-y-z axes:
///
///   Rot3(yaw,pitch,roll) * V = RotZ(yaw) * RotY(pitch) * RotX(roll) * V
class Rot3 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rot3)

  Rot3(double roll, double pitch, double yaw) : rpy_(roll, pitch, yaw) {}

  /// Applies the rotation to a 3-vector.
  V3 apply(const V3& in) const { return math::rpy2rotmat(rpy_) * in; }

  double yaw() const { return rpy_(2); }
  double pitch() const { return rpy_(1); }
  double roll() const { return rpy_(0); }

 private:
  Eigen::Matrix<double, 3, 1, Eigen::DontAlign> rpy_;
};


/// Base class for the multilane implementation of api::Lane.
class Lane : public api::Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

  /// Constructs a Lane.
  ///
  /// @param id the ID
  /// @param segment the Segment to which this Lane will belong, which must
  ///        remain valid for the lifetime of this class
  /// @param lane_bounds nominal bounds of the lane, uniform along the entire
  ///        reference path, which must be a subset of @p driveable_bounds
  /// @param driveable_bounds driveable bounds of the lane, uniform along the
  ///        entire reference path
  /// @param elevation_bounds elevation bounds of the lane, uniform along the
  ///        entire driveable surface
  /// @param road_curve The trajectory of the Lane over parent @p segment's
  ///        surface.
  /// @param r0 The lateral displacement with respect to the @p road_curve's
  ///        reference curve.
  ///
  /// N.B. The override Lane::ToLanePosition() is currently restricted to lanes
  /// in which superelevation and elevation change are both zero.
  Lane(const api::LaneId& id, const api::Segment* segment,
       const api::RBounds& lane_bounds,
       const api::RBounds& driveable_bounds,
       const api::HBounds& elevation_bounds,
       const RoadCurve *road_curve,
       double r0)
      : id_(id), segment_(segment),
        lane_bounds_(lane_bounds),
        driveable_bounds_(driveable_bounds),
        elevation_bounds_(elevation_bounds),
        road_curve_(road_curve),
        r0_(r0) {
    DRAKE_DEMAND(lane_bounds_.min() >= driveable_bounds_.min());
    DRAKE_DEMAND(lane_bounds_.max() <= driveable_bounds_.max());
    DRAKE_DEMAND(road_curve != nullptr);
  }

  // TODO(maddog@tri.global)  Allow superelevation to have a center-of-rotation
  //                          which is different from r = 0.

  const CubicPolynomial& elevation() const {
    return road_curve_->elevation();
  }

  const CubicPolynomial& superelevation() const {
    return road_curve_->superelevation();
  }

  void SetStartBp(BranchPoint* bp) { start_bp_ = bp; }
  void SetEndBp(BranchPoint* bp) { end_bp_ = bp; }

  BranchPoint* start_bp() { return start_bp_; }

  BranchPoint* end_bp() { return end_bp_; }

  ~Lane() override = default;

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

  api::HBounds do_elevation_bounds(double, double) const override {
    return elevation_bounds_;
  }

  api::GeoPosition DoToGeoPosition(
      const api::LanePosition& lane_pos) const override;

  api::Rotation DoGetOrientation(
      const api::LanePosition& lane_pos) const override;

  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const override;

  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_position,
      api::GeoPosition* nearest_position,
      double* distance) const override;

  // The geometry here revolves around an abstract "world function"
  //
  //    W: (p,r,h) --> (x,y,z)
  //
  // which maps a `Lane`-frame position to its corresponding representation in
  // world coordinates (with the caveat that instead of the lane's native
  // longitudinal coordinate 's', the reference curve parameter 'p' is used).
  //
  // W is derived from the three functions which define the lane:
  //
  //   G: p --> (x,y)     = the reference curve, a.k.a. xy_of_p()
  //   Z: p --> z / q_max = the elevation function, a.k.a. elevation_
  //   Θ: p --> θ / q_max = the superelevation function, a.k.a. superelevation_
  //
  // as:
  //
  //   (x,y,z) = W(p,r,h) = (G(p), Z(p)) + R_αβγ*(0,r,h)
  //
  // where R_αβγ is the roll/pitch/yaw rotation given by angles:
  //
  //   α = Θ(p)
  //   β = -atan(dZ/dp) at p
  //   γ = atan2(dG_y/dp, dG_x/dp) at p
  //
  // (R_αβγ is essentially the orientation of the (s,r,h) `Lane`-frame
  // at a location (s,0,0) on the reference-line of the lane.  However, it
  // is *not* necessarily the correct orientation at r != 0 or h != 0.)
  //
  // The following methods compute various terms derived from the above which
  // see repeated use.
  //
  // Note that xy_of_p(), xy_dot_of_p(), heading_of_p() and heading_dot_of_p(),
  // as well as the reference to the elevation and superelevation objects live
  // on the RoadCurve object.
  //
  // Note that the Lane includes r0 lateral offset of the reference curve, which
  // actually lives in RoadCurve object. This r0 distance makes the reference
  // curve be scaled and the superelevation polynomial act over the road curve
  // since it center of rotation is over the original reference curve. There is
  // support yet for Lanes that has both r0 and superelevation polynomial
  // different of zero.

  // Returns the rotation R_αβγ, evaluated at @p p along the reference curve.
  Rot3 Rabg_of_p(const double p) const;

  // Returns W' = ∂W/∂p, the partial differential of W with respect to p,
  // evaluated at @p p, @p r, @p h.
  //
  // (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  // avoid recomputing it.)
  V3 W_prime_of_prh(const double p, const double r, const double h,
                    const Rot3& Rabg) const;

  // Returns the s-axis unit-vector, expressed in the world frame,
  // of the (s,r,h) `Lane`-frame (with respect to the world frame).
  //
  // (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  // avoid recomputing it.)
  V3 s_hat_of_prh(const double p, const double r, const double h,
                  const Rot3& Rabg) const;

  // Returns the r-axis unit-vector, expressed in the world frame,
  // of the (s,r,h) `Lane`-frame (with respect to the world frame).
  //
  // (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  // avoid recomputing it.)
  V3 r_hat_of_Rabg(const Rot3& Rabg) const;

  const api::LaneId id_;
  const api::Segment* segment_{};
  BranchPoint* start_bp_{};
  BranchPoint* end_bp_{};

  const api::RBounds lane_bounds_;
  const api::RBounds driveable_bounds_;
  const api::HBounds elevation_bounds_;
  const RoadCurve* road_curve_{};
  const double r0_;
};


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
