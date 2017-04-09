#pragma once

#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace maliput {
namespace monolane {

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


/// A cubic polynomial, f(p) = a + b*p + c*p^2 + d*p^3.
class CubicPolynomial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CubicPolynomial)

  /// Default constructor, all zero coefficients.
  CubicPolynomial() : CubicPolynomial(0., 0., 0., 0.) {}

  /// Constructs a cubic polynomial given all four coefficients.
  CubicPolynomial(double a, double b, double c, double d)
      : a_(a), b_(b), c_(c), d_(d) {
    const double df = f_p(1.) - f_p(0.);
    s_1_ = std::sqrt(1. + (df * df));
  }

  // Returns the a coefficient.
  double a() const { return a_; }

  // Returns the b coefficient.
  double b() const { return b_; }

  // Returns the c coefficient.
  double c() const { return c_; }

  // Returns the d coefficient.
  double d() const { return d_; }

  /// Evaluates the polynomial f at @p p.
  double f_p(double p) const {
    return a_ + (b_ * p) + (c_ * p * p) + (d_ * p * p * p);
  }

  /// Evaluates the derivative df/dp at @p p.
  double f_dot_p(double p) const {
    return b_ + (2. * c_ * p) + (3. * d_ * p * p);
  }

  /// Evaluates the double-derivative d^2f/dp^2 at @p p.
  double f_ddot_p(double p) const {
    return (2. * c_) + (6. * d_ * p);
  }

  // TODO(maddog@tri.global)  s_p() and p_s() need to be replaced with a
  //                          properly integrated path-length parameterization.
  //                          For the moment, we are calculating the length by
  //                          approximating the curve with a single linear
  //                          segment from (0, f(0)) to (1, f(1)), which is
  //                          not entirely awful for relatively flat curves.
  /// Returns the path-length s along the curve (p, f(p)) from p = 0 to @p p.
  double s_p(double p) const {
    return s_1_ * p;
  }

  /// Returns the inverse of the path-length parameterization s_p(p).
  double p_s(double s) const {
    return s / s_1_;
  }

  // TODO(maddog@tri.global) Until s(p) is a properly integrated path-length
  //                         parameterization, we have a need to calculate the
  //                         derivative of the actual linear function
  //                         involved in our bogus path-length approximation.
  double fake_gprime(double p) const {
    // return df;  which is...
    return f_p(1.) - f_p(0.);
  }

 private:
  double a_{};
  double b_{};
  double c_{};
  double d_{};
  double s_1_{};
};


/// Base class for the monolane implementation of api::Lane.
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
  /// @param p_scale isotropic scale factor for elevation and superelevation
  /// @param elevation elevation function (see below)
  /// @param superelevation superelevation function (see below)
  ///
  /// This is the base class for subclasses, each of which describe a
  /// primitive reference curve in the xy ground-plane of the world frame.
  /// The specific curve is expressed by a subclass's implementations of
  /// private virtual functions; see the private method xy_of_p().
  ///
  /// @p elevation and @p superelevation are cubic-polynomial functions which
  /// define the elevation and superelevation as a function of position along
  /// the planar reference curve.  @p elevation specifies the z-component of
  /// the surface at (r,h) = (0,0).  @p superelevation specifies the angle
  /// of the r-axis with respect to the horizon, i.e., how the road twists.
  /// Thus, non-zero @p superelevation contributes to the z-component at
  /// r != 0.
  ///
  /// These two functions (@p elevation and @p superelevation) must be
  /// isotropically scaled to operate over the domain p in [0, 1], where
  /// p is linear in the path-length of the planar reference curve,
  /// p = 0 corresponds to the start and p = 1 to the end.  @p p_scale is
  /// the scale factor.  In other words...
  ///
  /// Given:
  ///  * a reference curve R(p) parameterized by p in domain [0, 1], which
  ///    has a path-length q(p) in range [0, q_max], linearly related to p,
  ///    where q_max is the total path-length of R (in real-world units);
  ///  * the true elevation function E_true(q), parameterized by the
  ///    path-length q of R;
  ///  * the true superelevation function S_true(q), parameterized by the
  ///    path-length q of R;
  ///
  /// then:
  ///  * @p p_scale is q_max (and p = q / p_scale);
  ///  * @p elevation is  E_scaled = (1 / p_scale) * E_true(p_scale * p);
  ///  * @p superelevation is  S_scaled = (1 / p_scale) * S_true(p_scale * p).
  ///
  /// N.B. The override Lane::ToLanePosition() is currently restricted to lanes
  /// in which superelevation and elevation change are both zero.
  Lane(const api::LaneId& id, const api::Segment* segment,
       const api::RBounds& lane_bounds,
       const api::RBounds& driveable_bounds,
       double p_scale,
       const CubicPolynomial& elevation,
       const CubicPolynomial& superelevation)
      : id_(id), segment_(segment),
        lane_bounds_(lane_bounds),
        driveable_bounds_(driveable_bounds),
        p_scale_(p_scale),
        elevation_(elevation),
        superelevation_(superelevation) {
    DRAKE_DEMAND(lane_bounds_.r_min >= driveable_bounds_.r_min);
    DRAKE_DEMAND(lane_bounds_.r_max <= driveable_bounds_.r_max);
  }

  // TODO(maddog@tri.global)  Allow superelevation to have a center-of-rotation
  //                          which is different from r = 0.

  const CubicPolynomial& elevation() const { return elevation_; }

  const CubicPolynomial& superelevation() const { return superelevation_; }

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

  api::GeoPosition DoToGeoPosition(
      const api::LanePosition& lane_pos) const override;

  api::Rotation DoGetOrientation(
      const api::LanePosition& lane_pos) const override;

  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const override;

  // The following virtual methods define a reference curve in the xy-plane
  // of the world frame (i.e., the Earth ground plane).  The curve is a
  // parametric curve:
  //
  //    F: p --> (x, y)  for p in [0, 1]
  //
  // defined such that parameter p is linear in the path-length of the curve.

  // Returns the reference curve itself, F(p).
  virtual V2 xy_of_p(const double p) const = 0;

  // Returns the derivative of the curve with respect to p, at p,
  // i.e., F'(p0) = (dx/dp, dy/dp) at p0
  virtual V2 xy_dot_of_p(const double p) const = 0;

  // Returns the heading of the curve at p, i.e., the angle of the tangent
  // vector (with respect to x-axis) in the increasing-p direction.
  virtual double heading_of_p(const double p) const = 0;

  // Returns the derivative of the heading with respect to p,
  // i.e., d_heading/dp evaluated at p.
  virtual double heading_dot_of_p(const double p) const = 0;


  // The geometry here revolves around an abstract "world function"
  //
  //    W: (p,r,h) --> (x,y,z)
  //
  // which maps a LANE-space position to its corresponding representation in
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
  // (R_αβγ is essentially the orientation of the (s,r,h) LANE-space frame
  // at a location (s,0,0) on the reference-line of the lane.  However, it
  // is *not* necessarily the correct orientation at r != 0 or h != 0.)
  //
  // The following methods compute various terms derived from the above which
  // see repeated use.

  // Returns the parametric position p along the reference curve corresponding
  // to longitudinal position @p s along the lane.
  double p_from_s(const double s) const;

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
  // of the (s,r,h) LANE-space frame (with respect to the world frame).
  //
  // (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  // avoid recomputing it.)
  V3 s_hat_of_prh(const double p, const double r, const double h,
                  const Rot3& Rabg) const;

  // Returns the r-axis unit-vector, expressed in the world frame,
  // of the (s,r,h) LANE-space frame (with respect to the world frame).
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
  const double p_scale_{};
  const CubicPolynomial elevation_;
  const CubicPolynomial superelevation_;
};


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
