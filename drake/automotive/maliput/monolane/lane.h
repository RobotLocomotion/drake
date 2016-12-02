#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"

#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace maliput {
namespace monolane {

class BranchPoint;
class Segment;

typedef Eigen::Matrix<double, 3, 1> V3;
typedef Eigen::Matrix<double, 2, 1> V2;

/// An R^3 rotation parameterized by roll, pitch, yaw.
///
/// This effects a compound rotation around space-fixed x-y-z axes:
///
///   Rot3(yaw,pitch,roll) * V = RotZ(yaw) * RotY(pitch) * RotX(roll) * V
class Rot3 {
 public:
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
  /// Default constructor, all zero coefficients.
  CubicPolynomial() : CubicPolynomial(0., 0., 0., 0.) {}

  /// Constructs a cubic polynomial given all four coefficients.
  CubicPolynomial(double a, double b, double c, double d)
      : a_(a), b_(b), c_(c), d_(d) {
    const double df = f_p(1.) - f_p(0.);
    s_1_ = std::sqrt(1. + (df * df));
  }

  /// Evaluates the polynomial at @p p.
  double f_p(double p) const {
    return a_ + (b_ * p) + (c_ * p * p) + (d_ * p * p * p);
  }

  /// Evaluates the derivative df/dp at @p p.
  double fdot_p(double p) const {
    return b_ + (2. * c_ * p) + (3. * d_ * p * p);
  }

  /// Evaluates the double-derivative d^2f/dp^2 at @p p.
  double fddot_p(double p) const {
    return (2. * c_) + (6. * d_ * p);
  }

  // TODO(maddog@tri.global)  s_p() and p_s() need to be replaced with a
  //                          properly integrated path-length parameterization.
  /// Returns the path-length s along the curve (p, f(p)) from p=0 to @p p.
  double s_p(double p) const {
    return s_1_ * p;
  }

  /// Returns the inverse of the path-length parameterization s_p(p).
  double p_s(double s) const {
    return s / s_1_;
  }

  // TODO(maddog@tri.global) Until s(p) is a properly integrated path-length
  //                         parameterization, we have need to calculate a
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


/// Base class for the monolane implementation of Lane.
class Lane : public api::Lane {
 public:
  /// Construct a base Lane.
  ///
  /// @param id the ID
  /// @param segment the Segment to which this Lane will belong
  /// @param lane_bounds nominal bounds of the lane, uniform along the entire
  ///        reference path
  /// @param driveable_bounds driveable bounds of the lane, uniform along the
  ///        entire reference path
  /// @param p_scale isotropic scale factor for elevation and superelevation
  /// @param elevation elevation function (see below)
  /// @param superelevation superelevation function (see below)
  ///
  /// @p elevation and @p superelevation are cubic-polynomial functions which
  /// define the elevation and superelevation as a function of position along
  /// the planar reference curve defined by the concrete subclass.
  /// (@p elevation specifies the z-component of the surface at r=0, h=0 ---
  /// @p superelevation contributes at r!=0.)  These two functions must be
  /// isotropically scaled to operate over the domain p in [0, 1], where p is
  /// linear in the path-length of the planar reference curve, p=0 corresponds
  /// to the start and p=1 to the end.  @p p_scale is the scale factor.
  /// In other words...
  ///
  /// Given
  ///  * a reference curve R(s) parameterized by path-length s in the
  ///    domain [0., s_max], where s_max is the total path-length of R;
  ///  * the true elevation function E_true(s), parameterized by the
  ///    path-length s of R;
  ///  * the true superelevation function S_true(s), parameterized by the
  ///    path-length s of R;
  /// then:
  ///  * @p p_scale is s_max
  ///  * @p elevation is  E_scaled = (1 / p_scale) * E_true(p_scale * p)
  ///  * @p superelevation is  S_scaled = (1 / p_scale) * S_true(p_scale * p)
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

  // TODO(maddog@tri.global)  Allow superelevation to have a center-of-rotation
  ///                         which is different from r=0.

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
  const CubicPolynomial elevation_;
  const CubicPolynomial superelevation_;
};


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
