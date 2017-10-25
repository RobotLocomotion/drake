#pragma once

#include <utility>

#include <Eigen/Dense>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace maliput {
namespace multilane {

/// An R^3 rotation parameterized by roll, pitch, yaw.
///
/// This effects a compound rotation around space-fixed x-y-z axes:
///
///   Rot3(roll, pitch, yaw) * V = RotZ(yaw) * RotY(pitch) * RotX(roll) * V
class Rot3 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rot3)

  Rot3(double roll, double pitch, double yaw) : rpy_(roll, pitch, yaw) {}

  /// Applies the rotation to a 3-vector.
  Vector3<double> apply(const Vector3<double>& in) const {
    return math::rpy2rotmat(rpy_) * in;
  }

  double yaw() const { return rpy_(2); }
  double pitch() const { return rpy_(1); }
  double roll() const { return rpy_(0); }

 private:
  Eigen::Matrix<double, 3, 1, Eigen::DontAlign> rpy_;
};

/// Defines an interface for a path in a Segment object surface. The path is
/// defined by an elevation and superelevation CubicPolynomial objects and a
/// reference curve. This reference curve is a C1 function in the z=0 plane.
/// Its domain is constrained in [0;1] interval and it should map a ℝ² curve.
/// As per notation, p is the parameter of the reference curve, not necessarily
/// arc length s, and function interpolations and function derivatives as well
/// as headings and heading derivatives are expressed in world coordinates,
/// which is the same frame as api::GeoPosition.
/// By implementing this interface the road curve is defined and complete.
///
/// The geometry here revolves around an abstract "world function"
///
///    W: (p,r,h) --> (x,y,z)
///
/// which maps a `Lane`-frame position to its corresponding representation in
/// world coordinates (with the caveat that instead of the lane's native
/// longitudinal coordinate 's', the reference curve parameter 'p' is used).
///
/// W is derived from the three functions which define the lane:
///
///   G: p --> (x,y)     = the reference curve, a.k.a. xy_of_p()
///   Z: p --> z / q_max = the elevation function, a.k.a. elevation_
///   Θ: p --> θ / q_max = the superelevation function, a.k.a. superelevation_
///
/// as:
///
///   (x,y,z) = W(p,r,h) = (G(p), Z(p)) + R_αβγ*(0,r,h)
///
/// where R_αβγ is the roll/pitch/yaw rotation given by angles:
///
///   α = Θ(p)
///   β = -atan(dZ/dp) at p
///   γ = atan2(dG_y/dp, dG_x/dp) at p
///
/// (R_αβγ is essentially the orientation of the (s,r,h) `Lane`-frame
/// at a location (s,0,0) on the reference-line of the lane.  However, it
/// is *not* necessarily the correct orientation at r != 0 or h != 0.)
///
/// The W(p,r,h) "world function" is defined by the RoadCurve referenced by a
/// Lane's Segment. A Lane is also defined by a r0 lateral offset with respect
/// to the reference curve of the RoadCurve. Thus, a mapping from the local
/// (s,r,h) lane-frame of the Lane becomes:
///
/// (x,y,z) = L(s,r,h) = W(P(s, r0), r + r0, h),
///
/// where P:(s, r0) --> (p) is a (potentially non-linear) function dependent on
/// the RoadCurve's reference-curve, elevation, and superelevation functions.
///
/// TODO(maddog-tri)  Add support for Lanes with both non-zero r0 and
///                   superelevation polynomial.
class RoadCurve {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadCurve)

  virtual ~RoadCurve() = default;

  const CubicPolynomial& elevation() const { return elevation_; }

  const CubicPolynomial& superelevation() const { return superelevation_; }

  /// Computes the parametric position p along the reference curve corresponding
  /// to longitudinal position (in path-length) `s` along a parallel curve
  /// laterally offset by `r` from the reference curve.
  /// @return The parametric position p along an offset of the reference curve.
  virtual double p_from_s(double s, double r) const = 0;

  /// Computes the path length integral in the interval of the parameter [0; p]
  /// and along a parallel curve laterally offset by `r` the planar reference
  /// curve.
  /// @return The path length integral of the curve composed with the elevation
  /// polynomial.
  virtual double s_from_p(double p, double r) const = 0;

  /// Computes the reference curve.
  /// @param p The reference curve parameter.
  /// @return The reference curve itself, F(p).
  virtual Vector2<double> xy_of_p(double p) const = 0;

  /// Computes the first derivative of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The derivative of the curve with respect to p, at @p p, i.e.,
  /// F'(p0) = (dx/dp, dy/dp) at p0.
  virtual Vector2<double> xy_dot_of_p(double p) const = 0;

  /// Computes the heading of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The heading of the curve at @p p, i.e.,
  /// the angle of the tangent vector (with respect to x-axis) in the
  /// increasing-p direction.
  virtual double heading_of_p(double p) const = 0;

  /// Computes the first derivative heading of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The derivative of the heading with respect to p, i.e.,
  /// d_heading/dp evaluated at @p p.
  virtual double heading_dot_of_p(double p) const = 0;

  /// Computes the path length integral of the reference curve for the interval
  /// [0;1] of p.
  /// @return The path length integral of the reference curve.
  // TODO(maddog-tri)  This method should be renamed to match the Maliput's
  //                   documentation as well as other variable names along the
  //                   implementation.
  virtual double p_scale() const = 0;

  /// Converts a @p geo_coordinate in the world frame to the composed curve
  /// frame, i.e., the superposition of the reference curve, elevation and
  /// superelevation polynomials. The resulting coordinates [p, r, h] are
  /// saturated in the following domain ranges.
  ///
  /// - p: [0, 1]
  /// - r: [@p r_min, @p r_max]
  /// - h: [@p height_bounds]
  /// @param geo_coordinate A 3D vector in the world frame to be converted to
  /// the composed curve frame.
  /// @param r_min Minimum lateral distance from the composed curve to saturate,
  /// if it is necessary, the result in the given direction.
  /// @param r_max Maximum lateral distance from the composed curve to evaluate,
  /// if it is necessary, the result in the given direction
  /// @param height_bounds An api::HBounds object that represents the elevation
  /// bounds of the surface mapping.
  /// @return A 3D vector [p, r, h], that represent the domain coordinates of
  /// the world function, that gives as world function output @p geo_cooridnate.
  virtual Vector3<double> ToCurveFrame(
      const Vector3<double>& geo_coordinate,
      double r_min, double r_max,
      const api::HBounds& height_bounds) const = 0;

  /// Checks that there are no self-intersections (singularities) in the volume
  /// created by applying the constant @p r_min, @p r_max and @p height_bounds
  /// to the RoadCurve.
  /// @param r_min Minimum lateral distance from the composed curve to evaluate
  /// the validity of the geometry.
  /// @param r_max Maximum lateral distance from the composed curve to evaluate
  /// the validity of the geometry.
  /// @param height_bounds An api::HBounds object that represents the elevation
  /// bounds of the surface mapping.
  /// @return True when there are no self-intersections.
  virtual bool IsValid(double r_min, double r_max,
                       const api::HBounds& height_bounds) const = 0;

  /// Returns W, the world function evaluated at @p p, @p r, @p h.
  Vector3<double> W_of_prh(double p, double r, double h) const;

  /// Returns W' = ∂W/∂p, the partial differential of W with respect to p,
  /// evaluated at @p p, @p r, @p h.
  ///
  /// (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  /// avoid recomputing it.)
  /// (@p g_prime must be the result of elevation().f_dot_p(p) --- passed in
  /// here to avoid recomputing it.)
  Vector3<double> W_prime_of_prh(double p, double r, double h, const Rot3& Rabg,
                                 double g_prime) const;

  /// Returns the rotation R_αβγ, evaluated at @p p along the reference curve.
  Rot3 Rabg_of_p(double p) const;

  /// Returns the rotation R_αβγ, evaluated at @p p, @p r and @p h.
  Rot3 Orientation(double p, double r, double h) const;

  /// Returns the s-axis unit-vector, expressed in the world frame,
  /// of the (s,r,h) `Lane`-frame (with respect to the world frame).
  ///
  /// (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  /// avoid recomputing it.)
  /// (@p g_prime must be the result of elevation().f_dot_p(p) --- passed in
  /// here to avoid recomputing it.)
  Vector3<double> s_hat_of_prh(double p, double r, double h, const Rot3& Rabg,
                               double g_prime) const;

  /// Returns the r-axis unit-vector, expressed in the world frame,
  /// of the (s,r,h) `Lane`-frame (with respect to the world frame).
  ///
  /// (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  /// avoid recomputing it.)
  Vector3<double> r_hat_of_Rabg(const Rot3& Rabg) const;

 protected:
  /// Constructs a road curve given elevation and superelevation curves.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// function (see below for more details).
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation function (see below for more details).
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
  /// p = 0 corresponds to the start and p = 1 to the end. p_scale() is
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
  ///  * p_scale is q_max (and p = q / p_scale);
  ///  * @p elevation is  E_scaled = (1 / p_scale) * E_true(p_scale * p);
  ///  * @p superelevation is  S_scaled = (1 / p_scale) * S_true(p_scale * p).
  RoadCurve(const CubicPolynomial& elevation,
            const CubicPolynomial& superelevation)
      : elevation_(elevation), superelevation_(superelevation) {}

 private:
  // A polynomial that represents the elevation change as a function of p.
  CubicPolynomial elevation_;
  // A polynomial that represents the superelevation angle change as a function
  // of p.
  CubicPolynomial superelevation_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
