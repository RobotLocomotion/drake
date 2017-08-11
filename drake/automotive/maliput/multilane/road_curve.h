#pragma once

#include <utility>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace multilane {

/// Defines an interface for a path in a Segment object surface. The path is
/// defined by an elevation and superelevation CubicPolynomial objects and a
/// reference curve. This reference curve is a C1 function over the z=0 plane.
/// Its domain is constrained in [0;1] interval and it should map a ℝ^2 curve.
/// As per notation, p is the parameter of the reference curve, and function
/// interpolations and function derivatives as well as headings and heading
/// derivatives are expressed in world coordinates, which is the same frame as
/// api::GeoPosition.
/// By implementing this interface the road curve is defined and a complete.
class RoadCurve {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoadCurve)

  /// Constructor.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// polynomial. Note that coefficients should be scaled to match the path
  /// length integral of the reference curve.
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation polynomial. Note that coefficients should be scaled to
  /// match the path length integral of the reference curve.
  RoadCurve(const CubicPolynomial& elevation,
                  const CubicPolynomial& superelevation)
      : elevation_(elevation), superelevation_(superelevation) {}

  virtual ~RoadCurve() = default;

  const CubicPolynomial& elevation() const { return elevation_; }

  const CubicPolynomial& superelevation() const { return superelevation_; }

  /// Computes the composed curve path integral in the interval of p = [0; 1].
  /// @return The path length integral of the curve composed with the elevation
  /// polynomial.
  double trajectory_length() const {
    return elevation_.s_p(1.0) * length();
  }

  /// Computes the interpolation of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The reference curve itself, F(p).
  virtual Vector2<double> xy_of_p(double p) const = 0;

  /// Computes the first derivative interpolation of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The derivative of the curve with respect to p, at @p p, i.e.,
  /// F'(p0) = (dx/dp, dy/dp) at p0.
  virtual Vector2<double> xy_dot_of_p(double p) const = 0;

  /// Computes the heading interpolation of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The heading of the curve at @p p, i.e.,
  /// the angle of the tangent vector (with respect to x-axis) in the
  /// increasing-p direction.
  virtual double heading_of_p(double p) const = 0;

  /// Computes the first derivative heading interpolation of the reference
  /// curve.
  /// @param p The reference curve parameter.
  /// @return The derivative of the heading with respect to p, i.e.,
  /// d_heading/dp evaluated at @p p.
  virtual double heading_dot_of_p(double p) const = 0;

  /// Computes the path length integral of the reference curve for the interval
  /// [0;1] of p.
  /// @return The path length integral of the reference curve.
  virtual double length() const = 0;

  /// Converts a @p geo_coordinate in the world frame to the composed curve
  /// frame, i.e., the superposition of the reference curve, elevation and
  /// superelevation polynomials. The resulting coordinates are saturated to
  /// @p lateral_bounds and @p height_bounds in the lateral and vertical
  /// directions over the composed curve trajectory. The path length coordinate
  /// is saturated in the interval [0; trajectory_length()].
  /// @param geo_coordinate A 3D vector in the world frame to be converted to
  /// the composed curve frame.
  /// @param lateral_bounds An api::RBounds object that represents the lateral
  /// bounds of the surface mapping.
  /// @param height_bounds An api::HBounds object that represents the elevation
  /// bounds of the surface mapping.
  /// @return A 3D vector that represents the coordinates with respect to the
  /// composed curve. The first dimension represents the path length coordinate,
  /// the second dimension is the lateral deviation from the composed curve and
  /// the third one is the vertical deviation from the composed curve too. The
  /// frame where this vector is defined is the same as api::LanePosition.
  virtual Vector3<double> ToCurveFrame(
      const Vector3<double>& geo_coordinate,
      const api::RBounds& lateral_bounds,
      const api::HBounds& height_bounds) const = 0;

  /// Checks if the surface does not fold due to the combination of elevation
  /// and superelevation polynomials. @p lateral_bounds and @p heigh_bounds are
  /// considered over the composed curve to derive the critical points to check.
  /// @param lateral_bounds An api::RBounds object that represents the lateral
  /// bounds of the surface mapping.
  /// @param height_bounds An api::HBounds object that represents the elevation
  /// bounds of the surface mapping.
  /// @return True when the surface does not fold.
  virtual bool IsValid(
      const api::RBounds& lateral_bounds,
      const api::HBounds& height_bounds) const = 0;

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
