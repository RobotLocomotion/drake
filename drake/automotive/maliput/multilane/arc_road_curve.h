#pragma once

#include <cmath>

#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {

/// RoadCurve specification for a reference curve that describes a piece
/// of an arc.
class ArcRoadCurve : public RoadCurve {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArcRoadCurve)

  /// Constructor. The reference curve is created from the circle @p center, the
  /// @p radius, initial angular position @p theta0 and the angle span
  /// @p d_theta. @p elevation and @p superelevation polynomials will be used to
  /// feed RoadGeometry parent class.
  /// @param center A 2D vector that describes the position of the arc's center
  /// of rotation in world coordinates (over the z=0 plane).
  /// @param radius Arc's radius length. It must be positive
  /// @param theta0 Angle in radians that describes the angular position that
  /// matches p=0 position interpolation.
  /// @param d_theta Angle in radians that describes the angular path of the
  /// reference curve.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// polynomial. Note that coefficients should be scaled to match the path
  /// length integral of the reference curve.
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation polynomial. Note that coefficients should be scaled to
  /// match the path length integral of the reference curve.
  /// @throws std::runtime_error When @p radius is not positive.
  explicit ArcRoadCurve(const Vector2<double>& center, double radius,
                        double theta0, double d_theta,
                        const CubicPolynomial& elevation,
                        const CubicPolynomial& superelevation)
      : RoadCurve(elevation, superelevation),
        center_(center),
        radius_(radius),
        theta0_(theta0),
        d_theta_(d_theta) {
    DRAKE_THROW_UNLESS(radius > 0.0);
  }

  ~ArcRoadCurve() override = default;

  /// Computes the position interpolation as a function of @p p.
  /// The result will be computed with the following function:
  ///      [x;y] = center + [radius * cos(θ); radius * sin(θ)]
  /// and:
  ///      θ = θ_0 + (p * Δθ)
  /// @param p Reference curve parameter.
  /// @return The position in the world frame of the reference curve.
  Vector2<double> xy_of_p(double p) const override {
    const double theta = theta_of_p(p);
    return center_ + Vector2<double>(radius_ * std::cos(theta),
                                     radius_ * std::sin(theta));
  }

  /// Computes the derivative of the position as a function @p p. Chain rule
  /// must be applied since the x and y coordinates of are functions of θ, which
  /// is a function of p at the same time.
  /// Given:
  ///      [x;y] = center + [radius * cos(θ); radius * sin(θ)]
  /// and:
  ///      θ = θ_0 + (p * Δθ)
  ///      dθ/dp = Δθ
  /// then:
  ///      [dx/dp; dy/dp] = [-radius * sin(θ) * dθ/dp; radius * cos(θ) * dθ/dp]
  ///                     = [-radius * sin(θ) * Δθ; radius * cos(θ) * Δθ]
  /// @param p Reference curve parameter.
  /// @return The tangent at @p p of the reference curve.
  Vector2<double> xy_dot_of_p(double p) const override {
    const double theta = theta_of_p(p);
    return Vector2<double>(-radius_ * std::sin(theta) * d_theta_,
                           radius_ * std::cos(theta) * d_theta_);
  }

  /// Computes the heading of the tangent at @p p. Since the reference geometry
  /// represents a piece of an arc, the tangent is normal to the angle position.
  /// Given:
  ///     θ = θ_0 + (p * Δθ)
  /// then:
  ///     heading = θ + sign(Δθ) * π / 2.0
  /// sign(Δθ) is used to express the increasing-p direction of the tangent.
  /// @param p The parameter of the heading function.
  /// @return The heading angle in radians.
  double heading_of_p(double p) const override {
    const double theta = theta_of_p(p);
    return theta + std::copysign(M_PI / 2.0, d_theta_);
  }

  /// Computes the heading derivative with respect to p at @p p. Note that the
  /// result is constant and Δθ becuase of the following.
  /// Given:
  ///      heading = θ + sign(Δθ) * π / 2.0
  /// and:
  ///      θ = θ_0 + (p * Δθ)
  /// then:
  ///      dheading/dp = dθ/dp = Δθ
  /// @param p The parameter of the heading derivative function.
  /// @return The heading derivative with respect to p at @p p.
  double heading_dot_of_p(double p) const override {
    unused(p);
    return d_theta_;
  }

  /// Computes the reference curve path length in the interval of p = [0; 1].
  /// @return The path length integral of the reference curve.
  double length() const override { return radius_ * std::abs(d_theta_); }

  /// Computes the coordinate frame conversion from world coordinates to
  /// composed curve frame (the same as api::LanePosition describes) for the arc
  /// reference curve, elevation and superelevation polynomials combination.
  /// @param geo_coordinate A 3D vector in the world frame to be converted to
  /// the composed curve frame.
  /// @param lateral_bounds An api::RBounds object that represents the lateral
  /// bounds of the surface mapping.
  /// @param height_bounds An api::HBounds object that represents the elevation
  /// bounds of the surface mapping.
  /// @return True.
  /// @return A 3D vector that represents the coordinates with respect to the
  /// composed curve. The first dimension represents the path length coordinate,
  /// the second dimension is the lateral deviation from the composed curve and
  /// the third one is the vertical deviation from the composed curve too. The
  /// frame where this vector is defined is the same as api::LanePosition.
  Vector3<double> ToCurveFrame(
      const Vector3<double>& geo_coordinate,
      const api::RBounds& lateral_bounds,
      const api::HBounds& height_bounds) const override;

  /// Evaluates extrema in superelevation polynomial to verify that for the
  /// given @p lateral_bounds the surface do not fold over itself.
  /// @param lateral_bounds An api::RBounds object that represents the lateral
  /// bounds of the surface mapping.
  /// @param height_bounds An api::HBounds object that represents the elevation
  /// bounds of the surface mapping.
  /// @return True.
  bool IsValid(
      const api::RBounds& lateral_bounds,
      const api::HBounds& height_bounds) const override;

 private:
  // Computes the absolute position along reference arc as an angle in
  // range [theta0_, (theta0 + d_theta_)],
  // as a function of parameter @p p (in domain [0, 1]).
  double theta_of_p(double p) const { return theta0_ + (p * d_theta_); }

  // Center of rotation in z=0 plane, world coordinates, for the arc reference
  // curve.
  const Vector2<double> center_;
  // The length of the radius of the arc.
  const double radius_{};
  // The angular position at which the piece of arc starts.
  const double theta0_{};
  // The aperture angle of the arc. Its sign is meaningful since it provides the
  // sense of direction.
  const double d_theta_{};
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
