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

  /// Constructor. The reference curve is created from the circle `center`, the
  /// `radius`, initial angular position `theta0` and the angle span
  /// `d_theta`. `elevation` and `superelevation` polynomials will be used to
  /// feed RoadGeometry parent class.
  /// @param center Center of the reference arc.
  /// @param radius Radius of the reference arc (must be positive).
  /// @param theta0 Angle of the start point of the reference arc with respect
  /// to `center` (0 == parallel to x-axis).
  /// @param d_theta Central angle of the arc, i.e., angular displacement
  /// from start to end.  d_theta > 0 is counter-clockwise.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// polynomial. See RoadCurve class constructor for more details.
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation polynomial. See RoadCurve class constructor for more
  /// details.
  /// @param linear_tolerance The linear tolerance, in meters, for all
  /// computations. See RoadCurve class constructor for more details.
  /// @param scale_length The minimum spatial period of variation in the curve,
  /// in meters. See RoadCurve class constructor for more details.
  /// @param computation_policy Policy to guide all computations. If geared
  /// towards speed, computations will make use of analytical expressions even
  /// if not actually correct for the curve as specified.
  /// @throws std::runtime_error if @p radius is not a positive number.
  /// @throws std::runtime_error if @p linear_tolerance is not a positive
  /// number.
  /// @throws std::runtime_error if @p scale_length is not a positive number.
  explicit ArcRoadCurve(
      const Vector2<double>& center, double radius,
      double theta0, double d_theta,
      const CubicPolynomial& elevation,
      const CubicPolynomial& superelevation,
      double linear_tolerance, double scale_length,
      ComputationPolicy computation_policy)
      : RoadCurve(linear_tolerance, scale_length,
                  elevation, superelevation,
                  computation_policy),
        center_(center), radius_(radius), theta0_(theta0), d_theta_(d_theta) {
    DRAKE_THROW_UNLESS(radius > 0.0);
  }

  ~ArcRoadCurve() override = default;

  Vector2<double> xy_of_p(double p) const override {
    // The result will be computed with the following function:
    //      [x;y] = center + [radius * cos(θ); radius * sin(θ)]
    // and:
    //      θ = θ₀ + (p * Δθ)
    const double theta = theta_of_p(p);
    return center_ + Vector2<double>(radius_ * std::cos(theta),
                                     radius_ * std::sin(theta));
  }

  Vector2<double> xy_dot_of_p(double p) const override {
    // Given:
    //      [x;y] = center + [radius * cos(θ); radius * sin(θ)]
    // and:
    //      θ = θ₀ + (p * Δθ)
    //      dθ/dp = Δθ
    // then:
    //      [dx/dp; dy/dp] = [-radius * sin(θ) * dθ/dp; radius * cos(θ) * dθ/dp]
    //                     = [-radius * sin(θ) * Δθ; radius * cos(θ) * Δθ]
    const double theta = theta_of_p(p);
    return Vector2<double>(-radius_ * std::sin(theta) * d_theta_,
                           radius_ * std::cos(theta) * d_theta_);
  }

  double heading_of_p(double p) const override {
    // Given:
    //     θ = θ_0 + (p * Δθ)
    // then:
    //     heading = θ + sign(Δθ) * π / 2.0
    // sign(Δθ) is used to express the increasing-p direction of the tangent.
    const double theta = theta_of_p(p);
    return theta + std::copysign(M_PI / 2.0, d_theta_);
  }

  double heading_dot_of_p(double p) const override {
    // Given:
    //      heading = θ + sign(Δθ) * π / 2.0
    // and:
    //      θ = θ₀ + (p * Δθ)
    // then:
    //      dheading/dp = dθ/dp = Δθ
    unused(p);
    return d_theta_;
  }

  double l_max() const override { return radius_ * std::abs(d_theta_); }

  Vector3<double> ToCurveFrame(
      const Vector3<double>& geo_coordinate,
      double r_min, double r_max,
      const api::HBounds& height_bounds) const override;

  bool IsValid(double r_min, double r_max,
               const api::HBounds& height_bounds) const override;

 private:
  double FastCalcPFromS(double s, double r) const override;

  double FastCalcSFromP(double p, double r) const override;

  double CalcMinimumRadiusAtOffset(double r) const override {
    return offset_radius(r);
  }

  // Computes the radius of the reference arc offset at a distance @p r.
  // Uses d_theta_'s sign (see ArcRoadCurve() for more details) to add or
  // or subtract @p r distance.
  // @param r Lateral offset of the reference curve over the z=0 plane.
  // @return The reference arc offset radius.
  double offset_radius(double r) const {
    return radius_ - std::copysign(1., d_theta_) * r;
  }

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
  // The aperture angle of the arc. Positive values are counter-clockwise.
  const double d_theta_{};
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
