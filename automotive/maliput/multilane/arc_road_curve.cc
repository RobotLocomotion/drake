#include "drake/automotive/maliput/multilane/arc_road_curve.h"

#include <algorithm>
#include <limits>

#include "drake/common/unused.h"
#include "drake/math/saturate.h"

namespace drake {
namespace maliput {
namespace multilane {

double ArcRoadCurve::FastCalcPFromS(double s, double r) const {
  const double effective_radius = offset_radius(r);
  const double elevation_domain = effective_radius / radius_;
  return s / (p_scale() * std::sqrt(elevation_domain * elevation_domain +
                                    elevation().fake_gprime(1.) *
                                    elevation().fake_gprime(1.)));
}

double ArcRoadCurve::FastCalcSFromP(double p, double r) const {
  const double effective_radius = offset_radius(r);
  const double elevation_domain = effective_radius / radius_;
  return p * p_scale() * std::sqrt(elevation_domain * elevation_domain +
                                   elevation().fake_gprime(p) *
                                   elevation().fake_gprime(p));
}

namespace {

// Wraps the input angle θ, casting it onto the range [-π, π].
double wrap(double theta) {
  double theta_new = std::fmod(theta + M_PI, 2. * M_PI);
  if (theta_new < 0.) theta_new += 2. * M_PI;
  return theta_new - M_PI;
}

// Implements the saturate function with respect to a range of angles.
// Specifically, given an angle θ ∈ [-π, π] along with lower and upper bounds
// θ_min, θ_max, such that -∞ < θ_min <= θ_max < +∞, this function returns the
// saturated angle sat(θ, θ_min, θ_max) within the range [-π, π].  If θ_max > 2π
// + θ_min, then no saturation occurs.
//
// Note that the interval [θ_min, θ_max] should be mapped into the cyclic
// range [-π, +π] --- and in doing so, it might remain a single closed
// interval [a, b], or it might split into a pair of intervals [-π, a] and [b,
// +π]. In both cases, -π <= a <= b <= +π. If the result is a single
// interval, saturating is the usual. If the result is two intervals, saturating
// involves the extra step picking the 'closest' interval to saturate
// within.
double saturate_on_wrapped_bounds(double theta, double theta_min,
                                  double theta_max) {
  DRAKE_DEMAND(-M_PI <= theta);
  DRAKE_DEMAND(theta <= M_PI);
  DRAKE_DEMAND(theta_min <= theta_max);

  if (theta_max >= theta_min + 2. * M_PI) return theta;

  // Wrap on the range [-π, π]. This is not order-preserving.
  const double theta_0 = wrap(theta_min);
  const double theta_1 = wrap(theta_max);

  // Criteria for returning the unsaturated result.
  // First, deal with the case where [θ_min, θ_max] wrapped does not cross the
  // ±π wrap-point (i.e. θ₀ ≤ θ₁).
  if (theta_0 <= theta && theta <= theta_1) return theta;
  // Next, deal with the case where [θ_min, θ_max] wrapped straddles ±π (i.e. θ₁
  // < θ₀).
  if (theta <= theta_1 && theta_1 < theta_0) return theta;
  if (theta_1 < theta_0 && theta_0 <= theta) return theta;

  // Saturate at the appropriate bound.
  const double delta_0 = std::min(std::abs(theta - theta_0),
                                  std::abs(theta - 2. * M_PI - theta_0));
  const double delta_1 = std::min(std::abs(theta - theta_1),
                                  std::abs(theta + 2. * M_PI - theta_1));
  return (delta_0 <= delta_1) ? theta_0 : theta_1;
}

}  // namespace

Vector3<double> ArcRoadCurve::ToCurveFrame(
    const Vector3<double>& geo_coordinate,
    double r_min, double r_max,
    const api::HBounds& height_bounds) const {
  DRAKE_DEMAND(r_min <= r_max);
  // TODO(jadecastro): Lift the zero superelevation and zero elevation gradient
  // restriction.
  const Vector2<double> q(geo_coordinate.x(), geo_coordinate.y());
  DRAKE_DEMAND(q != center_);

  // Define a vector from q to the center of the arc.
  const Vector2<double> v = q - center_;

  const double theta_min = std::min(theta0_, d_theta_ + theta0_);
  const double theta_max = std::max(theta0_, d_theta_ + theta0_);

  // First, find a saturated theta that is nearest to point q.
  const double theta_nearest =
      saturate_on_wrapped_bounds(std::atan2(v(1), v(0)), theta_min, theta_max);
  // Find the angle swept from the beginning of the lane (s = 0) to
  // theta_nearest.
  const double d_theta_nearest = (d_theta_ > 0.)
      ? theta_nearest - wrap(theta_min) : wrap(theta_max) - theta_nearest;
  // Then, unwrap this angle (if necessary) to deal with possible crossings with
  // the ±π wrap-around point.
  const double d_theta_nearest_unwrapped =
      (d_theta_nearest < 0.) ? d_theta_nearest + 2. * M_PI : d_theta_nearest;
  // Convert this angular displacement to arc length (s).
  const double p = d_theta_nearest_unwrapped / std::abs(d_theta_);
  // Compute r (its direction depends on the direction of the +s-coordinate)
  const double r_unsaturated = (d_theta_ >= 0.) ?
                               radius_ - v.norm() : v.norm() - radius_;
  // Saturate r within drivable bounds.
  const double r = math::saturate(r_unsaturated, r_min, r_max);

  // Calculate the (uniform) road elevation.
  // N.B. h is the geo z-coordinate referenced against the lane elevation (whose
  // `a` coefficient is normalized by lane length).
  const double h_unsaturated = geo_coordinate.z() - elevation().a() * p_scale();
  const double h = math::saturate(h_unsaturated, height_bounds.min(),
                                  height_bounds.max());
  return Vector3<double>(p, r, h);
}

bool ArcRoadCurve::IsValid(double r_min, double r_max,
                           const api::HBounds& height_bounds) const {
  // TODO(@agalbachicar)      There is no check on height constraints. When the
  //                          curve bends over itself, if it does, it must do it
  //                          with a difference in height greater than
  //                          (height_bounds.max() - height_bounds.min()).
  // TODO(maddog@tri.global)  Include height_bounds in checking against the
  //                          singularity at the arc center.
  // TODO(maddog@tri.global)  Check against singularities in curvature of
  //                          elevation.
  // TODO(maddog@tri.global)  Check for self-intersecting volumes (e.g., when
  //                          arc angle >= 2π).
  unused(height_bounds);
  DRAKE_DEMAND(r_min <= r_max);
  // Whether or not user code pays attention to driveable_bounds, at least
  // ensure that bounds are sane.  Given the singularity at the center of
  // the arc, it is not well-defined to consider parallel curves offset
  // from the reference by a distance greater than or equal to the
  // effective_radius (it takes into account r displacement with respect to the
  // reference radius radius_).
  //
  // In the presence of superelevation, the bounds are effectively scaled
  // by cos(superelevation) (the more tilted the road, the narrower the
  // road looks when projected onto the groundplane).  Thus, the worst case
  // (widest effective bounds) happens at the position p along the arc
  // with maximum cos(superelevation).
  //
  // In other words, the worst case happens at p in domain [0, 1] with the
  // minimum abs(superelevation).  There are up to four possible candidates
  // at which an extremum can occur, since superelevation is cubic.
  double theta_min = std::numeric_limits<double>::max();
  double theta_max = std::numeric_limits<double>::min();
  const CubicPolynomial& f_superelevation = superelevation();
  auto update_range_given_p = [&theta_min, &theta_max,
                               &f_superelevation](double p) {
    // Skip p if outside of domain [0, 1].
    if ((p < 0.) || (p > 1.)) {
      return;
    }
    const double theta_p = f_superelevation.f_p(p);
    theta_min = std::min(theta_min, theta_p);
    theta_max = std::max(theta_max, theta_p);
  };
  // ...possible extremum at p = 0.
  update_range_given_p(0.);
  // ...possible extremum at p = 1.
  update_range_given_p(1.);
  // ...possible extrema at the local min/max of superelevation (if the local
  // min/max occur within domain [0, 1]).
  if (f_superelevation.d() != 0) {
    const double p_plus =
        (-f_superelevation.c() +
         std::sqrt((f_superelevation.c() * f_superelevation.c()) -
                   (3. * f_superelevation.b() * f_superelevation.d()))) /
        (3. * f_superelevation.d());
    update_range_given_p(p_plus);
    const double p_minus =
        (-f_superelevation.c() -
         std::sqrt((f_superelevation.c() * f_superelevation.c()) -
                   (3. * f_superelevation.b() * f_superelevation.d()))) /
        (3. * f_superelevation.d());
    update_range_given_p(p_minus);
  } else if (f_superelevation.c() != 0) {
    const double p_extremum = -f_superelevation.b() /
                              (2. * f_superelevation.c());
    update_range_given_p(p_extremum);
  }
  // If theta_min and theta_max flank zero, then min(abs(theta)) is 0....
  const double max_cos_theta =
      std::cos(((theta_min < 0.) && (theta_max > 0.))
                   ? 0.
                   : std::min(std::abs(theta_min), std::abs(theta_max)));
  // TODO(maddog@tri.global)  When you have nothing better to do, handle the
  //                          improbable case of superelevation >= 90 deg, too.
  if (d_theta_ > 0.) {
    return (offset_radius(r_max * max_cos_theta) > 0.);
  } else {
    return (offset_radius(r_min * max_cos_theta) > 0.);
  }
  return true;
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
