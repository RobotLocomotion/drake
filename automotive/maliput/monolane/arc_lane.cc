#include "drake/automotive/maliput/monolane/arc_lane.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/math/saturate.h"

namespace drake {
namespace maliput {
namespace monolane {

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

ArcLane::ArcLane(const api::LaneId& id, const api::Segment* segment,
                 const V2& center, double radius, const double theta0,
                 const double d_theta, const api::RBounds& lane_bounds,
                 const api::RBounds& driveable_bounds,
                 const api::HBounds& elevation_bounds,
                 const CubicPolynomial& elevation,
                 const CubicPolynomial& superelevation)
    : Lane(id, segment,
           lane_bounds, driveable_bounds, elevation_bounds,
           radius * std::abs(d_theta),
           elevation, superelevation),
      r_(radius),
      cx_(center.x()),
      cy_(center.y()),
      theta0_(theta0),
      d_theta_(d_theta) {
  DRAKE_DEMAND(r_ > 0.);

  // Whether or not user code pays attention to driveable_bounds, at least
  // ensure that bounds are sane.  Given the singularity at the center of
  // the arc, it is not well-defined to consider parallel curves offset
  // from the reference by a distance greater than or equal to the radius r_.
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
  auto update_range_given_p = [&theta_min, &theta_max,
                               &superelevation](double p) {
    // Skip p if outside of domain [0, 1].
    if ((p < 0.) || (p > 1.)) {
      return;
    }
    const double theta_p = superelevation.f_p(p);
    theta_min = std::min(theta_min, theta_p);
    theta_max = std::max(theta_max, theta_p);
  };
  // ...possible extremum at p = 0.
  update_range_given_p(0.);
  // ...possible extremum at p = 1.
  update_range_given_p(1.);
  // ...possible extrema at the local min/max of superelevation (if the local
  // min/max occur within domain [0, 1]).
  if (superelevation.d() != 0) {
    const double p_plus =
        (-superelevation.c() +
         std::sqrt((superelevation.c() * superelevation.c()) -
                   (3. * superelevation.b() * superelevation.d()))) /
        (3. * superelevation.d());
    update_range_given_p(p_plus);
    const double p_minus =
        (-superelevation.c() -
         std::sqrt((superelevation.c() * superelevation.c()) -
                   (3. * superelevation.b() * superelevation.d()))) /
        (3. * superelevation.d());
    update_range_given_p(p_minus);
  } else if (superelevation.c() != 0) {
    const double p_extremum = -superelevation.b() / (2. * superelevation.c());
    update_range_given_p(p_extremum);
  }
  // If theta_min and theta_max flank zero, then min(abs(theta)) is 0....
  const double max_cos_theta =
      std::cos(((theta_min < 0.) && (theta_max > 0.))
                   ? 0.
                   : std::min(std::abs(theta_min), std::abs(theta_max)));
  // TODO(maddog@tri.global)  When you have nothing better to do, handle the
  //                          improbable case of superelevation >= 90 deg, too.
  if (d_theta > 0.) {
    DRAKE_DEMAND((driveable_bounds.max() * max_cos_theta) < r_);
  } else {
    DRAKE_DEMAND((driveable_bounds.min() * max_cos_theta) > -r_);
  }
}

// Evaluate absolute position along reference arc as an angle in
// range [theta0_, (theta0 + d_theta_)],
// as a function of parameter p (in domain [0, 1]).
double ArcLane::theta_of_p(const double p) const {
  return theta0_ + (p * d_theta_);
}

V2 ArcLane::xy_of_p(const double p) const {
  const double theta = theta_of_p(p);
  return V2(cx_ + (r_ * std::cos(theta)), cy_ + (r_ * std::sin(theta)));
}

V2 ArcLane::xy_dot_of_p(const double p) const {
  // Given:
  //      x = c_x + (r * cos(θ))
  // and:
  //      θ = θ_0 + (p * Δθ)
  // then:
  //  dx/dp = -r * sin(θ) * dθ/dp
  //        = -r * sin(θ) * Δθ
  //
  // A similar result holds for y.
  const double theta = theta_of_p(p);
  return V2(-r_ * std::sin(theta) * d_theta_, r_ * std::cos(theta) * d_theta_);
}

double ArcLane::heading_of_p(const double p) const {
  const double theta = theta_of_p(p);
  return theta + std::copysign(M_PI / 2.0, d_theta_);
}

double ArcLane::heading_dot_of_p(const double) const {
  // Given:
  //      H = θ ± (π / 2)
  // and:
  //      θ = θ_0 + (p * Δθ)
  // then:
  //  dH/dp = dθ/dp = Δθ
  return d_theta_;
}

api::LanePosition ArcLane::DoToLanePosition(
    const api::GeoPosition& geo_position, api::GeoPosition* nearest_position,
    double* distance) const {
  // TODO(jadecastro): Lift the zero superelevation and zero elevation gradient
  // restriction.
  const V2 center{cx_, cy_};
  const V2 p{geo_position.x(), geo_position.y()};
  DRAKE_DEMAND(p != center);

  // Define a vector from p to the center of the arc.
  const V2 v = p - center;

  const double theta_min = std::min(theta0_, d_theta_ + theta0_);
  const double theta_max = std::max(theta0_, d_theta_ + theta0_);

  // First, find a saturated theta that is nearest to point p.
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
  const double s = r_ * d_theta_nearest_unwrapped;

  // Compute r (its direction depends on the direction of the +s-coordinate)
  const double r_unsaturated = (d_theta_ >= 0.) ? r_ - v.norm() : v.norm() - r_;
  // Saturate r within drivable bounds.
  const double r = math::saturate(r_unsaturated, driveable_bounds(s).min(),
                                  driveable_bounds(s).max());

  // Calculate the (uniform) road elevation.
  const double p_scale = r_ * d_theta_;
  // N.B. h is the geo z-coordinate referenced against the lane elevation (whose
  // `a` coefficient is normalized by lane length).
  const double h_unsaturated = geo_position.z() - elevation().a() * p_scale;
  const double h = math::saturate(h_unsaturated,
                                  elevation_bounds(s, r).min(),
                                  elevation_bounds(s, r).max());

  const api::LanePosition lane_position{s, r, h};

  const api::GeoPosition nearest = ToGeoPosition(lane_position);
  if (nearest_position != nullptr) {
    *nearest_position = nearest;
  }
  if (distance != nullptr) {
    *distance = (nearest.xyz() - geo_position.xyz()).norm();
  }

  return lane_position;
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
