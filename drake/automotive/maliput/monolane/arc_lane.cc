#include "drake/automotive/maliput/monolane/arc_lane.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

ArcLane::ArcLane(const api::LaneId& id, const Segment* segment,
                 const V2& center, double radius,
                 const double theta0, const double d_theta,
                 const api::RBounds& lane_bounds,
                 const api::RBounds& driveable_bounds,
                 const CubicPolynomial& elevation,
                 const CubicPolynomial& superelevation)
    : Lane(id, segment,
           lane_bounds, driveable_bounds,
           radius * std::abs(d_theta),
           elevation, superelevation),
      r_(radius), cx_(center.x()), cy_(center.y()),
      theta0_(theta0), d_theta_(d_theta) {
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
  auto update_range_given_p =
      [&theta_min, &theta_max, &superelevation](double p) {
    // Skip p if outside of domain [0, 1].
    if ((p < 0.) || (p > 1.)) { return; }
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
         std::sqrt((superelevation.c() * superelevation.c())
                   - (3. * superelevation.b() * superelevation.d())))
        / (3. * superelevation.d());
    update_range_given_p(p_plus);
    const double p_minus =
        (-superelevation.c() -
         std::sqrt((superelevation.c() * superelevation.c())
                   - (3. * superelevation.b() * superelevation.d())))
        / (3. * superelevation.d());
    update_range_given_p(p_minus);
  } else if (superelevation.c() != 0) {
    const double p_extremum =
        -superelevation.b() / (2. * superelevation.c());
    update_range_given_p(p_extremum);
  }
  // If theta_min and theta_max flank zero, then min(abs(theta)) is 0....
  const double max_cos_theta =
      std::cos(((theta_min < 0.) && (theta_max > 0.)) ? 0.
               : std::min(std::abs(theta_min), std::abs(theta_max)));
  // TODO(maddog@tri.global)  When you have nothing better to do, handle the
  //                          improbable case of superelevation >= 90 deg, too.
  if (d_theta > 0.) {
    DRAKE_DEMAND((driveable_bounds.r_max * max_cos_theta) < r_);
  } else {
    DRAKE_DEMAND((driveable_bounds.r_min * max_cos_theta) > -r_);
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
  return V2(cx_ + (r_ * std::cos(theta)),
            cy_ + (r_ * std::sin(theta)));
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
  return V2(-r_ * std::sin(theta) * d_theta_,
            r_ * std::cos(theta) * d_theta_);
}


double ArcLane::heading_of_p(const double p) const {
  const double theta = theta_of_p(p);
  return theta + std::copysign(M_PI / 2.0, d_theta_);
}


double ArcLane::heading_dot_of_p(const double p) const {
  // Given:
  //      H = θ ± (π / 2)
  // and:
  //      θ = θ_0 + (p * Δθ)
  // then:
  //  dH/dp = dθ/dp = Δθ
  return d_theta_;
}


api::LanePosition ArcLane::DoToLanePosition(const api::GeoPosition&) const {
  DRAKE_ABORT();  // TODO(maddog@tri.global) Implement me.
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
