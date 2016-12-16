#include "drake/automotive/maliput/monolane/arc_lane.h"

#include <cmath>

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

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
  // TODO(maddog)  Does this still hold if r exceeds radius r_, e.g. if r
  //               approaches/crosses the singularity at center of arc?
  // Given:
  //      H = θ ± (π / 2)
  // and:
  //      θ = θ_0 + (p * Δθ)
  // then:
  //  dH/dp = dθ/dp = Δθ
  return d_theta_;
}


api::LanePosition ArcLane::DoToLanePosition(const api::GeoPosition&) const {
  DRAKE_ABORT();  // TODO(maddog) Implement me.
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
