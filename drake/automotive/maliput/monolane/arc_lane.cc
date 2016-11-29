#include "drake/automotive/maliput/monolane/arc_lane.h"

#include <cmath>

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

V2 ArcLane::xy_of_p_(const double p) const {
  const double theta = theta0_ + (p * d_theta_);
  return V2(cx_ + (r_ * std::cos(theta)),
            cy_ + (r_ * std::sin(theta)));
}


V2 ArcLane::xy_dot_of_p_(const double p) const {
  const double theta = theta0_ + (p * d_theta_);
  return V2(-r_ * d_theta_ * std::sin(theta),
            r_ * d_theta_ * std::cos(theta));
}


double ArcLane::heading_of_p_(const double p) const {
  const double theta = theta0_ + (p * d_theta_);
  return theta + std::copysign(M_PI / 2.0, d_theta_);
}


double ArcLane::heading_dot_of_p_(const double p) const {
  // TODO(maddog)  Does this still hold if r exceeds radius r_, e.g. if r
  //               approaches/crosses the singularity at center of arc?
  return d_theta_;
}


api::LanePosition ArcLane::DoToLanePosition(const api::GeoPosition&) const {
  DRAKE_ABORT();  // TODO(maddog) Implement me.
}



}  // namespace monolane
}  // namespace maliput
}  // namespace drake
