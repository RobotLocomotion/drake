#include "drake/automotive/maliput/monolane/line_lane.h"

#include <cmath>

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

V2 LineLane::xy_of_p(const double p) const {
  return V2(x0_ + (p * dx_),
            y0_ + (p * dy_));
}


V2 LineLane::xy_dot_of_p(const double p) const {
  return V2(dx_,
            dy_);
}


double LineLane::heading_of_p(const double) const { return heading_; }


double LineLane::heading_dot_of_p(const double) const { return 0.; }


api::LanePosition LineLane::DoToLanePosition(const api::GeoPosition&) const {
  DRAKE_ABORT();  // TODO(maddog@tri.global) Implement me.
}


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
