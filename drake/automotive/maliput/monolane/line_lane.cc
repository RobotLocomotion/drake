#include "drake/automotive/maliput/monolane/line_lane.h"

#include <cmath>

#include "drake/common/drake_assert.h"
#include "drake/math/saturate.h"

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


api::LanePosition LineLane::DoToLanePosition(
    const api::GeoPosition& geo_position, api::GeoPosition* nearest_position,
    double* distance) const {
  const V2 xy0{x0_, y0_};
  const V2 d_xy{dx_, dy_};
  const double length = this->length();
  const V2 unit_vector = d_xy / d_xy.norm();

  const V2 p{geo_position.x, geo_position.y};

  // Compute the distance from `p` to the start of the lane.
  const double s = math::saturate(-(xy0 - p).dot(unit_vector), 0., length);

  // The distance is the length of the vector sum between the s-coordinate in
  // geo coordinates and the vector connecting `p` and the start of the lane.
  *distance = ((xy0 - p) + s * unit_vector).norm();
  const V2 p_nearest = xy_of_p(s / length);
  (*nearest_position).x = p_nearest(0);
  (*nearest_position).y = p_nearest(1);
  (*nearest_position).z = 0.;

  return api::LanePosition(s, 0., 0.);
}


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
