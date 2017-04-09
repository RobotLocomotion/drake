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
  // TODO(jadecastro): Lift the zero superelevation and zero elevation gradient
  // restriction.
  const V2 xy0{x0_, y0_};
  const V2 d_xy{dx_, dy_};
  const double length = this->length();
  const V2 s_unit_vector = d_xy / d_xy.norm();
  const V2 r_unit_vector{-s_unit_vector(1), s_unit_vector(0)};

  const V2 p{geo_position.x, geo_position.y};
  const V2 lane_origin_to_p = p - xy0;

  // Compute the distance from `p` to the start of the lane.
  const double s =
      math::saturate(lane_origin_to_p.dot(s_unit_vector), 0., length);
  const double r_unsaturated = lane_origin_to_p.dot(r_unit_vector);
  const double r = math::saturate(r_unsaturated, driveable_bounds(s).r_min,
                                  driveable_bounds(s).r_max);
  const double h = geo_position.z - elevation().a() * length;

  const api::LanePosition lane_position{s, r, h};

  const api::GeoPosition nearest = ToGeoPosition(lane_position);
  if (nearest_position != nullptr) {
    *nearest_position = nearest;
  }
  if (distance != nullptr) {
    const V2 p_to_nearest{p(0) - nearest.x, p(1) - nearest.y};
    *distance = p_to_nearest.norm();
  }

  return lane_position;
}


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
