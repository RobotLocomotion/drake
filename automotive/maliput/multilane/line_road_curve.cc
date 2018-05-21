#include "drake/automotive/maliput/multilane/line_road_curve.h"

#include "drake/common/unused.h"
#include "drake/math/saturate.h"

namespace drake {
namespace maliput {
namespace multilane {

const double LineRoadCurve::kMinimumNorm = 1e-12;

double LineRoadCurve::FastCalcPFromS(double s, double r) const {
  unused(r);
  return elevation().p_s(s / p_scale());
}

double LineRoadCurve::FastCalcSFromP(double p, double r) const {
  unused(r);
  return p_scale() * elevation().s_p(p);
}

Vector3<double> LineRoadCurve::ToCurveFrame(
    const Vector3<double>& geo_coordinate,
    double r_min, double r_max,
    const api::HBounds& height_bounds) const {
  DRAKE_DEMAND(r_min <= r_max);
  // TODO(jadecastro): Lift the zero superelevation and zero elevation gradient
  // restriction.
  const Vector2<double> s_unit_vector = dp_ / dp_.norm();
  const Vector2<double> r_unit_vector{-s_unit_vector(1), s_unit_vector(0)};

  const Vector2<double> q(geo_coordinate.x(), geo_coordinate.y());
  const Vector2<double> lane_origin_to_q = q - p0_;

  // Compute the distance from `q` to the start of the lane.
  const double p_unsaturated = lane_origin_to_q.dot(s_unit_vector) / p_scale();
  const double p = math::saturate(p_unsaturated, 0., 1.);
  const double r_unsaturated = lane_origin_to_q.dot(r_unit_vector);
  const double r = math::saturate(r_unsaturated, r_min, r_max);
  // N.B. h is the geo z-coordinate referenced against the lane elevation (whose
  // `a` coefficient is normalized by lane length).
  const double h_unsaturated = geo_coordinate.z() - elevation().a() * p_scale();
  const double h = math::saturate(h_unsaturated, height_bounds.min(),
                                  height_bounds.max());
  return Vector3<double>(p, r, h);
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
