#include "drake/automotive/maliput/rndf/spline_lane.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/math/saturate.h"

namespace drake {
namespace maliput {
namespace rndf {

// This constant is set to provide the maximum tolerable error to the
// ArcLengthParameterizedSpline. It will use it, to provide a close
// approximation of the path length s coordinate to the spline's parameter.
const double SplineLane::kSplineErrorBound = 1e-6;
// Used to provide a nice visualization as for the general dimensions involved.
const double SplineLane::kTension = 0.8;

SplineLane::SplineLane(
    const api::LaneId& id, const api::Segment* segment,
    const std::vector<std::tuple<ignition::math::Vector3d,
                                 ignition::math::Vector3d>>& control_points,
    double width, int index)
    : Lane(id, segment, width, index) {
  // It first creates a spline based on the positions and its tangents.
  std::unique_ptr<ignition::math::Spline> spline =
      std::make_unique<ignition::math::Spline>();
  spline->Tension(kTension);
  spline->AutoCalculate(true);
  for (const auto& point : control_points) {
    spline->AddPoint(std::get<0>(point), std::get<1>(point));
  }
  // Then we move the spline to the ArcLengthParameterizedSpline to wrap the
  // inverse function from the t parameter of the ignition::math::Spline to the
  // Maliput s coordinate.
  spline_ = std::make_unique<ArcLengthParameterizedSpline>(std::move(spline),
                                                           kSplineErrorBound);
}

api::LanePosition SplineLane::DoToLanePosition(const api::GeoPosition&,
                                               api::GeoPosition*,
                                               double*) const {
  // TODO(@agalbachicar) We need to find a way to implement it.
  DRAKE_ABORT();
}

api::GeoPosition SplineLane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  const double s = drake::math::saturate(lane_pos.s(), 0., do_length());
  const api::RBounds driveable_bounds = do_driveable_bounds(s);
  const double r = drake::math::saturate(lane_pos.r(), driveable_bounds.r_min,
                                         driveable_bounds.r_max);
  // Calculate x,y of (s,0,0).
  const Vector2<double> xy = xy_of_s(s);
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 ypr = Rabg_of_s(s);
  // Rotate (0,r,h) and sum with mapped (s,0,h).
  const Vector3<double> xyz = ypr.apply({0., r, lane_pos.h()}) +
                              Vector3<double>(xy.x(), xy.y(), lane_pos.h());
  return {xyz.x(), xyz.y(), xyz.z()};
}

api::Rotation SplineLane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  const double s = math::saturate(lane_pos.s(), 0., this->do_length());
  // Recover linear parameter p from path length position s.
  const Rot3 Rabg = Rabg_of_s(s);
  return api::Rotation::FromRpy(0.0, 0.0, Rabg.yaw());
}

api::LanePosition SplineLane::DoEvalMotionDerivatives(
    const api::LanePosition&, const api::IsoLaneVelocity& velocity) const {
  return api::LanePosition(velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

Vector2<double> SplineLane::xy_of_s(double s) const {
  // xy_of_s it's called L which is a function
  // R --> R^2. We discard z component right now. We can say
  // L = f(s) = (x(s) ; y(s))
  const ignition::math::Vector3d point =
      spline_->InterpolateMthDerivative(0, s);
  return {point.X(), point.Y()};
}
Vector2<double> SplineLane::xy_dot_of_s(double s) const {
  // We get here the tangent, which is the first derivative of
  // L --> dL(s) / ds
  const ignition::math::Vector3d point =
      spline_->InterpolateMthDerivative(1, s);
  return {point.X(), point.Y()};
}
double SplineLane::heading_of_s(double s) const {
  // The tangent of the heading is the function of y(s) / x(s).
  // So, we can say that h(s) = arctg (y(s) / x(s)). This function
  // is a function like: h(s) = R --> R or h(f(x, y)) where f it's
  // a function defined like y / x. y and x are the components
  // of the first derivative of L. Then, we got: f: R^2 --> R
  const Vector2<double> tangent = xy_dot_of_s(s);
  return std::atan2(tangent.y(), tangent.x());
}

double SplineLane::heading_dot_of_s(double s) const {
  // Based on the explanation of heading_of_s, we got it applying the chain
  // rule:
  // dh / ds = d/ds {arctg (f(x(s), y(s)))}
  //         = 1 / (1 + f(x(s), y(s))^2) * d/ds {f(x(s), y(s))}
  // As x(s) and y(s) and independent polynomials, we can say that:
  // df(x(s), y(s)) / dp = (y' * x - y * x') / x^2
  // Where y and x are the components of the L' and, x' and y' are
  // the components of L'' as they are independent.
  const double heading = heading_of_s(s);
  const ignition::math::Vector3d first_derivative =
      spline_->InterpolateMthDerivative(1, s);
  const ignition::math::Vector3d second_derivative =
      spline_->InterpolateMthDerivative(2, s);
  const double m = (second_derivative.Y() * first_derivative.X() -
                    first_derivative.Y() * second_derivative.X()) /
                   (first_derivative.X() * first_derivative.X());
  return (m / (1.0 + heading * heading));
}

Rot3 SplineLane::Rabg_of_s(double s) const {
  return Rot3(0.0, 0.0, heading_of_s(s));
}

api::RBounds SplineLane::do_lane_bounds(double) const {
  return api::RBounds(-width_ / 2., width_ / 2.);
}

api::RBounds SplineLane::do_driveable_bounds(double s) const {
  if (segment_->num_lanes() == 1) {
    return api::RBounds(-width_ / 2., width_ / 2.);
  }
  // Get the position to the first lane
  const ignition::math::Vector3d position_first_lane = GetPositionToLane(s, 0);
  const ignition::math::Vector3d position_last_lane =
      GetPositionToLane(s, segment_->num_lanes() - 1);
  const double r_min = -std::abs(
      (position_first_lane - spline_->InterpolateMthDerivative(0, s)).Length() +
      segment_->lane(0)->lane_bounds(0.).r_max);
  const double r_max = std::abs(
      (position_last_lane - spline_->InterpolateMthDerivative(0, s)).Length() +
      segment_->lane(segment_->num_lanes() - 1)->lane_bounds(0.).r_max);
  return api::RBounds(r_min, r_max);
}

ignition::math::Vector3d SplineLane::GetPositionToLane(double s,
                                                       int lane_id) const {
  // This the geo position in the current lane for a LanePosition(s, 0., 0.).
  const ignition::math::Vector3d g_l_1 =
      spline_->InterpolateMthDerivative(0, s);
  // After continue working out the expression, if lane_id points to myself,
  // I just need to return my position.
  if (lane_id == index_) {
    return g_l_1;
  }
  // This is the tangent of the current position.
  ignition::math::Vector3d t_1 = spline_->InterpolateMthDerivative(1, s);
  t_1.Normalize();
  // And the normal.
  const ignition::math::Vector3d n_1(-t_1.Y(), t_1.X(), 0.);
  // Here we get the beginning and ending of the other lane, and then compute
  // the respective GeoPositions.
  const api::Lane* other_lane = segment_->lane(lane_id);
  const api::GeoPosition other_lane_beginning =
      other_lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
  const api::GeoPosition other_lane_ending = other_lane->ToGeoPosition(
      api::LanePosition(other_lane->length(), 0., 0.));
  // Beginning of other lane
  const ignition::math::Vector3d g_l_0_a(other_lane_beginning.x(),
                                         other_lane_beginning.y(), 0.);
  // Ending of other lane
  const ignition::math::Vector3d g_l_0_b(other_lane_ending.x(),
                                         other_lane_ending.y(), 0.);
  // As the lane is approximated as a line, we get the normalized tangent as:
  ignition::math::Vector3d t_0 = (g_l_0_b - g_l_0_a);
  t_0.Normalize();
  // We get here the intersection point between n_1 and the other_lane's line
  // approximation
  return g_l_1 +
         n_1 * (t_0.Cross(g_l_1 - g_l_0_a).Length() / t_0.Cross(n_1).Length());
}

double SplineLane::ComputeLength(
    const std::vector<std::tuple<ignition::math::Vector3d,
                                 ignition::math::Vector3d>>& points) {
  DRAKE_THROW_UNLESS(points.size() >= 2);
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  spline.Tension(kTension);
  for (const auto& point : points) {
    spline.AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline.ArcLength();
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
