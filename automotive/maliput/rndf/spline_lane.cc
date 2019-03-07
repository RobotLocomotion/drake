#include "drake/automotive/maliput/rndf/spline_lane.h"

#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/math/saturate.h"

namespace drake {
namespace maliput {
namespace rndf {

// This constant specifies the maximum tolerable error to
// ArcLengthParameterizedSpline. It will be used to provide a close
// approximation of the path length s coordinate to the spline's t parameter.
const double SplineLane::kSplineErrorBound = 1e-6;

SplineLane::SplineLane(
    const api::LaneId& id, const api::Segment* segment,
    const std::vector<std::tuple<ignition::math::Vector3d,
                                 ignition::math::Vector3d>>& control_points,
    double width, int index)
    : Lane(id, segment, index), width_(width) {
  DRAKE_THROW_UNLESS(control_points.size() >= 2);
  // Creates a spline based on the positions and their tangents.
  auto spline = std::make_unique<ignition::math::Spline>();
  spline->AutoCalculate(true);
  for (const auto& point : control_points) {
    spline->AddPoint(std::get<0>(point), std::get<1>(point));
  }
  // Wraps the ignition::math::Spline within an ArcLengthParamterizedSpline for
  // compatibility with Maliput's s parameter.
  spline_ = std::make_unique<ArcLengthParameterizedSpline>(std::move(spline),
                                                           kSplineErrorBound);
}

api::LanePosition SplineLane::DoToLanePosition(const api::GeoPosition&,
                                               api::GeoPosition*,
                                               double*) const {
  // TODO(@agalbachicar) We need to find a way to implement it.
  throw std::runtime_error("SplineLane::DoToLanePosition is not implemented");
}

api::GeoPosition SplineLane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  const double s = drake::math::saturate(lane_pos.s(), 0., do_length());
  const api::RBounds driveable_bounds = do_driveable_bounds(s);
  const double r = drake::math::saturate(lane_pos.r(), driveable_bounds.min(),
                                         driveable_bounds.max());
  // Calculate x,y of (s,0,0).
  const Vector2<double> xy = xy_of_s(s);
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const math::RotationMatrix<double> rotation_matrix = Rabg_of_s(s);
  // Rotate (0,r,h) and sum with mapped (s,0,h).
  const Vector3<double> xyz =
      rotation_matrix * Vector3<double>(0., r, lane_pos.h()) +
      Vector3<double>(xy.x(), xy.y(), lane_pos.h());
  return {xyz.x(), xyz.y(), xyz.z()};
}

api::Rotation SplineLane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  const double s = math::saturate(lane_pos.s(), 0., this->do_length());
  // Gets the rotation matrix at path length position s. As RNDF does not
  // provide information of height, only yaw angle is set.
  return api::Rotation::FromRpy(0.0, 0.0, heading_of_s(s));
}

api::LanePosition SplineLane::DoEvalMotionDerivatives(
    const api::LanePosition&, const api::IsoLaneVelocity& velocity) const {
  return api::LanePosition(velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

Vector2<double> SplineLane::xy_of_s(double s) const {
  // Let xy_of_s be called L, which is a function
  // R --> R^2. We discard the z component right now. We can say
  // L = f(s) = (x(s) ; y(s))
  const ignition::math::Vector3d point =
      spline_->InterpolateMthDerivative(0, s);
  return {point.X(), point.Y()};
}

Vector2<double> SplineLane::xy_dot_of_s(double s) const {
  // Computes the tangent, which is the first derivative of
  // L --> dL(s) / ds
  const ignition::math::Vector3d point =
      spline_->InterpolateMthDerivative(1, s);
  return {point.X(), point.Y()};
}

double SplineLane::heading_of_s(double s) const {
  // The tangent of the heading is the function of y(s) / x(s).
  // So, we can say that h(s) = arctg (y(s) / x(s)). This function
  // takes the form: h(s) = R --> R or h(f(x, y)) where f is
  // a function defined by y / x. y and x are the components
  // of the first derivative of L. Thus, we get: f: R^2 --> R
  const Vector2<double> tangent = xy_dot_of_s(s);
  return std::atan2(tangent.y(), tangent.x());
}

double SplineLane::heading_dot_of_s(double s) const {
  // Based on the explanation of heading_of_s, we get its derivative by
  // applying the chain rule:
  // dh / ds = d/ds {arctg (f(x(s), y(s)))}
  //         = 1 / (1 + f(x(s), y(s))^2) * d/ds {f(x(s), y(s))}
  // Since x(s) and y(s) are independent polynomials, we can say that:
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

math::RotationMatrix<double> SplineLane::Rabg_of_s(double s) const {
  const double angle = heading_of_s(s);
  return math::RotationMatrix<double>::MakeZRotation(angle);
}

api::RBounds SplineLane::do_lane_bounds(double) const {
  return api::RBounds(-width_ / 2., width_ / 2.);
}

api::RBounds SplineLane::do_driveable_bounds(double s) const {
  if (segment()->num_lanes() == 1) {
    return api::RBounds(-width_ / 2., width_ / 2.);
  }
  // Get the position to the first lane.
  const ignition::math::Vector3d position_first_lane = GetPositionToLane(s, 0);
  const ignition::math::Vector3d position_last_lane =
      GetPositionToLane(s, segment()->num_lanes() - 1);
  const double r_min = -std::abs(
      (position_first_lane - spline_->InterpolateMthDerivative(0, s)).Length() +
      segment()->lane(0)->lane_bounds(0.).max());
  const double r_max = std::abs(
      (position_last_lane - spline_->InterpolateMthDerivative(0, s)).Length() +
      segment()->lane(segment()->num_lanes() - 1)->lane_bounds(0.).max());
  return api::RBounds(r_min, r_max);
}

ignition::math::Vector3d SplineLane::GetPositionToLane(double s,
                                                       int lane_id) const {
  // Computes the geo position in the current lane at LanePosition(s, 0., 0.).
  const ignition::math::Vector3d p = spline_->InterpolateMthDerivative(0, s);
  // If lane_id points to myself I just need to return my position.
  if (lane_id == index()) {
    return p;
  }
  // This is the tangent of the current position.
  ignition::math::Vector3d t_p = spline_->InterpolateMthDerivative(1, s);
  t_p.Normalize();
  // Computes the normal with a right-handed frame. The normal is just a 90°
  // rotation over the z-axis.
  const ignition::math::Vector3d r(-t_p.Y(), t_p.X(), 0.);
  // Gets the beginning and ending of the other lane, and then computes the
  // respective GeoPositions.
  const SplineLane* other_lane = dynamic_cast<const SplineLane*>(
      segment()->lane(lane_id));
  DRAKE_DEMAND(other_lane != nullptr);
  const Vector2<double> other_lane_beginning = other_lane->xy_of_s(0.);
  const Vector2<double> other_lane_ending =
      other_lane->xy_of_s(other_lane->do_length());
  // Converts the beginning and ending positions of the other lane into
  // ignition::math::Vector3d objects.
  const ignition::math::Vector3d q(other_lane_beginning.x(),
                                   other_lane_beginning.y(),
                                   0.);
  const ignition::math::Vector3d q_ending(other_lane_ending.x(),
                                          other_lane_ending.y(),
                                          0.);
  // As the lane is approximated as a line, we get the normalized tangent as:
  ignition::math::Vector3d t_q = (q_ending - q);
  t_q.Normalize();
  // Checks if the lines are collinear.
  const ignition::math::Vector3d t_q_cross_r = t_q.Cross(r);
  const ignition::math::Vector3d p_to_q_cross_t_q = (p - q).Cross(t_q);
  const double kAlmostZero = 1e-6;
  // Lines are parallel or non-intersecting. We cannot handle correctly that
  // case for the purpose of this function.
  DRAKE_DEMAND(t_q_cross_r.Length() > kAlmostZero);
  // Computes the intersection point between r and the other_lane's line
  // approximation.
  return p - r * (p_to_q_cross_t_q.Length() / t_q_cross_r.Length());
}

double SplineLane::ComputeLength(
    const std::vector<std::tuple<ignition::math::Vector3d,
                                 ignition::math::Vector3d>>& control_points) {
  DRAKE_THROW_UNLESS(control_points.size() >= 2);
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  for (const auto& control_point : control_points) {
    spline.AddPoint(std::get<0>(control_point), std::get<1>(control_point));
  }
  return spline.ArcLength();
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
