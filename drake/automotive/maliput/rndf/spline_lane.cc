#include "drake/automotive/maliput/rndf/spline_lane.h"

#include <algorithm>
#include <tuple>
#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace rndf {

const double SplineLane::kSplineErrorBound = 1e-6;

// This tension value is used near 1 to make sure that the spline won't have any
// loops. This is being improved in issues:
// - https://bitbucket.org/ekumen/terminus-simulation/issues/149
// - https://bitbucket.org/ekumen/terminus-simulation/issues/134
// Once this issues are finished, code in builder.cc will improve the tangent
// interpolation so as to get a smoother base line than the current one.
const double SplineLane::kTension = 0.99;

SplineLane::SplineLane(
  const api::LaneId& id, const api::Segment* segment,
  const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>>& control_points,
  const double width, const int index) : Lane(id, segment, width, index) {
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
  api::GeoPosition*, double*) const {
  // TODO(@agalbachicar) We need to find a way to implement it.
  DRAKE_ABORT();
}

api::GeoPosition
SplineLane::DoToGeoPosition(const api::LanePosition& lane_pos) const {
  const double s = math::saturate(lane_pos.s(), 0., this->do_length());
  // Calculate x,y of (s,0,0).
  const V2 xy = xy_of_s(s);
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 ypr = Rabg_of_s(s);
  // Rotate (0,r,h) and sum with mapped (s,0,h).
  const V3 xyz = ypr.apply({0., lane_pos.r(), lane_pos.h()}) +
                 V3(xy.x(), xy.y(), lane_pos.h());
  return {xyz.x(), xyz.y(), xyz.z()};
}

api::Rotation
SplineLane::DoGetOrientation(const api::LanePosition& lane_pos) const {
  const double s = math::saturate(lane_pos.s(), 0., this->do_length());
  // Recover linear parameter p from arc-length position s.
  const Rot3 Rabg = Rabg_of_s(s);
  return api::Rotation::FromRpy(0.0, 0.0, Rabg.yaw());
}

api::LanePosition SplineLane::DoEvalMotionDerivatives(
    const api::LanePosition&,
    const api::IsoLaneVelocity& velocity) const {
  return api::LanePosition(velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

V3 SplineLane::s_hat_of_srh(const double s, const double r, const double h,
  const Rot3 &Rabg) const {
  // We compute the derivative at s, r, h and then get the unit vector.
  const V3 W_prime = W_prime_of_srh(s, r, h, Rabg);
  return W_prime * (1.0 / W_prime.norm());
}
V3 SplineLane::W_prime_of_srh(const double s, const double r, const double h,
  const Rot3& Rabg) const {
  const V2 G_prime = xy_dot_of_s(s);

  const Rot3 &R = Rabg;
  const double alpha = R.roll();
  const double beta = R.pitch();
  const double gamma = R.yaw();

  const double ca = std::cos(alpha);
  const double cb = std::cos(beta);
  const double cg = std::cos(gamma);
  const double sa = std::sin(alpha);
  const double sb = std::sin(beta);
  const double sg = std::sin(gamma);

  // Evaluate dα/dp, dβ/dp, dγ/dp...
  const double d_alpha = 0.0;
  const double d_beta = 0.0;
  const double d_gamma = heading_dot_of_s(s);

  // Recall that W is the lane-to-world transform, defined by
  //   (x,y,z)  = W(p,r,h) = (G(p), Z(p)) + R_αβγ*(0,r,h)
  // where G is the reference curve, Z is the elevation profile, and R_αβγ is
  // a rotation matrix derived from reference curve (heading), elevation,
  // and superelevation.
  //
  // Thus, ∂W/∂p = (∂G(p)/∂p, ∂Z(p)/∂p) + (∂R_αβγ/∂p)*(0,r,h), where
  //
  //   ∂G(p)/∂p = G'(p)
  //
  //   ∂Z(p)/∂p = p_scale * (z / p_scale) = p_scale * g'(p)
  //
  //   ∂R_αβγ/∂p = (∂R_αβγ/∂α ∂R_αβγ/∂β ∂R_αβγ/∂γ)*(dα/dp, dβ/dp, dγ/dp)
  return V3(G_prime.x(), G_prime.y(), 0.0) +

    V3((((sa * sg) + (ca * sb * cg)) * r +
       ((ca * sg) - (sa * sb * cg)) * h),
      (((-sa * cg) + (ca * sb * sg)) * r -
       ((ca * cg) + (sa * sb * sg)) * h),
      ((ca * cb) * r + (-sa * cb) * h)) *
       d_alpha +

    V3(((sa * cb * cg) * r + (ca * cb * cg) * h),
      ((sa * cb * sg) * r + (ca * cb * sg) * h),
      ((-sa * sb) * r - (ca * sb) * h)) *
       d_beta +

    V3((((-ca * cg) - (sa * sb * sg)) * r +
       ((+sa * cg) - (ca * sb * sg)) * h),
      (((-ca * sg) + (sa * sb * cg)) * r +
       ((sa * sg) + (ca * sb * cg)) * h),
      0) *
       d_gamma;
}

V2 SplineLane::xy_of_s(const double s) const {
  // xy_of_s it's called L which is a function
  // R --> R^2. We discard z component right now. We can say
  // L = f(s) = (x(s) ; y(s))
  const ignition::math::Vector3d point =
    spline_->InterpolateMthDerivative(0, s);
  return {point.X(), point.Y()};
}
V2 SplineLane::xy_dot_of_s(const double s) const {
  // We get here the tangent, which is the first derivative of
  // L --> dL(s) / ds
  const ignition::math::Vector3d point =
    spline_->InterpolateMthDerivative(1, s);
  return {point.X(), point.Y()};
}
double SplineLane::heading_of_s(const double s) const {
  // The tangent of the heading is the function of y(s) / x(s).
  // So, we can say that h(s) = arctg (y(s) / x(s)). This function
  // is a function like: h(s) = R --> R or h(f(x, y)) where f it's
  // a function defined like y / x. y and x are the components
  // of the first derivative of L. Then, we got: f: R^2 --> R
  const V2 tangent = xy_dot_of_s(s);
  return std::atan2(tangent.y(), tangent.x());
}

double SplineLane::heading_dot_of_s(const double s) const {
  // Based on the explanation of heading_of_s, we got applying the chain rule:
  // dh / ds = d/ds {arctg (f(x(s), y(s)))}
  //  = 1 / (1 + f(x(s), y(s))^2) * d/ds {f(x(s), y(s))}
  // As x(s) and y(s) and independant polynomials, we can say that:
  // df(x(s), y(s)) / dp = (y' * x - y * x') / x^2
  // Where y and x are the components of the L' and, x' and y' are
  // the components of L'' as they are independant.
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

Rot3 SplineLane::Rabg_of_s(const double s) const {
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
  const double r_min =
    -std::abs(
      (position_first_lane - spline_->InterpolateMthDerivative(0, s)).Length() +
      segment_->lane(0)->lane_bounds(0.).r_max);
  const double r_max = std::abs(
      (position_last_lane - spline_->InterpolateMthDerivative(0, s)).Length() +
      segment_->lane(segment_->num_lanes() - 1)->lane_bounds(0.).r_max);
  return api::RBounds(r_min, r_max);
}

ignition::math::Vector3d
SplineLane::GetPositionToLane(const double s, const int lane_id) const {
  // This the geo position in the current lane for a LanePosition(s, 0., 0.)
  const ignition::math::Vector3d g_l_1 =
    spline_->InterpolateMthDerivative(0, s);
  // After continue working out the expression, if lane_id points to myself,
  // I just need to return my position.
  if (lane_id == index_) {
    return g_l_1;
  }
  // This is the tangent of the current position
  ignition::math::Vector3d t_1 = spline_->InterpolateMthDerivative(1, s);
  t_1.Normalize();
  // and the normal
  const ignition::math::Vector3d n_1(-t_1.Y(), t_1.X(), 0.);
  // Here we get the beginning and ending of the other lane, and then compute
  // the respective GeoPositions.
  const api::Lane *other_lane = segment_->lane(lane_id);
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
  for (const auto &point : points) {
    spline.AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline.ArcLength();
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
