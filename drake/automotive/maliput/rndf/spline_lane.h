#pragma once

#include <cmath>
#include <memory>
#include <tuple>
#include <vector>
#include <utility>

#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/spline_helpers.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/math/saturate.h"

#include "ignition/math/Spline.hh"


namespace drake {
namespace maliput {
namespace rndf {

/// Specialization of Lane with a spline segment as its reference curve
/// in the xy-plane (the ground plane) of the world frame.
class SplineLane : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SplineLane)

  /// Constructs a SplineLane, a lane specified by a spline segment defined in
  /// the xy-plane (the ground plane) of the world frame.
  ///
  /// For @p id, @p segment, @p width and @p index please see the documentation
  /// of the Lane base class.
  ///
  /// @param control_points are a collection of points and the tangent value
  /// where the interpolated curve will pass. They must be at least two.
  ///
  /// This implementation uses ignition::math::Spline and
  /// ArcLengthParameterizedSpline which is used as the inverse function
  /// that maps the s parameter of s,r,h frame to ignition::math's Spline t
  /// parameter.
  SplineLane(const api::LaneId& id, const api::Segment* segment,
          const std::vector<std::tuple<ignition::math::Vector3d,
            ignition::math::Vector3d>>& control_points,
          const double width,
          const int index);

  ~SplineLane() override = default;

  /// It computes the length of a SplineLane based on the @p points set as
  /// control points. The first value are the points and the second is the
  /// value at that point.
  /// @param control_points are a collection of points and the tangent value
  /// where the interpolated curve will pass. They must be at least two.
  /// @return The length of the baseline computed as a spline.
  static double ComputeLength(
    const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>>& points);

  /// @return the tension of the curve, a bounded constant value between 0 to
  /// 1 which is a measure of the curvature of the interpolated spline. Given a
  /// bigger value of the tension, you'll get an interpolation more similar to a
  /// straight line.
  static double Tension() { return kTension; }

  /// @return the error bound that the arc length interpolator will
  /// attempt to attain when aproximating the inverse function that maps
  /// the s coordinate of s,r,h frame into the t parameter that
  /// ignition::math::Spline uses to evaluate the function.
  static double SplineErrorBound() { return kSplineErrorBound; }

 private:
  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_pos,
      api::GeoPosition* nearest_point,
      double* distance) const override;

  api::GeoPosition DoToGeoPosition(
      const api::LanePosition& lane_pos) const override;

  api::Rotation DoGetOrientation(
      const api::LanePosition& lane_pos) const override;

  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition&,
      const api::IsoLaneVelocity& velocity) const override;

  // It will provide the x,y coordinates based on the
  // ArcLengthParameterizedSpline that will provide the interpolation
  // image at a arc length s from the beginning of the SplineLane.
  V2 xy_of_s(const double s) const;

  // It will provide the x,y tangent values based on the
  // ArcLengthParameterizedSpline that will provide the interpolation
  // image at a arc length s from the beginning of the SplineLane.
  V2 xy_dot_of_s(const double s) const;

  // It will provide the angle of the tangent vector evaluated at an arc lenght
  // s distance from the beginning of the SplineLane.
  double heading_of_s(const double s) const;

  // It will provide the derivative of the angle evaluates at at an arc lenght
  // s distance from the beginning of the SplineLane. Given that x, y and z
  // components of ignition::math::Spline are independant from each other and a
  // a function of s (given the inverse function approximation provided by
  // ArcLengthParameterizedSpline) we can apply the chain rule and
  // the obtain derivative of the heading.
  double heading_dot_of_s(const double s) const;

  // It will provide a rotation vector in terms of Euler angles, with only
  // yaw angle set as RNDF is defined without any change in elevation.
  // See heading_of_s for more information.
  Rot3 Rabg_of_s(const double s) const;

  // Returns the s-axis unit-vector, expressed in the world frame,
  // of the (s,r,h) LANE-space frame (with respect to the world frame).
  //
  // (Rabg must be the result of Rabg_of_s(s) --- passed in here to
  // avoid recomputing it.)
  V3 s_hat_of_srh(const double s, const double r, const double h,
                      const Rot3& Rabg) const;

  // Returns W' = ∂W/∂s, the partial differential of W with respect to s,
  // evaluated at s, r, h.
  //
  // (Rabg must be the result of Rabg_of_s(s) --- passed in here to
  // avoid recomputing it.)
  V3 W_prime_of_srh(const double s, const double r, const double h,
                        const Rot3& Rabg) const;

  // It returns the length of the curve.
  double do_length() const override {
    return spline_->BaseSpline()->ArcLength();
  }

  // It will compute the lane_bounds taking into account the Lane::width. Based
  // on it, it will construct an api::RBounds object whose r_min variable is
  // half the width but negative, and the r_max variable is half the width.
  api::RBounds do_lane_bounds(double) const override;

  // If the segment has only one lane, this one, the return value will be
  // identical to do_lane_bounds(s). However, for the general case, each lane
  // is approximated by a straight line from the beginning to the end of it.
  // Then, we compute the intersection point of the current normal (following
  // the s direction it will be coincident with the r direction) at s with that
  // line approximation. Based on the intersection points with the lanes set as
  // extents of the segment (the first and last ones) we can determine the
  // distance to those limits and create the api::RBounds set as results.
  // Here, GetPositionToLane() is used to compute the line approximation and
  // intersection point.
  api::RBounds do_driveable_bounds(double s) const override;

  // It computes the intersection point of the normal line that intersects the
  // current lane at s coordinate (r = 0, h = 0) with the lane whose index is
  // lane_id. The function of lane_id is approximated to a line defined by the
  // api::GeoPosition values of both the beginning and ending of it.
  // It returns the intersection point on the lane whose index is lane_id.
  ignition::math::Vector3d GetPositionToLane(
  const double s, const int lane_id) const;

  std::unique_ptr<ArcLengthParameterizedSpline> spline_;

  static const double kSplineErrorBound;
  static const double kTension;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
