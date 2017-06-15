#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/spline_helpers.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"

namespace drake {
namespace maliput {
namespace rndf {

/// An R^3 rotation parameterized by roll, pitch, yaw.
///
/// This effects a compound rotation around space-fixed x-y-z axes:
///
///   Rot3(yaw,pitch,roll) * V = RotZ(yaw) * RotY(pitch) * RotX(roll) * V
class Rot3 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rot3)

  Rot3(double roll, double pitch, double yaw) : rpy_(roll, pitch, yaw) {}

  /// Applies the rotation to a 3-vector.
  Vector3<double> apply(const Vector3<double>& in) const {
    return math::rpy2rotmat(rpy_) * in;
  }

  double yaw() const { return rpy_(2); }
  double pitch() const { return rpy_(1); }
  double roll() const { return rpy_(0); }

 private:
  Eigen::Matrix<double, 3, 1, Eigen::DontAlign> rpy_;
};

/// Specialization of drake::maliput::rndf::Lane with a spline curve as its
/// reference path. RNDF specification lacks elevation and superelevation so
/// all the Lanes will depict a flat path just over the ground.
class SplineLane : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SplineLane)

  /// Constructs a SplineLane, a lane specified by a spline segment defined in
  /// the xy-plane (the ground plane) of the world frame.
  ///
  /// @param id The ID of the api::Lane.
  /// @param segment A pointer that refers to its parent, which must remain
  /// valid for the lifetime of this class.
  /// @param width The value of the width of the lane based on the RNDF
  /// lane_width parameter. It will be used to set a constant lane_bound
  /// value along the road.
  /// @param index The index that can be used to reference this Lane from
  /// api::Segment::lane() call.
  /// @param control_points A collection of knots and their tangents where
  /// the interpolated curve will pass. They must be at least two. The tuple
  /// will be interpreted as with the first value as the knot and the second as
  /// the tangent.
  ///
  /// This implementation uses ignition::math::Spline and
  /// ArcLengthParameterizedSpline which is used as the inverse function
  /// that maps the s parameter of s,r,h frame to ignition::math's Spline t
  /// parameter.
  ///
  /// @throws when the size of @p control_points is less than 2.
  SplineLane(
      const api::LaneId& id, const api::Segment* segment,
      const std::vector<std::tuple<ignition::math::Vector3d,
                                   ignition::math::Vector3d>>& control_points,
      double width, int index);

  ~SplineLane() override = default;

  /// Computes the length of a SplineLane based on the @p points set as
  /// control points. The first value are the points and the second is the
  /// value at that point.
  /// @param control_points A collection of knots and their tangents where
  /// the interpolated curve will pass. They must be at least two. The tuple
  /// will be interpreted as with the first value as the knot and the second as
  /// the tangent.
  /// @return The length of the baseline computed as a spline.
  /// @throws when the size of @p control_points is less than 2.
  static double ComputeLength(
      const std::vector<std::tuple<ignition::math::Vector3d,
                                   ignition::math::Vector3d>>& control_points);

  /// @return the tension of the curve, a bounded constant value between 0 to
  /// 1 which is a measure of the desired curvature of the interpolated spline.
  /// Given a bigger value of the tension, you'll get an interpolation more
  /// similar to a straight line.
  static double Tension() { return kTension; }

  /// @return the error bound that the path length interpolator will
  /// attempt to attain when approximating the inverse function that maps
  /// the s coordinate of api::LanePosition frame into the t parameter that
  /// ignition::math::Spline uses to evaluate the function.
  static double SplineErrorBound() { return kSplineErrorBound; }

 private:
  // This function is not implemented and will abort if called.
  // TODO(@agalbachicar) We need to find a way to implement it.
  api::LanePosition DoToLanePosition(const api::GeoPosition& geo_pos,
                                     api::GeoPosition* nearest_point,
                                     double* distance) const override;

  // Provides the api::GeoPosition at the @p lane_pos over the lane.
  // s coordinate of @p lane_pos will be saturated to be processed in the range
  // of 0.0 to the lane length. The same is applied to r coordinate which is
  // saturated to the driveable_bounds at s coordinate (after the saturation).
  api::GeoPosition DoToGeoPosition(
      const api::LanePosition& lane_pos) const override;

  // Provides the api::Rotation matrix against the origin of the world frame at
  // @p lane_pos coordinate over the lane. The s coordinate of @p lane_pose will
  // be saturated to be in the range of 0.0 to the lane length.
  api::Rotation DoGetOrientation(
      const api::LanePosition& lane_pos) const override;

  // As we are moving along the lane with path length coordinates we will
  // return the velocity as is and will not scale through the chain rule.
  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const override;

  // Provide the x,y coordinates based on the
  // ArcLengthParameterizedSpline that will provide the interpolation
  // image at a path length s from the beginning of the SplineLane.
  Vector2<double> xy_of_s(double s) const;

  // Provide the x,y tangent values based on the ArcLengthParameterizedSpline
  // that will provide the interpolation image at a path length s from the
  // beginning of the SplineLane.
  Vector2<double> xy_dot_of_s(double s) const;

  // Provide the angle of the tangent vector evaluated at a path length
  // s distance from the beginning of the SplineLane.
  double heading_of_s(double s) const;

  // Provides the derivative of the angle at a path length s distance from the
  // beginning of the SplineLane. Given that x, y and z components of
  // ignition::math::Spline are independent from each other and a function of s
  // (given the inverse function approximation provided by
  // ArcLengthParameterizedSpline) we can apply the chain rule and
  // the obtain derivative of the heading.
  double heading_dot_of_s(double s) const;

  // Provide a rotation vector in terms of Euler angles, with only
  // yaw angle set as RNDF is defined without any change in elevation.
  // See heading_of_s for more information.
  Rot3 Rabg_of_s(double s) const;

  // Returns the length of the curve.
  double do_length() const override {
    return spline_->BaseSpline()->ArcLength();
  }

  // TODO(@agalbachicar) Not implemented yet.
  api::HBounds do_elevation_bounds(double, double) const override {
    return api::HBounds(0., 0.);
  }

  // Computes the lane_bounds taking into account the Lane::width. Based
  // on it, it constructs an api::RBounds object whose r_min variable is
  // half the width but negative, and the r_max variable is half the width.
  api::RBounds do_lane_bounds(double s) const override;

  // If the segment has only one lane, this one, the return value will be
  // identical to do_lane_bounds(s). However, for the general case, each lane
  // is approximated by a straight line from the beginning to the end of it.
  // Then, we compute the intersection point of the current normal (following
  // the s direction it will be coincident with the r direction) at s with that
  // line approximation. Based on the intersection points with the lanes set as
  // extents of the segment (the first and last ones) we can determine the
  // distance to those limits and create the api::RBounds set as result.
  // Here, GetPositionToLane() is used to compute the line approximation and
  // intersection point.
  api::RBounds do_driveable_bounds(double s) const override;

  // Computes the intersection point of the normal line that intersects the
  // current lane at s coordinate (r = 0, h = 0) with the lane whose index is
  // lane_id. The function of lane_id is approximated to a line defined by the
  // api::GeoPosition values of both the beginning and ending of it.
  // It returns the intersection point on the lane whose index is lane_id.
  ignition::math::Vector3d GetPositionToLane(double s, int lane_id) const;

  std::unique_ptr<ArcLengthParameterizedSpline> spline_;

  static const double kSplineErrorBound;
  static const double kTension;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
