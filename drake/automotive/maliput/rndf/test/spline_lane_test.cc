#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"
#include "drake/automotive/maliput/rndf/spline_helpers.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"
#include "drake/automotive/maliput/rndf/test/rndf_test_utils.h"

namespace drake {
namespace maliput {
namespace rndf {

// Angular tolerance will be used to match the angle values from the orientation
// matrices.
const double kAngularTolerance = 1e-6;
// For path length computations we expect to have this error or less as the
// SplineLane class provides this accuracy.
const double kPathTolerance = 1e-6;
// Use this tolerance when the API call should answer the same exact result as
// the expected value without adding any new operation in the middle.
const double kVeryExact = 1e-12;

// This is a wrapper to easily create an ignition::math::Spline.
// @param points a tuple consisting of two ignition::math::Vector3d. The
// first one is the position of the point and the second one is the tangent
// vector or the spline first derivative at that point.
// @return a pointer to an ignition::math::Spline object.
std::unique_ptr<ignition::math::Spline> CreateSpline(
    const std::vector<std::tuple<ignition::math::Vector3d,
                                 ignition::math::Vector3d>>& points) {
  auto spline = std::make_unique<ignition::math::Spline>();
  spline->Tension(1.0);
  spline->AutoCalculate(true);
  for (const auto& point : points) {
    spline->AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline;
}

// Makes geometric tests over a straight line lane.
GTEST_TEST(RNDFSplineLanesTest, FlatLineLane) {
  const double width = 5.;
  RoadGeometry rg({"FlatLineLane"}, kPathTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(20.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  Lane* l1 = s1->NewSplineLane({"l1"}, control_points, width);
  // Check road geometry invariants
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());
  // Check lane length
  EXPECT_NEAR(l1->length(), std::sqrt((20. * 20.) + (0. * 0.)), kPathTolerance);
  // Check bounds
  EXPECT_NEAR(l1->lane_bounds(0.).r_min, -width / 2., kVeryExact);
  EXPECT_NEAR(l1->lane_bounds(0.).r_max, width / 2., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_min, -width / 2., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_max, width / 2., kVeryExact);

  // ToGeoPosition.
  // Reference line.
  // At the beginning, end and middle.
  EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(0., 0., 0.)),
                  api::GeoPosition(0., 0., 0.), kPathTolerance);
  EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(20., 0., 0.)),
                  api::GeoPosition(20., 0., 0.), kPathTolerance);
  EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(10., 0., 0.)),
                  api::GeoPosition(10., 0., 0.), kPathTolerance);

  // A couple of meters away of the reference baseline.
  EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(5., 2., 0.)),
                  api::GeoPosition(5., 2., 0.), kPathTolerance);
  EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(12., -2., 0.)),
                  api::GeoPosition(12., -2., 0.), kPathTolerance);
  EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(18., 1.234567, 0.)),
                  api::GeoPosition(18., 1.234567, 0.), kPathTolerance);

  // Orientation.
  EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(0., 0., 0.)),
                  api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);
  EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(20., 0., 0.)),
                  api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);
  EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(10., 2., 0.)),
                  api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);
  EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(10., -2., 0.)),
                  api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);

  // EvalMotionDerivatives.
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                   api::LanePosition(0., 0., 0.), kPathTolerance);
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                   api::LanePosition(1., 0., 0.), kPathTolerance);
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                   api::LanePosition(0., 1., 0.), kPathTolerance);
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                   api::LanePosition(0., 0., 1.), kPathTolerance);
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({10., 2., 5.}, {0., 0., 1.}),
                   api::LanePosition(0., 0., 1.), kPathTolerance);
}

// Makes geometric tests over a curved spline.
GTEST_TEST(RNDFSplineLanesTest, CurvedLineLane) {
  const double width = 5.;
  RoadGeometry rg({"CurvedLineLane"}, kPathTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(20.0, 20.0, 0.0),
                      ignition::math::Vector3d(0, 10.0, 0.0)));
  Lane* l1 = s1->NewSplineLane({"l1"}, control_points, width);
  // Check road geometry invariants.
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());
  // Create a spline to check the internal values.
  std::unique_ptr<ignition::math::Spline> spline = CreateSpline(control_points);
  auto arc_length_interpolator = std::make_unique<ArcLengthParameterizedSpline>(
      std::move(spline), SplineLane::SplineErrorBound());
  // Check length.
  const double s_total = arc_length_interpolator->BaseSpline()->ArcLength();
  EXPECT_NEAR(l1->length(), s_total, kPathTolerance);
  // Check bounds.
  EXPECT_NEAR(l1->lane_bounds(0.).r_min, -width / 2., kVeryExact);
  EXPECT_NEAR(l1->lane_bounds(0.).r_max, width / 2., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_min, -width / 2., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_max, width / 2., kVeryExact);

  // ToGeoPosition.
  // Reference line.
  // At the beginning, end and middle.
  {
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(0., 0., 0.)),
                    api::GeoPosition(0., 0., 0.), kPathTolerance);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(s_total, 0., 0.)),
                    api::GeoPosition(20., 20.0, 0.), kPathTolerance);
    const ignition::math::Vector3d point =
        arc_length_interpolator->InterpolateMthDerivative(0, s_total / 2.);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(s_total / 2., 0., 0.)),
                    api::GeoPosition(point.X(), point.Y(), 0.), kPathTolerance);
  }

  // On the plane but at any point over the road.
  {
    const ignition::math::Vector3d point =
        arc_length_interpolator->InterpolateMthDerivative(0, s_total / 2.);
    const ignition::math::Vector3d tangent =
        arc_length_interpolator->InterpolateMthDerivative(1, s_total / 2.);
    const double yaw = std::atan2(tangent.Y(), tangent.X());
    const double x_offset = -2. * std::sin(yaw);
    const double y_offset = 2. * std::cos(yaw);
    EXPECT_GEO_NEAR(
        l1->ToGeoPosition(api::LanePosition(s_total / 2., 2., 0.)),
        api::GeoPosition(point.X() + x_offset, point.Y() + y_offset, 0.),
        kPathTolerance);
  }
  // Outside the lane constraints.
  {
    const ignition::math::Vector3d point =
        arc_length_interpolator->InterpolateMthDerivative(0, s_total / 2.);
    const ignition::math::Vector3d tangent =
        arc_length_interpolator->InterpolateMthDerivative(1, s_total / 2.);
    const double yaw = std::atan2(tangent.Y(), tangent.X());
    const double x_offset = -width / 2. * std::sin(yaw);
    const double y_offset = width / 2. * std::cos(yaw);
    EXPECT_GEO_NEAR(
        l1->ToGeoPosition(api::LanePosition(s_total / 2., 15., 0.)),
        api::GeoPosition(point.X() + x_offset, point.Y() + y_offset, 0.),
        kPathTolerance);
  }
  // Orientation.
  {
    EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(0., 0., 0.)),
                    api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);
    EXPECT_ROT_NEAR(
        l1->GetOrientation(api::LanePosition(
            arc_length_interpolator->BaseSpline()->ArcLength(), 0., 0.)),
        api::Rotation::FromRpy(0., 0., M_PI / 2.), kAngularTolerance);
    const ignition::math::Vector3d point =
        arc_length_interpolator->InterpolateMthDerivative(
            1, arc_length_interpolator->BaseSpline()->ArcLength() / 2.);
    const double yaw = std::atan2(point.Y(), point.X());
    EXPECT_ROT_NEAR(
        l1->GetOrientation(api::LanePosition(
            arc_length_interpolator->BaseSpline()->ArcLength() / 2., 0., 0.)),
        api::Rotation::FromRpy(0., 0., yaw), kAngularTolerance);
    EXPECT_ROT_NEAR(
        l1->GetOrientation(api::LanePosition(
            arc_length_interpolator->BaseSpline()->ArcLength() / 2., 2., 0.)),
        api::Rotation::FromRpy(0., 0., yaw), kAngularTolerance);
    EXPECT_ROT_NEAR(
        l1->GetOrientation(api::LanePosition(
            arc_length_interpolator->BaseSpline()->ArcLength() / 2., -2., 0.)),
        api::Rotation::FromRpy(0., 0., yaw), kAngularTolerance);
  }
  // EvalMotionDerivatives.
  {
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                     api::LanePosition(0., 0., 0.), kPathTolerance);
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                     api::LanePosition(1., 0., 0.), kPathTolerance);
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                     api::LanePosition(0., 1., 0.), kPathTolerance);
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                     api::LanePosition(0., 0., 1.), kPathTolerance);
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({10., 2., 5.}, {0., 0., 1.}),
                     api::LanePosition(0., 0., 1.), kPathTolerance);
  }
}

// Tests the ComputeLength computation.
GTEST_TEST(RNDFSplineLanesTest, ComputeLength) {
  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(20.0, 20.0, 0.0),
                      ignition::math::Vector3d(0, 10.0, 0.0)));
  auto spline = CreateSpline(control_points);
  // Check that length is being correctly computed.
  EXPECT_NEAR(spline->ArcLength(), SplineLane::ComputeLength(control_points),
              kVeryExact);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
