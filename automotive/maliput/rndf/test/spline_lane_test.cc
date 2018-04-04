#include "drake/automotive/maliput/rndf/spline_lane.h"

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
#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"
#include "drake/automotive/maliput/rndf/spline_helpers.h"

namespace drake {
namespace maliput {
namespace rndf {
namespace {

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
// @param control_points A vector of tuples consisting of two
// ignition::math::Vector3d objects. The tuple's first value is the knot point
// while its second value is the tangent at the knot point.
// @return a pointer to an ignition::math::Spline object.
std::unique_ptr<ignition::math::Spline> CreateSpline(
    const std::vector<std::tuple<ignition::math::Vector3d,
                                 ignition::math::Vector3d>>& control_points) {
  auto spline = std::make_unique<ignition::math::Spline>();
  spline->AutoCalculate(true);
  for (const auto& control_point : control_points) {
    spline->AddPoint(std::get<0>(control_point), std::get<1>(control_point));
  }
  return spline;
}

// Performs geometric tests over a straight lane.
GTEST_TEST(RNDFSplineLanesTest, FlatLineLane) {
  const double kWidth = 5.;
  RoadGeometry rg(api::RoadGeometryId{"FlatLineLane"},
                  kPathTolerance, kAngularTolerance);
  Segment* s1 =
      rg.NewJunction(api::JunctionId{"j1"})->NewSegment(api::SegmentId{"s1"});
  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(20.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  const Lane* l1 = s1->NewSplineLane(api::LaneId{"l1"}, control_points, kWidth);
  // Checks road geometry invariants.
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());
  // Checks lane length.
  EXPECT_NEAR(l1->length(), std::sqrt((20. * 20.) + (0. * 0.)), kPathTolerance);
  // Checks bounds.
  EXPECT_TRUE(api::test::IsRBoundsClose(l1->lane_bounds(0.),
                                        api::RBounds(-kWidth / 2., kWidth / 2.),
                                        kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(l1->driveable_bounds(0.),
                                        api::RBounds(-kWidth / 2., kWidth / 2.),
                                        kVeryExact));

  // ToGeoPosition.
  // Reference line.
  // At the beginning, end and middle.
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      l1->ToGeoPosition(api::LanePosition(0., 0., 0.)),
      api::GeoPosition(0., 0., 0.),
      kPathTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      l1->ToGeoPosition(api::LanePosition(20., 0., 0.)),
      api::GeoPosition(20., 0., 0.),
      kPathTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      l1->ToGeoPosition(api::LanePosition(10., 0., 0.)),
      api::GeoPosition(10., 0., 0.),
      kPathTolerance));

  // A couple of meters away of the reference baseline.
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      l1->ToGeoPosition(api::LanePosition(5., 2., 0.)),
      api::GeoPosition(5., 2., 0.),
      kPathTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      l1->ToGeoPosition(api::LanePosition(12., -2., 0.)),
      api::GeoPosition(12., -2., 0.),
      kPathTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      l1->ToGeoPosition(api::LanePosition(18., 1.234567, 0.)),
      api::GeoPosition(18., 1.234567, 0.),
      kPathTolerance));

  // Orientation.
  EXPECT_TRUE(api::test::IsRotationClose(
      l1->GetOrientation(api::LanePosition(0., 0., 0.)),
      api::Rotation::FromRpy(0., 0., 0.),
      kAngularTolerance));
  EXPECT_TRUE(api::test::IsRotationClose(
      l1->GetOrientation(api::LanePosition(20., 0., 0.)),
      api::Rotation::FromRpy(0., 0., 0.),
      kAngularTolerance));
  EXPECT_TRUE(api::test::IsRotationClose(
      l1->GetOrientation(api::LanePosition(10., 2., 0.)),
      api::Rotation::FromRpy(0., 0., 0.),
      kAngularTolerance));
  EXPECT_TRUE(api::test::IsRotationClose(
      l1->GetOrientation(api::LanePosition(10., -2., 0.)),
      api::Rotation::FromRpy(0., 0., 0.),
      kAngularTolerance));

  // EvalMotionDerivatives.
  EXPECT_TRUE(api::test::IsLanePositionClose(
      l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
      api::LanePosition(0., 0., 0.),
      kPathTolerance));
  EXPECT_TRUE(api::test::IsLanePositionClose(
      l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
      api::LanePosition(1., 0., 0.),
      kPathTolerance));
  EXPECT_TRUE(api::test::IsLanePositionClose(
      l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
      api::LanePosition(0., 1., 0.),
      kPathTolerance));
  EXPECT_TRUE(api::test::IsLanePositionClose(
      l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
      api::LanePosition(0., 0., 1.),
      kPathTolerance));
  EXPECT_TRUE(api::test::IsLanePositionClose(
      l1->EvalMotionDerivatives({10., 2., 5.}, {0., 0., 1.}),
      api::LanePosition(0., 0., 1.),
      kPathTolerance));
}

// Performs geometric tests over a curved lane.
GTEST_TEST(RNDFSplineLanesTest, CurvedLineLane) {
  const double kWidth = 5.;
  RoadGeometry rg(api::RoadGeometryId{"CurvedLineLane"},
                  kPathTolerance, kAngularTolerance);
  Segment* s1 =
      rg.NewJunction(api::JunctionId{"j1"})->NewSegment(api::SegmentId{"s1"});
  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(20.0, 20.0, 0.0),
                      ignition::math::Vector3d(0, 10.0, 0.0)));
  const Lane* l1 = s1->NewSplineLane(api::LaneId{"l1"}, control_points, kWidth);
  // Checks road geometry invariants.
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());
  // Creates a spline to check the internal values.
  std::unique_ptr<ignition::math::Spline> spline = CreateSpline(control_points);
  auto arc_length_interpolator = std::make_unique<ArcLengthParameterizedSpline>(
      std::move(spline), SplineLane::SplineErrorBound());
  // Checks length.
  const double s_total = arc_length_interpolator->BaseSpline()->ArcLength();
  EXPECT_NEAR(l1->length(), s_total, kPathTolerance);
  // Checks bounds.
  EXPECT_TRUE(api::test::IsRBoundsClose(l1->lane_bounds(0.),
                                        api::RBounds(-kWidth / 2., kWidth / 2.),
                                        kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(l1->driveable_bounds(0.),
                                        api::RBounds(-kWidth / 2., kWidth / 2.),
                                        kVeryExact));

  // ToGeoPosition.
  // Reference line.
  // At the beginning, end and middle.
  {
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        l1->ToGeoPosition(api::LanePosition(0., 0., 0.)),
        api::GeoPosition(0., 0., 0.),
        kPathTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        l1->ToGeoPosition(api::LanePosition(s_total, 0., 0.)),
        api::GeoPosition(20., 20.0, 0.),
        kPathTolerance));
    const ignition::math::Vector3d point =
        arc_length_interpolator->InterpolateMthDerivative(0, s_total / 2.);
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        l1->ToGeoPosition(api::LanePosition(s_total / 2., 0., 0.)),
        api::GeoPosition(point.X(), point.Y(), 0.),
        kPathTolerance));
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
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        l1->ToGeoPosition(api::LanePosition(s_total / 2., 2., 0.)),
        api::GeoPosition(point.X() + x_offset, point.Y() + y_offset, 0.),
        kPathTolerance));
  }
  // Outside the lane constraints.
  {
    const ignition::math::Vector3d point =
        arc_length_interpolator->InterpolateMthDerivative(0, s_total / 2.);
    const ignition::math::Vector3d tangent =
        arc_length_interpolator->InterpolateMthDerivative(1, s_total / 2.);
    const double yaw = std::atan2(tangent.Y(), tangent.X());
    const double x_offset = -kWidth / 2. * std::sin(yaw);
    const double y_offset = kWidth / 2. * std::cos(yaw);
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        l1->ToGeoPosition(api::LanePosition(s_total / 2., 15., 0.)),
        api::GeoPosition(point.X() + x_offset, point.Y() + y_offset, 0.),
        kPathTolerance));
  }
  // Orientation.
  {
    EXPECT_TRUE(api::test::IsRotationClose(
        l1->GetOrientation(api::LanePosition(0., 0., 0.)),
        api::Rotation::FromRpy(0., 0., 0.),
        kAngularTolerance));
    EXPECT_TRUE(api::test::IsRotationClose(
        l1->GetOrientation(api::LanePosition(
            arc_length_interpolator->BaseSpline()->ArcLength(), 0., 0.)),
        api::Rotation::FromRpy(0., 0., M_PI / 2.),
        kAngularTolerance));
    const ignition::math::Vector3d point =
        arc_length_interpolator->InterpolateMthDerivative(
            1, arc_length_interpolator->BaseSpline()->ArcLength() / 2.);
    const double yaw = std::atan2(point.Y(), point.X());
    EXPECT_TRUE(api::test::IsRotationClose(
        l1->GetOrientation(api::LanePosition(
            arc_length_interpolator->BaseSpline()->ArcLength() / 2., 0., 0.)),
        api::Rotation::FromRpy(0., 0., yaw),
        kAngularTolerance));
    EXPECT_TRUE(api::test::IsRotationClose(
        l1->GetOrientation(api::LanePosition(
            arc_length_interpolator->BaseSpline()->ArcLength() / 2., 2., 0.)),
        api::Rotation::FromRpy(0., 0., yaw),
        kAngularTolerance));
    EXPECT_TRUE(api::test::IsRotationClose(
        l1->GetOrientation(api::LanePosition(
            arc_length_interpolator->BaseSpline()->ArcLength() / 2., -2., 0.)),
        api::Rotation::FromRpy(0., 0., yaw),
        kAngularTolerance));
  }
  // EvalMotionDerivatives.
  {
    EXPECT_TRUE(api::test::IsLanePositionClose(
        l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
        api::LanePosition(0., 0., 0.),
        kPathTolerance));
    EXPECT_TRUE(api::test::IsLanePositionClose(
        l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
        api::LanePosition(1., 0., 0.),
        kPathTolerance));
    EXPECT_TRUE(api::test::IsLanePositionClose(
        l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
        api::LanePosition(0., 1., 0.),
        kPathTolerance));
    EXPECT_TRUE(api::test::IsLanePositionClose(
        l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
        api::LanePosition(0., 0., 1.),
        kPathTolerance));
    EXPECT_TRUE(api::test::IsLanePositionClose(
        l1->EvalMotionDerivatives({10., 2., 5.}, {0., 0., 1.}),
        api::LanePosition(0., 0., 1.),
        kPathTolerance));
  }
}

// Constructor constraints.
GTEST_TEST(RNDFSplineLanesTest, Constructor) {
  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  // Check the constraint in size of control_points.
  EXPECT_THROW(SplineLane(api::LaneId{"l:1"}, nullptr, control_points, 5.0, 1),
               std::runtime_error);
  // Adding a second control_point should pass.
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(20.0, 20.0, 0.0),
                      ignition::math::Vector3d(0, 10.0, 0.0)));
  EXPECT_NO_THROW(
      SplineLane(api::LaneId{"l:1"}, nullptr, control_points, 5.0, 1));
}

// Tests the ComputeLength computation.
GTEST_TEST(RNDFSplineLanesTest, ComputeLength) {
  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  // Check the constraint in size of control_points.
  EXPECT_THROW(SplineLane::ComputeLength(control_points), std::runtime_error);

  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(20.0, 20.0, 0.0),
                      ignition::math::Vector3d(0, 10.0, 0.0)));
  auto spline = CreateSpline(control_points);
  // Check that length is being correctly computed.
  EXPECT_NEAR(spline->ArcLength(), SplineLane::ComputeLength(control_points),
              kVeryExact);
}

// Performs geometric tests over the bounds of a pair of straight lanes.
//
// Geometry of the segment is as follows:
//
// x = 0.0        x = 10.0
// +--------------+   <-- y = 12.5
// |------l2------|   <-- y = 10.0
// +--------------+   <-- y = 7.5
//
// +--------------+   <-- y = 2.5
// |------l1------|   <-- y = 0.0
// +--------------+   <-- y = -2.5
GTEST_TEST(RNDFSplineLanesTest, TwoFlatLineLanesBoundChecks) {
  const double kWidth = 5.;
  RoadGeometry rg(api::RoadGeometryId{"TwoFlatLineLanes"},
                  kPathTolerance, kAngularTolerance);
  Segment* s1 =
      rg.NewJunction(api::JunctionId{"j1"})->NewSegment(api::SegmentId{"s1"});
  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(1.0, 0.0, 0.0)));
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(10.0, 0.0, 0.0),
                      ignition::math::Vector3d(1.0, 0.0, 0.0)));
  const Lane* l1 = s1->NewSplineLane(api::LaneId{"l1"}, control_points, kWidth);

  control_points.clear();
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 10.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(10.0, 10.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  const Lane* l2 = s1->NewSplineLane(api::LaneId{"l2"}, control_points, kWidth);
  // Checks road geometry invariants.
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());
  // Checks bounds.
  EXPECT_TRUE(api::test::IsRBoundsClose(l1->lane_bounds(0.),
                                        api::RBounds(-kWidth / 2., kWidth / 2.),
                                        kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(l2->lane_bounds(0.),
                                        api::RBounds(-kWidth / 2., kWidth / 2.),
                                        kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(l1->driveable_bounds(0.),
                                        api::RBounds(-kWidth / 2.,
                                                     10.0 + kWidth / 2.),
                                        kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(l2->driveable_bounds(0.),
                                        api::RBounds(-10.0 -kWidth / 2.,
                                                     kWidth / 2.),
                                        kVeryExact));
}

}  // namespace
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
