#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-6;
const double kAngularTolerance = 1e-6;
const double kVeryExact = 1e-12;

std::unique_ptr<ignition::math::Spline>
CreateSpline(
  const std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>> &points) {
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();
  spline->Tension(SplineLane::Tension());
  spline->AutoCalculate(true);
  for (const auto &point : points) {
    spline->AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline;
}

#define EXPECT_GEO_NEAR(actual, expected, tolerance)         \
  do {                                                       \
    const api::GeoPosition _actual(actual);                  \
    const api::GeoPosition _expected(expected);               \
    const double _tolerance = (tolerance);                   \
    EXPECT_NEAR(_actual.x(), _expected.x(), _tolerance);         \
    EXPECT_NEAR(_actual.y(), _expected.y(), _tolerance);         \
    EXPECT_NEAR(_actual.z(), _expected.z(), _tolerance);         \
  } while (0)

#define EXPECT_LANE_NEAR(actual, expected, tolerance)         \
  do {                                                        \
    const api::LanePosition _actual(actual);                  \
    const api::LanePosition _expected expected;               \
    const double _tolerance = (tolerance);                    \
    EXPECT_NEAR(_actual.s(), _expected.s(), _tolerance);          \
    EXPECT_NEAR(_actual.r(), _expected.r(), _tolerance);          \
    EXPECT_NEAR(_actual.h(), _expected.h(), _tolerance);          \
  } while (0)

#define EXPECT_ROT_NEAR(actual, expected, tolerance)                 \
  do {                                                               \
    const api::Rotation _actual = actual;                            \
    const api::Rotation _expected = expected;                        \
    const double _tolerance = (tolerance);                           \
    EXPECT_NEAR(_actual.yaw(), _expected.yaw(), _tolerance);         \
    EXPECT_NEAR(_actual.pitch(), _expected.pitch(), _tolerance);     \
    EXPECT_NEAR(_actual.roll(), _expected.roll(), _tolerance);       \
  } while (0)

// It makes geometric tests over a straight line lane
GTEST_TEST(RNDFSplineLanesTest, FlatLineLane) {
  const double width = 5.;
  RoadGeometry rg({"FlatLineLane"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  Lane *l1 = s1->NewSplineLane(
    {"l1"},
    control_points,
    width);

  // Check road geometry invariants
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  // Check length
  {
    EXPECT_NEAR(l1->length(), std::sqrt((20. * 20.) + (0. * 0.)),
      kLinearTolerance);
  }

  // Check bounds
  {
    EXPECT_NEAR(l1->lane_bounds(0.).r_min, -width / 2., kVeryExact);
    EXPECT_NEAR(l1->lane_bounds(0.).r_max,  width / 2., kVeryExact);
    EXPECT_NEAR(l1->driveable_bounds(0.).r_min, -width / 2., kVeryExact);
    EXPECT_NEAR(l1->driveable_bounds(0.).r_max,  width / 2., kVeryExact);
  }

  // ToGeoPosition
  // Reference line
  // At the beginning, end and middle
  {
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(0., 0., 0.)),
      api::GeoPosition(0., 0., 0.), kLinearTolerance);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(20., 0., 0.)),
      api::GeoPosition(20., 0., 0.), kLinearTolerance);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(10., 0., 0.)),
      api::GeoPosition(10., 0., 0.), kLinearTolerance);
  }

  // A couple of meters away of the reference baseline
  {
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(5., 2., 0.)),
      api::GeoPosition(5., 2., 0.), kLinearTolerance);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(12., -2., 0.)),
      api::GeoPosition(12., -2., 0.), kLinearTolerance);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(18., 1.234567, 0.)),
      api::GeoPosition(18., 1.234567, 0.), kLinearTolerance);
  }

  // Orientation
  {
    EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(0., 0., 0.)),
      api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);
    EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(20., 0., 0.)),
      api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);
    EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(10., 2., 0.)),
      api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);
    EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(10., -2., 0.)),
      api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);
  }

  // EvalMotionDerivatives
  {
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
      (0., 0., 0.), kLinearTolerance);
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
      (1., 0., 0.), kLinearTolerance);
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
      (0., 1., 0.), kLinearTolerance);
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
      (0., 0., 1.), kLinearTolerance);
    EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({10., 2., 5.}, {0., 0., 1.}),
      (0., 0., 1.), kLinearTolerance);
  }
}

// It makes geometric tests over a curved spline.
GTEST_TEST(RNDFSplineLanesTest, CurvedLineLane) {
  const double width = 5.;
  RoadGeometry rg({"CurvedLineLane"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 20.0, 0.0),
      ignition::math::Vector3d(0, 10.0, 0.0)));
  Lane *l1 = s1->NewSplineLane(
    {"l1"},
    control_points,
    width);
  // Check road geometry invariants
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  auto spline = CreateSpline(control_points);
  auto arc_length_interpolator =
    std::make_unique<ArcLengthParameterizedSpline>(
      std::move(spline), SplineLane::SplineErrorBound());
  // Check length
  const double s_total = arc_length_interpolator->BaseSpline()->ArcLength();
  {
    EXPECT_NEAR(l1->length(), s_total, kLinearTolerance);
  }

  // Check bounds
  {
    EXPECT_NEAR(l1->lane_bounds(0.).r_min, -width / 2., kVeryExact);
    EXPECT_NEAR(l1->lane_bounds(0.).r_max,  width / 2., kVeryExact);
    EXPECT_NEAR(l1->driveable_bounds(0.).r_min, -width / 2., kVeryExact);
    EXPECT_NEAR(l1->driveable_bounds(0.).r_max,  width / 2., kVeryExact);
  }

  // ToGeoPosition
  // Reference line
  // At the beginning, end and middle
  {
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(0., 0., 0.)),
      api::GeoPosition(0., 0., 0.), kLinearTolerance);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(s_total, 0., 0.)),
      api::GeoPosition(20., 20.0, 0.), kLinearTolerance);
    const auto point =
      arc_length_interpolator->InterpolateMthDerivative(0, s_total/2.);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(s_total/2., 0., 0.)),
      api::GeoPosition(point.X(), point.Y(), 0.), kLinearTolerance);
  }

  // On the plane but at any point over the road
  {
    const auto point =
      arc_length_interpolator->InterpolateMthDerivative(0, s_total/2.);
    const auto tangent =
      arc_length_interpolator->InterpolateMthDerivative(1, s_total/2.);
    const double yaw = std::atan2(tangent.Y(), tangent.X());
    const double x_offset = -2. * std::sin(yaw);
    const double y_offset = 2. * std::cos(yaw);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(s_total/2., 2., 0.)),
      api::GeoPosition(point.X() + x_offset, point.Y() + y_offset, 0.),
      kLinearTolerance);
  }
  // Outside the lane constraints
  {
    const auto point =
      arc_length_interpolator->InterpolateMthDerivative(0, s_total/2.);
    const auto tangent =
      arc_length_interpolator->InterpolateMthDerivative(1, s_total/2.);
    const double yaw = std::atan2(tangent.Y(), tangent.X());
    const double x_offset = -15. * std::sin(yaw);
    const double y_offset = 15. * std::cos(yaw);
    EXPECT_GEO_NEAR(l1->ToGeoPosition(api::LanePosition(s_total/2., 15., 0.)),
      api::GeoPosition(point.X() + x_offset, point.Y() + y_offset, 0.),
      kLinearTolerance);
  }
  // Orientation
  {
    EXPECT_ROT_NEAR(l1->GetOrientation(api::LanePosition(0., 0., 0.)),
      api::Rotation::FromRpy(0., 0., 0.), kAngularTolerance);
    EXPECT_ROT_NEAR(
      l1->GetOrientation(
        api::LanePosition(
          arc_length_interpolator->BaseSpline()->ArcLength(), 0., 0.)),
      api::Rotation::FromRpy(0., 0., M_PI/2.), kAngularTolerance);
    const auto point =
      arc_length_interpolator->InterpolateMthDerivative(1,
        arc_length_interpolator->BaseSpline()->ArcLength() / 2.);
    const double yaw = std::atan2(point.Y(), point.X());
    EXPECT_ROT_NEAR(
      l1->GetOrientation(
        api::LanePosition(
          arc_length_interpolator->BaseSpline()->ArcLength() / 2., 0., 0.)),
      api::Rotation::FromRpy(0., 0., yaw), kAngularTolerance);
    EXPECT_ROT_NEAR(
      l1->GetOrientation(
        api::LanePosition(
          arc_length_interpolator->BaseSpline()->ArcLength() / 2., 2., 0.)),
      api::Rotation::FromRpy(0., 0., yaw), kAngularTolerance);
    EXPECT_ROT_NEAR(
      l1->GetOrientation(
        api::LanePosition(
          arc_length_interpolator->BaseSpline()->ArcLength() / 2., -2., 0.)),
      api::Rotation::FromRpy(0., 0., yaw), kAngularTolerance);
  }
  // EvalMotionDerivatives
  {
    EXPECT_LANE_NEAR(
      l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}), (0., 0., 0.),
      kLinearTolerance);
    EXPECT_LANE_NEAR(
      l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}), (1., 0., 0.),
      kLinearTolerance);
    EXPECT_LANE_NEAR(
      l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}), (0., 1., 0.),
      kLinearTolerance);
    EXPECT_LANE_NEAR(
      l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}), (0., 0., 1.),
      kLinearTolerance);
    EXPECT_LANE_NEAR(
      l1->EvalMotionDerivatives({10., 2., 5.}, {0., 0., 1.}), (0., 0., 1.),
      kLinearTolerance);
  }
}

GTEST_TEST(RNDFSplineLanesTest, ComputeLength) {
  std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 20.0, 0.0),
      ignition::math::Vector3d(0, 10.0, 0.0)));
  auto spline = CreateSpline(control_points);
  // Check road geometry invariants
  EXPECT_NEAR(spline->ArcLength(), SplineLane::ComputeLength(control_points),
    kLinearTolerance);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
