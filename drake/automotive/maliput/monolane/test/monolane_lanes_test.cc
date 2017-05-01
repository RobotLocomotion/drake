#include <cmath>
#include <iostream>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/junction.h"
#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/segment.h"
#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace maliput {
namespace monolane {

const double kLinearTolerance = 1e-6;
const double kAngularTolerance = 1e-6;
const double kVeryExact = 1e-11;


GTEST_TEST(MonolaneLanesTest, Rot3) {
  // Spot-check that Rot3 is behaving as advertised.
  Rot3 rpy90 {M_PI / 2., M_PI / 2., M_PI / 2.};
  EXPECT_TRUE(CompareMatrices(
      rpy90.apply({1., 0., 0.}), V3(0., 0., -1.), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      rpy90.apply({0., 1., 0.}), V3(0., 1., 0.), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      rpy90.apply({0., 0., 1.}), V3(1., 0., 0.), kVeryExact));
}


#define EXPECT_GEO_NEAR(actual, expected, tolerance)         \
  do {                                                       \
    const api::GeoPosition _actual(actual);                  \
    const api::GeoPosition _expected expected;               \
    const double _tolerance = (tolerance);                   \
    EXPECT_NEAR(_actual.x, _expected.x, _tolerance);         \
    EXPECT_NEAR(_actual.y, _expected.y, _tolerance);         \
    EXPECT_NEAR(_actual.z, _expected.z, _tolerance);         \
  } while (0)

#define EXPECT_LANE_NEAR(actual, expected, tolerance)         \
  do {                                                        \
    const api::LanePosition _actual(actual);                  \
    const api::LanePosition _expected expected;               \
    const double _tolerance = (tolerance);                    \
    EXPECT_NEAR(_actual.s, _expected.s, _tolerance);          \
    EXPECT_NEAR(_actual.r, _expected.r, _tolerance);          \
    EXPECT_NEAR(_actual.h, _expected.h, _tolerance);          \
  } while (0)

#define EXPECT_ROT_NEAR(actual, expected, tolerance)                 \
  do {                                                               \
    const api::Rotation _actual(actual);                             \
    const api::Rotation _expected expected;                          \
    const double _tolerance = (tolerance);                           \
    EXPECT_NEAR(_actual.yaw, _expected.yaw, _tolerance);             \
    EXPECT_NEAR(_actual.pitch, _expected.pitch, _tolerance);         \
    EXPECT_NEAR(_actual.roll, _expected.roll, _tolerance);           \
  } while (0)


GTEST_TEST(MonolaneLanesTest, FlatLineLane) {
  CubicPolynomial zp {0., 0., 0., 0.};
  const double half_width = 10.;
  RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  Lane* l1 = s1->NewLineLane(
      {"l1"},
      {100., -75.}, {100., 50.},
      {-5., 5.}, {-half_width, half_width},
      // Zero elevation, zero superelevation == flat.
      zp, zp);

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_EQ(l1->id().id, "l1");
  EXPECT_EQ(l1->segment(), s1);
  EXPECT_EQ(l1->index(), 0);
  EXPECT_EQ(l1->to_left(), nullptr);
  EXPECT_EQ(l1->to_right(), nullptr);

  EXPECT_NEAR(l1->length(), std::sqrt((100. * 100) + (50. * 50.)), kVeryExact);

  EXPECT_NEAR(l1->lane_bounds(0.).r_min, -5., kVeryExact);
  EXPECT_NEAR(l1->lane_bounds(0.).r_max,  5., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_min, -10., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_max,  10., kVeryExact);

  EXPECT_GEO_NEAR(l1->ToGeoPosition({0., 0., 0.}),
                  (100., -75., 0.), kLinearTolerance);

  // A little bit along the lane, but still on the reference line.
  EXPECT_GEO_NEAR(l1->ToGeoPosition({1., 0., 0.}),
                  (100. + ((100./ l1->length()) * 1.),
                   -75. + ((50. / l1->length()) * 1.),
                   0.), kLinearTolerance);

  // At the very beginning of the lane, but laterally off the reference line.
  EXPECT_GEO_NEAR(l1->ToGeoPosition({0., 3., 0.}),
                  (100. + ((-50. / l1->length()) * 3.),
                   -75. + ((100. / l1->length()) * 3.),
                   0.), kLinearTolerance);

  // At the very end of the lane.
  EXPECT_GEO_NEAR(l1->ToGeoPosition({l1->length(), 0., 0.}),
                  (200., -25., 0.), kLinearTolerance);

  // Case 1: Tests LineLane::ToLanePosition() with a closest point that lies
  // within the lane bounds.
  const api::GeoPosition point_within_lane{148., -46., 0.};
  api::GeoPosition nearest_position;
  double distance{};
  const double expected_s = 0.5 * l1->length();
  const double expected_r = std::sqrt(std::pow(2., 2.) + std::pow(4., 2.));
  EXPECT_LANE_NEAR(l1->ToLanePosition(point_within_lane, &nearest_position,
                                      &distance),
                   (expected_s, expected_r, 0.), kVeryExact);
  EXPECT_GEO_NEAR(nearest_position, (148., -46., 0.), kVeryExact);
  EXPECT_NEAR(distance, 0., kVeryExact);

  // Case 2: Tests LineLane::ToLanePosition() with a closest point that lies
  // outside of the lane bounds, verifying that the result saturates.
  const api::GeoPosition point_outside_lane{-75., 25., 0.};
  const double expected_r_outside = half_width;
  const double x_dist_to_edge = half_width * std::sin(std::atan(0.5));
  const double y_dist_to_edge = half_width * std::cos(std::atan(0.5));
  EXPECT_LANE_NEAR(l1->ToLanePosition(point_outside_lane, &nearest_position,
                                      &distance),
                   (0., expected_r_outside, 0.), kVeryExact);
  EXPECT_GEO_NEAR(nearest_position,
                  (100. - x_dist_to_edge, -75. + y_dist_to_edge, 0.),
                  kVeryExact);
  EXPECT_NEAR(distance, std::sqrt(std::pow(175. - x_dist_to_edge, 2.)
                                  + std::pow(100. - y_dist_to_edge, 2.)),
              kVeryExact);

  // Case 3: Tests LineLane::ToLanePosition() at a non-zero but flat elevation.
  const double elevation = 10.;
  const double length = std::sqrt(std::pow(100, 2.) + std::pow(50, 2.));
  Segment* s2 = rg.NewJunction({"j2"})->NewSegment({"s2"});
  Lane* l1_with_z = s2->NewLineLane(
      {"l1_with_z"},
      {100., -75.}, {100., 50.},
      {-5., 5.}, {-half_width, half_width},
      {-elevation / length, 0., 0., 0.} /* constant elevation */,
      zp /* zero superelevation */);
  EXPECT_LANE_NEAR(
      l1_with_z->ToLanePosition(point_outside_lane, &nearest_position,
                                &distance),
      (0., expected_r_outside, elevation), kVeryExact);
  EXPECT_GEO_NEAR(nearest_position,
                  (100. - x_dist_to_edge, -75. + y_dist_to_edge, 0.),
                  kVeryExact);
  EXPECT_NEAR(distance, std::sqrt(std::pow(175. - x_dist_to_edge, 2.)
                                  + std::pow(100. - y_dist_to_edge, 2.)),
              kVeryExact);

  // Tests the integrity of LineLane::ToLanePosition() with various null
  // argument combinations.
  EXPECT_NO_THROW(l1->ToLanePosition(point_within_lane, &nearest_position,
                                     nullptr));
  EXPECT_NO_THROW(l1->ToLanePosition(point_within_lane, nullptr, &distance));
  EXPECT_NO_THROW(l1->ToLanePosition(point_within_lane, nullptr, nullptr));

  // Verifies the output of LineLane::GetOrientation().
  EXPECT_ROT_NEAR(l1->GetOrientation({0., 0., 0.}),
                  (0., 0., std::atan2(50., 100.)), kVeryExact);

  EXPECT_ROT_NEAR(l1->GetOrientation({1., 0., 0.}),
                  (0., 0., std::atan2(50., 100.)), kVeryExact);

  EXPECT_ROT_NEAR(l1->GetOrientation({0., 1., 0.}),
                  (0., 0., std::atan2(50., 100.)), kVeryExact);

  EXPECT_ROT_NEAR(l1->GetOrientation({l1->length(), 0., 0.}),
                  (0., 0., std::atan2(50., 100.)), kVeryExact);

  // Derivative map should be identity (for a flat, straight road).
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                   (0., 0., 0.), kVeryExact);

  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                   (1., 0., 0.), kVeryExact);

  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                   (0., 1., 0.), kVeryExact);

  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                   (0., 0., 1.), kVeryExact);

  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {1., 1., 1.}),
                   (1., 1., 1.), kVeryExact);

  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({10., 5., 3.}, {1., 2., 3.}),
                   (1., 2., 3.), kVeryExact);
}


GTEST_TEST(MonolaneLanesTest, FlatArcLane) {
  CubicPolynomial zp {0., 0., 0., 0.};
  RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);
  const double theta0 = 0.25 * M_PI;
  const double d_theta = 1.5 * M_PI;
  const double radius = 100.;
  const V2 center{100., -75.};
  const double half_width = 10.;
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  Lane* l2 = s1->NewArcLane(
      {"l2"},
      center, radius, theta0, d_theta,
      {-5., 5.}, {-half_width, half_width},
      // Zero elevation, zero superelevation == flat.
      zp, zp);

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_EQ(l2->id().id, "l2");
  EXPECT_EQ(l2->segment(), s1);
  EXPECT_EQ(l2->index(), 0);
  EXPECT_EQ(l2->to_left(), nullptr);
  EXPECT_EQ(l2->to_right(), nullptr);

  EXPECT_NEAR(l2->length(), 100. * 1.5 * M_PI, kVeryExact);

  EXPECT_NEAR(l2->lane_bounds(0.).r_min, -5., kVeryExact);
  EXPECT_NEAR(l2->lane_bounds(0.).r_max,  5., kVeryExact);
  EXPECT_NEAR(l2->driveable_bounds(0.).r_min, -10., kVeryExact);  // half_width
  EXPECT_NEAR(l2->driveable_bounds(0.).r_max,  10., kVeryExact);  // half_width

  // Recall that the arc has center (100, -75) and radius 100.
  EXPECT_GEO_NEAR(l2->ToGeoPosition({0., 0., 0.}),
                  (100. + (100. * std::cos(0.25 * M_PI)),
                   -75. + (100. * std::sin(0.25 * M_PI)),
                   0.), kLinearTolerance);

  EXPECT_GEO_NEAR(
      l2->ToGeoPosition({1., 0., 0.}),
      (100. + (100. * std::cos((0.25 * M_PI) + (1.5 / l2->length() * M_PI))),
       -75. + (100. * std::sin((0.25 * M_PI) + (1.5 / l2->length() * M_PI))),
       0.), kLinearTolerance);

  EXPECT_GEO_NEAR(
      l2->ToGeoPosition({0., 1., 0.}),
      (100. + (100. * std::cos(0.25 * M_PI)) + (1. * std::cos(1.25 * M_PI)),
       -75. + (100. * std::sin(0.25 * M_PI)) + (1. * std::sin(1.25 * M_PI)),
       0.), kLinearTolerance);

  EXPECT_GEO_NEAR(l2->ToGeoPosition({l2->length(), 0., 0.}),
                  (100. + (100. * std::cos(1.75 * M_PI)),
                   -75. + (100. * std::sin(1.75 * M_PI)),
                   0.), kLinearTolerance);

  // Case 1: Tests ArcLane::ToLanePosition() with a closest point that lies
  // within the lane bounds.
  const api::GeoPosition point_within_lane{
    center(0) - 50., center(1) + 50., 0.};  // θ = 0.5π.
  api::GeoPosition nearest_position;
  double distance{};
  const double expected_s = 0.5 * M_PI / d_theta * l2->length();
  const double expected_r = std::min(radius - std::sqrt(2) * 50., half_width);
  EXPECT_LANE_NEAR(l2->ToLanePosition(point_within_lane, &nearest_position,
                                      &distance),
                   (expected_s, expected_r, 0.), kVeryExact);
  EXPECT_GEO_NEAR(nearest_position,
                  ((radius - half_width)
                   * std::cos(0.5 * M_PI + theta0) + center(0),
                   (radius - half_width)
                   * std::sin(0.5 * M_PI + theta0) + center(1), 0.),
                  kVeryExact);
  EXPECT_NEAR(distance,
              (radius - half_width) - std::sqrt(std::pow(50., 2.) +
                                                std::pow(50., 2.)),
              kVeryExact);

  // Case 2: Tests ArcLane::ToLanePosition() with a closest point that lies
  // outside of the lane bounds, verifying that the result saturates.
  const api::GeoPosition point_outside_lane{
    center(0) + 200., center(1) - 20., 0.};  // θ ~= 1.9π.
  const double expected_r_outside = -half_width;
  EXPECT_LANE_NEAR(l2->ToLanePosition(point_outside_lane, &nearest_position,
                                      &distance),
                   (l2->length(), expected_r_outside, 0.), kVeryExact);
  EXPECT_GEO_NEAR(nearest_position,
                  ((radius + half_width) * std::cos(theta0 + d_theta)
                   + center(0),
                   (radius + half_width) * std::sin(theta0 + d_theta)
                   + center(1), 0.),
                  kVeryExact);
  // Because we have saturated, expect the distance to the circle to be less
  // than the distance to the point on the arc.
  EXPECT_GT(distance,
            std::sqrt(std::pow(200., 2.) + std::pow(20., 2.)) - radius);

  // Case 3: Tests ArcLane::ToLanePosition() at a non-zero but flat elevation.
  const double elevation = 10.;
  Segment* s2 = rg.NewJunction({"j2"})->NewSegment({"s2"});
  Lane* l2_with_z = s2->NewArcLane(
      {"l2_with_z"},
      center, radius, theta0, d_theta,
      {-5., 5.}, {-half_width, half_width},
      {-elevation / radius / d_theta, 0., 0., 0.} /* constant elevation */,
      zp /* zero superelevation */);
  EXPECT_LANE_NEAR(
      l2_with_z->ToLanePosition(point_outside_lane, &nearest_position,
                                &distance),
      (l2_with_z->length(), expected_r_outside, elevation), kVeryExact);
  EXPECT_GEO_NEAR(nearest_position,
                  ((radius + half_width) * std::cos(theta0 + d_theta)
                   + center(0),
                   (radius + half_width) * std::sin(theta0 + d_theta)
                   + center(1), 0.),
                  kVeryExact);

  // Case 4: Tests ArcLane::ToLanePosition() with a lane that overlaps itself.
  // The result should be identical to Case 1.
  const double d_theta_overlap = 3 * M_PI;
  Segment* s3 = rg.NewJunction({"j3"})->NewSegment({"s3"});
  Lane* l2_overlapping = s3->NewArcLane(
      {"l2_overlapping"},
      center, radius, theta0, d_theta_overlap,
      {-5., 5.}, {-half_width, half_width},
      // Zero elevation, zero superelevation == flat.
      zp, zp);
  EXPECT_LANE_NEAR(l2_overlapping->ToLanePosition(point_within_lane,
                                                  &nearest_position,
                                                  &distance),
                   (expected_s, expected_r, 0.), kVeryExact);
  EXPECT_GEO_NEAR(nearest_position,
                  ((radius - half_width)
                   * std::cos(0.5 * M_PI + theta0) + center(0),
                   (radius - half_width)
                   * std::sin(0.5 * M_PI + theta0) + center(1), 0.),
                  kVeryExact);
  EXPECT_NEAR(distance,
              (radius - half_width) - std::sqrt(std::pow(50., 2.) +
                                                std::pow(50., 2.)),
              kVeryExact);

  // Case 5: Tests ArcLane::ToLanePosition() with a lane that starts in the
  // third quadrant and ends in the second quadrant; i.e. d_theta is negative
  // and crosses the ±π wrap-around value using a point that is within the lane
  // in the third quadrant.
  const double theta0_wrap = 1.2 * M_PI;
  const double d_theta_wrap = -0.4 * M_PI;
  Segment* s4 = rg.NewJunction({"j4"})->NewSegment({"s4"});
  Lane* l2_wrap = s4->NewArcLane(
      {"l2_wrap"},
      center, radius, theta0_wrap, d_theta_wrap,
      {-5., 5.}, {-half_width, half_width},
      // Zero elevation, zero superelevation == flat.
      zp, zp);
  const api::GeoPosition point_in_third_quadrant{
    center(0) - 90., center(1) - 25., 0.};  // θ ~= -0.9π.
  const double expected_s_wrap =
      (std::atan2(25, -90) - 0.8 * M_PI) / d_theta * l2->length();  // ~0.28L
  const double expected_r_wrap =
      std::sqrt(std::pow(90, 2.) + std::pow(25, 2.)) - radius;
  EXPECT_LANE_NEAR(l2_wrap->ToLanePosition(point_in_third_quadrant,
                                           &nearest_position,
                                           &distance),
                   (expected_s_wrap, expected_r_wrap, 0.), kVeryExact);
  EXPECT_GEO_NEAR(nearest_position, (-90 + center(0), -25 + center(1), 0.),
                  kVeryExact);
  EXPECT_NEAR(distance, 0. /* within lane */, kVeryExact);

  // Tests the integrity of ArcLaneWithConstantSuperelevation::ToLanePosition()
  // with various null argument combinations.
  EXPECT_NO_THROW(l2->ToLanePosition(point_within_lane, &nearest_position,
                                     nullptr));
  EXPECT_NO_THROW(l2->ToLanePosition(point_within_lane, nullptr, &distance));
  EXPECT_NO_THROW(l2->ToLanePosition(point_within_lane, nullptr, nullptr));

  // Verifies the output of ArcLane::GetOrientation().
  EXPECT_ROT_NEAR(l2->GetOrientation({0., 0., 0.}),
                  (0., 0., (0.25 + 0.5) * M_PI), kVeryExact);

  EXPECT_ROT_NEAR(l2->GetOrientation({0., 1., 0.}),
                  (0., 0., (0.25 + 0.5) * M_PI), kVeryExact);

  EXPECT_ROT_NEAR(l2->GetOrientation({l2->length(), 0., 0.}),
                  (0., 0, 0.25 * M_PI), kVeryExact);  // 0.25 + 1.5 + 0.5

  // For r=0, derivative map should be identity.
  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                   (0., 0., 0.), kVeryExact);

  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                   (1., 0., 0.), kVeryExact);

  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                   (0., 1., 0.), kVeryExact);

  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                   (0., 0., 1.), kVeryExact);

  EXPECT_LANE_NEAR(
      l2->EvalMotionDerivatives({l2->length(), 0., 0.}, {1., 1., 1.}),
      (1., 1., 1.), kVeryExact);

  // For a left-turning curve, r = +10 will decrease the radius of the path
  // from the original 100 down to 90.
  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., 10., 0.}, {1., 1., 1.}),
                   ((100. / 90.) * 1., 1., 1.), kVeryExact);
  // Likewise, r = -10 will increase the radius of the path from the
  // original 100 up to 110.
  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., -10., 0.}, {1., 1., 1.}),
                   ((100. / 110.) * 1., 1., 1.), kVeryExact);
  // ...and only r should matter for an otherwise flat arc.
  EXPECT_LANE_NEAR(
      l2->EvalMotionDerivatives({l2->length(), -10., 100.}, {1., 1., 1.}),
      ((100. / 110.) * 1., 1., 1.), kVeryExact);
}


GTEST_TEST(MonolaneLanesTest, ArcLaneWithConstantSuperelevation) {
  CubicPolynomial zp {0., 0., 0., 0.};
  const double kTheta = 0.10 * M_PI;  // superelevation
  RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  Lane* l2 = s1->NewArcLane(
      {"l2"},
      {100., -75.}, 100., 0.25 * M_PI, 1.5 * M_PI,
      {-5., 5.}, {-10., 10.},
      zp,
      { (kTheta) / (100. * 1.5 * M_PI), 0., 0., 0. });

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_NEAR(l2->length(), 100. * 1.5 * M_PI, kVeryExact);

  EXPECT_GEO_NEAR(l2->ToGeoPosition({0., 0., 0.}),
                  (100. + (100. * std::cos(0.25 * M_PI)),
                   -75. + (100. * std::sin(0.25 * M_PI)),
                   0.), kLinearTolerance);

  // NB: (1.25 * M_PI) is the direction of the r-axis at s = 0.
  EXPECT_GEO_NEAR(
      l2->ToGeoPosition({0., 10., 0.}),
      (100. + (100. * std::cos(0.25 * M_PI)) +
       (10. * std::cos(kTheta) * std::cos(1.25 * M_PI)),
       -75. + (100. * std::sin(0.25 * M_PI)) +
       (10. * std::cos(kTheta) * std::sin(1.25 * M_PI)),
       10. * std::sin(kTheta)), kLinearTolerance);

  // TODO(maddog@tri.global) Test ToLanePosition().

  EXPECT_ROT_NEAR(l2->GetOrientation({0., 0., 0.}),
                  (kTheta, 0., (0.25 + 0.5) * M_PI), kVeryExact);

  EXPECT_ROT_NEAR(l2->GetOrientation({0., 1., 0.}),
                  (kTheta, 0., (0.25 + 0.5) * M_PI), kVeryExact);

  EXPECT_ROT_NEAR(l2->GetOrientation({l2->length(), 0., 0.}),
                  (kTheta, 0., 0.25 * M_PI), kVeryExact);  // 0.25 + 1.5 + 0.5

  api::LanePosition pdot;
  // For r=0, derivative map should be identity.
  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                   (0., 0., 0.), kVeryExact);

  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                   (1., 0., 0.), kVeryExact);

  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                   (0., 1., 0.), kVeryExact);

  EXPECT_LANE_NEAR(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                   (0., 0., 1.), kVeryExact);

  EXPECT_LANE_NEAR(
      l2->EvalMotionDerivatives({l2->length(), 0., 0.}, {1., 1., 1.}),
      (1., 1., 1.), kVeryExact);

  // For a left-turning curve, r = +10 will decrease the radius of the path
  // from the original 100 down to almost 90.  (r is scaled by the cosine of
  // the superelevation since it is no longer horizontal).
  EXPECT_LANE_NEAR(
      l2->EvalMotionDerivatives({0., 10., 0.}, {1., 1., 1.}),
      ((100. / (100. - (10. * std::cos(kTheta)))) * 1., 1., 1.), kVeryExact);
  // Likewise, r = -10 will increase the radius of the path from the
  // original 100 up to almost 110 (since r is no longer horizontal).
  EXPECT_LANE_NEAR(
      l2->EvalMotionDerivatives({0., -10., 0.}, {1., 1., 1.}),
      ((100. / (100 + (10. * std::cos(kTheta)))) * 1., 1., 1.), kVeryExact);

  // h matters, too (because hovering above a tilted road changes one's
  // distance to the center of the arc).
  EXPECT_LANE_NEAR(
      l2->EvalMotionDerivatives({l2->length(), -10., 8.}, {1., 1., 1.}),
      ((100. / (100 + (10. * std::cos(kTheta))
                + (8. * std::sin(kTheta)))) * 1., 1., 1.), kVeryExact);
}

namespace {
api::LanePosition IntegrateTrivially(const api::Lane* lane,
                                     const api::LanePosition& lp_initial,
                                     const api::IsoLaneVelocity& velocity,
                                     const double time_step,
                                     const int step_count) {
  api::LanePosition lp_current = lp_initial;

  for (int i = 0; i < step_count; ++i) {
    const api::LanePosition lp_dot =
        lane->EvalMotionDerivatives(lp_current, velocity);
    lp_current.s += lp_dot.s * time_step;
    lp_current.r += lp_dot.r * time_step;
    lp_current.h += lp_dot.h * time_step;
  }
  return lp_current;
}
}  // namespace


GTEST_TEST(MonolaneLanesTest, HillIntegration) {
  CubicPolynomial zp {0., 0., 0., 0.};
  RoadGeometry rg({"apple"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  const double theta0 = 0.25 * M_PI;
  const double d_theta = 0.5 * M_PI;
  const double theta1 = theta0 + d_theta;
  const double p_scale = 100. * d_theta;
  const double z0 = 0.;
  const double z1 = 20.;
  // A cubic polynomial such that:
  //   f(0) = (z0 / p_scale), f(1) = (z1 / p_scale), and f'(0) = f'(1) = 0.
  const CubicPolynomial kHillPolynomial(z0 / p_scale,
                                        0.,
                                        (3. * (z1 - z0) / p_scale),
                                        (-2. * (z1 - z0) / p_scale));
  Lane* l1 = s1->NewArcLane(
      {"l2"},
      {-100., -100.}, 100., theta0, d_theta,
      {-5., 5.}, {-10., 10.},
      kHillPolynomial,
      zp);

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  const api::IsoLaneVelocity kVelocity { 1., 0., 0. };
  const double kTimeStep = 0.001;
  const int kStepsForZeroR = 158597;
  const double kIntegrationTolerance = 3e-4;

  const api::LanePosition kLpInitialA { 0., 0., 0. };
  EXPECT_GEO_NEAR(l1->ToGeoPosition(kLpInitialA),
                  (-100. + (100. * std::cos(theta0)),
                   -100. + (100. * std::sin(theta0)),
                   z0), kLinearTolerance);

  api::LanePosition lp_final_a =
      IntegrateTrivially(l1, kLpInitialA, kVelocity, kTimeStep,
                         kStepsForZeroR);
  EXPECT_GEO_NEAR(l1->ToGeoPosition(lp_final_a),
                  (-100. + (100. * std::cos(theta1)),
                   -100. + (100. * std::sin(theta1)),
                   z1), kIntegrationTolerance);

  const api::LanePosition kLpInitialB { 0., -10., 0. };
  EXPECT_GEO_NEAR(l1->ToGeoPosition(kLpInitialB),
                  (-100. + ((100. + 10.) * std::cos(theta0)),
                   -100. + ((100. + 10.) * std::sin(theta0)),
                   z0), kLinearTolerance);

  // NB:  '287' is a fudge-factor.  We know the steps should scale roughly
  //      as (r / r0), but not exactly because of the elevation curve.
  //      Mostly, we are testing that we end up in the right place in
  //      roughly the right number of steps.
  const int kStepsForR10 = ((100. + 10.) / 100. * kStepsForZeroR) - 287;
  api::LanePosition lp_final_b =
      IntegrateTrivially(l1, kLpInitialB, kVelocity, kTimeStep,
                         kStepsForR10);
  EXPECT_GEO_NEAR(l1->ToGeoPosition(lp_final_b),
                  (-100. + ((100. + 10.) * std::cos(theta1)),
                   -100. + ((100. + 10.) * std::sin(theta1)),
                   z1), kIntegrationTolerance);
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
