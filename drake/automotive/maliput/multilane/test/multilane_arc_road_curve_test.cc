/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/arc_road_curve.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {

// Checks ArcRoadCurve constructor constraints.
GTEST_TEST(MultilaneArcRoadCurve, ConstructorTest) {
  const Vector2<double> kCenter(10.0, 10.0);
  const double kRadius = 10.0;
  const double kTheta0 = M_PI / 4.0;
  const double kTheta1 = 3.0 * M_PI / 4.0;
  const double kDTheta = kTheta1 - kTheta0;
  const CubicPolynomial zp;
  EXPECT_THROW(ArcRoadCurve(kCenter, -kRadius, kTheta0, kDTheta, zp, zp),
               std::runtime_error);
  EXPECT_NO_THROW(ArcRoadCurve(kCenter, kRadius, kTheta0, kDTheta, zp, zp));
}

// Checks arc reference curve interpolations, derivatives, and lengths.
GTEST_TEST(MultilaneArcRoadCurve, ArcGeometryTest) {
  const Vector2<double> kCenter(10.0, 10.0);
  const double kRadius = 10.0;
  const double kTheta0 = M_PI / 4.0;
  const double kTheta1 = 3.0 * M_PI / 4.0;
  const double kDTheta = kTheta1 - kTheta0;
  const double kVeryExact = 1e-12;
  const CubicPolynomial zp;
  const ArcRoadCurve flat_arc_geometry(kCenter, kRadius, kTheta0, kDTheta, zp,
                                      zp);
  // Checks the length.
  EXPECT_NEAR(flat_arc_geometry.length(), kDTheta * kRadius, kVeryExact);
  EXPECT_NEAR(flat_arc_geometry.trajectory_length(), kDTheta * kRadius,
              kVeryExact);
  // Check the interpolation of p at different values over the reference curve.
  EXPECT_TRUE(
      CompareMatrices(flat_arc_geometry.xy_of_p(0.0),
                      kCenter + Vector2<double>(kRadius * std::cos(kTheta0),
                                                kRadius * std::sin(kTheta0)),
                      kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_arc_geometry.xy_of_p(0.5),
      kCenter + Vector2<double>(kRadius * std::cos(kTheta0 + kDTheta * 0.5),
                                kRadius * std::sin(kTheta0 + kDTheta * 0.5)),
      kVeryExact));
  EXPECT_TRUE(
      CompareMatrices(flat_arc_geometry.xy_of_p(1.0),
                      kCenter + Vector2<double>(kRadius * std::cos(kTheta1),
                                                kRadius * std::sin(kTheta1)),
                      kVeryExact));
  // Checks the derivative of p at different values over the reference curve.
  EXPECT_TRUE(
      CompareMatrices(flat_arc_geometry.xy_dot_of_p(0.0),
                      Vector2<double>(-kRadius * std::sin(kTheta0) * kDTheta,
                                      kRadius * std::cos(kTheta0) * kDTheta),
                      kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_arc_geometry.xy_dot_of_p(0.5),
      Vector2<double>(-kRadius * std::sin(kTheta0 + 0.5 * kDTheta) * kDTheta,
                      kRadius * std::cos(kTheta0 + 0.5 * kDTheta) * kDTheta),
      kVeryExact));
  EXPECT_TRUE(
      CompareMatrices(flat_arc_geometry.xy_dot_of_p(1.0),
                      Vector2<double>(-kRadius * std::sin(kTheta1) * kDTheta,
                                      kRadius * std::cos(kTheta1) * kDTheta),
                      kVeryExact));
  // Checks the heading at different values.
  EXPECT_NEAR(flat_arc_geometry.heading_of_p(0.0), kTheta0 + M_PI / 2.0,
              kVeryExact);
  EXPECT_NEAR(flat_arc_geometry.heading_of_p(0.5),
              kTheta0 + kDTheta / 2.0 + M_PI / 2.0, kVeryExact);
  EXPECT_NEAR(flat_arc_geometry.heading_of_p(1.0), kTheta1 + M_PI / 2.0,
              kVeryExact);
  // Checks the heading derivative of p at different values.
  EXPECT_NEAR(flat_arc_geometry.heading_dot_of_p(0.0), kDTheta, kVeryExact);
  EXPECT_NEAR(flat_arc_geometry.heading_dot_of_p(0.5), kDTheta, kVeryExact);
  EXPECT_NEAR(flat_arc_geometry.heading_dot_of_p(1.0), kDTheta, kVeryExact);
}

// Checks the validity of the surface for different lateral bounds and
// geometries.
GTEST_TEST(MultilaneArcRoadCurve, IsValidTest) {
  const Vector2<double> kCenter(0.0, 0.0);
  const double kRadius = 10.0;
  const double kTheta0 = 0.0;
  const double kTheta1 = 2.0 * M_PI;
  const double kDTheta1 = kTheta1 - kTheta0;
  const double kTheta2 = -2.0 * M_PI;
  const double kDTheta2 = kTheta2 - kTheta0;
  const CubicPolynomial zp;
  const CubicPolynomial constant_superelevation(M_PI / 4.0, 0.0, 0.0, 0.0);
  const api::RBounds lateral_bounds(-kRadius * 0.5, kRadius * 0.5);
  const api::RBounds large_lateral_bounds(-kRadius * 1.5, kRadius * 1.5);
  const api::RBounds critical_cone_lateral_bounds(
      -kRadius / std::cos(M_PI / 4.0), kRadius / std::cos(M_PI / 4.0));
  const api::HBounds height_bounds(0.0, 10.0);
  // Checks over a flat arc surface.
  const ArcRoadCurve flat_arc_geometry(kCenter, kRadius, kTheta0, kDTheta1, zp,
                                      zp);
  EXPECT_TRUE(flat_arc_geometry.IsValid(lateral_bounds, height_bounds));
  EXPECT_FALSE(flat_arc_geometry.IsValid(large_lateral_bounds, height_bounds));
  // Checks over a right handed cone.
  const ArcRoadCurve right_handed_cone_geometry(
      kCenter, kRadius, kTheta0, kDTheta1, zp, constant_superelevation);
  EXPECT_TRUE(
      right_handed_cone_geometry.IsValid(lateral_bounds, height_bounds));
  EXPECT_FALSE(
      right_handed_cone_geometry.IsValid(large_lateral_bounds, height_bounds));
  EXPECT_FALSE(right_handed_cone_geometry.IsValid(critical_cone_lateral_bounds,
                                                  height_bounds));
  // Checks over a left handed cone.
  const ArcRoadCurve left_handed_cone_geometry(
      kCenter, kRadius, kTheta0, kDTheta2, zp, constant_superelevation);
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(lateral_bounds, height_bounds));
  EXPECT_FALSE(
      left_handed_cone_geometry.IsValid(large_lateral_bounds, height_bounds));
  EXPECT_FALSE(left_handed_cone_geometry.IsValid(critical_cone_lateral_bounds,
                                                 height_bounds));
}

// Checks the ToCurve frame coordinate conversion for different points in world
// coordinates.
GTEST_TEST(MultilaneArcRoadCurve, ToCurveFrameTest) {
  const Vector2<double> kCenter(10.0, 10.0);
  const double kRadius = 10.0;
  const double kTheta0 = M_PI / 4.0;
  const double kTheta1 = 3.0 * M_PI / 4.0;
  const double kDTheta = kTheta1 - kTheta0;
  const double kVeryExact = 1e-12;
  const CubicPolynomial zp;
  const api::RBounds lateral_bounds(-5.0, 5.0);
  const api::HBounds height_bounds(0.0, 10.0);

  const ArcRoadCurve flat_arc_geometry(kCenter, kRadius, kTheta0, kDTheta, zp,
                                      zp);
  // Checks points over the composed curve.
  EXPECT_TRUE(CompareMatrices(
      flat_arc_geometry.ToCurveFrame(
          Vector3<double>(kCenter(0) + kRadius * std::cos(kTheta0),
                          kCenter(1) + kRadius * std::sin(kTheta0), 0.0),
          lateral_bounds, height_bounds),
      Vector3<double>(0.0, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_arc_geometry.ToCurveFrame(
          Vector3<double>(
              kCenter(0) + kRadius * std::cos(kTheta0 + kDTheta / 2.0),
              kCenter(1) + kRadius * std::sin(kTheta0 + kDTheta / 2.0), 0.0),
          lateral_bounds, height_bounds),
      Vector3<double>(kRadius * kDTheta * 0.5, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_arc_geometry.ToCurveFrame(
          Vector3<double>(kCenter(0) + kRadius * std::cos(kTheta1),
                          kCenter(1) + kRadius * std::sin(kTheta1), 0.0),
          lateral_bounds, height_bounds),
      Vector3<double>(kRadius * kDTheta, 0.0, 0.0), kVeryExact));
  // Checks with lateral and vertical deviations.
  EXPECT_TRUE(CompareMatrices(
      flat_arc_geometry.ToCurveFrame(
          Vector3<double>(
              kCenter(0) + (kRadius + 1.0) * std::cos(kTheta0 + M_PI / 8.0),
              kCenter(1) + (kRadius + 1.0) * std::sin(kTheta0 + M_PI / 8.0),
              6.0),
          lateral_bounds, height_bounds),
      Vector3<double>(kRadius * (M_PI / 8.0), -1.0, 6.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_arc_geometry.ToCurveFrame(
          Vector3<double>(
              kCenter(0) +
                  (kRadius - 2.0) *
                      std::cos(kTheta0 + kDTheta / 2.0 + M_PI / 8.0),
              kCenter(1) +
                  (kRadius - 2.0) *
                      std::sin(kTheta0 + kDTheta / 2.0 + M_PI / 8.0),
              3.0),
          lateral_bounds, height_bounds),
      Vector3<double>(kRadius * (kDTheta / 2.0 + M_PI / 8.0), 2.0, 3.0),
      kVeryExact));
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
