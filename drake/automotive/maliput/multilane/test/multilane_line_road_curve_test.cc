/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/line_road_geometry.h"
/* clang-format on */

#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {

// Checks line reference curve interpolations, derivatives, and lengths.
GTEST_TEST(MultilaneLineRoadCurve, LineRoadCurve) {
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const CubicPolynomial zp;
  const double kHeading = std::atan2(kDirection.y(), kDirection.x());
  const double kHeadingDerivative = 0.0;
  const double kVeryExact = 1e-12;

  const LineRoadCurve flat_line_geometry(kOrigin, kDirection, zp, zp);
  // Checks the length.
  EXPECT_NEAR(flat_line_geometry.length(),
              std::sqrt(kDirection.x() * kDirection.x() +
                        kDirection.y() * kDirection.y()),
              kVeryExact);
  // Check the interpolation of p at different p values.
  EXPECT_TRUE(
      CompareMatrices(flat_line_geometry.xy_of_p(0.0), kOrigin, kVeryExact));
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_of_p(0.5),
                              kOrigin + 0.5 * kDirection, kVeryExact));
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_of_p(1.0),
                              kOrigin + kDirection, kVeryExact));
  // Check the derivative of p at different p values.
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_dot_of_p(0.0), kDirection,
                              kVeryExact));
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_dot_of_p(0.5), kDirection,
                              kVeryExact));
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_dot_of_p(1.0), kDirection,
                              kVeryExact));
  // Check the heading at different p values.
  EXPECT_NEAR(flat_line_geometry.heading_of_p(0.0), kHeading, kVeryExact);
  EXPECT_NEAR(flat_line_geometry.heading_of_p(0.5), kHeading, kVeryExact);
  EXPECT_NEAR(flat_line_geometry.heading_of_p(1.0), kHeading, kVeryExact);
  // Check the heading derivative of p at different p values.
  EXPECT_NEAR(flat_line_geometry.heading_dot_of_p(0.0), kHeadingDerivative,
              kVeryExact);
  EXPECT_NEAR(flat_line_geometry.heading_dot_of_p(0.5), kHeadingDerivative,
              kVeryExact);
  EXPECT_NEAR(flat_line_geometry.heading_dot_of_p(1.0), kHeadingDerivative,
              kVeryExact);
}

// Checks that LineRoadCurve::IsValid() returns true.
GTEST_TEST(MultilaneLineRoadCurve, IsValidTest) {
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const CubicPolynomial zp;
  const api::Rbounds lateral_bounds(-10.0, 10.0);
  const api::HBounds elevation_bounds(0.0, 10.0);
  const LineRoadCurve flat_line_geometry(kOrigin, kDirection, zp, zp);
  EXPECT_TRUE(flat_line_geometry.IsValid(lateral_bounds, elevation_bounds));
}

// Checks the validity of the surface for different lateral bounds and
// geometries.
GTEST_TEST(MultilaneLineRoadCurve, ToCurveFrameTest) {
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const double kVeryExact = 1e-12;
  const CubicPolynomial zp;
  const api::RBounds lateral_bounds(-10.0, 10.0);
  const api::HBounds elevation_bounds(0.0, 10.0);

  const LineRoadCurve flat_line_geometry(kOrigin, kDirection, zp, zp);
  // Checks over the base line.
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(10.0, 10.0, 0.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(0.0, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(20.0, 20.0, 0.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(std::sqrt(2) * 10.0, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(15.0, 15.0, 0.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(std::sqrt(2) * 5.0, 0.0, 0.0), kVeryExact));
  // Check with lateral and vertical deviation.
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(11.0, 12.0, 5.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(2.12132034355964, 0.707106781186547, 5.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(11.0, 10.0, 7.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(0.707106781186547, -0.707106781186547, 7.0), kVeryExact));
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
