/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/line_road_curve.h"
/* clang-format on */

#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace maliput {
namespace multilane {

class MultilaneLineRoadCurveTest : public ::testing::Test {
 protected:
  const Vector2<double> kOrigin{10.0, 10.0};
  const Vector2<double> kDirection{10.0, 10.0};
  const CubicPolynomial zp;
  const double kHeading{M_PI / 4.0};
  const double kHeadingDerivative{0.0};
  const double kVeryExact{1e-12};
  const double kRMin{-10.0};
  const double kRMax{10.0};
  const api::HBounds elevation_bounds{0.0, 10.0};
};

// Checks line reference curve interpolations, derivatives, and lengths.
TEST_F(MultilaneLineRoadCurveTest, LineRoadCurve) {
  const LineRoadCurve dut(kOrigin, kDirection, zp, zp);
  // Checks the length.
  EXPECT_NEAR(dut.p_scale(),
              std::sqrt(kDirection.x() * kDirection.x() +
                        kDirection.y() * kDirection.y()),
              kVeryExact);
  // Check the evaluation of xy at different p values.
  EXPECT_TRUE(
      CompareMatrices(dut.xy_of_p(0.0), kOrigin, kVeryExact));
  EXPECT_TRUE(CompareMatrices(dut.xy_of_p(0.5),
                              kOrigin + 0.5 * kDirection, kVeryExact));
  EXPECT_TRUE(CompareMatrices(dut.xy_of_p(1.0),
                              kOrigin + kDirection, kVeryExact));
  // Check the derivative of xy with respect to p at different p values.
  EXPECT_TRUE(CompareMatrices(dut.xy_dot_of_p(0.0), kDirection,
                              kVeryExact));
  EXPECT_TRUE(CompareMatrices(dut.xy_dot_of_p(0.5), kDirection,
                              kVeryExact));
  EXPECT_TRUE(CompareMatrices(dut.xy_dot_of_p(1.0), kDirection,
                              kVeryExact));
  // Check the heading at different p values.
  EXPECT_NEAR(dut.heading_of_p(0.0), kHeading, kVeryExact);
  EXPECT_NEAR(dut.heading_of_p(0.5), kHeading, kVeryExact);
  EXPECT_NEAR(dut.heading_of_p(1.0), kHeading, kVeryExact);
  // Check the heading derivative of p at different p values.
  EXPECT_NEAR(dut.heading_dot_of_p(0.0), kHeadingDerivative, kVeryExact);
  EXPECT_NEAR(dut.heading_dot_of_p(0.5), kHeadingDerivative, kVeryExact);
  EXPECT_NEAR(dut.heading_dot_of_p(1.0), kHeadingDerivative, kVeryExact);
}

// Checks that LineRoadCurve::IsValid() returns true.
TEST_F(MultilaneLineRoadCurveTest, IsValidTest) {
  const LineRoadCurve dut(kOrigin, kDirection, zp, zp);
  EXPECT_TRUE(dut.IsValid(kRMin, kRMax, elevation_bounds));
}

// Checks the validity of the surface for different lateral bounds and
// geometries.
TEST_F(MultilaneLineRoadCurveTest, ToCurveFrameTest) {
  const LineRoadCurve dut(kOrigin, kDirection, zp, zp);
  // Checks over the base line.
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(Vector3<double>(10.0, 10.0, 0.0), kRMin, kRMax,
                       elevation_bounds),
      Vector3<double>(0.0, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(Vector3<double>(20.0, 20.0, 0.0), kRMin, kRMax,
                       elevation_bounds),
      Vector3<double>(1., 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(Vector3<double>(15.0, 15.0, 0.0), kRMin, kRMax,
                       elevation_bounds),
      Vector3<double>(0.5, 0.0, 0.0), kVeryExact));
  // Check with lateral and vertical deviation.
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(Vector3<double>(11.0, 12.0, 5.0), kRMin, kRMax,
                       elevation_bounds),
      Vector3<double>(0.15, 0.707106781186547, 5.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(Vector3<double>(11.0, 10.0, 7.0), kRMin, kRMax,
                       elevation_bounds),
      Vector3<double>(0.05, -0.707106781186547, 7.0), kVeryExact));
}

// Checks that p_scale(), p_from_s() and s_from_p() with constant superelevation
// polynomial and up to linear elevation polynomial behave properly.
TEST_F(MultilaneLineRoadCurveTest, OffsetTest) {
  const std::vector<double> r_vector{-10., 0., 10.};
  const std::vector<double> p_vector{0., 0.1, 0.2, 0.5, 0.7, 1.0};

  // Checks for flat LineRoadCurve.
  const LineRoadCurve flat_dut(kOrigin, kDirection, zp, zp);
  EXPECT_DOUBLE_EQ(flat_dut.p_scale(), kDirection.norm());
  // Evaluates inverse function for different path length and offset values.
  for (double r : r_vector) {
    for (double p : p_vector) {
      EXPECT_DOUBLE_EQ(flat_dut.p_from_s(p * kDirection.norm(), r), p);
    }
  }
  // Evaluates the path length integral for different offset values.
  for (double r : r_vector) {
    for (double p : p_vector) {
      EXPECT_DOUBLE_EQ(flat_dut.s_from_p(p, r), p * kDirection.norm());
    }
  }

  // Checks for linear elevation applied to the LineRoadCurve.
  const double slope = 10. / kDirection.norm();
  const CubicPolynomial linear_elevation(10., slope, 0., 0.);
  const LineRoadCurve elevated_dut(kOrigin, kDirection, linear_elevation, zp);
  EXPECT_DOUBLE_EQ(elevated_dut.p_scale(), kDirection.norm());
  // Evaluates inverse function and path length integral for different values of
  // p and r lateral offsets.
  for (double r : r_vector) {
    for (double p : p_vector) {
      const double s = p * kDirection.norm() * std::sqrt(1. + slope * slope);
      EXPECT_DOUBLE_EQ(elevated_dut.p_from_s(s, r), p);
      EXPECT_DOUBLE_EQ(elevated_dut.s_from_p(p, r), s);
    }
  }
}

// Checks World function evaluation at different values of [p, r, h].
TEST_F(MultilaneLineRoadCurveTest, WorldFunction) {
  const std::vector<double> r_vector{-10., 0., 10.};
  const std::vector<double> p_vector{0., 0.1, 0.2, 0.5, 0.7, 1.0};
  const std::vector<double> h_vector{-5., 0., 5.};

  // Checks for a flat curve.
  const LineRoadCurve flat_dut(kOrigin, kDirection, zp, zp);
  const Vector3<double> p_versor =
      Vector3<double>(kDirection.x(), kDirection.y(), 0.).normalized();
  const Rot3 flat_rotation(0., 0., kHeading);
  const Vector3<double> kGeoOrigin(kOrigin.x(), kOrigin.y(), 0.);
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Vector3<double> geo_position = kGeoOrigin +
                                             p * kDirection.norm() * p_versor +
                                             flat_rotation.apply({0., r, h});
        EXPECT_TRUE(CompareMatrices(flat_dut.W_of_prh(p, r, h), geo_position,
                                    kVeryExact));
      }
    }
  }

  // Checks for linearly elevated curve.
  const double kElevationSlope = 15. / kDirection.norm();
  const double kElevationOffset = 10. / kDirection.norm();
  const CubicPolynomial linear_elevation(kElevationOffset, kElevationSlope, 0.,
                                         0.);
  const LineRoadCurve elevated_dut(kOrigin, kDirection, linear_elevation, zp);
  // Computes the rotation along the RoadCurve.
  const Rot3 elevated_rotation(0., -std::atan(linear_elevation.f_dot_p(0.)),
                               kHeading);
  const Vector3<double> z_vector(0., 0., kDirection.norm());
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Vector3<double> geo_position =
            kGeoOrigin + p * kDirection.norm() * p_versor +
            linear_elevation.f_p(p) * z_vector +
            elevated_rotation.apply({0., r, h});
        EXPECT_TRUE(CompareMatrices(elevated_dut.W_of_prh(p, r, h),
                                    geo_position, kVeryExact));
      }
    }
  }

  // Checks for a curve with constant non zero superelevation.
  const double kSuperelevationOffset = M_PI / 4.;
  const CubicPolynomial constant_offset_superelevation(
      kSuperelevationOffset / kDirection.norm(), 0., 0., 0.);
  const LineRoadCurve superelevated_dut(kOrigin, kDirection, zp,
                                        constant_offset_superelevation);
  const Rot3 superelevated_rotation(kSuperelevationOffset, 0., kHeading);
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Vector3<double> geo_position =
            kGeoOrigin + p * kDirection.norm() * p_versor +
            superelevated_rotation.apply({0., r, h});
        EXPECT_TRUE(CompareMatrices(superelevated_dut.W_of_prh(p, r, h),
                                    geo_position, kVeryExact));
      }
    }
  }
}

// Checks Orientation for different values of [p, r, h].
TEST_F(MultilaneLineRoadCurveTest, Orientation) {
  const std::vector<double> r_vector{-10., 0., 10.};
  const std::vector<double> p_vector{0., 0.1, 0.2, 0.5, 0.7, 1.0};
  const std::vector<double> h_vector{-5., 0., 5.};

  // Checks for a flat curve.
  const LineRoadCurve flat_dut(kOrigin, kDirection, zp, zp);
  const double kZeroRoll{0.};
  const double kZeroPitch{0.};
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Rot3 rotation = flat_dut.Orientation(p, r, h);
        EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
        EXPECT_NEAR(rotation.pitch(), kZeroPitch, kVeryExact);
        EXPECT_NEAR(rotation.yaw(), kHeading, kVeryExact);
      }
    }
  }

  // Checks for a linearly elevated curve.
  const double kElevationSlope = 15. / kDirection.norm();
  const double kElevationOffset = 10. / kDirection.norm();
  const CubicPolynomial linear_elevation(kElevationOffset, kElevationSlope, 0.,
                                         0.);
  const LineRoadCurve elevated_dut(kOrigin, kDirection, linear_elevation, zp);
  const double kLinearPitch{-std::atan(linear_elevation.f_dot_p(0.))};
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Rot3 rotation = elevated_dut.Orientation(p, r, h);
        EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
        EXPECT_NEAR(rotation.pitch(), kLinearPitch, kVeryExact);
        EXPECT_NEAR(rotation.yaw(), kHeading, kVeryExact);
      }
    }
  }

  // Checks for a curve with constant non zero superelevation.
  const double kSuperelevationOffset = M_PI / 4.;
  const CubicPolynomial constant_offset_superelevation(
      kSuperelevationOffset / kDirection.norm(), 0., 0., 0.);
  const LineRoadCurve superelevated_dut(kOrigin, kDirection, zp,
                                        constant_offset_superelevation);
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Rot3 rotation = superelevated_dut.Orientation(p, r, h);
        EXPECT_NEAR(rotation.roll(), kSuperelevationOffset, kVeryExact);
        EXPECT_NEAR(rotation.pitch(), kZeroPitch, kVeryExact);
        EXPECT_NEAR(rotation.yaw(), kHeading, kVeryExact);
      }
    }
  }
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
