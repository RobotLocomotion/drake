#include "drake/multibody/multibody_tree/implicit_stribeck/implicit_stribeck_solver.h"

#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {
namespace {

// A test fixture to test LimitDirectionChange for a very standard configuration
// of parameters.
class DirectionLimiter : public ::testing::Test {
 protected:
  // Helper to make a 2D rotation matrix.
  static Matrix2<double> Rotation(double theta) {
    return Eigen::Rotation2D<double>(theta).toRotationMatrix();
  }

  // Limiter parameters. See LimitDirectionChange for further details.
  const double v_stribeck = 1.0e-4;  // m/s
  const double theta_max = M_PI / 6.0;  // radians.
  const double cos_min = std::cos(theta_max);
  const double tolerance = 0.01;  // Dimensionless. A factor of v_stribeck.
  // Tolerance to perform comparisons close to machine precision.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
};

// Verify results when vt and dvt are exactly zero.
TEST_F(DirectionLimiter, ZeroVandZeroDv) {
  const Vector2<double> vt = Vector2<double>::Zero();
  const Vector2<double> dvt = Vector2<double>::Zero();
  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Verify implementation when vt = 0 and the update dvt takes the velocity
// to within the Stribeck circle.
TEST_F(DirectionLimiter, ZeroVtoWithinStribeckCircle) {
  const Vector2<double> vt = Vector2<double>::Zero();
  const Vector2<double> dvt = Vector2<double>(-0.5, 0.7) * v_stribeck;
  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Perfect stiction (vt = 0) to sliding.
TEST_F(DirectionLimiter, ZeroVtoOutsideStribeckCircle) {
  const Vector2<double> vt = Vector2<double>::Zero();
  const Vector2<double> dvt = Vector2<double>(0.3, -0.1);
  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);
  const Vector2<double> vt_alpha_expected = dvt.normalized() * v_stribeck / 2.0;
  const Vector2<double> vt_alpha = vt + alpha * dvt;
  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// Sliding to perfect stiction with vt = 0.
TEST_F(DirectionLimiter, OutsideStribeckCircletoZero) {
  const Vector2<double> vt = Vector2<double>(0.3, -0.1);
  const Vector2<double> dvt = -vt;
  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);
  // LimitDirectionChange does not allow changes from outside the stribeck
  // circle (where friction is constant) to exactly zero velociy, since this
  // would imply leaving the solver in a state where gradients are negligible
  // (not strong). The solver can recover from this, but placing the velocity
  // within the Stribeck circle in the direction of the intial v, helps the
  // iterative process even more.
  const Vector2<double> vt_alpha = vt + alpha * dvt;
  const Vector2<double> vt_alpha_expected = vt.normalized() * v_stribeck / 2.0;
  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// A vt that lies outside the Stribeck circle lies somewhere within the circle
// after the update v_alpha = v + dv, alpha = 1. Since gradients are strong
// within this region, the limiter allows it.
TEST_F(DirectionLimiter, OutsideStribeckToWithinCircle) {
  const Vector2<double> vt = Vector2<double>(1.2, 0.4);
  const Vector2<double> vt_alpha_expected =
      Vector2<double>(-0.3, 0.45) * v_stribeck;
  const Vector2<double> dvt = vt_alpha_expected - vt;
  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Similar to ZeroVtoOutsideStribeckCircle, a velocity vt within the Stribeck
// region (but not to zero) is updated to a sliding configuration. Since vt
// falls in a region of strong gradients, the limiter allows it.
TEST_F(DirectionLimiter, WithinStribeckCircleToOutsideStribeckCircle) {
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7) * v_stribeck;
  const Vector2<double> dvt = Vector2<double>(0.9, -0.3);
  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Similar to test ZeroVtoOutsideStribeckCircle, but vt is not exactly zero
// but negligibly small with norm/v_stribeck < tolerance.
TEST_F(DirectionLimiter, StictionToSliding) {
  const Vector2<double> vt =
      Vector2<double>(-0.5, 0.3) * v_stribeck * tolerance;
  const Vector2<double> dvt(0.3, 0.15);

  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);

  // For this case LimitDirectionChange neglects the very small initial vt
  // (since we always have tolerance << 1.0) so that:
  // vα = vt + αΔvt ≈ αΔvt = Δvt/‖Δvt‖⋅vₛ/2.
  // Therefore we expect α = 1 / ‖Δvt‖⋅vₛ/2.
  double alpha_expected = 1.0 / dvt.norm() * v_stribeck / 2.0;

  EXPECT_NEAR(alpha, alpha_expected, kTolerance);
}

// Verifies that the limiter allows negligible changes dvt with alpha = 1.
TEST_F(DirectionLimiter, VerySmallDeltaV) {
  const Vector2<double> vt(0.1, 0.05);
  const Vector2<double> dvt =
      Vector2<double>(-0.5, 0.3) * v_stribeck * tolerance;
  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);
  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// A very specific scenario when the update vt + dvt crosses zero exactly.
// This is a very common case in 1D-like problems and therefore it does happen
// often.
TEST_F(DirectionLimiter, StraightCrossThroughZero) {
  const Vector2<double> vt(0.1, 0.05);
  const Vector2<double> dvt(-0.3, -0.15);  // dvt = -3 * vt.

  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);

  // Since the change crosses zero exactly, we expect
  // v_alpha = v + alpha * dv = v/‖v‖⋅vₛ/2.
  const Vector2<double> vt_alpha_expected = vt.normalized() * v_stribeck / 2.0;

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// Test a direction change from vt to v1 = vt + dvt that crosses through the
// Stribeck circle. In this case the limiter will find a scalar 0< alpha < 1
// such that v_alpha = vt + alpha * dvt is the closest vector to the origin.
TEST_F(DirectionLimiter, CrossStribeckCircleFromTheOutside) {
  // We construct a v_alpha expected to be within the Stribeck circle.
  const Vector2<double> vt_alpha_expected =
      Vector2<double>(0.3, 0.2) * v_stribeck;

  // A unit vector normal to vt_alpha_expected.
  const Vector2<double> vt_normal =
      Vector2<double>(vt_alpha_expected(1), -vt_alpha_expected(0)).normalized();

  // Construct a vt away from the circle in a large magnitude (>>v_stribec) in
  // the direction normal to vt_alpha_expected.
  const Vector2<double> vt = vt_alpha_expected + 0.5 * vt_normal;

  // Construct a v1 away from the circle in a large magnitude (>>v_stribec) in
  // the direction normal to vt_alpha_expected. This time in the opposite
  // direction to that of vt.
  const Vector2<double> v1 = vt_alpha_expected - 0.8 * vt_normal;

  // Velocity change from vt to v1 (this is what the implicit Stribeck iteration
  // would compute).
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);

  // Verify the result from the limiter.
  const Vector2<double> vt_alpha = vt + alpha * dvt;
  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

// Tests the limiter for a case in which both vt and v1 = vt + dvt are both
// outside the Stribeck circle. In this test, the angle formed by vt and v1 is
// smaller than theta_max and the limiter allows it, i.e. it returns alpha = 1.
TEST_F(DirectionLimiter, ChangesOutsideTheStribeckCircle) {
  // an angle smaller that theta_max = M_PI / 6.
  const double theta = M_PI / 8.0;

  // A vt outside the Stribeck circle
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A v1 at forming an angle of theta with vt.
  const Vector2<double> v1 = Rotation(theta) * vt;

  // Delta dvt that takes vt to v1.
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);

  EXPECT_NEAR(alpha, 1.0, kTolerance);
}

// Tests the limiter for a case in which both vt and v1 = vt + dvt are both
// outside the Stribeck circle. In this test, the angle formed by vt and v1 is
// larger than theta_max and the limiter will limit the change to a
// vt_alpha = vt + alpha * dvt so that vt_alpha forms an angle theta_max with
// vt.
// Internal detail note: the limiter computed two roots with different signs and
// returns the positive root.
TEST_F(DirectionLimiter, ChangesOutsideTheStribeckCircle_LargeTheta) {
  // Angle formed by v1 and vt, an angle larger that theta_max = M_PI / 6.
  const double theta1 = M_PI / 3.0;

  // A vt outside the Stribeck circle
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A v1 at forming an angle of theta with vt.
  const Vector2<double> v1 = Rotation(theta1) * vt;

  // Delta dvt that takes vt to v1.
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  // Compute the angle between vt_alpha and vt.
  double cos_theta = vt.dot(vt_alpha)/vt.norm()/vt_alpha.norm();
  double theta = std::acos(cos_theta);

  // Verify the result was limited to form an angle theta_max.
  EXPECT_NEAR(theta, theta_max, kTolerance);
}

// Tests the limiter for a case in which both vt and v1 = vt + dvt are both
// outside the Stribeck circle. In this test, the angle formed by vt and v1 is
// MUCH larger than theta_max and the limiter will limit the change to a
// vt_alpha = vt + alpha * dvt so that vt_alpha forms an angle theta_max with
// vt.
// Internal detail note: the limiter computed two positive roots and returns
// the smallest of the two.
TEST_F(DirectionLimiter, ChangesOutsideTheStribeckCircle_VeryLargeTheta) {
  // Angle formed by v1 and vt, an angle larger that theta_max = M_PI / 6.
  const double theta1 = 5.0 * M_PI / 6.0;

  // A vt outside the Stribeck circle
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A v1 at forming an angle of theta with vt.
  const Vector2<double> v1 = Rotation(theta1) * vt;

  // Delta dvt that takes vt to v1.
  const Vector2<double> dvt = v1 - vt;

  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  // Compute the angle between vt_alpha and vt.
  double cos_theta = vt.dot(vt_alpha)/vt.norm()/vt_alpha.norm();
  double theta = std::acos(cos_theta);

  // Verify the result was limited to form an angle theta_max.
  EXPECT_NEAR(theta, theta_max, kTolerance);
}

// This is a very degenerate case in which the angle formed by vt and dvt
// equals (whithin machine epsilon) to theta_max. It so happens than in this
// case there is a single solution to the quadratic equation solved by the
// limiter (the equation becomes linear).
// Even though this will rarely (or impossibly) happen, we make sure we consider
// it for maximum robustness.
TEST_F(DirectionLimiter, ChangesOutsideTheStribeckCircle_SingleSolution) {
  // A vt outside the Stribeck circle
  const Vector2<double> vt = Vector2<double>(-0.5, 0.7);

  // A dvt forming an angle of theta_max with vt not necessarily having the same
  // magnitude as vt.
  const Vector2<double> dvt = -3.0 * Rotation(theta_max) * vt;

  // Before proceeding with the test, assert that we are in a case where theta1
  // is larger than theta_max.
  const Vector2<double> v1 = vt + dvt;
  double theta1 = std::cos(vt.dot(v1)/vt.norm()/v1.norm());
  ASSERT_GT(theta1, theta_max);

  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  // Compute the angle between vt_alpha and vt.
  double cos_theta = vt.dot(vt_alpha)/vt.norm()/vt_alpha.norm();
  double theta = std::acos(cos_theta);

  // Verify the result was limited to form an angle theta_max.
  EXPECT_NEAR(theta, theta_max, kTolerance);
}

}  // namespace
}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake

