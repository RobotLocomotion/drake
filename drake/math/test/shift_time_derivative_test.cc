#include "drake/math/shift_time_derivative.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// Given an angular velocity w_AB of a frame B in A, verifies that the time
// derivative of this angular velocity is the same in both frames. That is:
//   d_A(w_AB)/dt = d_B(w_AB)/dt
// Input parameters:
//   w_AB: angular velocity of B in A.
//   DB_w_AB: Time derivative of w_AB in the B frame.
void ShiftTimeDerivativeOfAngularVelocity(
    const Vector3d& w_AB, const Vector3d& DB_w_AB) {
  const double kAbsoluteTolerance = 2 * std::numeric_limits<double>::epsilon();
  Vector3d DA_w_AB = ShiftTimeDerivative(w_AB, DB_w_AB, w_AB);
  EXPECT_TRUE(CompareMatrices(DA_w_AB, DB_w_AB, kAbsoluteTolerance,
                              MatrixCompareType::absolute));
}

GTEST_TEST(ShiftTimeDerivative, OnAngularVelocity) {
  // Make a number of random tests.
  ShiftTimeDerivativeOfAngularVelocity(
      Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
  ShiftTimeDerivativeOfAngularVelocity(
      Vector3d(-1.0, 0.0, 2.0), Vector3d(3.0, 1.0, 4.0));
}

// Consider a carousel rotating with angular velocity w_WC, where W is the
// (inertial) world frame and C is the (non-inertial) carousel frame with its
// origin Co at the center of the carousel.
// Now consider a stationary wooden horse at location p_CoHo from the center of
// the carousel frame Co. Since the horse is stationary in C we know
// that its velocity in frame C is zero, that is v_CHo = DC_p_CoHo = 0.
// This unit test verifies we can compute the velocity v_WHo in the world frame
// by shifting the time derivative as:
//   v_WHo = DW_p_CoHo = DC_p_CoHo + w_WC x p_CoHo
//
// Note: The carousel is on the x-y plane with an angular velocity around the
//       z-axis.
//
// Input parameters:
//   horse_radius: the radial position of the horse in the carousel.
//   theta: the rotation angle of the carousel. for theta = 0 C coincides with W
//          and C rotates according to the right-hand-rule around the z-axis.
//   theta_dot: the rate of change of theta.
void HorseOnCarousel(double horse_radius,
                     double theta, double theta_dot) {
  using std::cos;
  using std::sin;
  const double kAbsoluteTolerance = 2 * std::numeric_limits<double>::epsilon();

  const Vector3d w_WC = theta_dot * Vector3d::UnitZ();
  const Matrix3d R_WC = AngleAxisd(theta, Vector3d::UnitZ()).matrix();
  // Horse position in the carousel frame.
  const Vector3d& p_CoHo_C = horse_radius * Vector3d::UnitX();

  // Re-express horse position in the world frame.
  Vector3d p_CoHo_W = R_WC * p_CoHo_C;

  Vector3d v_WHo = ShiftTimeDerivative(
      p_CoHo_W, Vector3d::Zero() /* DC_p_CoHo */, w_WC);

  // Compute the expected value.
  Vector3d v_WHo_expected =
      horse_radius * theta_dot * Vector3d(-sin(theta), cos(theta), 0);

  EXPECT_TRUE(CompareMatrices(v_WHo, v_WHo_expected, kAbsoluteTolerance,
                              MatrixCompareType::absolute));
}

GTEST_TEST(ShiftTimeDerivative, HorseOnCarousel) {
  // Make a number of random tests.
  HorseOnCarousel(1.5, 0.0, 3.0);
  HorseOnCarousel(1.5, M_PI / 3.0, 3.0);
  HorseOnCarousel(3.0, 7.0 * M_PI / 8.0, -1.0);
}

}  // namespace
}  // namespace math
}  // namespace drake
