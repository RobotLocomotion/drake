#include "drake/math/convert_time_derivative.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// Given an angular velocity w_AB of a frame B in A, verifies that the time
// derivative of this angular velocity is the same in both frames. That is:
//   ᴬd/dt(w_AB) = ᴮd/dt(w_AB)
// Input parameters:
//   w_AB: angular velocity of B in A.
//   DtB_w_AB: Time derivative of w_AB in the B frame.
void ConvertTimeDerivativeOfAngularVelocity(
    const Vector3d& w_AB, const Vector3d& DtB_w_AB) {
  const double kAbsoluteTolerance = 2 * std::numeric_limits<double>::epsilon();
  Vector3d DtA_w_AB = ConvertTimeDerivativeToOtherFrame(w_AB, DtB_w_AB, w_AB);
  EXPECT_TRUE(CompareMatrices(DtA_w_AB, DtB_w_AB, kAbsoluteTolerance,
                              MatrixCompareType::absolute));
}

GTEST_TEST(ConvertTimeDerivativeToOtherFrame, OnAngularVelocity) {
  // Make a number of random tests.
  ConvertTimeDerivativeOfAngularVelocity(
      Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
  ConvertTimeDerivativeOfAngularVelocity(
      Vector3d(-1.0, 0.0, 2.0), Vector3d(3.0, 1.0, 4.0));
}

// Consider a carousel rotating with angular velocity w_WC, where W is the
// (inertial) world frame and C is the (non-inertial) carousel frame with its
// origin Co at the center of the carousel.
// The carousel is on the x-y plane with an angular velocity around the z-axis.
// Now consider a wooden horse moving up and down (along the z-axis) at location
// p_CoHo from the center of the carousel frame Co. Since the horse moves up and
// down in C we know that its velocity in frame C is along the vertical z-axis,
// that is v_CHo = DtC_p_CoHo = swing_up_speed * zhat, with swing_up_speed the
// (signed) magnitude of v_CHo and zhat the z-axis versor.
// This unit test verifies we can compute the velocity v_WHo in the world frame
// as:
//   v_WHo = DtW_p_CoHo = DtC_p_CoHo + w_WC x p_CoHo
//
// Input parameters:
//   horse_radius: the radial position of the horse in the carousel.
//   theta: the rotation angle of the carousel. for theta = 0 C coincides with W
//          and C rotates according to the right-hand-rule around the z-axis.
//   theta_dot: the rate of change of theta.
//   swing_up_speed: the (signed) magnitude of the horse's up and down motion.
void HorseOnCarousel(double horse_radius,
                     double theta, double theta_dot, double swing_up_speed) {
  using std::cos;
  using std::sin;
  const double kAbsoluteTolerance = 2 * std::numeric_limits<double>::epsilon();

  const Vector3d w_WC = theta_dot * Vector3d::UnitZ();
  const Matrix3d R_WC = AngleAxisd(theta, Vector3d::UnitZ()).matrix();
  // Horse position in the carousel frame.
  const Vector3d p_CoHo_C = horse_radius * Vector3d::UnitX();

  // Re-express horse position in the world frame.
  const Vector3d p_CoHo_W = R_WC * p_CoHo_C;

  // Horse velocity in the C frame.
  const Vector3d v_CHo = swing_up_speed * Vector3d::UnitZ();

  const Vector3d v_WHo = ConvertTimeDerivativeToOtherFrame(
      p_CoHo_W, v_CHo /* DtC_p_CoHo */, w_WC);

  // Compute the expected value. Note that since rotation is only along the
  // z-axis we can directly add the contribution v_CHo_z.
  const Vector3d v_WHo_expected =
      horse_radius * theta_dot * Vector3d(-sin(theta), cos(theta), 0) + v_CHo;

  EXPECT_TRUE(CompareMatrices(v_WHo, v_WHo_expected, kAbsoluteTolerance,
                              MatrixCompareType::absolute));
}

GTEST_TEST(ConvertTimeDerivativeToOtherFrame, HorseOnCarousel) {
  // Make a number of random tests.
  HorseOnCarousel(1.5, 0.0, 3.0, -1.0);
  HorseOnCarousel(1.5, M_PI / 3.0, 3.0, 3.5);
  HorseOnCarousel(3.0, 7.0 * M_PI / 8.0, -1.0, 2.0);
}

}  // namespace
}  // namespace math
}  // namespace drake
