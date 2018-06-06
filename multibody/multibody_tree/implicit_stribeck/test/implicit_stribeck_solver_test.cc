#include "drake/multibody/multibody_tree/implicit_stribeck/implicit_stribeck_solver.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

namespace drake {
namespace multibody {
namespace implicit_stribeck {
namespace {

class DirectionLimiter : public ::testing::Test {
 public:
  void SetUp() override {
  }

 protected:
  // Limiter parameters. See LimitDirectionChange for further details.
  const double v_stribeck = 1.0e-4;  // m/s
  const double theta_max = M_PI / 6.0;  // radians.
  const double cos_min = std::cos(theta_max);
  const double tolerance = 0.01;  // Dimensionless. A factor of v_stribeck.
  // Tolerance to perform comparisons close to machine precission.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
};

TEST_F(DirectionLimiter, StraightCrossThroughZero) {
  const Vector2<double> vt(0.1, 0.05);
  const Vector2<double> dvt(-0.3, -0.15);  // dvt = -3 * vt.

  const double alpha = internal::LimitDirectionChange<double>::run(
      vt, dvt, cos_min, v_stribeck, tolerance);

  PRINT_VAR(alpha);

  // Since the change crosses zero exactly, we expect
  // v_alpha = v + alpha * dv = v/‖v‖⋅vₛ/2.
  const Vector2<double> vt_alpha_expected = vt.normalized() * v_stribeck / 2.0;

  const Vector2<double> vt_alpha = vt + alpha * dvt;

  EXPECT_TRUE(CompareMatrices(
      vt_alpha, vt_alpha_expected, kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake

