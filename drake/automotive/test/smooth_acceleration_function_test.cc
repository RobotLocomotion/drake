#include "drake/automotive/smooth_acceleration_function.h"

#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {
namespace {

template <typename T>
void do_test() {
  // A positive desired acceleration when the current velocity is not close to
  // the max velocity should result in a positive acceleration.
  T result{};
  calc_smooth_acceleration(
          T(10.),                     // desired_acceleration
          T(10.),                     // current_velocity
          T(50.),                     // max velocity
          T(4.), &result);            // velocity_limit_kp
  EXPECT_GT(result, T(0));

  // A positive desired acceleration when the current velocity is equal to the
  // max velocity should result in zero acceleration.
  calc_smooth_acceleration(
          T(10.),                     // desired_acceleration
          T(50.),                     // current_velocity
          T(50.),                     // max velocity
          T(4.), &result);            // velocity_limit_kp
  EXPECT_EQ(result, T(0));

  // A negative desired acceleration when the current velocity is far from zero
  // should result in negative acceleration.
  calc_smooth_acceleration(
          T(-10.),                     // desired_acceleration
          T(50.),                     // current_velocity
          T(50.),                     // max velocity
          T(4.), &result);            // velocity_limit_kp
  EXPECT_LT(result, T(0));

  // A desired acceleration of zero result in zero acceleration regardless of
  // the current velocity.
  const std::vector<double> test_velocities{0, 10, 20, 30, 40, 50};
  for (const auto& velocity : test_velocities) {
    calc_smooth_acceleration(
            T(0.),                     // desired_acceleration
            T(velocity),               // current_velocity
            T(50.),                    // max velocity
            T(4.), &result);           // velocity_limit_kp
    EXPECT_EQ(result, T(0));
  }
}

GTEST_TEST(SmoothAccelerationFunctionTest, TestDoubleType) {
  do_test<double>();
}

GTEST_TEST(SmoothAccelerationFunctionTest, TestAutoDiffXType) {
  do_test<drake::AutoDiffXd>();
}

}  // namespace
}  // namespace automotive
}  // namespace drake
