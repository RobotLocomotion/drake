#include "drake/automotive/calc_smooth_acceleration.h"

#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"

namespace drake {

using symbolic::Expression;
using symbolic::Environment;
using symbolic::Variable;

namespace automotive {
namespace {

template <typename T>
void do_test() {
  // A positive desired acceleration when the current velocity is not close to
  // the max velocity should result in a positive acceleration.
  EXPECT_GT(
      calc_smooth_acceleration(
          T(10.),                     // desired_acceleration
          T(50.),                     // max velocity
          T(4.),                      // velocity_limit_kp
          T(10.)),                    // current_velocity
      T(0));

  // A positive desired acceleration when the current velocity is equal to the
  // max velocity should result in zero acceleration.
  EXPECT_EQ(
      calc_smooth_acceleration(
          T(10.),                     // desired_acceleration
          T(50.),                     // max velocity
          T(4.),                      // velocity_limit_kp
          T(50.)),                    // current_velocity
      T(0));

  // A negative desired acceleration when the current velocity is far from zero
  // should result in negative acceleration.
  EXPECT_LT(
      calc_smooth_acceleration(
          T(-10.),                    // desired_acceleration
          T(50.),                     // max velocity
          T(4.),                      // velocity_limit_kp
          T(50.)),                    // current_velocity
      T(0));

  // A desired acceleration of zero should result in zero acceleration
  // regardless of the current velocity.
  const std::vector<double> test_velocities{0, 10, 20, 30, 40, 50};
  for (const auto& velocity : test_velocities) {
    EXPECT_EQ(
        calc_smooth_acceleration(
            T(0.),                      // desired_acceleration
            T(50.),                     // max velocity
            T(4.),                      // velocity_limit_kp
            T(velocity)),               // current_velocity
        T(0));
  }
}

GTEST_TEST(SmoothAccelerationFunctionTest, TestDoubleType) {
  do_test<double>();
}

GTEST_TEST(SmoothAccelerationFunctionTest, TestAutoDiffXType) {
  do_test<drake::AutoDiffXd>();

  // Checks that the Jacobian can be taken across the limit-activation boundary.
  // Given the parameters below, the limit-activation boundary occurs when
  // the current velocity equals the max velocity. We thus check whether the
  // derivative at this point is valid. In this case, its valid value is zero.
  double kMaxVelocity{10};
  double kVelocityLimitKp{1};
  double kDesiredAcceleration{2};

  drake::AutoDiffXd result = calc_smooth_acceleration(
      drake::AutoDiffXd{kDesiredAcceleration, Vector1d(1)},
      drake::AutoDiffXd{kMaxVelocity, Vector1d(1)},
      drake::AutoDiffXd{kVelocityLimitKp, Vector1d(1)},
      drake::AutoDiffXd{kMaxVelocity /* current velocity */,
          Vector1d(1)});

  EXPECT_EQ(result.derivatives()(0), 0);
}

// Tests that `calc_smooth_acceleration()` can be evaluated symbolically.
GTEST_TEST(SmoothAccelerationFunctionTest, TestExpressionType) {
  do_test<Expression>();

  Variable desired_acceleration{"desired_acceleration"};
  Variable current_velocity{"current_velocity"};
  Variable max_velocity{"max_velocity"};
  Variable velocity_limit_kp{"velocity_limit_kp"};

  auto smooth_acceleration_expression = calc_smooth_acceleration(
      Expression{desired_acceleration},
      Expression{max_velocity},
      Expression{velocity_limit_kp},
      Expression{current_velocity});
  EXPECT_EQ(
      smooth_acceleration_expression.Evaluate(
          Environment{{desired_acceleration, 10},
                      {max_velocity, 50},
                      {velocity_limit_kp, 4},
                      {current_velocity, 50}}),
      0);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
