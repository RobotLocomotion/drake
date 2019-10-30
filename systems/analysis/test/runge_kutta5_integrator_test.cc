#include "drake/systems/analysis/runge_kutta5_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/test_utilities/explicit_error_controlled_integrator_test.h"
#include "drake/systems/analysis/test_utilities/generic_integrator_test.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/quartic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/quintic_scalar_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

typedef ::testing::Types<RungeKutta5Integrator<double>> Types;
// NOLINTNEXTLINE(whitespace/line_length)
INSTANTIATE_TYPED_TEST_SUITE_P(My, ExplicitErrorControlledIntegratorTest, Types);
INSTANTIATE_TYPED_TEST_SUITE_P(My, PleidesTest, Types);
INSTANTIATE_TYPED_TEST_SUITE_P(My, GenericIntegratorTest, Types);

// Tests accuracy for integrating the quintic system (with the state at time t
// corresponding to f(t) ≡ t⁵ + 2t⁴ + 3t³ + 4t² + 5t + C) over t ∈ [0, 1].
// RK5 is a fifth order integrator, meaning that it uses the Taylor Series
// expansion: f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t) + ⅙h³f'''(t) + ... + h⁵/120
// f'''''(t) + O(h⁶). The formula above indicates that the approximation error
// will be zero if d⁶f/dt⁶ = 0, which is true for the quintic equation.
GTEST_TEST(RK5IntegratorErrorEstimatorTest, QuinticTest) {
  QuinticScalarSystem quintic;
  auto quintic_context = quintic.CreateDefaultContext();
  const double C = quintic.Evaluate(0);
  quintic_context->SetTime(0.0);
  quintic_context->get_mutable_continuous_state_vector()[0] = C;

  RungeKutta5Integrator<double> rk5(quintic, quintic_context.get());
  const double t_final = 1.0;
  rk5.set_maximum_step_size(t_final);
  rk5.set_fixed_step_mode(true);
  rk5.Initialize();
  ASSERT_TRUE(rk5.IntegrateWithSingleFixedStepToTime(t_final));

  // Check for near-exact 5th-order results. The measure of accuracy is a
  // tolerance that scales with expected answer at t_final.
  const double expected_answer =
      t_final * (t_final * (t_final * (t_final * (t_final + 2) + 3) + 4) + 5) +
      6;
  const double allowable_5th_order_error =
      expected_answer * std::numeric_limits<double>::epsilon();
  const double actual_answer =
      quintic_context->get_continuous_state_vector()[0];
  EXPECT_NEAR(actual_answer, expected_answer, allowable_5th_order_error);

  // This integrator calculates error by subtracting a 4th-order integration
  // result from a 5th-order integration result. Since the 4th-order integrator
  // has a Taylor series that is accurate to O(h⁵) and since we have no terms
  // beyond order h⁵, halving the step size should improve the error estimate
  // by a factor of 2⁵ = 32. We verify this.

  // First obtain the error estimate using a single step of h. Note that the
  // actual integration error is essentially zero, per the check above.
  const double err_est_h = rk5.get_error_estimate()->get_vector().GetAtIndex(0);

  // Now obtain the error estimate using two half steps of h/2.
  quintic_context->SetTime(0.0);
  quintic_context->get_mutable_continuous_state_vector()[0] = C;
  rk5.Initialize();
  ASSERT_TRUE(rk5.IntegrateWithSingleFixedStepToTime(t_final / 2));
  ASSERT_TRUE(rk5.IntegrateWithSingleFixedStepToTime(t_final));
  const double err_est_2h_2 =
      rk5.get_error_estimate()->get_vector().GetAtIndex(0);

  EXPECT_NEAR(err_est_2h_2, 1.0 / 32 * err_est_h,
              10 * std::numeric_limits<double>::epsilon());
}

// Tests accuracy for integrating the quartic system (with the state at time t
// corresponding to f(t) ≡ t⁴ + 2t³ + 3t² + 4t + C, where C is the initial
// state) over t ∈ [0, 1]. The error estimator from RK5 is fourth order, meaning
// that it uses the Taylor Series expansion: f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t)
// + ... + h⁴/24 f''''(t) + O(h⁵). This formula indicates that the approximation
// error will be zero if d⁵f/dt⁵ = 0, which is true for the quartic equation. We
// check that the error estimator gives a perfect error estimate for this
// function.
GTEST_TEST(RK5IntegratorErrorEstimatorTest, QuarticTest) {
  QuarticScalarSystem quartic;
  auto quartic_context = quartic.CreateDefaultContext();
  const double C = quartic.Evaluate(0);
  quartic_context->SetTime(0.0);
  quartic_context->get_mutable_continuous_state_vector()[0] = C;

  RungeKutta5Integrator<double> rk5(quartic, quartic_context.get());
  const double t_final = 1.0;
  rk5.set_maximum_step_size(t_final);
  rk5.set_fixed_step_mode(true);
  rk5.Initialize();
  ASSERT_TRUE(rk5.IntegrateWithSingleFixedStepToTime(t_final));

  const double err_est = rk5.get_error_estimate()->get_vector().GetAtIndex(0);

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C, t_final, or polynomial coefficients.
  EXPECT_NEAR(err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
