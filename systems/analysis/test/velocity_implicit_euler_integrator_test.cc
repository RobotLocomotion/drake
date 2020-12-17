#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/test_utilities/implicit_integrator_test.h"
#include "drake/systems/analysis/test_utilities/quadratic_scalar_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

// Tests accuracy for integrating the quadratic system (with the state at time t
// corresponding to f(t) ≡ 7t² + 7t + f₀, where f₀ is the initial state) over
// t ∈ [0, 1]. Since the error estimate has a Taylor series that is
// accurate to O(h²) and since we have no terms beyond this order,
// halving the step size should improve the error estimate by a factor of 4.
// Furthermore, we test that the error estimate gives the exact error, with
// both the correct sign and magnitude.
// Note: this test differs from RadauIntegratorTest::QuadraticTest in that
// the error estimation for the two-stage Radau integrator is third-order
// accurate, so that test checks to make sure the error estimate (and error)
// are zero, while this integrator has a second-order-accurate error estimate,
// so this test checks to make sure the error estimate is exact (not necessarily
// zero).
GTEST_TEST(VelocityImplicitEulerIntegratorTest,
           QuadraticSystemErrorEstimatorAccuracy) {
  QuadraticScalarSystem quadratic(7);
  auto quadratic_context = quadratic.CreateDefaultContext();
  const double C = quadratic.Evaluate(0);
  quadratic_context->SetTime(0.0);
  quadratic_context->get_mutable_continuous_state_vector()[0] = C;

  VelocityImplicitEulerIntegrator<double> vie(quadratic,
                                              quadratic_context.get());

  // Ensure that the velocity-implicit Euler integrator supports error
  // estimation.
  ASSERT_TRUE(vie.supports_error_estimation());

  // Per the description in IntegratorBase::get_error_estimate_order(), this
  // should return "2", in accordance with the order of the polynomial in the
  // Big-O term.
  ASSERT_EQ(vie.get_error_estimate_order(), 2);

  const double t_final = 1.5;
  vie.set_maximum_step_size(t_final);
  vie.set_fixed_step_mode(true);
  vie.Initialize();
  ASSERT_TRUE(vie.IntegrateWithSingleFixedStepToTime(t_final));

  const double err_est_h = vie.get_error_estimate()->get_vector().GetAtIndex(0);
  const double expected_answer = quadratic.Evaluate(t_final);
  const double actual_answer =
      quadratic_context->get_continuous_state_vector()[0];

  // Verify that the error estimate gets the exact error value. Note the tight
  // tolerance used.
  EXPECT_NEAR(err_est_h, actual_answer - expected_answer,
              10 * std::numeric_limits<double>::epsilon());

  // Now obtain the error estimate corresponding to two half-sized "steps" of
  // size h/2, to verify the error estimate order. To clarify, this means that
  // the velocity-implicit Euler integrator would take four small steps and two
  // large steps here.
  quadratic_context->SetTime(0.0);
  quadratic_context->get_mutable_continuous_state_vector()[0] = C;
  vie.Initialize();
  ASSERT_TRUE(vie.IntegrateWithSingleFixedStepToTime(t_final / 2));
  ASSERT_TRUE(vie.IntegrateWithSingleFixedStepToTime(t_final));

  const double err_est_2h_2 =
      vie.get_error_estimate()->get_vector().GetAtIndex(0);

  // Since the error estimate is second order, the estimate from half-steps
  // should be a quarter the size for a quadratic system.
  EXPECT_NEAR(err_est_2h_2, 1.0 / 4 * err_est_h,
              10 * std::numeric_limits<double>::epsilon());

  // Verify the validity of general statistics.
  ImplicitIntegratorTest<
      VelocityImplicitEulerIntegrator<double>>::CheckGeneralStatsValidity(&vie);
}

// Test Velocity-Implicit Euler integrator on common implicit tests.
typedef ::testing::Types<VelocityImplicitEulerIntegrator<double>> MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, ImplicitIntegratorTest, MyTypes);

}  // namespace analysis_test

}  // namespace systems
}  // namespace drake
