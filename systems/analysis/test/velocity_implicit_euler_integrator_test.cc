#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/test_utilities/cubic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/discontinuous_spring_mass_damper_system.h"
#include "drake/systems/analysis/test_utilities/implicit_integrator_test.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/quadratic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/robertson_system.h"
#include "drake/systems/analysis/test_utilities/spring_mass_damper_system.h"
#include "drake/systems/analysis/test_utilities/stationary_system.h"
#include "drake/systems/analysis/test_utilities/stiff_double_mass_spring_system.h"
#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

using analysis_test::CubicScalarSystem;
using analysis_test::LinearScalarSystem;
using analysis_test::QuadraticScalarSystem;
using analysis_test::StationarySystem;
using implicit_integrator_test::DiscontinuousSpringMassDamperSystem;
using implicit_integrator_test::SpringMassDamperSystem;

// Checks the validity of general integrator statistics and resets statistics.
void CheckGeneralStatsValidity(
    VelocityImplicitEulerIntegrator<double>* integrator) {
  EXPECT_GT(integrator->get_num_newton_raphson_iterations(), 0);
  EXPECT_GT(integrator->get_num_error_estimator_newton_raphson_iterations(), 0);
  EXPECT_GT(integrator->get_previous_integration_step_size(), 0.0);
  EXPECT_GT(integrator->get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator->get_num_steps_taken(), 0);
  EXPECT_GT(integrator->get_num_derivative_evaluations(), 0);
  EXPECT_GE(integrator->get_num_error_estimator_derivative_evaluations(), 0);
  EXPECT_GT(integrator->get_num_derivative_evaluations_for_jacobian(), 0);
  EXPECT_GE(
      integrator->get_num_error_estimator_derivative_evaluations_for_jacobian(),
      0);
  EXPECT_GE(integrator->get_num_jacobian_evaluations(), 0);
  EXPECT_GE(integrator->get_num_error_estimator_jacobian_evaluations(), 0);
  EXPECT_GE(integrator->get_num_iteration_matrix_factorizations(), 0);
  EXPECT_GE(
      integrator->get_num_error_estimator_iteration_matrix_factorizations(), 0);
  EXPECT_GE(integrator->get_num_substep_failures(), 0);
  EXPECT_GE(integrator->get_num_step_shrinkages_from_substep_failures(), 0);
  EXPECT_GE(integrator->get_num_step_shrinkages_from_error_control(), 0);
  integrator->ResetStatistics();
}

// Tests accuracy for integrating the quadratic system (with the state at time t
// corresponding to f(t) ≡ 4t² + 4t + C, where C is the initial state) over
// t ∈ [0, 1]. Since the error estimate has a Taylor series that is
// accurate to O(h²) and since we have no terms beyond this order,
// halving the step size should improve the error estimate by a factor of 4.
// Furthemore, we test that the error estimate gives the exact error, with
// both the correct sign and magnitude.
GTEST_TEST(ImplicitIntegratorErrorEstimatorTest, QuadraticSystemAccuracy) {
  QuadraticScalarSystem quadratic(7);
  auto quadratic_context = quadratic.CreateDefaultContext();
  const double C = quadratic.Evaluate(0);
  quadratic_context->SetTime(0.0);
  quadratic_context->get_mutable_continuous_state_vector()[0] = C;

  VelocityImplicitEulerIntegrator<double> vie(quadratic,
                                              quadratic_context.get());

  // Per the description in IntegratorBase::get_error_estimate_order(), this
  // should return "2", in accordance with the order of the polynomial in the
  // Big-Oh term.
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

  // Verify that our error estimate gets the exact error value.
  EXPECT_NEAR(err_est_h, actual_answer - expected_answer,
              10 * std::numeric_limits<double>::epsilon());
  // Now obtain the error estimate using two half steps of h/2.
  quadratic_context->SetTime(0.0);
  quadratic_context->get_mutable_continuous_state_vector()[0] = C;
  vie.Initialize();
  ASSERT_TRUE(vie.IntegrateWithSingleFixedStepToTime(t_final / 2));
  ASSERT_TRUE(vie.IntegrateWithSingleFixedStepToTime(t_final));

  const double err_est_2h_2 =
      vie.get_error_estimate()->get_vector().GetAtIndex(0);

  // Since the error estimate is second order, the estimate from a half-step
  // should be a quarter the size for a quadratic system.
  EXPECT_NEAR(err_est_2h_2, 1.0 / 4 * err_est_h,
              10 * std::numeric_limits<double>::epsilon());
  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C, t_final, or polynomial coefficients.
  CheckGeneralStatsValidity(&vie);
}

// Test Velocity-Implicit Euler integrator on common implicit tests.
typedef ::testing::Types<VelocityImplicitEulerIntegrator<double>> MyTypes;
INSTANTIATE_TYPED_TEST_CASE_P(My, ImplicitIntegratorTest, MyTypes);

}  // namespace analysis_test

}  // namespace systems
}  // namespace drake
