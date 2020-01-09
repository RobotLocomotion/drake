#include "drake/systems/analysis/implicit_euler_integrator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/test_utilities/implicit_integrator_test.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/robertson_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

// Tests the Jacobian and iteration matrix reuse strategies using a test
// problem and integrator for which we have knowledge of the convergence
// behavior from the initial state.
GTEST_TEST(ImplicitEulerIntegratorTest, Reuse) {
  std::unique_ptr<analysis::test::RobertsonSystem<double>> robertson =
    std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Create the Euler integrator.
  ImplicitEulerIntegrator<double> euler(*robertson, context.get());

  euler.set_maximum_step_size(1e-2);  // Maximum step that will be attempted.
  euler.set_throw_on_minimum_step_size_violation(false);
  euler.set_fixed_step_mode(true);
  euler.set_reuse(true);    // The whole point of this.

  // Attempt to integrate the system. Our past experience indicates that this
  // system fails to converge from the initial state for this large step size.
  // This tests the case where the Jacobian matrix has yet to be formed. There
  // should be two Jacobian matrix evaluations- once at trial 1 and another
  // at trial 3. There should be three iteration matrix factorizations: once
  // at trial 1, another at trial 2, and the third at trial 3.
  euler.Initialize();
  ASSERT_FALSE(euler.IntegrateWithSingleFixedStepToTime(1e-2));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(), 3);
  EXPECT_EQ(euler.get_num_jacobian_evaluations(), 2);

  // Now integrate again but with a smaller size. Again, past experience
  // indicates that this step size should be sufficiently small for the
  // integrator to converge. The Jacobian matrix will be "fresh"; we assume no
  // knowledge of the number of iteration matrix factorizations.
  euler.ResetStatistics();
  ASSERT_TRUE(euler.IntegrateWithSingleFixedStepToTime(1e-6));
  EXPECT_EQ(euler.get_num_jacobian_evaluations(), 0);

  // Again try taking a large step, which we expect will be too large to
  // converge. There should be one Jacobian matrix evaluation- once at trial 3.
  // There should be two iteration matrix factorizations: one at trial 2 and
  // another at trial 3.
  euler.ResetStatistics();
  ASSERT_FALSE(euler.IntegrateWithSingleFixedStepToTime(1e-2));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(), 2);
  EXPECT_EQ(euler.get_num_jacobian_evaluations(), 1);
}

// Tests that the full-Newton approach computes a Jacobian matrix and factorizes
// the iteration matrix on every Newton-Raphson iteration.
GTEST_TEST(ImplicitEulerIntegratorTest, FullNewton) {
  std::unique_ptr<analysis::test::RobertsonSystem<double>> robertson =
    std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Create the Euler integrator.
  ImplicitEulerIntegrator<double> euler(*robertson, context.get());

  euler.request_initial_step_size_target(1e0);
  euler.set_throw_on_minimum_step_size_violation(false);
  euler.set_fixed_step_mode(true);
  euler.set_use_full_newton(true);    // The whole point of this test.

  // Attempt to integrate the system. Our past experience indicates that this
  // system fails to converge from the initial state for this large step size.
  // This tests the case where the Jacobian matrix has yet to be formed.
  euler.Initialize();
  ASSERT_FALSE(euler.IntegrateWithSingleFixedStepToTime(1e0));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(),
            euler.get_num_newton_raphson_iterations());
  EXPECT_EQ(euler.get_num_jacobian_evaluations(),
            euler.get_num_newton_raphson_iterations());

  // Now integrate again but with a smaller size. Again, past experience tells
  // us that this step size should be sufficiently small for the integrator to
  // converge.
  euler.ResetStatistics();
  ASSERT_TRUE(euler.IntegrateWithSingleFixedStepToTime(1e-6));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(),
            euler.get_num_newton_raphson_iterations());
  EXPECT_EQ(euler.get_num_jacobian_evaluations(),
            euler.get_num_newton_raphson_iterations());

  // Again try taking a large step, which we expect will be too large to
  // converge.
  euler.ResetStatistics();
  ASSERT_FALSE(euler.IntegrateWithSingleFixedStepToTime(1e0));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(),
            euler.get_num_newton_raphson_iterations());
  EXPECT_EQ(euler.get_num_jacobian_evaluations(),
            euler.get_num_newton_raphson_iterations());
}

// Tests the implicit integrator on a stationary system problem, which
// stresses numerical differentiation (since the state does not change).
GTEST_TEST(ImplicitEulerIntegratorTest, Stationary) {
  auto stationary = std::make_unique<StationarySystem>();
  std::unique_ptr<Context<double>> context = stationary->CreateDefaultContext();

  // Set the initial condition for the stationary system.
  VectorBase<double>& state = context->get_mutable_continuous_state().
      get_mutable_vector();
  state.SetAtIndex(0, 0.0);
  state.SetAtIndex(1, 0.0);

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*stationary, context.get());
  integrator.set_maximum_step_size(1.0);
  integrator.set_target_accuracy(1e-3);
  integrator.request_initial_step_size_target(1e-4);

  // Integrate the system
  integrator.Initialize();
  integrator.IntegrateWithMultipleStepsToTime(1.0);

  // Verify the solution.
  EXPECT_NEAR(state.GetAtIndex(0), 0, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(state.GetAtIndex(1), 0, std::numeric_limits<double>::epsilon());
}

// Tests the implicit integrator on Robertson's stiff chemical reaction
// problem, which has been used to benchmark various implicit integrators.
// This problem is particularly good at testing large step sizes (since the
// solution quickly converges) and long simulation times.
GTEST_TEST(ImplicitEulerIntegratorTest, Robertson) {
  std::unique_ptr<analysis::test::RobertsonSystem<double>> robertson =
    std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  const double t_final = robertson->get_end_time();
  const double tol = 5e-5;

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*robertson, context.get());

  // Very large step is necessary for this problem since given solution is
  // at t = 1e11. However, the current initial step size selection algorithm
  // will use a large factor of the maximum step size, which can result in
  // too large an initial step for this problem. Accordingly, we explicitly
  // select a small initial step size.
  // @TODO(edrumwri): Explore a better algorithm for selecting the initial
  //                  step size (see issue #6329).
  integrator.set_maximum_step_size(10000000.0);
  integrator.set_throw_on_minimum_step_size_violation(false);
  integrator.set_target_accuracy(tol);
  integrator.request_initial_step_size_target(1e-4);

  // Integrate the system
  integrator.Initialize();
  integrator.IntegrateWithMultipleStepsToTime(t_final);

  // Verify the solution.
  const VectorBase<double>& state = context->get_continuous_state().
      get_vector();
  const Eigen::Vector3d sol = robertson->GetSolution(t_final);
  EXPECT_NEAR(state.GetAtIndex(0), sol(0), tol);
  EXPECT_NEAR(state.GetAtIndex(1), sol(1), tol);
  EXPECT_NEAR(state.GetAtIndex(2), sol(2), tol);
}

GTEST_TEST(ImplicitEulerIntegratorTest, FixedStepThrowsOnMultiStep) {
  auto robertson = std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Relatively large step size that we know fails to converge from the initial
  // state.
  const double h = 1e-2;

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*robertson, context.get());

  // Make sure integrator can take the size we want.
  integrator.set_maximum_step_size(h);

  // Enable fixed stepping.
  integrator.set_fixed_step_mode(true);

  // Values we have used successfully in other Robertson system tests.
  integrator.set_target_accuracy(5e-5);

  // Integrate to the desired step time. We expect this to return false because
  // the integrator is generally unlikely to converge for such a relatively
  // large step.
  integrator.Initialize();
  EXPECT_FALSE(integrator.IntegrateWithSingleFixedStepToTime(
      context->get_time() + h));
}

// Tests accuracy for integrating linear systems (with the state at time t
// corresponding to f(t) ≡ St + C, where S is a scalar and C is the initial
// state) over t ∈ [0, 1]. The asymptotic term in ImplicitEulerIntegrator's
// error estimate is second order, meaning that it uses the Taylor Series
// expansion:
// f(t+h) ≈ f(t) + hf'(t) + O(h²).
// This formula indicates that the approximation error will be zero if
// f''(t) = 0, which is true for linear systems. We check that the
// error estimator gives a perfect error estimate for this function.
GTEST_TEST(ImplicitIntegratorErrorEstimatorTest, LinearTest) {
  LinearScalarSystem linear;
  auto linear_context = linear.CreateDefaultContext();
  const double C = linear.Evaluate(0);
  linear_context->SetTime(0.0);
  linear_context->get_mutable_continuous_state_vector()[0] = C;

  ImplicitEulerIntegrator<double> ie(linear, linear_context.get());
  const double t_final = 1.0;
  ie.set_maximum_step_size(t_final);
  ie.set_fixed_step_mode(true);
  ie.Initialize();
  ASSERT_TRUE(ie.IntegrateWithSingleFixedStepToTime(t_final));

  const double err_est = ie.get_error_estimate()->get_vector()[0];

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C, t_final, or polynomial coefficients.
  EXPECT_NEAR(err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());

  // Repeat this test, but using a final time that is below the working minimum
  // step size (thereby triggering the implicit integrator's alternate, explicit
  // mode). To retain our existing tolerances, we change the scale factor (S)
  // for the linear system.
  ie.get_mutable_context()->SetTime(0);
  const double working_min = ie.get_working_minimum_step_size();
  LinearScalarSystem scaled_linear(4.0/working_min);
  auto scaled_linear_context = scaled_linear.CreateDefaultContext();
  ImplicitEulerIntegrator<double> ie2(
      scaled_linear, scaled_linear_context.get());
  const double updated_t_final = working_min / 2;
  ie2.set_maximum_step_size(updated_t_final);
  ie2.set_fixed_step_mode(true);
  ie2.Initialize();
  ASSERT_TRUE(ie2.IntegrateWithSingleFixedStepToTime(updated_t_final));

  const double updated_err_est = ie2.get_error_estimate()->get_vector()[0];

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C, t_final, or polynomial coefficients.
  EXPECT_NEAR(updated_err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());
}

// Test Euler integrator.
typedef ::testing::Types<ImplicitEulerIntegrator<double>>
    MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, ImplicitIntegratorTest, MyTypes);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

