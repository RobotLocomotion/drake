#include "drake/systems/analysis/radau_integrator.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/systems/analysis/test_utilities/cubic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/robertson_system.h"
#include "drake/systems/analysis/test_utilities/stationary_system.h"

using drake::systems::analysis_test::CubicScalarSystem;
using drake::systems::analysis_test::LinearScalarSystem;
using drake::systems::analysis_test::StationarySystem;

namespace drake {
namespace systems {
namespace {

// Tests the Jacobian and iteration matrix reuse strategies using a test
// problem and integrator for which we have knowledge of the convergence
// behavior from the initial state.
GTEST_TEST(RadauIntegratorTest, Reuse) {
  std::unique_ptr<analysis::test::RobertsonSystem<double>> robertson =
    std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Create the Euler integrator.
  RadauIntegrator<double, 1> euler(*robertson, context.get());

  euler.set_maximum_step_size(1e-2);  // Maximum step that will be attempted.
  euler.set_throw_on_minimum_step_size_violation(false);
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
  // that this step size should be sufficiently small for the integrator to
  // converge. The Jacobian matrix will be "fresh"; we assume no knowledge
  // of the number of iteration matrix factorizations.
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

// Tests the implicit integrator on a stationary system problem, which
// stresses numerical differentiation (since the state does not change).
GTEST_TEST(RadauIntegratorTest, Stationary) {
  StationarySystem stationary;
  std::unique_ptr<Context<double>> context = stationary.CreateDefaultContext();

  // Set the initial condition for the stationary system.
  VectorBase<double>& state = context->get_mutable_continuous_state().
      get_mutable_vector();
  state.SetAtIndex(0, 0.0);
  state.SetAtIndex(1, 0.0);

  // Create the integrator.
  RadauIntegrator<double> integrator(stationary, context.get());
  integrator.set_maximum_step_size(0.1);

  // Integrate the system
  integrator.Initialize();
  integrator.IntegrateWithMultipleStepsToTime(1.0);

  // Verify the solution.
  EXPECT_NEAR(state.GetAtIndex(0), 0, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(state.GetAtIndex(1), 0, std::numeric_limits<double>::epsilon());
}

// Tests accuracy for integrating a scalar cubic system (with the state at time
// t corresponding to f(t) ≡ C₃t³ + C₂t² + C₁t + C₀) over
// t ∈ [0, 1]. Radau3 is a third order integrator, meaning that it uses the
// Taylor Series expansion:
// f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t) + ⅙h³f'''(t) + O(h⁴)
// The formula above indicates that the approximation error will be zero if
// f''''(t) = 0, which is true for the cubic equation. We also test that the
// single-stage Radau integrator (i.e., implicit Euler integrator) fails to
// integrate this function using a single step.
GTEST_TEST(RadauIntegratorTest, CubicSystem) {
  CubicScalarSystem cubic;
  std::unique_ptr<Context<double>> context = cubic.CreateDefaultContext();

  // Create the integrator: assumes that the default number of stages is 2.
  RadauIntegrator<double> radau3(cubic, context.get());

  const double dt = 1.0;
  radau3.set_maximum_step_size(dt);

  // Integrate the system
  radau3.Initialize();
  ASSERT_TRUE(radau3.IntegrateWithSingleFixedStepToTime(dt));

  // Verify the solution.
  VectorX<double> state =
      context->get_continuous_state().get_vector().CopyToVector();
  EXPECT_NEAR(state[0], cubic.Evaluate(dt),
      std::numeric_limits<double>::epsilon());

  // Reset the state.
  context = cubic.CreateDefaultContext();

  // Create an implicit Euler integrator using 1 stage.
  const int num_stages = 1;
  RadauIntegrator<double, num_stages> euler(cubic, context.get());

  euler.set_maximum_step_size(dt);

  // Integrate the system
  euler.Initialize();
  ASSERT_TRUE(euler.IntegrateWithSingleFixedStepToTime(dt));

  // Verify the integrator failed to produce the solution.
  state = context->get_continuous_state().get_vector().CopyToVector();
  EXPECT_GT(std::abs(state[0] - cubic.Evaluate(dt)),
      1e9 * std::numeric_limits<double>::epsilon());
}

// Tests accuracy for integrating a scalar linear system (with the state at time
// t corresponding to f(t) ≡ C₁t + C₀) over
// t ∈ [0, 1] using the single-stage Radau integrator, meaning that it uses the
// Taylor Series expansion:
// f(t+h) ≈ f(t) + hf'(t) + O(h²)
// The formula above indicates that the approximation error will be zero if
// f''(t) = 0, which is true for the linear equation.
GTEST_TEST(RadauIntegratorTest, LinearSystem) {
  LinearScalarSystem linear;
  std::unique_ptr<Context<double>> context = linear.CreateDefaultContext();

  // Create the integrator.
  const int num_stages = 1;
  RadauIntegrator<double, num_stages> euler(linear, context.get());

  const double dt = 1.0;
  euler.set_maximum_step_size(dt);

  // Integrate the system
  euler.Initialize();
  ASSERT_TRUE(euler.IntegrateWithSingleFixedStepToTime(dt));

  // Verify the solution.
  VectorX<double> state =
      context->get_continuous_state().get_vector().CopyToVector();
  EXPECT_NEAR(state[0], linear.Evaluate(dt),
      std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace systems
}  // namespace drake

