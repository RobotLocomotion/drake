#include "drake/systems/analysis/radau_integrator.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/test_utilities/cubic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/stationary_system.h"

using drake::systems::analysis_test::CubicScalarSystem;
using drake::systems::analysis_test::LinearScalarSystem;
using drake::systems::analysis_test::StationarySystem;

namespace drake {
namespace systems {
namespace {

// Tests the implicit integrator on a stationary system problem, which
// stresses numerical differentiation (since the state does not change).
GTEST_TEST(RadauIntegratorTest, Stationary) {
  StationarySystem stationary;
  std::unique_ptr<Context<double>> context = stationary.CreateDefaultContext();
  context->EnableCaching();

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
  radau3.IntegrateWithSingleFixedStepToTime(dt);

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
  euler.IntegrateWithSingleFixedStepToTime(dt);

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
  euler.IntegrateWithSingleFixedStepToTime(dt);

  // Verify the solution.
  VectorX<double> state =
      context->get_continuous_state().get_vector().CopyToVector();
  EXPECT_NEAR(state[0], linear.Evaluate(dt),
      std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace systems
}  // namespace drake

