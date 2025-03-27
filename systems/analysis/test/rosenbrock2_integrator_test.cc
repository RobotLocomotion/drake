#include "drake/systems/analysis/rosenbrock2_integrator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/test_utilities/explicit_error_controlled_integrator_test.h"
#include "drake/systems/analysis/test_utilities/generic_integrator_test.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/quadratic_scalar_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

// Test that the integrator gives the expected result on a simple spring-mass
// system.
GTEST_TEST(Rosenbrock2IntegratorTest, SpringMass) {
  const double k = 300.0;  // N/m
  const double m = 2.0;    // kg

  // Create the spring-mass system.
  SpringMassSystem<double> spring_mass(k, m, 0.0);
  auto context = spring_mass.CreateDefaultContext();

  // Set initial conditions
  const double initial_position = 0.1;
  const double initial_velocity = 2.1;

  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);

  // Set up the integrator
  const double h = 1e-1;
  const double accuracy = 0.01;
  Rosenbrock2Integrator<double> integrator(spring_mass, context.get());
  integrator.set_target_accuracy(accuracy);
  integrator.set_maximum_step_size(h);
  integrator.set_fixed_step_mode(false);
  integrator.Initialize();

  // Integrate the system
  const double t_final = 1.0;
  integrator.IntegrateWithMultipleStepsToTime(t_final);
  const double q = spring_mass.get_position(*context);
  const double v = spring_mass.get_velocity(*context);

  // Compare with the reference solution
  double q_ref;
  double v_ref;
  spring_mass.GetClosedFormSolution(initial_position, initial_velocity, t_final,
                                    &q_ref, &v_ref);

  EXPECT_NEAR(q, q_ref, accuracy);
  EXPECT_NEAR(v, v_ref, accuracy);
}

// Tests accuracy by integrating a quadratic system with a known solution,
// x(t) = 7t² + 7t + x₀, over t ∈ [0, 1]. Our Rosenbrock2 integrator is second
// order, and uses the Taylor Series expansion
//   x(t+h) ≈ x(t) + hx'(t) + ½h²x''(t) + O(h³).
// This indicates that approximation error should be zero if x'''(t) = 0, which
// is the case for this quadratic system.
GTEST_TEST(Rosenbrock2IntegratorTest, QuadraticSystem) {
  QuadraticScalarSystem quadratic(7);
  auto context = quadratic.CreateDefaultContext();

  const double x0 = quadratic.Evaluate(0);
  const double t_final = 1.0;
  context->get_mutable_continuous_state_vector()[0] = x0;

  Rosenbrock2Integrator<double> integrator(quadratic, context.get());
  integrator.set_maximum_step_size(t_final);
  integrator.set_fixed_step_mode(true);
  integrator.Initialize();
  ASSERT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(t_final));

  const double x1_ref = quadratic.Evaluate(t_final);
  const double x1 = context->get_continuous_state_vector()[0];

  EXPECT_NEAR(x1, x1_ref, 10 * std::numeric_limits<double>::epsilon());
}

// Generic integrator tests. Note that while the Rosenbrock2 integrator is
// templated on ImplicitIntegrator, it behaves more like an explicit integrator
// in the sense that we don't need to worry about convergence and never perform
// multiple iterations.
typedef ::testing::Types<Rosenbrock2Integrator<double>> MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, ExplicitErrorControlledIntegratorTest,
                               MyTypes);
INSTANTIATE_TYPED_TEST_SUITE_P(My, GenericIntegratorTest, MyTypes);
INSTANTIATE_TYPED_TEST_SUITE_P(My, PleidesTest, MyTypes);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
