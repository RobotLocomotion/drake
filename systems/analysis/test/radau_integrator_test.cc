#include "drake/systems/analysis/radau_integrator.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/test_utilities/cubic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/implicit_integrator_test.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/quadratic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/robertson_system.h"
#include "drake/systems/analysis/test_utilities/stationary_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

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

  // Create the integrator using two stages.
  RadauIntegrator<double, 2> radau3(cubic, context.get());

  const double h = 1.0;
  radau3.set_maximum_step_size(h);
  radau3.set_fixed_step_mode(true);

  // Integrate the system
  radau3.Initialize();
  ASSERT_TRUE(radau3.IntegrateWithSingleFixedStepToTime(h));

  // Verify the solution.
  VectorX<double> state =
      context->get_continuous_state().get_vector().CopyToVector();
  EXPECT_NEAR(state[0], cubic.Evaluate(h),
      std::numeric_limits<double>::epsilon());

  // Reset the state.
  context = cubic.CreateDefaultContext();

  // Create an implicit Euler integrator using 1 stage and running in fixed step
  // mode.
  const int num_stages = 1;
  RadauIntegrator<double, num_stages> euler(cubic, context.get());
  euler.set_fixed_step_mode(true);
  euler.set_maximum_step_size(h);

  // Integrate the system
  euler.Initialize();
  ASSERT_TRUE(euler.IntegrateWithSingleFixedStepToTime(h));

  // Verify the integrator failed to produce the solution.
  state = context->get_continuous_state().get_vector().CopyToVector();
  EXPECT_GT(std::abs(state[0] - cubic.Evaluate(h)),
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

  const double h = 1.0;
  euler.set_maximum_step_size(h);

  // Integrate the system
  euler.Initialize();
  euler.set_fixed_step_mode(true);
  ASSERT_TRUE(euler.IntegrateWithSingleFixedStepToTime(h));

  // Verify the solution.
  VectorX<double> state =
      context->get_continuous_state().get_vector().CopyToVector();
  EXPECT_NEAR(state[0], linear.Evaluate(h),
      std::numeric_limits<double>::epsilon());
}

// Tests accuracy for integrating the quadratic system (with the state at time t
// corresponding to f(t) ≡ C₂t² + C₁t + C₀, where C₀ is the initial state) over
// t ∈ [0, 1]. The error estimate from 2-stage Radau is second order accurate,
// meaning that the approximation error will be zero if f'''(t) = 0, which is
// true for the quadratic equation. We check that the error estimate is perfect
// for this function.
GTEST_TEST(RadauIntegratorTest, QuadraticTest) {
  QuadraticScalarSystem quadratic;
  auto quadratic_context = quadratic.CreateDefaultContext();
  const double C0 = quadratic.Evaluate(0);
  quadratic_context->SetTime(0.0);
  quadratic_context->get_mutable_continuous_state_vector()[0] = C0;

  // Create the integrator.
  const int num_stages = 2;
  RadauIntegrator<double, num_stages> radau(quadratic, quadratic_context.get());
  const double t_final = 1.0;
  radau.set_maximum_step_size(t_final);
  radau.set_fixed_step_mode(true);
  radau.Initialize();
  ASSERT_TRUE(radau.IntegrateWithSingleFixedStepToTime(t_final));

  // Ensure that Radau, and not BS3, was used by counting the number of
  // function evaluations.  Note that this number is somewhat brittle and might
  // need to be revised in the future.
  EXPECT_EQ(radau.get_num_derivative_evaluations(), 9);

  // Per the description in IntegratorBase::get_error_estimate_order(), this
  // should return "3", in accordance with the order of the polynomial in the
  // Big-Oh term; for the single-stage Radau integrator, this value will be
  // different.
  ASSERT_EQ(radau.get_error_estimate_order(), 3);

  const double err_est =
      radau.get_error_estimate()->get_vector().GetAtIndex(0);

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C0, t_final, or polynomial coefficients.
  EXPECT_NEAR(err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());

  // Verify the solution too.
  EXPECT_NEAR(
      quadratic_context->get_continuous_state().get_vector().CopyToVector()[0],
      quadratic.Evaluate(t_final),
      std::numeric_limits<double>::epsilon());

  // Repeat this test, but using a final time that is below the working minimum
  // step size (thereby triggering the implicit integrator's alternate, explicit
  // mode). To retain our existing tolerances, we change the scale factor (S)
  // for the quadratic system.
  radau.get_mutable_context()->SetTime(0);
  const double working_min = radau.get_working_minimum_step_size();
  QuadraticScalarSystem scaled_quadratic(4.0/working_min);
  auto scaled_quadratic_context = scaled_quadratic.CreateDefaultContext();
  RadauIntegrator<double> scaled_radau(
      scaled_quadratic, scaled_quadratic_context.get());
  const double updated_t_final = working_min / 2;
  scaled_radau.set_maximum_step_size(updated_t_final);
  scaled_radau.set_fixed_step_mode(true);
  scaled_radau.Initialize();
  ASSERT_TRUE(scaled_radau.IntegrateWithSingleFixedStepToTime(updated_t_final));

  // Ensure that BS3, and not Radau, was used by counting the number of
  // function evaluations.
  EXPECT_EQ(scaled_radau.get_num_derivative_evaluations(), 4);

  const double updated_err_est =
      scaled_radau.get_error_estimate()->get_vector()[0];

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C0, t_final, or polynomial coefficients.
  EXPECT_NEAR(updated_err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());

  // Verify the solution too.
  EXPECT_NEAR(
      scaled_quadratic_context->get_continuous_state().get_vector().
          CopyToVector()[0],
      scaled_quadratic.Evaluate(updated_t_final),
      10 * std::numeric_limits<double>::epsilon());
}

// Tests accuracy for integrating the linear system (with the state at time t
// corresponding to f(t) ≡ C₁t + C₀, where C₀ is the initial state) over
// t ∈ [0, 1]. The error estimate from 1-stage Radau is first order accurate,
// meaning that the approximation error will be zero if f''(t) = 0, which is
// true for the linear equation. We check that the error estimate is perfect
// for this function.
GTEST_TEST(RadauIntegratorTest, LinearTest) {
  LinearScalarSystem linear;
  auto linear_context = linear.CreateDefaultContext();
  const double C0 = linear.Evaluate(0);
  linear_context->SetTime(0.0);
  linear_context->get_mutable_continuous_state_vector()[0] = C0;

  // Create the integrator.
  const int num_stages = 1;
  RadauIntegrator<double, num_stages> radau(linear, linear_context.get());
  const double t_final = 1.0;
  radau.set_maximum_step_size(t_final);
  radau.set_fixed_step_mode(true);
  radau.Initialize();
  ASSERT_TRUE(radau.IntegrateWithSingleFixedStepToTime(t_final));

  // Ensure that Radau, and not Euler+RK2, was used by counting the number of
  // function evaluations. Note that this number is somewhat brittle and might
  // need to be revised in the future.
  EXPECT_EQ(radau.get_num_derivative_evaluations(), 7);

  // Per the description in IntegratorBase::get_error_estimate_order(), this
  // should return "2", in accordance with the order of the polynomial in the
  // Big-Oh term; for the two-stage Radau integrator, this value will be
  // different.
  ASSERT_EQ(radau.get_error_estimate_order(), 2);

  const double err_est =
      radau.get_error_estimate()->get_vector().GetAtIndex(0);

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C0, t_final, or polynomial coefficients.
  EXPECT_NEAR(err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());

  // Verify the solution too.
  EXPECT_NEAR(
      linear_context->get_continuous_state().get_vector().CopyToVector()[0],
      linear.Evaluate(t_final),
      std::numeric_limits<double>::epsilon());

  // Repeat this test, but using a final time that is below the working minimum
  // step size (thereby triggering the implicit integrator's alternate, explicit
  // mode). To retain our existing tolerances, we change the scale factor (S)
  // for the linear system.
  radau.get_mutable_context()->SetTime(0);
  const double working_min = radau.get_working_minimum_step_size();
  LinearScalarSystem scaled_linear(4.0/working_min);
  auto scaled_linear_context = scaled_linear.CreateDefaultContext();
  RadauIntegrator<double> scaled_radau(
      scaled_linear, scaled_linear_context.get());
  const double updated_t_final = working_min / 2;
  scaled_radau.set_maximum_step_size(updated_t_final);
  scaled_radau.set_fixed_step_mode(true);
  scaled_radau.Initialize();
  ASSERT_TRUE(scaled_radau.IntegrateWithSingleFixedStepToTime(updated_t_final));

  // Ensure that explicit Euler + RK2, and not Radau, was used by counting the
  // number of function evaluations.
  EXPECT_EQ(scaled_radau.get_num_derivative_evaluations(), 4);

  const double updated_err_est =
      scaled_radau.get_error_estimate()->get_vector()[0];

  // Note the very tight tolerance used, which will likely not hold for
  // arbitrary values of C0, t_final, or polynomial coefficients.
  EXPECT_NEAR(updated_err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());

  // Verify the solution too.
  EXPECT_NEAR(
      scaled_linear_context->get_continuous_state().get_vector().
          CopyToVector()[0],
      scaled_linear.Evaluate(updated_t_final),
      10 * std::numeric_limits<double>::epsilon());
}

// Integrate the modified mass-spring-damping system, which exhibits a
// discontinuity in the velocity derivative at spring position x = 0, and
// compares the results at the final state between implicit Euler and Radau-1.
GTEST_TEST(RadauIntegratorTest, Radau1MatchesImplicitEuler) {
  std::unique_ptr<Context<double>> context_;

  // The mass of the particle connected by the spring and damper to the world.
  const double mass = 2.0;

  // The magnitude of the constant force pushing the particle toward -inf.
  const double constant_force_mag = 10;

  // Spring constant for a semi-stiff spring. Corresponds to a frequency of
  // 35.588 cycles per second without damping, assuming that mass = 2 (using
  // formula f = sqrt(k/mass)/(2*pi), where k is the spring constant, and f is
  // the frequency in cycles per second).
  const double semistiff_spring_k = 1e5;

  // Construct the discontinuous mass-spring-damper system, using critical
  // damping.
  implicit_integrator_test::DiscontinuousSpringMassDamperSystem<double>
      spring_damper(semistiff_spring_k, std::sqrt(semistiff_spring_k / mass),
                    mass, constant_force_mag);

  // Create two contexts, one for each integrator.
  auto context_radau1 = spring_damper.CreateDefaultContext();
  auto context_ie = spring_damper.CreateDefaultContext();

  // Construct the two integrators.
  const int num_stages = 1;    // Yields implicit Euler.
  RadauIntegrator<double, num_stages> radau1(spring_damper,
                                             context_radau1.get());
  ImplicitEulerIntegrator<double> ie(spring_damper, context_ie.get());

  // Set maximum step sizes that are reasonable for this system.
  radau1.set_maximum_step_size(1e-2);
  ie.set_maximum_step_size(1e-2);

  // Set the same target accuracy for both integrators.
  radau1.set_target_accuracy(1e-5);
  ie.set_target_accuracy(1e-5);

  // Make both integrators re-use Jacobians and iteration matrices, when
  // possible.
  radau1.set_reuse(true);
  ie.set_reuse(true);

  // Setting the minimum step size speeds the unit test without (in this case)
  // affecting solution accuracy.
  radau1.set_requested_minimum_step_size(1e-8);
  ie.set_requested_minimum_step_size(1e-8);

  // Set initial position.
  const double initial_position = 1e-8;
  spring_damper.set_position(context_radau1.get(), initial_position);
  spring_damper.set_position(context_ie.get(), initial_position);

  // Set the initial velocity.
  const double initial_velocity = 0;
  spring_damper.set_velocity(context_radau1.get(), initial_velocity);
  spring_damper.set_velocity(context_ie.get(), initial_velocity);

  // Initialize both integrators.
  radau1.Initialize();
  ie.Initialize();

  // Integrate for 1 second.
  const double t_final = 1.0;
  radau1.IntegrateWithMultipleStepsToTime(t_final);
  ie.IntegrateWithMultipleStepsToTime(t_final);

  // NOTE: A preliminary investigation indicates that these are not bitwise
  // identical due to different ordering of arithmetic operations.

  // Verify the final position and accuracy are identical to machine precision.
  const double tol = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(context_radau1->get_continuous_state().get_vector().GetAtIndex(0),
              context_ie->get_continuous_state().get_vector().GetAtIndex(0),
              tol);
  EXPECT_NEAR(context_radau1->get_continuous_state().get_vector().GetAtIndex(1),
              context_ie->get_continuous_state().get_vector().GetAtIndex(1),
              tol);

  // NOTE: The preliminary investigation cited above indicates that the
  // different order of arithmetic operations is responsible for a slight
  // discrepancy in number of derivative evaluations and Newton-Raphson
  // iterations, so we don't verify that these stats are identical.

  // Verify that the remaining statistics are identical.
  EXPECT_EQ(radau1.get_largest_step_size_taken(),
            ie.get_largest_step_size_taken());
  EXPECT_EQ(radau1.get_num_derivative_evaluations_for_jacobian(),
            ie.get_num_derivative_evaluations_for_jacobian());
  EXPECT_EQ(radau1.get_num_error_estimator_derivative_evaluations(),
            ie.get_num_error_estimator_derivative_evaluations());
  EXPECT_EQ(
      radau1.get_num_error_estimator_derivative_evaluations_for_jacobian(),
      ie.get_num_error_estimator_derivative_evaluations_for_jacobian());
  EXPECT_EQ(radau1.get_num_error_estimator_iteration_matrix_factorizations(),
            ie.get_num_error_estimator_iteration_matrix_factorizations());
  EXPECT_EQ(radau1.get_num_error_estimator_jacobian_evaluations(),
            ie.get_num_error_estimator_jacobian_evaluations());
  EXPECT_EQ(radau1.get_num_error_estimator_newton_raphson_iterations(),
            ie.get_num_error_estimator_newton_raphson_iterations());
  EXPECT_EQ(radau1.get_num_iteration_matrix_factorizations(),
            ie.get_num_iteration_matrix_factorizations());
  EXPECT_EQ(radau1.get_num_jacobian_evaluations(),
            ie.get_num_jacobian_evaluations());
  EXPECT_EQ(radau1.get_num_step_shrinkages_from_error_control(),
            ie.get_num_step_shrinkages_from_error_control());
  EXPECT_EQ(radau1.get_num_step_shrinkages_from_substep_failures(),
            ie.get_num_step_shrinkages_from_substep_failures());
  EXPECT_EQ(radau1.get_num_steps_taken(), ie.get_num_steps_taken());
  EXPECT_EQ(radau1.get_num_substep_failures(), ie.get_num_substep_failures());

  // Verify that we've had at least one step shrinkage.
  EXPECT_GT(radau1.get_num_step_shrinkages_from_substep_failures(), 0);
}

// Test both 1-stage (1st order) and 2-stage (3rd order) Radau integrators.
typedef ::testing::Types<RadauIntegrator<double, 1>, RadauIntegrator<double, 2>>
    MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, ImplicitIntegratorTest, MyTypes);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

