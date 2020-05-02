#include "drake/systems/analysis/radau_integrator.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/test_utilities/cubic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/implicit_integrator_test.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/quadratic_scalar_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

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

// Tests accuracy for integrating the quadratic system (with the state at time t
// corresponding to f(t) ≡ C₂t² + C₁t + C₀, where C₀ is the initial state) over
// t ∈ [0, 1]. The error estimate from 2-stage Radau is third-order accurate,
// meaning that the approximation error will be zero if f'''(t) = 0, which is
// true for the quadratic equation. We check that the error estimate is zero
// for this function.
// Note: this test differs from [Velocity]ImplicitEulerIntegratorTest::
// QuadraticSystemErrorEstimatorAccuracy in that the error estimation for the
// two-stage Radau integrator is third-order accurate, so this test checks to
// make sure the error estimate (and error) are zero, while both the implicit
// Euler and the velocity-implicit Euler integrators have a second-order-
// accurate error estimate, so their tests check to make sure the error
// estimate is exact (not necessarily zero).
GTEST_TEST(RadauIntegratorTest, QuadraticTest) {
  QuadraticScalarSystem quadratic;
  auto quadratic_context = quadratic.CreateDefaultContext();
  const double C0 = quadratic.Evaluate(0);
  quadratic_context->SetTime(0.0);
  quadratic_context->get_mutable_continuous_state_vector()[0] = C0;

  // Create the integrator.
  const int num_stages = 2;
  RadauIntegrator<double, num_stages> radau(quadratic, quadratic_context.get());

  // Ensure that the Radau3 integrator supports error estimation.
  EXPECT_TRUE(radau.supports_error_estimation());

  const double t_final = 1.0;
  radau.set_maximum_step_size(t_final);
  radau.set_fixed_step_mode(true);
  radau.Initialize();
  ASSERT_TRUE(radau.IntegrateWithSingleFixedStepToTime(t_final));

  // Ensure that Radau, and not BS3, was used by counting the number of
  // function evaluations.  Note that this number is somewhat brittle and might
  // need to be revised in the future.
  EXPECT_EQ(radau.get_num_derivative_evaluations(), 8);

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

// Tests that Radau1 can successfully integrate the linear system (with the
// state at time t corresponding to f(t) ≡ C₁t + C₀, where C₀ is the initial
// state) over t ∈ [0, 1], without falling back to Euler+RK2. In
// ImplicitIntegratorTest::LinearTest, we also test the accuracy of Radau1 and
// Radau3 while integrating the same system.
GTEST_TEST(RadauIntegratorTest, LinearRadauTest) {
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
  EXPECT_EQ(radau.get_num_derivative_evaluations(), 6);

  // Per the description in IntegratorBase::get_error_estimate_order(), this
  // should return "2", in accordance with the order of the polynomial in the
  // Big-Oh term; for the two-stage Radau integrator, this value will be
  // different.
  ASSERT_EQ(radau.get_error_estimate_order(), 2);
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

  // Tell IE to use implicit trapezoid for error estimation to match
  // Radau1.
  ie.set_use_implicit_trapezoid_error_estimation(true);

  // Ensure that both integrators support error estimation.
  EXPECT_TRUE(radau1.supports_error_estimation());
  EXPECT_TRUE(ie.supports_error_estimation());

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

