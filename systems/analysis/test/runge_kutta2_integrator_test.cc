#include "drake/systems/analysis/runge_kutta2_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(IntegratorTest, MiscAPI) {
  // Create the spring-mass system.
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);

  // Setup integration step.
  const double h  = 1e-3;

  // Create the integrator.
  RungeKutta2Integrator<double> integrator(spring_mass, h);

  // Test that setting the target accuracy fails.
  EXPECT_THROW(integrator.set_target_accuracy(1.0), std::logic_error);
  EXPECT_THROW(integrator.request_initial_step_size_target(1.0),
               std::logic_error);

  // Verifies that starting dense integration (i.e. demanding a dense
  // output) fails if the integrator has not been initialized yet.
  EXPECT_THROW(integrator.StartDenseIntegration(), std::logic_error);

  // Verifies that stopping dense integration (i.e. precluding further updates
  // of the dense output known to the integrator) fails if it has not
  // been started (via StartDenseIntegration()) since last Initialize() call
  // or construction.
  EXPECT_THROW(integrator.StopDenseIntegration(), std::logic_error);
}

GTEST_TEST(IntegratorTest, ContextAccess) {
  // Create the mass spring system.
  SpringMassSystem<double> spring_mass(1., 1., 0.);

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Setup integration step.
  const double h  = 1e-3;

  // Create the integrator
  RungeKutta2Integrator<double> integrator(spring_mass, h, context.get());
  integrator.get_mutable_context()->SetTime(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(context->get_time(), 3.);
}

/// Verifies error estimation is unsupported.
GTEST_TEST(IntegratorTest, ErrorEst) {
  // Spring-mass system is necessary only to setup the problem.
  SpringMassSystem<double> spring_mass(1., 1., 0.);
  const double h = 1e-3;
  auto context = spring_mass.CreateDefaultContext();
  RungeKutta2Integrator<double> integrator(
      spring_mass, h, context.get());

  EXPECT_EQ(integrator.get_error_estimate_order(), 0);
  EXPECT_EQ(integrator.supports_error_estimation(), false);
  EXPECT_THROW(integrator.set_fixed_step_mode(false), std::logic_error);
  EXPECT_THROW(integrator.set_target_accuracy(1e-1), std::logic_error);
  EXPECT_THROW(integrator.request_initial_step_size_target(h),
               std::logic_error);
}

// Checks the validity of integrator statistics.
void CheckStatsValidity(RungeKutta2Integrator<double>* integrator) {
  EXPECT_GE(integrator->get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator->get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator->get_num_steps_taken(), 0);
  EXPECT_EQ(integrator->get_error_estimate(), nullptr);
  EXPECT_GT(integrator->get_num_derivative_evaluations(), 0);
}

// Try a purely continuous system with no sampling.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
GTEST_TEST(IntegratorTest, SpringMassStep) {
  const double spring_k = 300.0;  // N/m
  const double mass = 2.0;      // kg

  // Create the spring-mass system
  SpringMassSystem<double> spring_mass(spring_k, mass, 0.);

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Create the integrator.
  const double h = 1.0/1024;
  RungeKutta2Integrator<double> integrator(spring_mass, h, context.get());

  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;

  // Set initial conditions using integrator's internal Context.
  spring_mass.set_position(integrator.get_mutable_context(),
                           initial_position);
  spring_mass.set_velocity(integrator.get_mutable_context(),
                           initial_velocity);

  // Take all the defaults.
  integrator.Initialize();

  // Build a dense output while integrating the solution.
  integrator.StartDenseIntegration();

  // Integrate for 1 second.
  const double t_final = 1.0;
  integrator.IntegrateWithMultipleStepsToTime(t_final);
  EXPECT_NEAR(context->get_time(), t_final, h);  // Should be exact.

  // Get the final position and velocity.
  const VectorBase<double>& xc_final = context->get_continuous_state_vector();
  double x_final = xc_final.GetAtIndex(0);

  // Get the closed form solution.
  double x_final_true, unused_v_final_true;
  spring_mass.GetClosedFormSolution(initial_position, initial_velocity,
                                    t_final, &x_final_true,
                                    &unused_v_final_true);

  // Check the solution.
  const double xtol = 5e-3;
  EXPECT_NEAR(x_final_true, x_final, xtol);

  // Reclaim dense output and prevent further updates to it.
  std::unique_ptr<trajectories::PiecewisePolynomial<double>> dense_output =
      integrator.StopDenseIntegration();

  // Verify that the built dense output is valid.
  for (double t = 0; t <= t_final; t += h / 2.) {
    double x_true, unused_v_true;
    spring_mass.GetClosedFormSolution(initial_position, initial_velocity,
                                      t, &x_true, &unused_v_true);
    const VectorX<double> x = dense_output->value(t);
    EXPECT_NEAR(x_true, x(0), xtol);
  }

  // Verify that integrator statistics are valid.
  CheckStatsValidity(&integrator);
}

// System where the state at t corresponds to the quadratic equation
// 4t² + 4t + C, where C is the initial value (the state at t=0).
class Quadratic : public LeafSystem<double> {
 public:
  Quadratic() { this->DeclareContinuousState(1); }

 private:
  void DoCalcTimeDerivatives(
    const Context<double>& context,
    ContinuousState<double>* deriv) const override {
    const double t = context.get_time();
    (*deriv)[0] = 8 * t + 4;
  }
};

// Tests accuracy for integrating the quadratic system (with the state at time t
// corresponding to f(t) ≡ 4t² + 4t + C, where C is the initial state) over
// t ∈ [0, 1]. The RK2 integrator is second order, meaning it uses the Taylor
// Series expansion:
// f(t+h) ≈ f(t) + hf'(t) + ½h²f''(t) + O(h³)
// This formula indicates that the approximation error will be zero if
// f'''(t) = 0, which is true for the quadratic equation. We check that the
// RK2 integrator is indeed able to obtain the true result.
GTEST_TEST(RK3IntegratorErrorEstimatorTest, QuadraticTest) {
  Quadratic quadratic;

  auto quadratic_context = quadratic.CreateDefaultContext();
  const double C = 0.0;
  quadratic_context->get_mutable_continuous_state_vector()[0] = C;

  const double t_final = 1.0;

  RungeKutta2Integrator<double> rk2(
      quadratic, t_final, quadratic_context.get());
  rk2.set_maximum_step_size(t_final);
  rk2.set_fixed_step_mode(true);
  rk2.Initialize();
  ASSERT_TRUE(rk2.IntegrateWithSingleFixedStepToTime(t_final));

  const double expected_result = t_final * (4 * t_final + 4) + C;
  EXPECT_NEAR(
      quadratic_context->get_continuous_state_vector()[0], expected_result,
      10 * std::numeric_limits<double>::epsilon());
}

GTEST_TEST(IntegratorTest, Symbolic) {
  using symbolic::Expression;
  using symbolic::Variable;

  // Create the mass spring system.
  SpringMassSystem<Expression> spring_mass(1., 1.);
  // Set the maximum step size.
  const double max_h = .01;
  // Create a context.
  auto context = spring_mass.CreateDefaultContext();
  // Create the integrator.
  RungeKutta2Integrator<Expression> integrator(
      spring_mass, max_h, context.get());
  integrator.Initialize();

  const Variable q("q");
  const Variable v("v");
  const Variable work("work");
  const Variable h("h");
  context->SetContinuousState(Vector3<Expression>(q, v, work));
  EXPECT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(h));
}

}  // namespace
}  // namespace systems
}  // namespace drake
