#include"drake/systems/analysis/runge_kutta2_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/systems/analysis/test/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(IntegratorTest, MiscAPI) {
  // Create the spring-mass system.
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);

  // Setup integration step.
  const double dt  = 1e-3;

  // Create the integrator.
  RungeKutta2Integrator<double> integrator(spring_mass, dt);

  // Test that setting the target accuracy fails.
  EXPECT_THROW(integrator.set_target_accuracy(1.0), std::logic_error);
  EXPECT_THROW(integrator.request_initial_step_size_target(1.0),
               std::logic_error);
}

GTEST_TEST(IntegratorTest, ContextAccess) {
  // Create the mass spring system.
  SpringMassSystem<double> spring_mass(1., 1., 0.);

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Setup integration step.
  const double dt  = 1e-3;

  // Create the integrator
  RungeKutta2Integrator<double> integrator(spring_mass, dt, context.get());
  integrator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(context->get_time(), 3.);
}

/// Verifies error estimation is unsupported.
GTEST_TEST(IntegratorTest, ErrorEst) {
  // Spring-mass system is necessary only to setup the problem.
  SpringMassSystem<double> spring_mass(1., 1., 0.);
  const double dt = 1e-3;
  auto context = spring_mass.CreateDefaultContext();
  RungeKutta2Integrator<double> integrator(
      spring_mass, dt, context.get());

  EXPECT_EQ(integrator.get_error_estimate_order(), 0);
  EXPECT_EQ(integrator.supports_error_estimation(), false);
  EXPECT_THROW(integrator.set_fixed_step_mode(false), std::logic_error);
  EXPECT_THROW(integrator.set_target_accuracy(1e-1), std::logic_error);
  EXPECT_THROW(integrator.request_initial_step_size_target(dt),
               std::logic_error);
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
  const double dt = 1.0/1024;
  const double inf = std::numeric_limits<double>::infinity();
  RungeKutta2Integrator<double> integrator(spring_mass, dt, context.get());

  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(spring_k / mass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(integrator.get_mutable_context(),
                           initial_position);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double C1 = initial_position;
  const double C2 = initial_velocity / omega;

  // Integrate for 1 second.
  const double t_final = 1.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) > dt; t += dt)
    integrator.StepOnceAtMost(inf, inf, dt);

  EXPECT_NEAR(context->get_time(), 1., dt);  // Should be exact.

  // Get the final position.
  const double x_final = context->get_continuous_state_vector().GetAtIndex(0);

  // Check the solution.
  double true_sol = C1 * std::cos(omega * t) + C2 * std::sin(omega * t);
  EXPECT_NEAR(true_sol, x_final, 5e-3);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator.get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator.get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator.get_num_steps_taken(), 0);
  EXPECT_EQ(integrator.get_error_estimate(), nullptr);
}

}  // namespace
}  // namespace systems
}  // namespace drake
