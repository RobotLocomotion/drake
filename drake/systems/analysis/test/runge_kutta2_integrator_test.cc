#include"drake/systems/analysis/runge_kutta2_integrator.h"

#include <cmath>

#include "gtest/gtest.h"

#include "drake/systems/analysis/test/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(IntegratorTest, MiscAPI) {
  // Create the spring-mass system.
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);

  // Setup integration step.
  const double DT  = 1e-3;

  // Create the integrator.
  RungeKutta2Integrator<double> integrator(spring_mass, DT);

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
  const double DT  = 1e-3;

  // Create the integrator
  RungeKutta2Integrator<double> integrator(spring_mass, DT, context.get());
  integrator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(context->get_time(), 3.);
}

// Try a purely continuous system with no sampling.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
GTEST_TEST(IntegratorTest, SpringMassStep) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // Create the spring-mass system
  SpringMassSystem<double> spring_mass(kSpring, kMass, 0.);

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Create the integrator.
  const double DT = 1.0/1024;
  const double INF = std::numeric_limits<double>::infinity();
  RungeKutta2Integrator<double> integrator(spring_mass, DT, context.get());

  // Setup the initial position and initial velocity.
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.0;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(integrator.get_mutable_context(),
                           kInitialPosition);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double C1 = kInitialPosition;
  const double C2 = kInitialVelocity / kOmega;

  // Integrate for 1 second.
  const double T_FINAL = 1.0;
  double t;
  for (t = 0.0; std::abs(t - T_FINAL) > DT; t += DT)
    integrator.Step(INF, INF);

  EXPECT_NEAR(context->get_time(), 1., DT);  // Should be exact.

  // Get the final position.
  const double kXFinal = context->get_continuous_state_vector().GetAtIndex(0);

  // Check the solution.
  double true_sol = C1 * std::cos(kOmega * t) + C2 * std::sin(kOmega * t);
  EXPECT_NEAR(true_sol, kXFinal, 1e-5);
}

}  // namespace
}  // namespace systems
}  // namespace drake
