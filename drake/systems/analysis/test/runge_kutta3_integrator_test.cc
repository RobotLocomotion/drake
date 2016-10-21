#include "drake/systems/analysis/runge_kutta3_integrator.h"

// TODO(edrumwri): Remove this and fix corresponding build errors.
#include "drake/systems/analysis/runge_kutta3_integrator-inl.h"

#include <cmath>

#include "gtest/gtest.h"

#include "drake/systems/analysis/test/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(IntegratorTest, MiscAPI) {
  MySpringMassSystem<double> spring_mass(1., 1., 0.);

  // Setup the integration step size.
  const double DT = 1e-3;

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Create the integrator.
  RungeKutta3Integrator<double> integrator(spring_mass, context.get());

  // Set the accuracy.
  integrator.request_initial_step_size_target(DT);

  EXPECT_EQ(integrator.get_initial_step_size_target(), DT);
}

GTEST_TEST(IntegratorTest, ContextAccess) {
  // Create the mass spring system
  MySpringMassSystem<double> spring_mass(1., 1., 0.);

  // Create a context
  auto context = spring_mass.CreateDefaultContext();

  // Create the integrator
  RungeKutta3Integrator<double> integrator(
      spring_mass, context.get());  // Use default Context.

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
  MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);

  // Create a context
  auto context = spring_mass.CreateDefaultContext();

  // Create the integrator.
  RungeKutta3Integrator<double> integrator(
      spring_mass, context.get());  // Use default Context.

  // Set integrator parameters.
  const double DT = 1e-3;
  integrator.set_maximum_step_size(DT);
  integrator.set_minimum_step_size(DT);
  integrator.set_target_accuracy(1e-5);

  // Setup the initial position and initial velocity
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.0;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(integrator.get_mutable_context(), kInitialPosition);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double C1 = kInitialPosition;
  const double C2 = kInitialVelocity / kOmega;

  // Integrate for 1 second.
  const double T_FINAL = 1.0;
  for (double t = 0.0; t < T_FINAL; t += DT) integrator.Step(DT, DT);

  // Get the final position.
  const double kXFinal =
      context->get_state().get_continuous_state()->get_state().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(C1 * std::cos(kOmega * T_FINAL) + C2 * std::sin(kOmega * T_FINAL),
              kXFinal, 1e-5);
}

// Integrate a purely continuous system with no sampling using error control.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
GTEST_TEST(IntegratorTest, SpringMassStepEC) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // Create the spring-mass system
  MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);

  // Create a context
  auto context = spring_mass.CreateDefaultContext();

  // Create the integrator.
  RungeKutta3Integrator<double> integrator(
      spring_mass, context.get());  // Use default Context.

  // Set reasonable integrator parameters.
  integrator.set_maximum_step_size(0.1);
  integrator.set_minimum_step_size(1e-6);
  integrator.set_target_accuracy(1e-5);

  // Setup the initial position and initial velocity
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.0;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(integrator.get_mutable_context(), kInitialPosition);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double C1 = kInitialPosition;
  const double C2 = kInitialVelocity / kOmega;

  // Integrate for 1 second.
  const double T_FINAL = 1.0;
  double t_remaining = T_FINAL - context->get_time();
  do {
    integrator.Step(t_remaining, t_remaining);
    t_remaining = T_FINAL - context->get_time();
  } while (t_remaining > 0.0);

  // Get the final position.
  const double kXFinal =
      context->get_state().get_continuous_state()->get_state().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(C1 * std::cos(kOmega * T_FINAL) + C2 * std::sin(kOmega * T_FINAL),
              kXFinal, 1e-5);
}

}  // namespace
}  // namespace systems
}  // namespace drake
