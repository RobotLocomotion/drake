#include <cmath>
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/bead_on_a_wire/bead_on_a_wire.h"
#include "drake/examples/bead_on_a_wire/bead_on_a_wire-inl.h"
#include "drake/systems/analysis/half_explicit_DAE1_solver.h"
#include "drake/systems/analysis/test/my_spring_mass_system.h"
#include "gtest/gtest.h"

using drake::examples::bead_on_a_wire::BeadOnAWire;

namespace drake {
namespace systems {
namespace {

// Try a purely continuous system with no sampling.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
GTEST_TEST(IntegratorTest, SpringMassStep) {
  const double spring_k = 300.0;  // N/m
  const double mass = 2.0;      // kg

  // Create the spring-mass system.
  SpringMassSystem<double> spring_mass(spring_k, mass, 0.);

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Setup the integration size and infinity.
  const double dt = 1e-6;
  const double inf = std::numeric_limits<double>::infinity();

  // Create the integrator.
  HalfExplicitDAE1Solver<double> integrator(
      spring_mass, dt, context.get());  // Use default Context.

  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(spring_k / mass);

  // Set initial condition.
  spring_mass.set_position(context.get(), initial_position);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double c1 = initial_position;
  const double c2 = initial_velocity / omega;

  // Integrate for 1 second.
  const double t_final = 1.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) > dt; t += dt)
    integrator.StepOnceAtMost(inf, inf, dt);

  EXPECT_NEAR(context->get_time(), t, dt);  // Should be exact.

  // Get the final position.
  const double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(c1 * std::cos(omega * t) + c2 * std::sin(omega * t), x_final,
              5e-3);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator.get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator.get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator.get_num_steps_taken(), 0);
  EXPECT_EQ(integrator.get_error_estimate(), nullptr);
}

// Simulate the bead on the wire example.
GTEST_TEST(IntegratorTest, BeadOnAWire) {
  // Setup the integration size and infinity.
  const double inf = std::numeric_limits<double>::infinity();
  const double dt = 1e-5;

  // Create and initialize the beads on the wire.
  std::unique_ptr<BeadOnAWire<double>> bw_abs =
      std::make_unique<BeadOnAWire<double>>(
          BeadOnAWire<double>::kAbsoluteCoordinates);
  std::unique_ptr<BeadOnAWire<double>> bw_min =
      std::make_unique<BeadOnAWire<double>>(
          BeadOnAWire<double>::kMinimalCoordinates);

  // Create the context.
  auto context_min = bw_min->CreateDefaultContext();
  auto context_abs = bw_abs->CreateDefaultContext();

  // Fix unused input ports.
  std::unique_ptr<BasicVector<double>> input_abs =
      std::make_unique<BasicVector<double>>(3);
  std::unique_ptr<BasicVector<double>> input_min =
      std::make_unique<BasicVector<double>>(1);
  input_abs->SetAtIndex(0, 0.0);
  input_abs->SetAtIndex(1, 0.0);
  input_abs->SetAtIndex(2, 0.0);
  input_min->SetAtIndex(0, 0.0);
  context_abs->FixInputPort(0, std::move(input_abs));
  context_min->FixInputPort(0, std::move(input_min));

  // Create the integrators.
  HalfExplicitDAE1Solver<double> integrator_abs(*bw_abs, dt, context_abs.get());
  HalfExplicitDAE1Solver<double> integrator_min(*bw_min, dt, context_min.get());

  // Set the tolerance to very tight on the constraint tolerance for the bead on
  // the wire in absolute coordinates.
  integrator_abs.set_constraint_error_tolerance(1e-9);

  // Initialize the integrators.
  integrator_abs.Initialize();
  integrator_min.Initialize();

  // Integrate for a single step.
  integrator_abs.StepOnceAtMost(inf, inf, dt);
  integrator_min.StepOnceAtMost(inf, inf, dt);

  // Since the parametric sinusoidal function [ cos(s) sin(s) s ] is default,
  // we should expect last index of absolute coordinates and only index of
  // minimal coordinate to match.
  const double tol = 25*integrator_abs.get_constraint_error_tolerance();
  EXPECT_NEAR(context_abs->get_continuous_state_vector().GetAtIndex(2),
              context_min->get_continuous_state_vector().GetAtIndex(0), tol);
}

}  // namespace
}  // namespace systems
}  // namespace drake
