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

class HalfExplicitDAE1SolverTest : public ::testing::Test {
 public:
  HalfExplicitDAE1SolverTest() {
    // Create and initialize the bead on the wire.
    bead_on_a_wire_ = std::make_unique<BeadOnAWire<double>>(
        BeadOnAWire<double>::kAbsoluteCoordinates);

    // Create the context.
    context_ = bead_on_a_wire_->CreateDefaultContext();

    // Fix an input port for gravity.
    std::unique_ptr<BasicVector<double>> grav_input =
      std::make_unique<BasicVector<double>>(3);
    grav_input->SetAtIndex(0, 0.0);
    grav_input->SetAtIndex(1, 0.0);
    grav_input->SetAtIndex(2, -1.0);
    context_->FixInputPort(0, std::move(grav_input));

    // Create and initialize the integrator.
    integrator_ = std::make_unique<HalfExplicitDAE1Solver<double>>(
        *bead_on_a_wire_, dt_, context_.get());  // Use default Context.
  }

  std::unique_ptr<BeadOnAWire<double>> bead_on_a_wire_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<HalfExplicitDAE1Solver<double>> integrator_;
  const double dt_ = 1e-3;        // Integration step size.
};

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
TEST_F(HalfExplicitDAE1SolverTest, BeadOnAWire) {
  // Setup the integration size and infinity.
  const double inf = std::numeric_limits<double>::infinity();

  // Take all the defaults.
  integrator_->Initialize();

  // Integrate for 1 second.
  const double t_final = 1.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) >= dt_; t += dt_)
    integrator_->StepOnceAtMost(inf, inf, dt_);

  // TODO(edrumwri): Insert a proper test in here.
}

}  // namespace
}  // namespace systems
}  // namespace drake
