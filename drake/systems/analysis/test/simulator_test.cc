#include "drake/systems/analysis/simulator.h"
#include <cmath>
#include "gtest/gtest.h"
#include "drake/systems/analysis/test/my_spring_mass_system.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(SimulatorTest, MiscAPI) {
  MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // set the integrator default step size
  const double DT = 1e-3;

  // create a context
  auto context = spring_mass.CreateDefaultContext();

  // create the integrator
  std::unique_ptr<IntegratorBase<double>> integrator(
      new ExplicitEulerIntegrator<double>(
          spring_mass, DT, context.get()));  // Use default Context.
  simulator.reset_integrator(integrator);

  // initialize the simulator first
  simulator.Initialize();

  EXPECT_EQ(simulator.get_ideal_next_step_size(), DT);
}

GTEST_TEST(SimulatorTest, ContextAccess) {
  MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // set the integrator default step size
  const double DT = 1e-3;

  // get the context
  auto context = simulator.get_mutable_context();

  // create the integrator
  std::unique_ptr<IntegratorBase<double>> integrator(
      new ExplicitEulerIntegrator<double>(spring_mass, DT,
                                          context));  // Use default Context.
  simulator.reset_integrator(integrator);

  // initialize the simulator first
  simulator.Initialize();

  simulator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
  simulator.release_context();
  EXPECT_TRUE(simulator.get_mutable_context() == nullptr);
}

// Try a purely continuous system with no sampling.
GTEST_TEST(SimulatorTest, SpringMassNoSample) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // set the integrator default step size
  const double DT = 1e-3;

  MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // get the context
  auto context = simulator.get_mutable_context();

  // create the integrator and initialize it
  std::unique_ptr<IntegratorBase<double>> integrator(
      new ExplicitEulerIntegrator<double>(spring_mass, DT, context));
  integrator->Initialize();

  // set the integrator and initialize the simulator
  simulator.reset_integrator(integrator);
  simulator.Initialize();

  // Simulate for 1 second.
  simulator.StepTo(1.);

  EXPECT_EQ(context->get_time(), 1.);  // Should be exact.
  EXPECT_EQ(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_updates(), 0);
  EXPECT_LE(simulator.get_smallest_step_size_taken(),
            simulator.get_largest_step_size_taken());

  // Publish() should get called at start and finish.
  EXPECT_EQ(spring_mass.get_publish_count(), 1001);
  EXPECT_EQ(spring_mass.get_update_count(), 0);

  // Current time is 1. An earlier final time should fail.
  EXPECT_THROW(simulator.StepTo(0.5), std::runtime_error);
}

// Repeat the previous test but now the continuous steps are interrupted
// by a discrete sample every 1/30 second. The step size doesn't divide that
// evenly so we should get some step size modification here.
GTEST_TEST(SimulatorTest, SpringMass) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // set the integrator default step size
  const double DT = 1e-3;

  // create the mass spring system and the simulator
  MySpringMassSystem<double> spring_mass(kSpring, kMass, 30.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // get the context
  auto context = simulator.get_mutable_context();

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // create the integrator and initialize it
  std::unique_ptr<IntegratorBase<double>> integrator(
      new ExplicitEulerIntegrator<double>(spring_mass, DT, context));
  integrator->Initialize();

  // set the integrator and initialize the simulator
  simulator.reset_integrator(integrator);
  simulator.Initialize();

  // simulate up to one second
  simulator.StepTo(1.);

  EXPECT_GT(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_updates(), 30);
  EXPECT_LE(simulator.get_smallest_step_size_taken(),
            simulator.get_largest_step_size_taken());

  // We're calling Publish() every step, and extra steps have to be taken
  // since the step size doesn't divide evenly into the sample rate. Shouldn't
  // require more than one extra step per sample though.
  EXPECT_LE(spring_mass.get_publish_count(), 1030);
  EXPECT_EQ(spring_mass.get_update_count(), 30);
}

}  // namespace
}  // namespace systems
}  // namespace drake
