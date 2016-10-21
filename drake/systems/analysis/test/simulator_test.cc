#include "drake/systems/analysis/simulator.h"
#include <cmath>
#include <complex>

#include <unsupported/Eigen/AutoDiff>

#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/test/my_spring_mass_system.h"
#include "drake/systems/analysis/test/controlled_spring_mass_system/controlled_spring_mass_system.h"
#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"
#include "gtest/gtest.h"

using Eigen::AutoDiffScalar;
using Eigen::NumTraits;
using std::complex;

namespace drake {
namespace systems {
namespace {

GTEST_TEST(SimulatorTest, SecondConstructor) {
  // Create the spring-mass sytem and context.
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  auto context = spring_mass.CreateDefaultContext();

  // Mark the context with an arbitrary value
  context->set_time(3.);

  /// Construct the simulator with the created context.
  Simulator<double> simulator(spring_mass, std::move(context));

  // Verify that context pointers are equivalent.
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
}

GTEST_TEST(SimulatorTest, MiscAPI) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set the integrator default step size.
  const double DT = 1e-3;

  // Create a context.
  auto context = simulator.get_mutable_context();

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, DT,
                                                              context);

  // Initialize the simulator first.
  simulator.Initialize();
}

GTEST_TEST(SimulatorTest, ContextAccess) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // set the integrator default step size
  const double DT = 1e-3;

  // get the context
  auto context = simulator.get_mutable_context();

  // create the integrator
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, DT,
                                                              context);

  // initialize the simulator first
  simulator.Initialize();

  // try some other context stuff
  simulator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
  simulator.release_context();
  EXPECT_TRUE(simulator.get_mutable_context() == nullptr);
  EXPECT_THROW(simulator.Initialize(), std::logic_error);

  // create another context
  auto ucontext = spring_mass.CreateDefaultContext();
  ucontext->set_time(3.);
  simulator.reset_context(std::move(ucontext));
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
  simulator.reset_context(nullptr);
  EXPECT_TRUE(simulator.get_mutable_context() == nullptr);
}

// Try a purely continuous system with no sampling.
GTEST_TEST(SimulatorTest, SpringMassNoSample) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // set the integrator default step size
  const double DT = 1e-3;

  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // get the context
  auto context = simulator.get_mutable_context();

  // create the integrator
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, DT,
                                                              context);

  // set the integrator and initialize the simulator
  simulator.Initialize();

  // Simulate for 1 second.
  simulator.StepTo(1.);

  EXPECT_NEAR(context->get_time(), 1., 1e-8);
  EXPECT_EQ(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_updates(), 0);

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
  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 30.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // get the context
  auto context = simulator.get_mutable_context();

  // TODO(edrumwri): remove this when discrete state has been created
  // automatically
  // Create the discrete state
  context->set_difference_state(std::make_unique<DifferenceState<double>>());

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // create the integrator and initialize it
  auto integrator = simulator.reset_integrator<ExplicitEulerIntegrator<double>>(
      spring_mass, DT, context);
  integrator->Initialize();

  // set the integrator and initialize the simulator
  simulator.Initialize();

  // simulate up to one second
  simulator.StepTo(1.);

  EXPECT_GT(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_updates(), 30);

  // We're calling Publish() every step, and extra steps have to be taken
  // since the step size doesn't divide evenly into the sample rate. Shouldn't
  // require more than one extra step per sample though.
  EXPECT_LE(spring_mass.get_publish_count(), 1030);
  EXPECT_EQ(spring_mass.get_update_count(), 30);
}

// Tests Simulator for a Diagram system consisting of a tree of systems.
// In this case the System is a PidControlledSpringMassSystem which is a
// Diagram containing a SpringMassSystem (the plant) and a PidController which
// in turn is a Diagram composed of primitives such as Gain and Adder systems.
GTEST_TEST(SimulatorTest, ControlledSpringMass) {
  typedef complex<double> complexd;
  typedef AutoDiffScalar<Vector1d> SingleVarAutoDiff;

  // SpringMassSystem parameters.
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg
  // System's open loop frequency.
  const double wol = std::sqrt(kSpring / kMass);

  // Initial conditions.
  const double x0 = 0.1;
  const double v0 = 0.0;

  // Choose some controller constants.
  const double kp = kSpring;
  const double ki = 0.0;
  // System's undamped frequency (when kd = 0).
  const double w0 = std::sqrt(wol * wol + kp / kMass);
  // Damping ratio (underdamped).
  const double zeta = 0.5;
  const double kd = 2.0 * kMass * w0 * zeta;
  const double x_target = 0.0;

  PidControlledSpringMassSystem<double> spring_mass(kSpring, kMass, kp, ki, kd,
                                                    x_target);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Sets initial conditions to zero.
  spring_mass.SetDefaultState(simulator.get_mutable_context());

  // Sets initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), x0);
  spring_mass.set_velocity(simulator.get_mutable_context(), v0);

  // Takes all the defaults for the simulator.
  simulator.Initialize();

  // Computes analytical solution.
  // 1) Roots of the characteristic equation.
  complexd lambda1 = -zeta * w0 + w0 * std::sqrt(complexd(zeta * zeta - 1));
  complexd lambda2 = -zeta * w0 - w0 * std::sqrt(complexd(zeta * zeta - 1));

// Roots should be the complex conjugate of each other.
#ifdef __APPLE__
  // The factor of 20 is needed for OS X builds where the comparison needs a
  // looser tolerance, see #3636.
  auto abs_error = 20.0 * NumTraits<double>::epsilon();
#else
  auto abs_error = NumTraits<double>::epsilon();
#endif
  EXPECT_NEAR(lambda1.real(), lambda2.real(), abs_error);
  EXPECT_NEAR(lambda1.imag(), -lambda2.imag(), abs_error);

  // The damped frequency corresponds to the absolute value of the imaginary
  // part of any of the roots.
  double wd = std::abs(lambda1.imag());

  // 2) Differential equation's constants of integration.
  double C1 = x0;
  double C2 = (zeta * w0 * x0 + v0) / wd;

  // 3) Computes analytical solution at time final_time.
  // Velocity is computed using AutoDiffScalar.
  double final_time = 0.2;
  double x_final{}, v_final{};
  {
    // At the end of this local scope x_final and v_final are properly
    // initialized.
    // Auxiliary AutoDiffScalar variables are confined to this local scope so
    // that we don't pollute the test's scope with them.
    SingleVarAutoDiff time(final_time);
    time.derivatives() << 1.0;
    auto x =
        exp(-zeta * w0 * time) * (C1 * cos(wd * time) + C2 * sin(wd * time));
    x_final = x.value();
    v_final = x.derivatives()[0];
  }

  // Simulates to final_time.
  simulator.StepTo(final_time);

  EXPECT_EQ(simulator.get_num_steps_taken(), 200);

  const auto& context = simulator.get_context();
  EXPECT_NEAR(context.get_time(), final_time, 1e-8);

  // Compares with analytical solution (to numerical integration error).
  EXPECT_NEAR(spring_mass.get_position(context), x_final, 3.0e-6);
  EXPECT_NEAR(spring_mass.get_velocity(context), v_final, 1.0e-5);
}

}  // namespace
}  // namespace systems
}  // namespace drake
