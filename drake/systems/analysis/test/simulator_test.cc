#include "drake/systems/analysis/simulator.h"

#include <cmath>
#include <complex>

#include <unsupported/Eigen/AutoDiff>

#include "gtest/gtest.h"

#include "drake/systems/framework/examples/controlled_spring_mass_system.h"
#include "drake/systems/framework/examples/spring_mass_system.h"

using Eigen::AutoDiffScalar;
using Eigen::NumTraits;
using std::complex;

namespace drake {
namespace systems {
namespace {

class MySpringMassSystem : public SpringMassSystem<double> {
 public:
  // Pass through to SpringMassSystem, except add sample rate in samples/s.
  MySpringMassSystem(double stiffness, double mass, double sample_rate)
      : SpringMassSystem<double>(stiffness, mass, false /*no input force*/),
        sample_rate_(sample_rate) {}

  int get_publish_count() const { return publish_count_; }
  int get_update_count() const { return update_count_; }

 private:
  // Publish t q u to standard output.
  void DoPublish(const Context<double>& context) const override {
    ++publish_count_;
  }

  void DoUpdate(Context<double>* context,
                const SampleActions& actions) const override {
    ++update_count_;
  }

  // Force a sample at the next multiple of the sample rate. If the current
  // time is exactly at a sample time, we assume the sample has already been
  // done and return the following sample time. That means we don't get a
  // sample at 0 but will get one at the end.
  void DoCalcNextSampleTime(const Context<double>& context,
                            SampleActions* actions) const override {
    if (sample_rate_ <= 0.) {
      actions->time = std::numeric_limits<double>::infinity();
      return;
    }

    // For reliable behavior, convert floating point times into integer
    // sample counts. We want the ceiling unless it is the same as the floor.
    const int prev =
        static_cast<int>(std::floor(context.get_time() * sample_rate_));
    const int next =
        static_cast<int>(std::ceil(context.get_time() * sample_rate_));
    const int which = next == prev ? next + 1 : next;

    // Convert the next sample count back to a time to return.
    const double next_sample = which / sample_rate_;
    actions->time = next_sample;
  }

  double sample_rate_{0.};  // Default is "don't sample".

  mutable int publish_count_{0};
  mutable int update_count_{0};
};

GTEST_TEST(SimulatorTest, MiscAPI) {
  MySpringMassSystem spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  simulator.set_integrator_type(IntegratorType::ExplicitEuler);
  simulator.set_accuracy(1e-6);
  simulator.request_initial_step_size_attempt(1e-8);

  simulator.Initialize();
  EXPECT_TRUE(simulator.get_integrator_type_in_use() ==
              IntegratorType::ExplicitEuler);
  EXPECT_EQ(simulator.get_accuracy_in_use(), 1e-6);
  EXPECT_EQ(simulator.get_initial_step_size_attempt_in_use(), 1e-8);

  EXPECT_EQ(simulator.get_ideal_next_step_size(), 1e-8);
}

GTEST_TEST(SimulatorTest, ContextAccess) {
  MySpringMassSystem spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  simulator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(simulator.get_context().get_time(), 3.);

  auto context = simulator.release_context();
  EXPECT_TRUE(simulator.get_mutable_context() == nullptr);
  EXPECT_EQ(context->get_time(), 3.);
}

// Try a purely continuous system with no sampling.
GTEST_TEST(SimulatorTest, SpringMassNoSample) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  MySpringMassSystem spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // Take all the defaults.
  simulator.Initialize();

  EXPECT_TRUE(simulator.get_integrator_type_in_use() ==
              IntegratorType::RungeKutta2);

  // Simulate for 1 second.
  simulator.StepTo(1.);

  const auto& context = simulator.get_context();
  EXPECT_EQ(context.get_time(), 1.);  // Should be exact.

  EXPECT_EQ(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_samples_taken(), 0);
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

  MySpringMassSystem spring_mass(kSpring, kMass, 30.);

  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  simulator.request_initial_step_size_attempt(1e-3);
  simulator.set_integrator_type(IntegratorType::RungeKutta2);
  simulator.Initialize();

  EXPECT_TRUE(simulator.get_integrator_type_in_use() ==
              IntegratorType::RungeKutta2);

  simulator.StepTo(1.);

  EXPECT_GT(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_samples_taken(), 30);
  EXPECT_LE(simulator.get_smallest_step_size_taken(),
            simulator.get_largest_step_size_taken());

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

  PidControlledSpringMassSystem<double> spring_mass(
      kSpring, kMass, kp, ki, kd, x_target);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Sets initial conditions to zero.
  spring_mass.SetDefaultState(simulator.get_mutable_context());

  // Sets initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), x0);
  spring_mass.set_velocity(simulator.get_mutable_context(), v0);

  // Takes all the defaults for the simulator.
  simulator.Initialize();

  EXPECT_TRUE(simulator.get_integrator_type_in_use() ==
      IntegratorType::RungeKutta2);

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
  EXPECT_NEAR(lambda1.real(),  lambda2.real(), abs_error);
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
  { // At the end of this local scope x_final and v_final are properly
    // initialized.
    // Auxiliary AutoDiffScalar variables are confined to this local scope so
    // that we don't pollute the test's scope with them.
    SingleVarAutoDiff time(final_time);
    time.derivatives() << 1.0;
    auto x = exp(-zeta * w0 * time) * (
        C1 * cos(wd * time) + C2 * sin(wd * time));
    x_final = x.value();
    v_final = x.derivatives()[0];
  }

  // Simulates to final_time.
  simulator.StepTo(final_time);

  EXPECT_EQ(simulator.get_num_steps_taken(), 200);
  EXPECT_NEAR(simulator.get_smallest_step_size_taken(),
              simulator.get_largest_step_size_taken(),
              NumTraits<double>::epsilon());

  const auto& context = simulator.get_context();
  EXPECT_EQ(context.get_time(), final_time);  // Should be exact.

  // Compares with analytical solution (to numerical integration error).
  EXPECT_NEAR(spring_mass.get_position(context), x_final, 3.0e-6);
  EXPECT_NEAR(spring_mass.get_velocity(context), v_final, 1.0e-5);
}

}  // namespace
}  // namespace systems
}  // namespace drake
