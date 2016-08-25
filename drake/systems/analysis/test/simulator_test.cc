#include "drake/systems/analysis/simulator.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "drake/systems/framework/examples/spring_mass_system.h"

#include "gtest/gtest.h"

using std::cout;
using std::endl;

namespace drake {
namespace systems {
namespace {

class MySpringMassSystem : public SpringMassSystem {
 public:
  // Pass through to SpringMassSystem, except add sample rate in samples/s.
  MySpringMassSystem(double stiffness, double mass, double sample_rate)
      : SpringMassSystem(stiffness, mass, false /*no input force*/),
        sample_rate_(sample_rate) {}

  // Set this true to publish trajectory to stdout.
  void set_publish(bool publish) { publish_ = publish; }

 private:
  // Publish t q u to standard output.
  void DoPublish(const ContextBase<double>& context) const override {
    if (!publish_) return;

    const StateVector<double>& xc =
        context.get_state().continuous_state->get_state();

    cout << context.get_time() << " " << xc.GetAtIndex(0) << " "
         << xc.GetAtIndex(1) << endl;
  }

  void DoUpdate(ContextBase<double>* context,
                const SampleActions& actions) const override {}

  // Force a sample at the next multiple of the sample rate. If the current
  // time is exactly at a sample time, we assume the sample has already been
  // done and return the following sample time. That means we don't get a
  // sample at 0 but will get one at the end.
  double DoCalcNextSampleTime(const ContextBase<double>& context,
                              SampleActions* actions) const override {
    if (sample_rate_ <= 0.) {
      actions->time = std::numeric_limits<double>::infinity();
      return actions->time;
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
    return actions->time;
  }

 private:
  bool publish_{false};     // Whether to write to stdout.
  double sample_rate_{0.};  // Default is "don't sample".
};

// Try a purely continuous system with no sampling.
GTEST_TEST(SimulatorTest, SpringMassNoSample) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  MySpringMassSystem spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // use default Context

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
  EXPECT_EQ(simulator.get_num_discrete_samples(), 0);

  cout << "num steps taken=" << simulator.get_num_steps_taken() << endl;
  cout << "num discrete samples=" << simulator.get_num_discrete_samples()
       << endl;
  cout << "actual initial step="
       << simulator.get_actual_initial_step_size_taken() << endl;
  cout << "min/max step=" << simulator.get_smallest_step_size_taken() << " / "
       << simulator.get_largest_step_size_taken() << endl;
}

// Repeat the previous test but now the continuous steps are interrupted
// by a discrete sample every 1/30 second. The step size doesn't divide that
// evenly so we should get some step size modification here.
GTEST_TEST(SimulatorTest, SpringMass) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  MySpringMassSystem spring_mass(kSpring, kMass, 30.);
  // spring_mass.set_publish(true);

  Simulator<double> simulator(spring_mass);  // use default Context

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  simulator.request_initial_stepsize(1e-3);
  simulator.set_integrator_type(IntegratorType::RungeKutta2);
  simulator.Initialize();

  EXPECT_TRUE(simulator.get_integrator_type_in_use() ==
              IntegratorType::RungeKutta2);

  simulator.StepTo(1.);

  EXPECT_GT(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_discrete_samples(), 30);

  cout << "num steps taken=" << simulator.get_num_steps_taken() << endl;
  cout << "num discrete samples=" << simulator.get_num_discrete_samples()
       << endl;
  cout << "actual initial step="
       << simulator.get_actual_initial_step_size_taken() << endl;
  cout << "min/max step=" << simulator.get_smallest_step_size_taken() << " / "
       << simulator.get_largest_step_size_taken() << endl;
}

}  // namespace
}  // namespace systems
}  // namespace drake
