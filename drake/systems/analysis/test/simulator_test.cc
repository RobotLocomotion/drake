#include "drake/systems/analysis/simulator.h"

#include <cmath>
#include <complex>
#include <functional>
#include <map>

#include <gtest/gtest.h>

#include <unsupported/Eigen/AutoDiff>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/text_logging.h"
#include "drake/common/test/is_dynamic_castable.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/test/controlled_spring_mass_system/controlled_spring_mass_system.h"
#include "drake/systems/analysis/test/empty_system.h"
#include "drake/systems/analysis/test/logistic_system.h"
#include "drake/systems/analysis/test/my_spring_mass_system.h"
#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

using drake::systems::WitnessFunction;
using drake::systems::Simulator;
using drake::systems::RungeKutta3Integrator;
using drake::systems::ImplicitEulerIntegrator;
using LogisticSystem = drake::systems::analysis_test::LogisticSystem<double>;
using EmptySystem = drake::systems::analysis_test::EmptySystem<double>;
using LogisticWitness = drake::systems::analysis_test::LogisticWitness<double>;
using ClockWitness = drake::systems::analysis_test::ClockWitness<double>;
using Eigen::AutoDiffScalar;
using Eigen::NumTraits;
using std::complex;

namespace drake {
namespace systems {
namespace {

// @TODO(edrumwri): Use test fixtures to streamline this file and promote reuse.

// A composite system using the logistic system with the clock-based
// witness function.
class CompositeSystem : public LogisticSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompositeSystem)

  CompositeSystem(double k, double alpha, double nu, double trigger_time) :
      LogisticSystem(k, alpha, nu) {
    this->DeclareContinuousState(1);
    logistic_witness_ = std::make_unique<LogisticWitness>(*this);
    clock_witness_ = std::make_unique<ClockWitness>(trigger_time, *this,
                          WitnessFunction<double>::DirectionType::kCrossesZero);
  }

 protected:
  void DoGetWitnessFunctions(
      const Context<double>&,
      std::vector<const systems::WitnessFunction<double>*>* w) const override {
    w->push_back(clock_witness_.get());
    w->push_back(logistic_witness_.get());
  }

 private:
  std::unique_ptr<LogisticWitness> logistic_witness_;
  std::unique_ptr<ClockWitness> clock_witness_;
};

// An empty system using two clock witnesses.
class TwoWitnessEmptySystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TwoWitnessEmptySystem)

  explicit TwoWitnessEmptySystem(double off1, double off2) {
    const auto dir_type = WitnessFunction<double>::DirectionType::kCrossesZero;
    witness1_ = std::make_unique<ClockWitness>(off1, *this, dir_type);
    witness2_ = std::make_unique<ClockWitness>(off2, *this, dir_type);
  }

  void set_publish_callback(
      std::function<void(const Context<double>&)> callback) {
    publish_callback_ = callback;
  }

 protected:
  void DoCalcOutput(const Context<double>&,
                    SystemOutput<double>*) const override {}

  void DoGetWitnessFunctions(
      const systems::Context<double>&,
      std::vector<const systems::WitnessFunction<double>*>* w) const override {
    w->push_back(witness1_.get());
    w->push_back(witness2_.get());
  }

  void DoPublish(
      const drake::systems::Context<double>& context) const override {
    if (publish_callback_ != nullptr) publish_callback_(context);
  }

 private:
  std::unique_ptr<ClockWitness> witness1_, witness2_;
  std::function<void(const Context<double>&)> publish_callback_{nullptr};
};

// Disables non-witness based publishing for witness function testing.
void DisableDefaultPublishing(Simulator<double>* s) {
  s->set_publish_at_initialization(false);
  s->set_publish_every_time_step(false);
}

// Initializes the Simulator's integrator to fixed step mode for witness
// function related tests.
void InitFixedStepIntegratorForWitnessTesting(Simulator<double>* s, double dt) {
  const System<double>& system = s->get_system();
  auto context = s->get_mutable_context();
  s->reset_integrator<RungeKutta2Integrator<double>>(system, dt, context);
  s->get_mutable_integrator()->set_fixed_step_mode(true);
  s->get_mutable_integrator()->set_maximum_step_size(dt);
  DisableDefaultPublishing(s);
}

// Initializes the Simulator's integrator to variable step mode for witness
// function related tests.
void InitVariableStepIntegratorForWitnessTesting(Simulator<double>* s) {
  const System<double>& system = s->get_system();
  auto context = s->get_mutable_context();
  s->reset_integrator<RungeKutta3Integrator<double>>(system, context);
  DisableDefaultPublishing(s);
}

// Tests witness function isolation when operating in fixed step mode without
// specifying accuracy (i.e., no isolation should be performed, and the witness
// function should trigger at the end of the step). See
// Simulator::GetCurrentWitnessTimeIsolation() for more information.
GTEST_TEST(SimulatorTest, FixedStepNoIsolation) {
  LogisticSystem system(1e-8, 100, 1);
  double publish_time = 0;
  system.set_publish_callback([&](const Context<double>& context) {
    publish_time = context.get_time();
  });

  const double dt = 1e-3;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, dt);

  Context<double>* context = simulator.get_mutable_context();
  (*context->get_mutable_continuous_state())[0] = -1;
  simulator.StepTo(dt);

  // Verify that no witness isolation is done.
  EXPECT_FALSE(simulator.GetCurrentWitnessTimeIsolation());

  // Verify that the witness function triggered at dt.
  EXPECT_EQ(publish_time, dt);
}

// Tests the witness function isolation window gets smaller *when Simulator
// uses a variable step integrator* as the accuracy in the context becomes
// tighter. See Simulator::GetCurrentWitnessTimeIsolation() for documentation of
// this effect. Note that we cannot guarantee that the witness function zero is
// isolated to greater accuracy because variable step integration implies that
// we will not know the brackets on the interval input to the witness isolation
// function- simulating with low accuracy could inadvertently isolate a
// zero to perfect tolerance because the step taken by the integrator
// fortuitously lands on the zero.
GTEST_TEST(SimulatorTest, VariableStepIsolation) {
  // Set empty system to trigger when time is +1 (setting is arbitrary for this
  // test).
  EmptySystem system(1.0, WitnessFunction<double>::DirectionType::kCrossesZero);
  double publish_time = 0;
  system.set_publish_callback([&](const Context<double>& context){
    publish_time = context.get_time();
  });

  Simulator<double> simulator(system);
  InitVariableStepIntegratorForWitnessTesting(&simulator);
  Context<double>* context = simulator.get_mutable_context();

  // Set the initial accuracy in the context.
  double accuracy = 1.0;
  context->set_accuracy(accuracy);

  // Set the initial empty system evaluation.
  double eval = std::numeric_limits<double>::infinity();

  // Loop, decreasing accuracy as we go.
  while (accuracy > 1e-8) {
    // Verify that the isolation window is computed.
    optional<double> iso_win = simulator.GetCurrentWitnessTimeIsolation();
    EXPECT_TRUE(iso_win);

    // Verify that the new evaluation is closer to zero than the old one.
    EXPECT_LT(iso_win.value(), eval);
    eval = iso_win.value();

    // Increase the accuracy, which should shrink the isolation window when
    // next retrieved.
    accuracy *= 0.1;
    context->set_accuracy(accuracy);
  }
}

// Tests that witness function isolation accuracy increases with increasing
// accuracy in the context. This test uses fixed step integration, which implies
// a particular mechanism for the witness window isolation length (see
// Simulator::GetCurrentWitnessTimeIsolation()). This function tests that the
// accuracy of the witness function isolation time increases as accuracy
// (in the Context) is increased.
GTEST_TEST(SimulatorTest, FixedStepIncreasingIsolationAccuracy) {
  // Set empty system to trigger when time is +1.
  EmptySystem system(1.0, WitnessFunction<double>::DirectionType::kCrossesZero);
  double publish_time = 0;
  system.set_publish_callback([&](const Context<double>& context){
    publish_time = context.get_time();
  });

  const double dt = 10;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, dt);
  Context<double>* context = simulator.get_mutable_context();

  // Get the (one) witness function.
  std::vector<const systems::WitnessFunction<double>*> witness;
  system.GetWitnessFunctions(*context, &witness);
  DRAKE_DEMAND(witness.size() == 1);

  // Set the initial accuracy in the context.
  double accuracy = 1.0;
  context->set_accuracy(accuracy);

  // Set the initial empty system evaluation.
  double eval = std::numeric_limits<double>::infinity();

  // Loop, decreasing accuracy as we go.
  while (accuracy > 1e-8) {
    // (Re)set the time and initial state.
    context->set_time(0);

    // Simulate to dt.
    simulator.StepTo(dt);

    // Evaluate the witness function.
    context->set_time(publish_time);
    double new_eval = witness.front()->Evaluate(*context);

    // Verify that the new evaluation is closer to zero than the old one.
    EXPECT_LT(new_eval, eval);
    eval = new_eval;

    // Increase the accuracy.
    accuracy *= 0.1;
    context->set_accuracy(accuracy);
  }
}

// Tests ability of simulation to identify the witness function triggering
// over an interval *where both witness functions change sign from the beginning
// to the end of the interval.
GTEST_TEST(SimulatorTest, MultipleWitnesses) {
  // Set up the trigger time.
  const double trigger_time = 1e-2;

  // Create a CompositeSystem, which uses two witness functions.
  CompositeSystem system(1e-8, 100, 1, trigger_time);
  std::vector<std::pair<double, const WitnessFunction<double> *>> triggers;
  system.set_publish_callback([&](const Context<double> &context) {
    // Get the witness functions.
    std::vector<const WitnessFunction<double> *> witnesses;
    system.GetWitnessFunctions(context, &witnesses);
    DRAKE_DEMAND(witnesses.size() == 2);

    // Evaluate them.
    double clock_eval = witnesses.front()->Evaluate(context);
    double logistic_eval = witnesses.back()->Evaluate(context);

    // Store the one that evaluates closest to zero.
    if (std::abs(clock_eval) < std::abs(logistic_eval)) {
      triggers.emplace_back(context.get_time(), witnesses.front());
    } else {
      // They should not be very close to one another.
      const double tol = 1e-8;
      DRAKE_ASSERT(std::abs(logistic_eval - clock_eval) > tol);
      triggers.emplace_back(context.get_time(), witnesses.back());
    }
  });

  const double dt = 1e-3;
  Simulator<double> simulator(system);
  DisableDefaultPublishing(&simulator);
  simulator.reset_integrator<ImplicitEulerIntegrator<double>>(system,
                                              simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(dt);
  simulator.get_mutable_integrator()->set_target_accuracy(0.1);

  // Set initial time and state.
  Context<double>* context = simulator.get_mutable_context();
  (*context->get_mutable_continuous_state())[0] = -1;
  context->set_time(0);

  // Isolate witness functions to high accuracy.
  const double tol = 1e-10;
  context->set_accuracy(tol);

  // Simulate.
  simulator.StepTo(0.1);

  // We expect exactly two triggerings.
  ASSERT_EQ(triggers.size(), 2);

  // Check that the witnesses triggered in the order we expect.
  EXPECT_TRUE(is_dynamic_castable<const LogisticWitness>(
      triggers.front().second));
  EXPECT_TRUE(is_dynamic_castable<const ClockWitness>(triggers.back().second));

  // We expect that the clock witness will trigger second at a time of ~1s.
  EXPECT_NEAR(triggers.back().first, trigger_time, tol);
}

// Tests ability of simulation to identify two witness functions triggering
// at the identical time over an interval.
GTEST_TEST(SimulatorTest, MultipleWitnessesIdentical) {
  // Create an EmptySystem that uses two identical witness functions.
  TwoWitnessEmptySystem system(1.0, 1.0);
  bool published = false;
  std::unique_ptr<Simulator<double>> simulator;
  system.set_publish_callback([&](const Context<double> &context) {
    // Get the witness functions.
    std::vector<const WitnessFunction<double> *> witnesses;
    system.GetWitnessFunctions(context, &witnesses);
    DRAKE_DEMAND(witnesses.size() == 2);

    // Evaluate them.
    double w1 = witnesses.front()->Evaluate(context);
    double w2 = witnesses.back()->Evaluate(context);

    // Verify both are equivalent.
    EXPECT_EQ(w1, w2);

    // Verify that they are triggering.
    // NOTE: value_or(999) necessary to work around Mac OS X bug where value()
    // function is declared but not defined.
    optional<double> iso_time = simulator->GetCurrentWitnessTimeIsolation();
    EXPECT_TRUE(iso_time);
    EXPECT_LT(std::abs(w1), iso_time.value_or(999));

    // Indicate that the method has been called.
    published = true;
  });

  const double dt = 2;
  simulator = std::make_unique<Simulator<double>>(system);
  DisableDefaultPublishing(simulator.get());
  simulator->get_mutable_integrator()->set_maximum_step_size(dt);

  // Isolate witness functions to high accuracy.
  const double tol = 1e-12;
  simulator->get_mutable_context()->set_accuracy(tol);

  // Simulate.
  simulator->StepTo(10);

  // Verify one publish.
  EXPECT_TRUE(published);
}

// Tests ability of simulation to identify two witness functions triggering
// over an interval where (a) both functions change sign over the interval and
// (b) after the interval is "chopped down" (to isolate the first witness
// triggering), the second witness does not trigger.
//
// How this function tests this functionality: Both functions will change sign
// over the interval [0, 2.1]. Since the first witness function triggers at
// t=1.0, isolating the firing time should cause the second witness to no longer
// trigger (at t=1.0). When the Simulator continues stepping (i.e., after the
// event corresponding to the first witness function is handled), the second
// witness should then be triggered at t=2.0.
GTEST_TEST(SimulatorTest, MultipleWitnessesStaggered) {
  // Set the trigger times.
  const double first_time = 1.0;
  const double second_time = 2.0;

  // Create an EmptySystem that uses clock witnesses.
  TwoWitnessEmptySystem system(first_time, second_time);
  std::vector<double> publish_times;
  system.set_publish_callback([&](const Context<double> &context) {
    publish_times.push_back(context.get_time());
  });

  const double dt = 3;
  Simulator<double> simulator(system);
  DisableDefaultPublishing(&simulator);
  simulator.get_mutable_integrator()->set_maximum_step_size(dt);

  // Isolate witness functions to high accuracy.
  const double tol = 1e-12;
  simulator.get_mutable_context()->set_accuracy(tol);

  // Get the isolation interval tolerance.
  const optional<double> iso_tol = simulator.GetCurrentWitnessTimeIsolation();
  EXPECT_TRUE(iso_tol);

  // Simulate to right after the second one should have triggered.
  simulator.StepTo(2.1);

  // Verify two publishes.
  EXPECT_EQ(publish_times.size(), 2);

  // Verify that the publishes are at the expected times.
  EXPECT_NEAR(publish_times.front(), first_time, iso_tol.value());
  EXPECT_NEAR(publish_times.back(), second_time, iso_tol.value());
}

// Tests ability of simulation to identify the proper number of witness function
// triggerings going from negative to non-negative witness function evaluation.
// This particular example uses an empty system and a clock as the witness
// function, which makes it particularly easy to determine when the witness
// function should trigger.
GTEST_TEST(SimulatorTest, WitnessTestCountSimpleNegToZero) {
  // Set empty system to trigger when time is +1.
  EmptySystem system(1.0, WitnessFunction<double>::DirectionType::kCrossesZero);
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    num_publishes++;
  });

  const double dt = 1;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, dt);
  Context<double>* context = simulator.get_mutable_context();
  context->set_time(0);
  simulator.StepTo(1);

  // Publication should occur at witness function crossing.
  EXPECT_EQ(1, num_publishes);
}

// Tests ability of simulation to identify the proper number of witness function
// triggerings going from zero to positive witness function evaluation. This
// particular example uses an empty system and a clock as the witness function,
// which makes it particularly easy to determine when the witness function
// should trigger.
GTEST_TEST(SimulatorTest, WitnessTestCountSimpleZeroToPos) {
  // Set empty system to trigger when time is zero.
  EmptySystem system(0, WitnessFunction<double>::DirectionType::kCrossesZero);
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    num_publishes++;
  });

  const double dt = 1;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, dt);
  Context<double>* context = simulator.get_mutable_context();
  context->set_time(0);
  simulator.StepTo(1);

  // Verify that no publication is performed when stepping to 1.
  EXPECT_EQ(0, num_publishes);
}

// Tests ability of simulation to identify the proper number of witness function
// triggerings (zero) for a positive-to-negative trigger. Uses the same empty
// system from WitnessTestCountSimple.
GTEST_TEST(SimulatorTest, WitnessTestCountSimplePositiveToNegative) {
  // Set empty system to trigger when time is +1.
  EmptySystem system(1.0, WitnessFunction<double>::DirectionType::
      kPositiveThenNonPositive);
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    num_publishes++;
  });

  const double dt = 1;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, dt);
  Context<double>* context = simulator.get_mutable_context();
  context->set_time(0);
  simulator.StepTo(2);

  // Publication should not occur (witness function should initially evaluate
  // to a negative value, then will evolve to a positive value).
  EXPECT_EQ(0, num_publishes);
}

// Tests ability of simulation to identify the proper number of witness function
// triggerings (zero) for a negative-to-positive trigger. Uses the same empty
// system from WitnessTestCountSimple.
GTEST_TEST(SimulatorTest, WitnessTestCountSimpleNegativeToPositive) {
  EmptySystem system(0, WitnessFunction<double>::DirectionType::
      kNegativeThenNonNegative);
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    num_publishes++;
  });

  const double dt = 1;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, dt);
  Context<double>* context = simulator.get_mutable_context();
  context->set_time(-1);
  simulator.StepTo(1);

  // Publication should occur at witness function crossing.
  EXPECT_EQ(1, num_publishes);
}

// Tests ability of simulation to identify the proper number of witness function
// triggerings. This particular example, the logistic function, is particularly
// challenging for detecting exactly one zero crossing under the
// parameterization in use. The logic system's state just barely crosses zero
// (at t << 1) and then hovers around zero afterward.
GTEST_TEST(SimulatorTest, WitnessTestCountChallenging) {
  LogisticSystem system(1e-8, 100, 1);
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    num_publishes++;
  });

  const double dt = 1e-6;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, dt);
  Context<double>* context = simulator.get_mutable_context();
  (*context->get_mutable_continuous_state())[0] = -1;
  simulator.StepTo(1e-4);

  // Publication should occur only at witness function crossing.
  EXPECT_EQ(1, num_publishes);
}

// TODO(edrumwri): Add tests for verifying that correct interval returned
// in the case of multiple witness functions. See issue #6184.

GTEST_TEST(SimulatorTest, SecondConstructor) {
  // Create the spring-mass sytem and context.
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  auto context = spring_mass.CreateDefaultContext();

  // Mark the context with an arbitrary value.
  context->set_time(3.);

  /// Construct the simulator with the created context.
  Simulator<double> simulator(spring_mass, std::move(context));

  // Verify that context pointers are equivalent.
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
}

GTEST_TEST(SimulatorTest, MiscAPI) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Default realtime rate should be zero.
  EXPECT_TRUE(simulator.get_target_realtime_rate() == 0.);

  simulator.set_target_realtime_rate(1.25);
  EXPECT_TRUE(simulator.get_target_realtime_rate() == 1.25);

  EXPECT_TRUE(std::isnan(simulator.get_actual_realtime_rate()));

  // Set the integrator default step size.
  const double dt = 1e-3;

  // Create a context.
  auto context = simulator.get_mutable_context();

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, dt,
                                                              context);

  // Initialize the simulator first.
  simulator.Initialize();
}

GTEST_TEST(SimulatorTest, ContextAccess) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set the integrator default step size.
  const double dt = 1e-3;

  // Get the context.
  auto context = simulator.get_mutable_context();

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, dt,
                                                              context);

  // Initialize the simulator first.
  simulator.Initialize();

  // Try some other context stuff.
  simulator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
  simulator.release_context();
  EXPECT_TRUE(simulator.get_mutable_context() == nullptr);
  EXPECT_THROW(simulator.Initialize(), std::logic_error);

  // Create another context.
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

  // Set the integrator default step size.
  const double dt = 1e-3;

  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // Get the context.
  auto context = simulator.get_mutable_context();

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, dt,
                                                              context);

  simulator.set_target_realtime_rate(0.5);
  // Set the integrator and initialize the simulator.
  simulator.Initialize();

  // Simulate for 1 second.
  simulator.StepTo(1.);

  EXPECT_NEAR(context->get_time(), 1., 1e-8);
  EXPECT_EQ(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_discrete_updates(), 0);

  EXPECT_EQ(spring_mass.get_publish_count(), 1001);
  EXPECT_EQ(spring_mass.get_update_count(), 0);

  // Current time is 1. An earlier final time should fail.
  EXPECT_THROW(simulator.StepTo(0.5), std::runtime_error);
}

// Test ability to swap integrators mid-stream.
GTEST_TEST(SimulatorTest, ResetIntegratorTest) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // set the integrator default step size
  const double dt = 1e-3;

  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // Get the context.
  auto context = simulator.get_mutable_context();

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, dt,
                                                              context);

  // set the integrator and initialize the simulator
  simulator.Initialize();

  // Simulate for 1/2 second.
  simulator.StepTo(0.5);

  // Reset the integrator.
  simulator.reset_integrator<RungeKutta2Integrator<double>>(
      simulator.get_system(), dt, simulator.get_mutable_context());

  // Simulate to 1 second..
  simulator.StepTo(1.);

  EXPECT_NEAR(context->get_time(), 1., 1e-8);

  // Number of steps will have been reset.
  EXPECT_EQ(simulator.get_num_steps_taken(), 500);
}

// Because of arbitrary possible delays we can't do a very careful test of
// the realtime rate control. However, we can at least say that the simulation
// should not proceed much *faster* than the rate we select.
GTEST_TEST(SimulatorTest, RealtimeRate) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  simulator.set_target_realtime_rate(1.);  // No faster than 1X real time.
  simulator.get_mutable_context()->set_time(0.);
  simulator.Initialize();
  simulator.StepTo(1.);  // Simulate for 1 simulated second.
  EXPECT_TRUE(simulator.get_actual_realtime_rate() <= 1.1);

  simulator.set_target_realtime_rate(5.);  // No faster than 5X real time.
  simulator.get_mutable_context()->set_time(0.);
  simulator.Initialize();
  simulator.StepTo(1.);  // Simulate for 1 more simulated second.
  EXPECT_TRUE(simulator.get_actual_realtime_rate() <= 5.1);
}

// Tests that if publishing every timestep is disabled, publish only happens
// on initialization.
GTEST_TEST(SimulatorTest, DisablePublishEveryTimestep) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.
  simulator.set_publish_every_time_step(false);

  simulator.get_mutable_context()->set_time(0.);
  simulator.Initialize();
  // Publish should happen on initialization.
  EXPECT_EQ(1, simulator.get_num_publishes());

  // Simulate for 1 simulated second.  Publish should not happen.
  simulator.StepTo(1.);
  EXPECT_EQ(1, simulator.get_num_publishes());
}

// Repeat the previous test but now the continuous steps are interrupted
// by a discrete sample every 1/30 second. The step size doesn't divide that
// evenly so we should get some step size modification here.
GTEST_TEST(SimulatorTest, SpringMass) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // Set the integrator default step size.
  const double dt = 1e-3;

  // Create the mass spring system and the simulator.
  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 30.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Get the context.
  auto context = simulator.get_mutable_context();

  // TODO(edrumwri): Remove this when discrete state has been created
  // automatically.
  // Create the discrete state.
  context->set_discrete_state(std::make_unique<DiscreteValues<double>>());

  // Set initial condition using the Simulator's internal context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // Create the integrator and initialize it.
  auto integrator = simulator.reset_integrator<ExplicitEulerIntegrator<double>>(
      spring_mass, dt, context);
  integrator->Initialize();

  // Set the integrator and initialize the simulator.
  simulator.Initialize();

  // Simulate to one second.
  simulator.StepTo(1.);

  EXPECT_GT(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_discrete_updates(), 30);

  // We're calling Publish() every step, and extra steps have to be taken
  // since the step size doesn't divide evenly into the sample rate. Shouldn't
  // require more than one extra step per sample though.
  EXPECT_LE(spring_mass.get_publish_count(), 1030);
  EXPECT_EQ(spring_mass.get_update_count(), 30);
}

// A mock System that requests a single publication at a prespecified time.
namespace {
class UnrestrictedUpdater : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrestrictedUpdater)

  explicit UnrestrictedUpdater(double t_upd) : t_upd_(t_upd) {
  }

  ~UnrestrictedUpdater() override {}

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  void DoCalcNextUpdateTime(const systems::Context<double>& context,
                            systems::UpdateActions<double>* actions)
                              const override {
    const double inf = std::numeric_limits<double>::infinity();
    actions->time = (context.get_time() < t_upd_) ? t_upd_ : inf;
    actions->events.push_back(systems::DiscreteEvent<double>());
    actions->events.back().action = systems::DiscreteEvent<double>::
                                               kUnrestrictedUpdateAction;
  }

  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const override {
    if (unrestricted_update_callback_ != nullptr)
      unrestricted_update_callback_(context, state);
  }

  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    if (derivatives_callback_ != nullptr) derivatives_callback_(context);
  }

  void set_unrestricted_update_callback(
      std::function<void(const Context<double>&, State<double>*)> callback) {
    unrestricted_update_callback_ = callback;
  }

  void set_derivatives_callback(
      std::function<void(const Context<double>&)> callback) {
    derivatives_callback_ = callback;
  }

 private:
  const double t_upd_{0.0};
  std::function<void(const Context<double>&, State<double>*)>
                                      unrestricted_update_callback_{nullptr};
  std::function<void(const Context<double>&)> derivatives_callback_{nullptr};
};
}  // namespace

// Tests that the simulator captures an unrestricted update at the exact time
// (i.e., without accumulating floating point error).
GTEST_TEST(SimulatorTest, ExactUpdateTime) {
  // Create the UnrestrictedUpdater system.
  const double t_upd = 1e-10;                // Inexact floating point rep.
  UnrestrictedUpdater unrest_upd(t_upd);
  Simulator<double> simulator(unrest_upd);  // Use default Context.

  // Set time to an exact floating point representation; we want t_upd to
  // be much smaller in magnitude than the time, hence the negative time.
  simulator.get_mutable_context()->set_time(-1.0/1024);

  // Capture the time at which an update is done using a callback function.
  std::vector<double> updates;
  unrest_upd.set_unrestricted_update_callback(
      [&updates](const Context<double>& context, State<double>* state) {
    updates.push_back(context.get_time());
  });

  // Simulate forward.
  simulator.Initialize();
  simulator.StepTo(1.);

  // Check that the update occurs at exactly the desired time.
  EXPECT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates.front(), t_upd);
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


// A mock System that requests discrete update at 1 kHz, and publishes at 400
// Hz. Calls user-configured callbacks on DoPublish,
// DoCalcDiscreteVariableUpdates, and EvalTimeDerivatives.
class DiscreteSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteSystem)

  DiscreteSystem() {
    // Deliberately choose a period that is identical to, and therefore courts
    // floating-point error with, the default max step size.
    const double offset = 0.0;
    this->DeclarePeriodicDiscreteUpdate(kUpdatePeriod, offset);
    this->DeclarePublishPeriodSec(kPublishPeriod);

    set_name("TestSystem");
  }

  ~DiscreteSystem() override {}

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* updates) const override {
    if (update_callback_ != nullptr) update_callback_(context);
  }

  void DoPublish(
      const drake::systems::Context<double>& context) const override {
    if (publish_callback_ != nullptr) publish_callback_(context);
  }

  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    if (derivatives_callback_ != nullptr) derivatives_callback_(context);
  }

  void set_update_callback(
      std::function<void(const Context<double>&)> callback) {
    update_callback_ = callback;
  }

  void set_publish_callback(
      std::function<void(const Context<double>&)> callback) {
    publish_callback_ = callback;
  }

  void set_derivatives_callback(
      std::function<void(const Context<double>&)> callback) {
    derivatives_callback_ = callback;
  }

  double update_period() const { return kUpdatePeriod; }
  double publish_period() const { return kPublishPeriod; }

 private:
  const double kUpdatePeriod{0.001};
  const double kPublishPeriod{0.0025};
  std::function<void(const Context<double>&)> update_callback_{nullptr};
  std::function<void(const Context<double>&)> publish_callback_{nullptr};
  std::function<void(const Context<double>&)> derivatives_callback_{nullptr};
};

// Returns true if the time in the @p context is a multiple of the @p period.
bool CheckSampleTime(const Context<double>& context, double period) {
  const double k = context.get_time() / period;
  const double int_k = std::round(k);
  const double kTolerance = 1e-8;
  return std::abs(k - int_k) < kTolerance;
}

// Tests that the Simulator invokes the DiscreteSystem's update method every
// 0.001 sec, and its publish method every 0.0025 sec, without missing any
// updates.
GTEST_TEST(SimulatorTest, DiscreteUpdateAndPublish) {
  DiscreteSystem system;
  int num_disc_updates = 0;
  system.set_update_callback([&](const Context<double>& context){
    ASSERT_TRUE(CheckSampleTime(context, system.update_period()));
    num_disc_updates++;
  });
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    ASSERT_TRUE(CheckSampleTime(context, system.publish_period()));
    num_publishes++;
  });

  drake::systems::Simulator<double> simulator(system);
  simulator.set_publish_every_time_step(false);
  simulator.StepTo(0.5);
  EXPECT_EQ(500, num_disc_updates);
  // Publication occurs at 400Hz, and also at initialization.
  EXPECT_EQ(200 + 1, num_publishes);
}

// Tests that the order of events in a simulator time step is first update
// discrete state, then publish, then integrate.
GTEST_TEST(SimulatorTest, UpdateThenPublishThenIntegrate) {
  DiscreteSystem system;
  drake::systems::Simulator<double> simulator(system);
  enum EventType {
    kUpdate = 0,
    kPublish = 1,
    kIntegrate = 2
  };

  // Write down the order in which the DiscreteSystem is asked to compute
  // discrete updates, do publishes, or compute derivatives at each time step.
  std::map<int, std::vector<EventType>> events;
  system.set_update_callback(
      [&events, &simulator](const Context<double>& context) {
    events[simulator.get_num_steps_taken()].push_back(kUpdate);
  });
  system.set_publish_callback(
      [&events, &simulator](const Context<double>& context) {
    events[simulator.get_num_steps_taken()].push_back(kPublish);
  });
  system.set_derivatives_callback(
      [&events, &simulator](const Context<double>& context) {
    events[simulator.get_num_steps_taken()].push_back(kIntegrate);
  });

  // Run a simulation.
  simulator.set_publish_every_time_step(true);
  simulator.StepTo(0.5);

  // Check that all the update events precede all the publish events, and all
  // the publish events precede all the eval-derivatives events, for each
  // time step in the simulation.
  for (const auto& log : events) {
    ASSERT_GE(log.second.size(), 0u);
    EventType state = log.second[0];
    for (const EventType& event : log.second) {
      ASSERT_TRUE(event >= state);
      state = event;
    }
  }
}

// A basic sanity check that AutoDiff works.
GTEST_TEST(SimulatorTest, AutodiffBasic) {
  SpringMassSystem<AutoDiffXd> spring_mass(1., 1., 0.);
  Simulator<AutoDiffXd> simulator(spring_mass);
  simulator.Initialize();
  simulator.StepTo(1);
}

// Tests per step publish, discrete and unrestricted update actions. Each
// action handler logs the context time when it's called, and the test compares
// the time stamp against the integrator's dt.
GTEST_TEST(SimulatorTest, PerStepAction) {
  class PerStepActionTestSystem : public LeafSystem<double> {
   public:
    PerStepActionTestSystem() {}

    void AddPerStepAction(
        const typename DiscreteEvent<double>::ActionType& action) {
      this->DeclarePerStepAction(action);
    }

    const std::vector<double>& get_publish_times() const {
      return publish_times_;
    }

    const std::vector<double>& get_discrete_update_times() const {
      return discrete_update_times_;
    }

    const std::vector<double>& get_unrestricted_update_times() const {
      return unrestricted_update_times_;
    }

   private:
    void DoCalcOutput(const Context<double>& context,
        SystemOutput<double>* output) const override {}

    void DoCalcDiscreteVariableUpdates(const Context<double>& context,
        DiscreteValues<double>* discrete_state) const override {
      discrete_update_times_.push_back(context.get_time());
    }

    void DoCalcUnrestrictedUpdate(const Context<double>& context,
        State<double>* state) const override {
      unrestricted_update_times_.push_back(context.get_time());
    }

    void DoPublish(const Context<double>& context) const override {
      publish_times_.push_back(context.get_time());
    }

    // A hack to test actions easily.
    // Note that these should really be part of the Context, and users should
    // NOT use this as an example code.
    //
    // Since Publish only takes a const Context, the only way to log time is
    // through some side effects. Thus, using a mutable vector can be justified.
    //
    // One conceptually correct implementation for discrete_update_times_ is
    // to pre allocate a big DiscreteState in Context, and store all the time
    // stamps there.
    //
    // unrestricted_update_times_ can be put in the AbstractState in Context
    // and mutated similarly to this implementation.
    //
    // The motivation for keeping them as mutable are for simplicity and
    // easiness to understand.
    mutable std::vector<double> publish_times_;
    mutable std::vector<double> discrete_update_times_;
    mutable std::vector<double> unrestricted_update_times_;
  };

  PerStepActionTestSystem sys;
  sys.AddPerStepAction(DiscreteEvent<double>::kPublishAction);
  sys.AddPerStepAction(DiscreteEvent<double>::kDiscreteUpdateAction);
  sys.AddPerStepAction(DiscreteEvent<double>::kUnrestrictedUpdateAction);
  Simulator<double> sim(sys);

  // Disables all simulator induced publish events, so that all publish calls
  // are intiated by sys.
  sim.set_publish_at_initialization(false);
  sim.set_publish_every_time_step(false);
  sim.Initialize();
  sim.StepTo(0.1);

  double dt = sim.get_integrator()->get_maximum_step_size();
  int N = static_cast<int>(0.1 / dt);
  // Need to change this if the default integrator step size is not 1ms.
  EXPECT_EQ(N, 100);

  auto& publish_times = sys.get_publish_times();
  auto& discrete_update_times = sys.get_discrete_update_times();
  auto& unrestricted_update_times = sys.get_unrestricted_update_times();
  EXPECT_EQ(publish_times.size(), N);
  EXPECT_EQ(sys.get_discrete_update_times().size(), N);
  EXPECT_EQ(sys.get_unrestricted_update_times().size(), N);
  for (size_t i = 0; i < publish_times.size(); ++i) {
    EXPECT_NEAR(publish_times[i], i * dt, 1e-12);
    EXPECT_NEAR(discrete_update_times[i], i * dt, 1e-12);
    EXPECT_NEAR(unrestricted_update_times[i], i * dt, 1e-12);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake

