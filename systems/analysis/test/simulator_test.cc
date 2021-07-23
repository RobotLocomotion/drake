#include "drake/systems/analysis/simulator.h"

#include <cmath>
#include <complex>
#include <functional>
#include <map>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/test_utilities/controlled_spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/logistic_system.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/stateless_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/vector_log_sink.h"

using drake::systems::WitnessFunction;
using drake::systems::Simulator;
using drake::systems::RungeKutta3Integrator;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::ExplicitEulerIntegrator;
using LogisticSystem = drake::systems::analysis_test::LogisticSystem<double>;
using StatelessSystem = drake::systems::analysis_test::StatelessSystem<double>;
using Eigen::AutoDiffScalar;
using Eigen::NumTraits;
using std::complex;

// N.B. internal::GetPreviousNormalizedValue() is tested separately in
// simulator_denorm_test.cc.

namespace drake {
namespace systems {
namespace {

// @TODO(edrumwri): Use test fixtures to streamline this file and promote reuse.

// Stateless system with a DoCalcTimeDerivatives implementation. This class
// will serve to confirm that the time derivative calculation is not called.
class StatelessSystemPlusDerivs : public systems::LeafSystem<double> {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StatelessSystemPlusDerivs)

 public:
  StatelessSystemPlusDerivs() {}

  bool was_do_calc_time_derivatives_called() const {
    return do_calc_time_derivatives_called_;
  }

 private:
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    // Modifying system members in DoCalcTimeDerivatives() is an anti-pattern.
    // It is done here only to simplify the testing code.
    do_calc_time_derivatives_called_ = true;
  }

  mutable bool do_calc_time_derivatives_called_{false};
};

// Empty diagram
class StatelessDiagram : public Diagram<double> {
 public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StatelessDiagram)

  explicit StatelessDiagram(double offset) {
    DiagramBuilder<double> builder;

    // Add the empty system (and its witness function).
    stateless_ = builder.AddSystem<StatelessSystem>(offset,
        WitnessFunctionDirection::kCrossesZero);
    stateless_->set_name("stateless_diagram");
    builder.BuildInto(this);
  }

  void set_publish_callback(
      std::function<void(const Context<double>&)> callback) {
    stateless_->set_publish_callback(callback);
  }

 private:
  StatelessSystem* stateless_ = nullptr;
};

// Diagram for testing witness functions.
class ExampleDiagram : public Diagram<double> {
 public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExampleDiagram)

  explicit ExampleDiagram(double offset) {
    DiagramBuilder<double> builder;

    // Add the empty system (and its witness function).
    stateless_diag_ = builder.AddSystem<StatelessDiagram>(offset);
    stateless_diag_->set_name("diagram_of_stateless_diagram");
    builder.BuildInto(this);
  }

  void set_publish_callback(
      std::function<void(const Context<double>&)> callback) {
    stateless_diag_->set_publish_callback(callback);
  }

 private:
  StatelessDiagram* stateless_diag_ = nullptr;
};

// Tests that DoCalcTimeDerivatives() is not called when the system has no
// continuous state.
GTEST_TEST(SimulatorTest, NoUnexpectedDoCalcTimeDerivativesCall) {
  // Construct the simulation using the RK2 (fixed step) integrator with a small
  // time step.
  StatelessSystemPlusDerivs system;
  const double final_time = 1.0;
  const double h = 1e-3;
  Simulator<double> simulator(system);
  simulator.reset_integrator<RungeKutta2Integrator<double>>(h);
  simulator.AdvanceTo(final_time);

  // Verify no derivative calculations.
  EXPECT_FALSE(system.was_do_calc_time_derivatives_called());
}

// Tests that simulation only takes a single step when there is no continuous
// state, regardless of the integrator maximum step size (and no discrete state
// or events).
GTEST_TEST(SimulatorTest, NoContinuousStateYieldsSingleStep) {
  const double final_time = 1.0;
  StatelessSystem system(
     final_time + 1, /* publish time *after* final time */
     WitnessFunctionDirection::kCrossesZero);

  // Construct the simulation using the RK2 (fixed step) integrator with a small
  // time step.
  const double h = 1e-3;
  Simulator<double> simulator(system);
  simulator.reset_integrator<RungeKutta2Integrator<double>>(h);
  simulator.AdvanceTo(final_time);

  EXPECT_EQ(simulator.get_num_steps_taken(), 1);
}

// Tests ability of simulation to identify the proper number of witness function
// triggerings going from negative to non-negative witness function evaluation
// using a Diagram. This particular example uses an empty system and a clock as
// the witness function, which makes it particularly easy to determine when the
// witness function should trigger.
GTEST_TEST(SimulatorTest, DiagramWitness) {
  // Set empty system to trigger when time is +1.
  const double trigger_time = 1.0;
  ExampleDiagram system(trigger_time);
  double publish_time = -1;
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context) {
    num_publishes++;
    publish_time = context.get_time();
  });

  const double h = 1;
  Simulator<double> simulator(system);
  simulator.reset_integrator<RungeKutta2Integrator<double>>(h);
  simulator.set_publish_at_initialization(false);
  simulator.set_publish_every_time_step(false);

  simulator.get_mutable_context().SetTime(0);
  simulator.AdvanceTo(1);

  // Publication should occur at witness function crossing.
  EXPECT_EQ(1, num_publishes);
  EXPECT_EQ(publish_time, trigger_time);
}

// A composite system using the logistic system with the clock-based
// witness function.
class CompositeSystem : public analysis_test::LogisticSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompositeSystem)

  CompositeSystem(double k, double alpha, double nu, double trigger_time)
      : LogisticSystem(k, alpha, nu), trigger_time_(trigger_time) {
    this->DeclareContinuousState(1);

    logistic_witness_ = this->MakeWitnessFunction(
        "logistic witness", WitnessFunctionDirection::kCrossesZero,
        &CompositeSystem::GetStateValue,
        &CompositeSystem::CallLogisticsCallback);
    clock_witness_ = this->MakeWitnessFunction(
        "clock witness", WitnessFunctionDirection::kCrossesZero,
        &CompositeSystem::CalcClockWitness,
        &CompositeSystem::CallLogisticsCallback);
  }

  const WitnessFunction<double>* get_logistic_witness() const {
    return logistic_witness_.get();
  }

  const WitnessFunction<double>* get_clock_witness() const {
    return clock_witness_.get();
  }

 protected:
  void DoGetWitnessFunctions(
      const Context<double>&,
      std::vector<const WitnessFunction<double>*>* w) const override {
    w->push_back(clock_witness_.get());
    w->push_back(logistic_witness_.get());
  }

 private:
  double GetStateValue(const Context<double>& context) const {
    return context.get_continuous_state()[0];
  }

  // The witness function is the time value itself plus the offset value.
  double CalcClockWitness(const Context<double>& context) const {
    return context.get_time() - trigger_time_;
  }

  void CallLogisticsCallback(const Context<double>& context,
                            const PublishEvent<double>& event) const {
    LogisticSystem<double>::InvokePublishCallback(context, event);
  }

  const double trigger_time_;
  std::unique_ptr<WitnessFunction<double>> logistic_witness_;
  std::unique_ptr<WitnessFunction<double>> clock_witness_;
};

// An empty system using two clock witnesses.
class TwoWitnessStatelessSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TwoWitnessStatelessSystem)

  explicit TwoWitnessStatelessSystem(double off1, double off2)
      : offset1_(off1), offset2_(off2) {
    auto witness_handler = [this](const Context<double>& context,
                                  const PublishEvent<double>& event) {
      this->PublishOnWitness(context, event);
    };
    witness1_ = this->MakeWitnessFunction(
        "clock witness1", WitnessFunctionDirection::kCrossesZero,
        &TwoWitnessStatelessSystem::CalcClockWitness1,
        PublishEvent<double>(witness_handler));
    witness2_ = this->MakeWitnessFunction(
        "clock witness2", WitnessFunctionDirection::kCrossesZero,
        &TwoWitnessStatelessSystem::CalcClockWitness2,
        PublishEvent<double>(witness_handler));
  }

  void set_publish_callback(
      std::function<void(const Context<double>&)> callback) {
    publish_callback_ = callback;
  }

 private:
  void DoGetWitnessFunctions(
      const Context<double>&,
      std::vector<const WitnessFunction<double>*>* w) const override {
    w->push_back(witness1_.get());
    w->push_back(witness2_.get());
  }

  void PublishOnWitness(const Context<double>& context,
                        const PublishEvent<double>&) const {
    if (publish_callback_ != nullptr) publish_callback_(context);
  }

  // The witness function is the time value itself minus the offset value.
  double CalcClockWitness1(const Context<double>& context) const {
    return context.get_time() - offset1_;
  }

  // The witness function is the time value itself minus the offset value.
  double CalcClockWitness2(const Context<double>& context) const {
    return context.get_time() - offset2_;
  }

  std::unique_ptr<WitnessFunction<double>> witness1_, witness2_;
  const double offset1_;
  const double offset2_;
  std::function<void(const Context<double>&)> publish_callback_{nullptr};
};

// Disables non-witness based publishing for witness function testing.
void DisableDefaultPublishing(Simulator<double>* s) {
  s->set_publish_at_initialization(false);
  s->set_publish_every_time_step(false);
}

// Initializes the Simulator's integrator to fixed step mode for witness
// function related tests.
void InitFixedStepIntegratorForWitnessTesting(Simulator<double>* s, double h) {
  s->reset_integrator<RungeKutta2Integrator<double>>(h);
  s->get_mutable_integrator().set_fixed_step_mode(true);
  s->get_mutable_integrator().set_maximum_step_size(h);
  DisableDefaultPublishing(s);
}

// Initializes the Simulator's integrator to variable step mode for witness
// function related tests.
void InitVariableStepIntegratorForWitnessTesting(Simulator<double>* s) {
  s->reset_integrator<RungeKutta3Integrator<double>>();
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

  const double h = 1e-3;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, h);

  Context<double>& context = simulator.get_mutable_context();
  context.get_mutable_continuous_state()[0] = -1;
  simulator.AdvanceTo(h);

  // Verify that no witness isolation is done.
  EXPECT_FALSE(simulator.GetCurrentWitnessTimeIsolation());

  // Verify that the witness function triggered at h.
  EXPECT_EQ(publish_time, h);
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
  StatelessSystem system(1.0, WitnessFunctionDirection::kCrossesZero);
  double publish_time = 0;
  system.set_publish_callback([&](const Context<double>& context){
    publish_time = context.get_time();
  });

  Simulator<double> simulator(system);
  InitVariableStepIntegratorForWitnessTesting(&simulator);
  Context<double>& context = simulator.get_mutable_context();

  // Set the initial accuracy in the context.
  double accuracy = 1.0;
  context.SetAccuracy(accuracy);

  // Set the initial empty system evaluation.
  double eval = std::numeric_limits<double>::infinity();

  // Loop, decreasing accuracy as we go.
  while (accuracy > 1e-8) {
    // Verify that the isolation window is computed.
    std::optional<double> iso_win = simulator.GetCurrentWitnessTimeIsolation();
    EXPECT_TRUE(iso_win);

    // Verify that the new evaluation is closer to zero than the old one.
    EXPECT_LT(iso_win.value(), eval);
    eval = iso_win.value();

    // Increase the accuracy, which should shrink the isolation window when
    // next retrieved.
    accuracy *= 0.1;
    context.SetAccuracy(accuracy);
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
  StatelessSystem system(1.0, WitnessFunctionDirection::kCrossesZero);
  double publish_time = 0;
  system.set_publish_callback([&](const Context<double>& context){
    publish_time = context.get_time();
  });

  const double h = 10;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, h);
  Context<double>& context = simulator.get_mutable_context();

  // Get the (one) witness function.
  std::vector<const WitnessFunction<double>*> witness;
  system.GetWitnessFunctions(context, &witness);
  DRAKE_DEMAND(witness.size() == 1);

  // Set the initial accuracy in the context.
  double accuracy = 1.0;
  context.SetAccuracy(accuracy);

  // Set the initial empty system evaluation.
  double eval = std::numeric_limits<double>::infinity();

  // Loop, decreasing accuracy as we go.
  while (accuracy > 1e-8) {
    // (Re)set the time and initial state.
    context.SetTime(0);
    simulator.Initialize();

    // Simulate to h.
    simulator.AdvanceTo(h);

    // CalcWitnessValue the witness function.
    context.SetTime(publish_time);
    double new_eval = witness.front()->CalcWitnessValue(context);

    // Verify that the new evaluation is closer to zero than the old one.
    EXPECT_LT(new_eval, eval);
    eval = new_eval;

    // Increase the accuracy.
    accuracy *= 0.1;
    context.SetAccuracy(accuracy);
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

    // CalcWitnessValue them.
    double clock_eval = witnesses.front()->CalcWitnessValue(context);
    double logistic_eval = witnesses.back()->CalcWitnessValue(context);

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

  const double h = 1e-3;
  Simulator<double> simulator(system);
  DisableDefaultPublishing(&simulator);
  simulator.reset_integrator<ImplicitEulerIntegrator<double>>();
  simulator.get_mutable_integrator().set_maximum_step_size(h);
  simulator.get_mutable_integrator().set_target_accuracy(0.1);

  // Set initial time and state.
  Context<double>& context = simulator.get_mutable_context();
  context.get_mutable_continuous_state()[0] = -1;
  context.SetTime(0);

  // Isolate witness functions to high accuracy.
  const double tol = 1e-10;
  context.SetAccuracy(tol);

  // Simulate.
  simulator.AdvanceTo(0.1);

  // We expect exactly two triggerings.
  ASSERT_EQ(triggers.size(), 2);

  // Check that the witnesses triggered in the order we expect.
  EXPECT_EQ(system.get_logistic_witness(), triggers.front().second);
  EXPECT_EQ(system.get_clock_witness(), triggers.back().second);

  // We expect that the clock witness will trigger second at a time of ~1s.
  EXPECT_NEAR(triggers.back().first, trigger_time, tol);
}

// Tests ability of simulation to identify two witness functions triggering
// at the identical time over an interval.
GTEST_TEST(SimulatorTest, MultipleWitnessesIdentical) {
  // Create a StatelessSystem that uses two identical witness functions.
  TwoWitnessStatelessSystem system(1.0, 1.0);
  bool published = false;
  std::unique_ptr<Simulator<double>> simulator;
  system.set_publish_callback([&](const Context<double> &context) {
    // Get the witness functions.
    std::vector<const WitnessFunction<double> *> witnesses;
    system.GetWitnessFunctions(context, &witnesses);
    DRAKE_DEMAND(witnesses.size() == 2);

    // CalcWitnessValue them.
    double w1 = witnesses.front()->CalcWitnessValue(context);
    double w2 = witnesses.back()->CalcWitnessValue(context);

    // Verify both are equivalent.
    EXPECT_EQ(w1, w2);

    // Verify that they are triggering.
    std::optional<double> iso_time =
        simulator->GetCurrentWitnessTimeIsolation();
    EXPECT_TRUE(iso_time);
    EXPECT_LT(std::abs(w1), iso_time.value());

    // Indicate that the method has been called.
    published = true;
  });

  const double h = 2;
  simulator = std::make_unique<Simulator<double>>(system);
  DisableDefaultPublishing(simulator.get());
  simulator->get_mutable_integrator().set_maximum_step_size(h);

  // Isolate witness functions to high accuracy.
  const double tol = 1e-12;
  simulator->get_mutable_context().SetAccuracy(tol);

  // Simulate.
  simulator->AdvanceTo(10);

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

  // Create a StatelessSystem that uses clock witnesses.
  TwoWitnessStatelessSystem system(first_time, second_time);
  std::vector<double> publish_times;
  system.set_publish_callback([&](const Context<double> &context) {
    publish_times.push_back(context.get_time());
  });

  const double h = 3;
  Simulator<double> simulator(system);
  DisableDefaultPublishing(&simulator);
  simulator.get_mutable_integrator().set_maximum_step_size(h);

  // Isolate witness functions to high accuracy.
  const double tol = 1e-12;
  simulator.get_mutable_context().SetAccuracy(tol);

  // Get the isolation interval tolerance.
  const std::optional<double> iso_tol =
      simulator.GetCurrentWitnessTimeIsolation();
  EXPECT_TRUE(iso_tol);

  // Simulate to right after the second one should have triggered.
  simulator.AdvanceTo(2.1);

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
  StatelessSystem system(+1, WitnessFunctionDirection::kCrossesZero);
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    num_publishes++;
  });

  const double h = 1;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, h);
  Context<double>& context = simulator.get_mutable_context();
  context.SetTime(0);
  simulator.AdvanceTo(1);

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
  StatelessSystem system(0, WitnessFunctionDirection::kCrossesZero);
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    num_publishes++;
  });

  const double h = 1;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, h);
  Context<double>& context = simulator.get_mutable_context();
  context.SetTime(0);
  simulator.AdvanceTo(1);

  // Verify that no publication is performed when stepping to 1.
  EXPECT_EQ(0, num_publishes);
}

// Tests ability of simulation to identify the proper number of witness function
// triggerings (zero) for a positive-to-negative trigger. Uses the same empty
// system from WitnessTestCountSimple.
GTEST_TEST(SimulatorTest, WitnessTestCountSimplePositiveToNegative) {
  // Set empty system to trigger when time is +1.
  StatelessSystem system(+1, WitnessFunctionDirection::
      kPositiveThenNonPositive);
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    num_publishes++;
  });

  const double h = 1;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, h);
  Context<double>& context = simulator.get_mutable_context();
  context.SetTime(0);
  simulator.AdvanceTo(2);

  // Publication should not occur (witness function should initially evaluate
  // to a negative value, then will evolve to a positive value).
  EXPECT_EQ(0, num_publishes);
}

// Tests ability of simulation to identify the proper number of witness function
// triggerings (zero) for a negative-to-positive trigger. Uses the same empty
// system from WitnessTestCountSimple.
GTEST_TEST(SimulatorTest, WitnessTestCountSimpleNegativeToPositive) {
  StatelessSystem system(0, WitnessFunctionDirection::
      kNegativeThenNonNegative);
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    num_publishes++;
  });

  const double h = 1;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, h);
  Context<double>& context = simulator.get_mutable_context();
  context.SetTime(-1);
  simulator.AdvanceTo(1);

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

  const double h = 1e-6;
  Simulator<double> simulator(system);
  InitFixedStepIntegratorForWitnessTesting(&simulator, h);
  Context<double>& context = simulator.get_mutable_context();
  context.get_mutable_continuous_state()[0] = -1;
  simulator.AdvanceTo(1e-4);

  // Publication should occur only at witness function crossing.
  EXPECT_EQ(1, num_publishes);
}

// A system that publishes with a period of 1s and offset of 0.5s and also
// publishes when a witness function triggers (happens when a time is crossed).
class PeriodicPublishWithTimedWitnessSystem final : public LeafSystem<double> {
 public:
  explicit PeriodicPublishWithTimedWitnessSystem(double witness_trigger_time) {
    // Declare the periodic publish.
    this->DeclarePeriodicPublishEvent(
      1.0, 0.5, &PeriodicPublishWithTimedWitnessSystem::PublishPeriodic);

    // Declare the publish event for the witness trigger.
    auto fn = [this](const Context<double>& context,
        const PublishEvent<double>& witness_publish_event) {
      return this->PublishWitness(context, witness_publish_event);
    };
    PublishEvent<double> witness_publish(fn);
    witness_ = this->MakeWitnessFunction(
      "timed_witness", WitnessFunctionDirection::kPositiveThenNonPositive,
      [witness_trigger_time](const Context<double>& context) {
        return witness_trigger_time - context.get_time();
      }, witness_publish);
  }

  double periodic_publish_time() const { return periodic_publish_time_; }
  double witness_publish_time() const { return witness_publish_time_; }

 private:
  std::unique_ptr<WitnessFunction<double>> witness_;
  mutable double periodic_publish_time_{-1.0};
  mutable double witness_publish_time_{-1.0};

  void DoGetWitnessFunctions(
      const Context<double>&,
      std::vector<const WitnessFunction<double>*>* w) const {
    *w = { witness_.get() };
  }

  // Modifying system members in System callbacks is an anti-pattern.
  // It is done here (and below) only to simplify the testing code.
  void PublishPeriodic(const Context<double>& context) const {
    ASSERT_LT(periodic_publish_time_, 0.0);
    periodic_publish_time_ = context.get_time();
  }

  void PublishWitness(const Context<double>& context,
      const PublishEvent<double>&) const {
    ASSERT_LT(witness_publish_time_, 0.0);
    witness_publish_time_ = context.get_time();
  }
};

// Tests ability of simulation to properly handle a sequence of a timed event
// and a witnessed triggered event. All three combinations of sequences are
// tested: witnessed then timed, timed then witnessed, and both triggering
// simultaneously.
GTEST_TEST(SimulatorTest, WitnessAndTimedSequences) {
  // System will publish at 0.5. Witness function triggers at the specified
  // time.
  PeriodicPublishWithTimedWitnessSystem sys_25(0.25), sys_50(0.5), sys_75(0.75);

  // Note: setting accuracy in the Context will not guarantee that the values
  // in the tests are satisfied to the same accuracy. The test tolerances might
  // need to be slightly adjusted in the future (with different integrators,
  // adjustments to the witness isolation interval, etc.)
  const double accuracy = 1e-12;
  Simulator<double> sim_25(sys_25);
  sim_25.get_mutable_context().SetAccuracy(accuracy);
  sim_25.AdvanceTo(1.0);
  EXPECT_NEAR(sys_25.witness_publish_time(), 0.25, accuracy);

  // Timed events (here and below) should be bit exact.
  EXPECT_EQ(sys_25.periodic_publish_time(), 0.5);

  Simulator<double> sim_50(sys_50);
  sim_50.get_mutable_context().SetAccuracy(accuracy);
  sim_50.AdvanceTo(1.0);
  EXPECT_NEAR(sys_50.witness_publish_time(), 0.5, accuracy);
  EXPECT_EQ(sys_50.periodic_publish_time(), 0.5);

  Simulator<double> sim_75(sys_75);
  sim_75.get_mutable_context().SetAccuracy(accuracy);
  sim_75.AdvanceTo(1.0);
  EXPECT_NEAR(sys_75.witness_publish_time(), 0.75, accuracy);
  EXPECT_EQ(sys_75.periodic_publish_time(), 0.5);
}

// TODO(edrumwri): Add tests for verifying that correct interval returned
// in the case of multiple witness functions. See issue #6184.

GTEST_TEST(SimulatorTest, SecondConstructor) {
  // Create the spring-mass system and context.
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  auto context = spring_mass.CreateDefaultContext();

  // Mark the context with an arbitrary value.
  context->SetTime(3.);

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
  const double h = 1e-3;

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(h);

  // Initialize the simulator first.
  simulator.Initialize();
}

GTEST_TEST(SimulatorTest, ContextAccess) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set the integrator default step size.
  const double h = 1e-3;

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(h);

  // Initialize the simulator first.
  simulator.Initialize();

  // Try some other context stuff.
  simulator.get_mutable_context().SetTime(3.);
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
  EXPECT_TRUE(simulator.has_context());
  simulator.release_context();
  EXPECT_FALSE(simulator.has_context());
  DRAKE_EXPECT_THROWS_MESSAGE(simulator.Initialize(), std::logic_error,
      ".*Initialize.*Context.*not.*set.*");

  // Create another context.
  auto ucontext = spring_mass.CreateDefaultContext();
  ucontext->SetTime(3.);
  simulator.reset_context(std::move(ucontext));
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
  EXPECT_TRUE(simulator.has_context());
  simulator.reset_context(nullptr);
  EXPECT_FALSE(simulator.has_context());
}

// Try a purely continuous system with no sampling.
GTEST_TEST(SimulatorTest, SpringMassNoSample) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // Set the integrator default step size.
  const double h = 1e-3;

  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(&simulator.get_mutable_context(), 0.1);

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(h);

  simulator.set_target_realtime_rate(0.5);
  // Request forced-publishes at every internal step.
  simulator.set_publish_at_initialization(true);
  simulator.set_publish_every_time_step(true);

  // Set the integrator and initialize the simulator.
  simulator.Initialize();

  // Simulate for 1 second.
  simulator.AdvanceTo(1.);

  EXPECT_NEAR(simulator.get_context().get_time(), 1., 1e-8);
  EXPECT_EQ(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_discrete_updates(), 0);

  EXPECT_EQ(spring_mass.get_publish_count(), 1001);
  EXPECT_EQ(spring_mass.get_update_count(), 0);

  // Current time is 1. An earlier final time should fail.
  EXPECT_THROW(simulator.AdvanceTo(0.5), std::runtime_error);
}

// Test ability to swap integrators mid-stream.
GTEST_TEST(SimulatorTest, ResetIntegratorTest) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // set the integrator default step size
  const double h = 1e-3;

  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(&simulator.get_mutable_context(), 0.1);

  // Get the context.
  Context<double>& context = simulator.get_mutable_context();

  // Create the integrator with the simple spelling.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(h);

  // Set the integrator and initialize the simulator
  simulator.Initialize();

  // Simulate for 1/2 second.
  simulator.AdvanceTo(0.5);

  // Reset the integrator.
  simulator.reset_integrator<RungeKutta2Integrator<double>>(h);

  // Simulate to 1 second..
  simulator.AdvanceTo(1.);

  EXPECT_NEAR(context.get_time(), 1., 1e-8);

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
  simulator.get_mutable_integrator().set_maximum_step_size(0.001);
  simulator.get_mutable_context().SetTime(0.);
  simulator.Initialize();
  simulator.AdvanceTo(1.);  // Simulate for 1 simulated second.
  EXPECT_TRUE(simulator.get_actual_realtime_rate() <= 1.1);

  simulator.set_target_realtime_rate(5.);  // No faster than 5X real time.
  simulator.get_mutable_context().SetTime(0.);
  simulator.Initialize();
  simulator.AdvanceTo(1.);  // Simulate for 1 more simulated second.
  EXPECT_TRUE(simulator.get_actual_realtime_rate() <= 5.1);
}

// Tests that if publishing every timestep is disabled and publish on
// initialization is enabled, publish only happens on initialization.
GTEST_TEST(SimulatorTest, DisablePublishEveryTimestep) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.
  simulator.set_publish_at_initialization(true);
  simulator.set_publish_every_time_step(false);

  simulator.get_mutable_context().SetTime(0.);
  simulator.Initialize();
  // Publish should happen on initialization.
  EXPECT_EQ(1, simulator.get_num_publishes());

  // Simulate for 1 simulated second.  Publish should not happen.
  simulator.AdvanceTo(1.);
  EXPECT_EQ(1, simulator.get_num_publishes());
}

// Repeat the previous test but now the continuous steps are interrupted
// by a discrete sample every 1/30 second. The step size doesn't divide that
// evenly so we should get some step size modification here.
GTEST_TEST(SimulatorTest, SpringMass) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // Set the integrator default step size.
  const double h = 1e-3;

  // Create the mass spring system and the simulator.
  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 30.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal context.
  spring_mass.set_position(&simulator.get_mutable_context(), 0.1);

  // Create the integrator and initialize it.
  auto& integrator =
      simulator.reset_integrator<ExplicitEulerIntegrator<double>>(h);
  integrator.Initialize();

  // Set the integrator and initialize the simulator.
  simulator.Initialize();

  // Simulate to one second.
  simulator.AdvanceTo(1.);

  EXPECT_GT(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_discrete_updates(), 30);

  // We're calling Publish() every step, and extra steps have to be taken
  // since the step size doesn't divide evenly into the sample rate. Shouldn't
  // require more than one extra step per sample though.
  EXPECT_LE(spring_mass.get_publish_count(), 1030);
  EXPECT_EQ(spring_mass.get_update_count(), 30);
}

// This is the example from discrete_systems.h. Let's make sure it works
// as advertised there! The discrete system is:
//    x_{n+1} = x_n + 1
//    y_n     = 10 x_n
//    x_0     = 0
// which should produce 0 10 20 30 ... .
//
// Don't change this unit test without making a corresponding change to the
// doxygen example in the systems/discrete_systems.h module. This class should
// be as identical to the code there as possible; ideally, just a
// copy-and-paste.
class ExampleDiscreteSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExampleDiscreteSystem)

  ExampleDiscreteSystem() {
    DeclareDiscreteState(1);  // Just one state variable, x[0], default=0.

    // Update to x_{n+1} using a Drake "discrete update" event (occurs
    // at the beginning of step n+1).
    DeclarePeriodicDiscreteUpdateEvent(kPeriod, kOffset,
                                       &ExampleDiscreteSystem::Update);

    // Present y_n (=S_n) at the output port.
    DeclareVectorOutputPort("Sn", 1, &ExampleDiscreteSystem::Output);
  }

  static constexpr double kPeriod = 1 / 50.;  // Update at 50Hz (h=1/50).
  static constexpr double kOffset = 0.;       // Trigger events at n=0.

 private:
  void Update(const systems::Context<double>& context,
              systems::DiscreteValues<double>* xd) const {
    const double x_n = context.get_discrete_state()[0];
    (*xd)[0] = x_n + 1.;
  }

  void Output(const systems::Context<double>& context,
              systems::BasicVector<double>* result) const {
    const double x_n = context.get_discrete_state()[0];
    const double S_n = 10 * x_n;
    (*result)[0] = S_n;
  }
};

// Tests the code fragment shown in the systems/discrete_systems.h module. Make
// this as copypasta-identical as possible to the code there, and make matching
// changes there if you change anything here.
GTEST_TEST(SimulatorTest, ExampleDiscreteSystem) {
  // Build a Diagram containing the Example system and a data logger that
  // samples the Sn output port exactly at the update times.
  DiagramBuilder<double> builder;
  auto example = builder.AddSystem<ExampleDiscreteSystem>();
  auto logger = LogVectorOutput(example->GetOutputPort("Sn"), &builder,
                                ExampleDiscreteSystem::kPeriod);
  auto diagram = builder.Build();

  // Create a Simulator and use it to advance time until t=3*h.
  Simulator<double> simulator(*diagram);
  simulator.AdvanceTo(3 * ExampleDiscreteSystem::kPeriod);

  testing::internal::CaptureStdout();  // Not in example.

  // Print out the contents of the log.
  const auto& log = logger->FindLog(simulator.get_context());
  for (int n = 0; n < log.sample_times().size(); ++n) {
    const double t = log.sample_times()[n];
    std::cout << n << ": " << log.data()(0, n)
              << " (" << t << ")\n";
  }

  // Not in example (although the expected output is there).
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(output, "0: 0 (0)\n"
                    "1: 10 (0.02)\n"
                    "2: 20 (0.04)\n"
                    "3: 30 (0.06)\n");
}

// A hybrid discrete-continuous system:
//   x_{n+1} = sin(1.234*t)
//   y_n     = x_n
// With proper initial conditions, this should produce a one-step-delayed
// sample of the periodic function, so that y_n = sin(1.234 * (n-1)*h).
class SinusoidalDelayHybridSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SinusoidalDelayHybridSystem)

  SinusoidalDelayHybridSystem() {
    this->DeclarePeriodicDiscreteUpdateEvent(
        kUpdatePeriod, 0.0, &SinusoidalDelayHybridSystem::Update);
    this->DeclareDiscreteState(1 /* single state variable */);
    this->DeclareVectorOutputPort("y", 1,
                                  &SinusoidalDelayHybridSystem::CalcOutput);
  }

  static constexpr double kSinusoidalFrequency = 1.234;
  static constexpr double kUpdatePeriod = 0.25;

 private:
  void Update(const Context<double>& context,
              DiscreteValues<double>* x_next) const {
    const double t = context.get_time();
    (*x_next)[0] = std::sin(kSinusoidalFrequency * t);
  }

  void CalcOutput(const Context<double>& context,
                  BasicVector<double>* output) const {
    (*output)[0] = context.get_discrete_state()[0];  // y = x.
  }
};

// Tests that sinusoidal hybrid system that is not periodic in the update period
// produces the result from simulating the discrete system:
//   x_{n+1} = sin(f * n * h)
//   y_n     = x_n
//   x_0     = sin(f * -1 * h)
// where h is the update period and f is the frequency of the sinusoid.
// This should be a one-step delayed discrete sampling of the sinusoid.
GTEST_TEST(SimulatorTest, SinusoidalHybridSystem) {
  const double h = SinusoidalDelayHybridSystem::kUpdatePeriod;
  const double f = SinusoidalDelayHybridSystem::kSinusoidalFrequency;

  // Build the diagram.
  DiagramBuilder<double> builder;
  auto sinusoidal_system = builder.AddSystem<SinusoidalDelayHybridSystem>();
  auto logger = builder.AddSystem<VectorLogSink<double>>(1 /* input size */, h);
  builder.Connect(*sinusoidal_system, *logger);
  auto diagram = builder.Build();

  // Simulator.
  const double t_final = 10.0;
  const double initial_value = std::sin(-f * h);  // = sin(f*-1*h)
  Simulator<double> simulator(*diagram);

  simulator.get_mutable_context().get_mutable_discrete_state()[0] =
      initial_value;
  simulator.AdvanceTo(t_final);

  // Set a very tight tolerance value.
  const double eps = 1e-14;

  // Get the output from the signal logger. It will look like this in the
  // signal logger:
  //
  // y value    n    corresponding time   signal logger value (delayed)
  // -------   ---   ------------------   -----------------------------
  // y₀         0    0                    sin(-f h)
  // y₁         1    t = h                sin(0)
  // y₂         2    t = 2 h              sin(f h)
  // y₃         3    t = 3 h              sin(f 2 h)
  // ...
  const auto& log = logger->FindLog(simulator.get_context());
  const VectorX<double> times = log.sample_times();
  const MatrixX<double> data = log.data();

  ASSERT_EQ(times.size(), std::round(t_final/h) + 1);
  ASSERT_EQ(data.rows(), 1);
  for (int n = 0; n < times.size(); ++n) {
    const double t_n = times[n];
    const double y_n = data(0, n);
    EXPECT_NEAR(t_n, n * h, eps);
    EXPECT_NEAR(std::sin(f * (n - 1) * h), y_n, eps);
  }
}

// A continuous system that outputs unity plus time.
class ShiftedTimeOutputter : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ShiftedTimeOutputter)

  ShiftedTimeOutputter() {
    this->DeclareVectorOutputPort("time", 1, &ShiftedTimeOutputter::OutputTime);
  }

 private:
  void OutputTime(
      const Context<double>& context, BasicVector<double>* output) const {
    (*output)[0] = context.get_time() + 1;
  }
};

// A hybrid discrete-continuous system:
//   x_{n+1} = x_n + u(t)
//   x_0     = 0
class SimpleHybridSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleHybridSystem)

  explicit SimpleHybridSystem(double offset) {
    this->DeclarePeriodicDiscreteUpdateEvent(kPeriod, offset,
        &SimpleHybridSystem::Update);
    this->DeclareDiscreteState(1 /* single state variable */);
    this->DeclareVectorInputPort("u", 1);
  }

 private:
  void Update(const Context<double>& context,
              DiscreteValues<double>* x_next) const {
    const double u = this->get_input_port(0).Eval(context)[0];  // u(t)
    double x = context.get_discrete_state()[0];                 // x_n
    (*x_next)[0] = x + u;
  }

  const double kPeriod = 1.0;
};

// Tests the update sequence for a simple mixed discrete/continuous system
// that uses the prescribed updating offset of 0.0.
GTEST_TEST(SimulatorTest, SimpleHybridSystemTestOffsetZero) {
  DiagramBuilder<double> builder;
  // Connect a system that outputs u(t) = t to the hybrid system
  // x_{n+1} = x_n + u(t).
  auto shifted_time_outputter = builder.AddSystem<ShiftedTimeOutputter>();
  const double updating_offset_time = 0.0;
  auto hybrid_system = builder.AddSystem<SimpleHybridSystem>(
      updating_offset_time);
  builder.Connect(*shifted_time_outputter, *hybrid_system);
  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);

  // Set the initial condition x_0 (the subscript notation reflects the
  // discrete step number as described in discrete_systems.h).
  const double initial_condition = 0.0;
  simulator.get_mutable_context().get_mutable_discrete_state()[0] =
      initial_condition;

  // Simulate forward. The first update occurs at t=0, meaning AdvanceTo(1)
  // updates the discrete state to x⁺(0) (i.e., x_1) before updating time to 1.
  simulator.AdvanceTo(1.0);

  // Check that the expected state value was attained. The value should be
  // x_0 + u(0) since we expect the discrete update to occur at t = 0 when
  // u(t) = 1.
  const double u0 = 1;
  EXPECT_EQ(simulator.get_context().get_discrete_state()[0],
            initial_condition + u0);

  // Check that the expected number of updates (one) was performed.
  EXPECT_EQ(simulator.get_num_discrete_updates(), 1);
}

// A "Delta function" system that outputs zero except at the instant (the spike
// time) when the output is 1. This function of time is continuous otherwise.
// We'll verify that the output of this system into a discrete system produces
// samples at the expected instant in time.
class DeltaFunction : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeltaFunction)

  explicit DeltaFunction(double spike_time) : spike_time_(spike_time) {
    this->DeclareVectorOutputPort("spike", 1, &DeltaFunction::Output);
  }

  // Change the spike time. Be sure to re-initialize after calling this.
  void set_spike_time(double spike_time) {
    spike_time_ = spike_time;
  }

 private:
  void Output(
      const Context<double>& context, BasicVector<double>* output) const {
    (*output)[0] = context.get_time() == spike_time_ ? 1. : 0.;
  }

  double spike_time_{};
};

// This is a mixed continuous/discrete system:
//    x_{n+1} = x_n + u(t)
//    y_n     = x_n
//    x_0     = 0
// By plugging interesting things into the input we can test whether we're
// sampling the continuous input at the appropriate times.
class DiscreteInputAccumulator : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteInputAccumulator)

  DiscreteInputAccumulator() {
    DeclareDiscreteState(1);  // Just one state variable, x[0].

    DeclareVectorInputPort("u", 1);

    // Set initial condition x_0 = 0, and clear the result.
    DeclareInitializationEvent(
        DiscreteUpdateEvent<double>([this](const Context<double>&,
                                           const DiscreteUpdateEvent<double>&,
                                           DiscreteValues<double>* x_0) {
          (*x_0)[0] = 0.;
          result_.clear();
        }));

    // Output y_n using a Drake "publish" event (occurs at the end of step n).
    DeclarePeriodicEvent(
        kPeriod, kPublishOffset,
        PublishEvent<double>(
            [this](const Context<double>& context,
                   const PublishEvent<double>&) {
              result_.push_back(get_x(context));  // y_n = x_n
            }));

    // Update to x_{n+1} (x_np1), using a Drake "discrete update" event (occurs
    // at the beginning of step n+1).
    DeclarePeriodicEvent(
        kPeriod, kPublishOffset,
        DiscreteUpdateEvent<double>([this](const Context<double>& context,
                                           const DiscreteUpdateEvent<double>&,
                                           DiscreteValues<double>* x_np1) {
          const double x_n = get_x(context);
          const double u = get_input_port(0).Eval(context)[0];
          (*x_np1)[0] = x_n + u;  // x_{n+1} = x_n + u(t)
        }));
  }

  const std::vector<double>& result() const { return result_; }

  static constexpr double kPeriod = 0.125;
  static constexpr double kPublishOffset = 0.;

 private:
  double get_x(const Context<double>& context) const {
    return context.get_discrete_state()[0];
  }

  std::vector<double> result_;
};

// Build a diagram that takes a DeltaFunction input and then simulates this
// mixed discrete/continuous system:
//    x_{n+1} = x_n + u(t)
//    y_n     = x_n
//    x_0     = 0
// Let t_s be the chosen "spike time" for the delta function. We expect the
// output y_n to be zero for all n unless a sample at k*h occurs exactly at
// tₛ for some k. In that case y_n=0, n ≤ k and y_n=1, n > k. Important cases
// to check are: t_s=0, t_s=k*h for some k>0, and t_s≠k*h for any k.
GTEST_TEST(SimulatorTest, SpikeTest) {
  DiagramBuilder<double> builder;

  auto delta = builder.AddSystem<DeltaFunction>(0.);
  auto hybrid_system = builder.AddSystem<DiscreteInputAccumulator>();
  builder.Connect(delta->get_output_port(0), hybrid_system->get_input_port(0));
  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);

  // Test with spike time = 0.
  delta->set_spike_time(0);
  simulator.Initialize();
  simulator.AdvanceTo(5 * DiscreteInputAccumulator::kPeriod);
  EXPECT_EQ(hybrid_system->result(), std::vector<double>({0, 1, 1, 1, 1, 1}));

  // Test with spike time = 3*h.
  delta->set_spike_time(3 * DiscreteInputAccumulator::kPeriod);
  simulator.get_mutable_context().SetTime(0.);
  simulator.Initialize();
  simulator.AdvanceTo(5 * DiscreteInputAccumulator::kPeriod);
  EXPECT_EQ(hybrid_system->result(), std::vector<double>({0, 0, 0, 0, 1, 1}));

  // Test with spike time not coinciding with a sample time.
  delta->set_spike_time(2.7 * DiscreteInputAccumulator::kPeriod);
  simulator.get_mutable_context().SetTime(0.);
  simulator.Initialize();
  simulator.AdvanceTo(5 * DiscreteInputAccumulator::kPeriod);
  EXPECT_EQ(hybrid_system->result(), std::vector<double>({0, 0, 0, 0, 0, 0}));
}

// A mock System that requests a single update at a prespecified time.
class UnrestrictedUpdater : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrestrictedUpdater)

  explicit UnrestrictedUpdater(double t_upd) : t_upd_(t_upd) {}

  ~UnrestrictedUpdater() override {}

  void DoCalcNextUpdateTime(const Context<double>& context,
                            CompositeEventCollection<double>* event_info,
                            double* time) const override {
    const double inf = std::numeric_limits<double>::infinity();
    *time = (context.get_time() < t_upd_) ? t_upd_ : inf;
    UnrestrictedUpdateEvent<double> event(
        TriggerType::kPeriodic);
    event.AddToComposite(event_info);
  }

  void DoCalcUnrestrictedUpdate(
      const Context<double>& context,
      const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
      State<double>* state) const override {
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

// Tests that the simulator captures an unrestricted update at the exact time
// (i.e., without accumulating floating point error).
GTEST_TEST(SimulatorTest, ExactUpdateTime) {
  // Create the UnrestrictedUpdater system.
  const double t_upd = 1e-10;  // Inexact floating point rep.
  UnrestrictedUpdater unrest_upd(t_upd);
  Simulator<double> simulator(unrest_upd);  // Use default Context.

  // Set time to an exact floating point representation; we want t_upd to
  // be much smaller in magnitude than the time, hence the negative time.
  simulator.get_mutable_context().SetTime(-1.0 / 1024);

  // Capture the time at which an update is done using a callback function.
  std::vector<double> updates;
  unrest_upd.set_unrestricted_update_callback(
      [&updates](const Context<double>& context, State<double>* state) {
        updates.push_back(context.get_time());
      });

  // Simulate forward.
  simulator.Initialize();
  simulator.AdvanceTo(1.);

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

  // Forces simulator to use fixed-step integration at 1ms (to keep assumptions
  // below accurate).
  simulator.get_mutable_integrator().set_fixed_step_mode(true);
  simulator.get_mutable_integrator().set_maximum_step_size(0.001);

  // Sets initial condition using the Simulator's internal Context.
  spring_mass.set_position(&simulator.get_mutable_context(), x0);
  spring_mass.set_velocity(&simulator.get_mutable_context(), v0);

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
  simulator.AdvanceTo(final_time);

  EXPECT_EQ(simulator.get_num_steps_taken(), 200);

  const auto& context = simulator.get_context();
  EXPECT_NEAR(context.get_time(), final_time, 1e-8);

  // Compares with analytical solution (to numerical integration error).
  EXPECT_NEAR(spring_mass.get_position(context), x_final, 3.0e-6);
  EXPECT_NEAR(spring_mass.get_velocity(context), v_final, 1.0e-5);
}

// A mock hybrid continuous-discrete System with time as its only continuous
// variable, discrete updates at 1 kHz, and requests publishes at 400 Hz. Calls
// user-configured callbacks in the publish and discrete variable update event
// handlers and in EvalTimeDerivatives. This hybrid system will be used to
// verify expected state update ordering -- discrete, continuous (i.e.,
// integration), then publish.
class MixedContinuousDiscreteSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MixedContinuousDiscreteSystem)

  MixedContinuousDiscreteSystem() {
    // Deliberately choose a period that is identical to, and therefore courts
    // floating-point error with, the default max step size.
    const double offset = 0.0;
    DeclarePeriodicDiscreteUpdateEvent(
        kUpdatePeriod, offset,
        &MixedContinuousDiscreteSystem::HandleDiscreteVariableUpdates);
    DeclarePeriodicPublishEvent(kPublishPeriod, 0.0,
                                &MixedContinuousDiscreteSystem::HandlePublish);

    // We need some continuous state (which will be unused) so that the
    // continuous state integration will not be bypassed.
    this->DeclareContinuousState(1);

    set_name("TestSystem");
  }

  ~MixedContinuousDiscreteSystem() override {}

  void HandleDiscreteVariableUpdates(const Context<double>& context,
                                     DiscreteValues<double>*) const {
    if (update_callback_ != nullptr) update_callback_(context);
  }

  void HandlePublish(const Context<double>& context) const {
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

// Tests that the Simulator invokes the MixedContinuousDiscreteSystem's
// update method every 0.001 sec, and its publish method every 0.0025 sec,
// without missing any updates.
GTEST_TEST(SimulatorTest, DiscreteUpdateAndPublish) {
  MixedContinuousDiscreteSystem system;
  int num_disc_updates = 0;
  system.set_update_callback([&](const Context<double>& context) {
    ASSERT_TRUE(CheckSampleTime(context, system.update_period()));
    num_disc_updates++;
  });
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context) {
    ASSERT_TRUE(CheckSampleTime(context, system.publish_period()));
    num_publishes++;
  });

  Simulator<double> simulator(system);
  simulator.set_publish_at_initialization(false);
  simulator.set_publish_every_time_step(false);
  simulator.AdvanceTo(0.5);

  // Update occurs at 1000Hz, at the beginning of each step (there is no
  // discrete event update during initialization nor a final update at the
  // final time).
  EXPECT_EQ(500, num_disc_updates);
  // Publication occurs at 400Hz, at the end of initialization and the end
  // of each step.
  EXPECT_EQ(200 + 1, num_publishes);
}

// Tests that the order of events in a simulator time step is first update
// discrete state, then publish, then integrate.
GTEST_TEST(SimulatorTest, UpdateThenPublishThenIntegrate) {
  MixedContinuousDiscreteSystem system;
  Simulator<double> simulator(system);
  enum EventType { kPublish = 0, kUpdate = 1, kIntegrate = 2, kTypeCount = 3};

  // Record the order in which the MixedContinuousDiscreteSystem is asked to
  // do publishes, compute discrete updates, or compute derivatives at each
  // time step.
  std::map<int, std::vector<EventType>> events;
  system.set_publish_callback(
      [&events, &simulator](const Context<double>& context) {
        events[simulator.get_num_steps_taken()].push_back(kPublish);
      });
  system.set_update_callback(
      [&events, &simulator](const Context<double>& context) {
        events[simulator.get_num_steps_taken()].push_back(kUpdate);
      });
  system.set_derivatives_callback(
      [&events, &simulator](const Context<double>& context) {
        events[simulator.get_num_steps_taken()].push_back(kIntegrate);
      });

  // Run a simulation with per-step forced-publishing enabled.
  simulator.set_publish_at_initialization(true);
  simulator.set_publish_every_time_step(true);
  simulator.AdvanceTo(0.5);

  // Verify that at least one of each event type was triggered.
  bool triggers[kTypeCount] = { false, false, false };

  // Check that all of the publish events precede all of the update events
  // (since "publish on init" was activated), which in turn precede all of the
  // derivative evaluation events, for each time step in the simulation.
  for (const auto& log : events) {
    ASSERT_GE(log.second.size(), 0u);
    EventType state = log.second[0];
    for (const EventType& event : log.second) {
      triggers[event] = true;
      ASSERT_TRUE(event >= state);
      state = event;
    }
  }
  EXPECT_TRUE(triggers[kUpdate]);
  EXPECT_TRUE(triggers[kPublish]);
  EXPECT_TRUE(triggers[kIntegrate]);
}

// A basic sanity check that AutoDiff works.
GTEST_TEST(SimulatorTest, AutodiffBasic) {
  SpringMassSystem<AutoDiffXd> spring_mass(1., 1., 0.);
  Simulator<AutoDiffXd> simulator(spring_mass);
  simulator.Initialize();
  simulator.AdvanceTo(1);
}

// Verifies that an integrator will stretch its integration step in the case
// that a directed step would end right before an event.
GTEST_TEST(SimulatorTest, StretchedStep) {
  // Setting the update rate to 1.0 will cause the spring mass to update at
  // 1.0s.
  analysis_test::MySpringMassSystem<double> spring_mass(
      1., 1., 1. /* update rate */);
  Simulator<double> simulator(spring_mass);

  // We will direct the integrator to take a single step of t_final, which
  // will be very near the mass-spring system's update event at 1.0s. 1e-8
  // is so small that any reasonable degree of step "stretching" should jump
  // to 1.0 proper.
  const double expected_t_final = 1.0;
  const double directed_t_final = expected_t_final - 1e-8;

  // Initialize a fixed step integrator and the simulator.
  simulator.reset_integrator<RungeKutta2Integrator<double>>(directed_t_final);
  simulator.Initialize();

  // Set initial condition using the Simulator's internal Context.
  simulator.get_mutable_context().SetTime(0);
  spring_mass.set_position(&simulator.get_mutable_context(), 0.1);
  spring_mass.set_velocity(&simulator.get_mutable_context(), 0);

  // Now step.
  simulator.AdvanceTo(expected_t_final);

  // Verify that the step size was stretched and that exactly one "step" was
  // taken to integrate the continuous variables forward.
  EXPECT_EQ(simulator.get_context().get_time(), expected_t_final);
  EXPECT_EQ(simulator.get_num_steps_taken(), 1);
}

// This test specifically tests for correct handling of issue #10443, in which
// an event can be missed.
GTEST_TEST(SimulatorTest, Issue10443) {
  // NOTE: This bug arose from a "perfect storm" of conditions- which
  // occurred due to the interactions of an error controlled integrator, the
  // maximum step size setting, and the particular publish period used. The
  // maintainer should assume that every line below is critical to reproducing
  // those conditions.

  // Log the output of a simple diagram containing a constant
  // source and an integrator.
  DiagramBuilder<double> builder;
  const double kValue = 2.4;
  const auto& source = *builder.AddSystem<ConstantVectorSource<double>>(kValue);
  const int kSize = 1;
  const auto& integrator = *builder.AddSystem<Integrator<double>>(kSize);
  builder.Connect(source.get_output_port(),
      integrator.get_input_port());

  // Add a periodic logger.
  const int kFrequency = 10;  // 10 cycles per second.
  auto& periodic_logger = *builder.AddSystem<VectorLogSink<double>>(
      kSize, 1.0 / kFrequency);
  builder.Connect(integrator.get_output_port(),
      periodic_logger.get_input_port());

  // Finish constructing the Diagram.
  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  // Construct the Simulator with an RK3 integrator and settings that reproduce
  // the behavior.
  Simulator<double> simulator(*diagram);
  auto& rk3 = simulator.reset_integrator<RungeKutta3Integrator<double>>();
  rk3.set_maximum_step_size(1.0 / kFrequency);
  rk3.request_initial_step_size_target(1e-4);
  rk3.set_target_accuracy(1e-4);
  rk3.set_fixed_step_mode(false);

  // Simulate.
  const int kTime = 1;
  simulator.AdvanceTo(static_cast<double>(kTime));

  // Should log exactly once every kPeriod, up to and including
  // kTime.
  Eigen::VectorBlock<const VectorX<double>> t_periodic =
      periodic_logger.FindLog(simulator.get_context()).sample_times();
  EXPECT_EQ(t_periodic.size(), kTime * kFrequency + 1);
}

// Verifies that an integrator will *not* stretch its integration step in the
// case that a directed step would be before- but not too close- to an event.
GTEST_TEST(SimulatorTest, NoStretchedStep) {
  // Setting the update rate to 1.0 will cause the spring mass to update at
  // 1.0s.
  analysis_test::MySpringMassSystem<double> spring_mass(
      1., 1., 1. /* update rate */);
  Simulator<double> simulator(spring_mass);

  // We will direct the integrator to take a single step of 0.9, which
  // will be not so near the mass-spring system's update event at 1.0s. 0.1
  // (the difference) is so large that any reasonable approach should avoid
  // stretching to 1.0 proper.
  const double event_t_final = 1.0;
  const double directed_t_final = event_t_final - 0.1;

  // Initialize a fixed step integrator.
  simulator.reset_integrator<RungeKutta2Integrator<double>>(directed_t_final);

  // Set initial condition using the Simulator's internal Context.
  simulator.get_mutable_context().SetTime(0);
  spring_mass.set_position(&simulator.get_mutable_context(), 0.1);
  spring_mass.set_velocity(&simulator.get_mutable_context(), 0);

  // Now step.
  simulator.AdvanceTo(event_t_final);

  // Verify that the step size was not stretched and that exactly two "steps"
  // were taken to integrate the continuous variables forward.
  EXPECT_EQ(simulator.get_context().get_time(), event_t_final);
  EXPECT_EQ(simulator.get_num_steps_taken(), 2);
}

// Verifies that artificially limiting a step does not change the ideal next
// step for error controlled integrators.
GTEST_TEST(SimulatorTest, ArtificalLimitingStep) {
  // Setting the update rate to 1.0 will cause the spring mass to update at
  // 1.0s.
  analysis_test::MySpringMassSystem<double> spring_mass(
    1., 1., 1. /* update rate */);
  Simulator<double> simulator(spring_mass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(&simulator.get_mutable_context(), 1.0);
  spring_mass.set_velocity(&simulator.get_mutable_context(), 0.1);

  // Accuracy tolerances are extremely loose.
  const double accuracy = 1e-1;

  // Requested step size should start out two orders of magnitude larger than
  // desired_h (defined below), in order to prime the ideal next step size.
  const double req_initial_step_size = 1e-2;

  // Initialize the error controlled integrator and the simulator.
  simulator.reset_integrator<RungeKutta3Integrator<double>>();
  IntegratorBase<double>& integrator = simulator.get_mutable_integrator();
  integrator.request_initial_step_size_target(req_initial_step_size);
  integrator.set_target_accuracy(accuracy);
  simulator.Initialize();

  // Mark the event time.
  const double event_time = 1.0;

  // Take a single step with the integrator.
  const double inf = std::numeric_limits<double>::infinity();
  const Context<double>& context = integrator.get_context();
  integrator.IntegrateNoFurtherThanTime(inf,
      context.get_time() + 1.0, context.get_time() + 1.0);

  // Verify that the integrator has stepped before the event time.
  EXPECT_LT(context.get_time(), event_time);

  // Get the ideal next step size and verify that it is not NaN.
  const double ideal_next_step_size = integrator.get_ideal_next_step_size();

  // Set the time to right before an event, which should trigger artificial
  // limiting.
  const double desired_h = req_initial_step_size * 1e-2;
  simulator.get_mutable_context().SetTime(event_time - desired_h);

  // Step to the event time.
  integrator.IntegrateNoFurtherThanTime(
    inf, context.get_time() + desired_h, inf);

  // Verify that the context is at the event time.
  EXPECT_EQ(context.get_time(), event_time);

  // Verify that artificial limiting did not change the ideal next step size.
  EXPECT_EQ(integrator.get_ideal_next_step_size(), ideal_next_step_size);
}

// Verifies that an error controlled integrator will stretch its integration
// step when it is near an update action and (a) error control is used, (b)
// minimum step size exceptions are suppressed, (c) the integration step size
// necessary to realize error tolerances is below the minimum step size.
GTEST_TEST(SimulatorTest, StretchedStepPerfectStorm) {
  // Setting the update rate to 1.0 will cause the spring mass to update at
  // 1.0s.
  analysis_test::MySpringMassSystem<double> spring_mass(
      1., 1., 1. /* update rate */);
  Simulator<double> simulator(spring_mass);

  // We will direct the integrator to take a single step of t_final, which
  // will be very near the mass-spring system's update event at 1.0s. 1e-8
  // is so small that any reasonable degree of step "stretching" should jump
  // to 1.0 proper.
  const double expected_t_final = 1.0;
  const double directed_t_final = expected_t_final - 1e-8;

  // Accuracy tolerances are tight so that the integrator is sure to decrease
  // the step size.
  const double accuracy = 1e-8;
  const double req_min_step_size = directed_t_final;

  // Initialize the error controlled integrator and the simulator.
  simulator.reset_integrator<RungeKutta3Integrator<double>>();
  IntegratorBase<double>& integrator = simulator.get_mutable_integrator();
  integrator.set_requested_minimum_step_size(req_min_step_size);
  integrator.request_initial_step_size_target(directed_t_final);
  integrator.set_target_accuracy(accuracy);

  // Set initial condition using the Simulator's internal Context.
  simulator.get_mutable_context().SetTime(0);
  spring_mass.set_position(&simulator.get_mutable_context(), 0.1);
  spring_mass.set_velocity(&simulator.get_mutable_context(), 0);

  // Activate exceptions on violating the minimum step size to verify that
  // error control is a limiting factor.
  integrator.set_throw_on_minimum_step_size_violation(true);
  EXPECT_THROW(simulator.AdvanceTo(expected_t_final), std::runtime_error);

  // Now disable exceptions on violating the minimum step size and step again.
  // Since we are changing the state between successive AdvanceTo(.) calls, it
  // is wise to call Initialize() prior to the second call.
  simulator.get_mutable_context().SetTime(0);
  spring_mass.set_position(&simulator.get_mutable_context(), 0.1);
  spring_mass.set_velocity(&simulator.get_mutable_context(), 0);
  integrator.set_throw_on_minimum_step_size_violation(false);
  simulator.Initialize();
  simulator.AdvanceTo(expected_t_final);

  // Verify that the step size was stretched and that exactly one "step" was
  // taken to integrate the continuous variables forward.
  EXPECT_EQ(simulator.get_context().get_time(), expected_t_final);
  EXPECT_EQ(simulator.get_num_steps_taken(), 1);
}

// Tests per step publish, discrete and unrestricted update actions. Each
// action handler logs the context time when it's called, and the test compares
// the time stamp against the integrator's h.
GTEST_TEST(SimulatorTest, PerStepAction) {
  class PerStepActionTestSystem : public LeafSystem<double> {
   public:
    PerStepActionTestSystem() {
      // We need some continuous state (which will be unused) so that the
      // continuous state integration will not be bypassed.
      this->DeclareContinuousState(1);
    }

    void AddPerStepPublishEvent() {
      DeclarePerStepPublishEvent(&PerStepActionTestSystem::HandlePublish);
    }

    void AddPerStepDiscreteUpdateEvent() {
      DeclarePerStepDiscreteUpdateEvent(
          &PerStepActionTestSystem::HandleDiscrete);
    }

    void AddPerStepUnrestrictedUpdateEvent() {
      DeclarePerStepUnrestrictedUpdateEvent(
          &PerStepActionTestSystem::HandleUnrestricted);
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
    void DoCalcTimeDerivatives(
        const Context<double>& context,
        ContinuousState<double>* derivatives) const override {
      // Derivative will always be zero, making the system stationary.
      derivatives->get_mutable_vector().SetAtIndex(0, 0.0);
    }

    // TODO(15465) When per-step event declaration sugar allows for callbacks
    // whose result is "assumed to succeed", change these to void return types.
    EventStatus HandleDiscrete(const Context<double>& context,
                               DiscreteValues<double>*) const {
      discrete_update_times_.push_back(context.get_time());
      return EventStatus::Succeeded();
    }

    EventStatus HandleUnrestricted(const Context<double>& context,
                                   State<double>*) const {
      unrestricted_update_times_.push_back(context.get_time());
      return EventStatus::Succeeded();
    }

    EventStatus HandlePublish(const Context<double>& context) const {
      publish_times_.push_back(context.get_time());
      return EventStatus::Succeeded();
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
  sys.AddPerStepPublishEvent();
  sys.AddPerStepUnrestrictedUpdateEvent();
  sys.AddPerStepDiscreteUpdateEvent();
  Simulator<double> sim(sys);

  // Forces simulator to use fixed-step integration.
  sim.get_mutable_integrator().set_fixed_step_mode(true);
  sim.get_mutable_integrator().set_maximum_step_size(0.001);

  // Disables all simulator induced publish events, so that all publish calls
  // are initiated by sys.
  sim.set_publish_at_initialization(false);
  sim.set_publish_every_time_step(false);
  sim.Initialize();
  sim.AdvanceTo(0.1);

  const double h = sim.get_integrator().get_maximum_step_size();
  const int N = static_cast<int>(0.1 / h);
  // Step size was set to 1ms above; make sure we don't have roundoff trouble.
  EXPECT_EQ(N, 100);
  ASSERT_EQ(sim.get_num_steps_taken(), N);

  auto& publish_times = sys.get_publish_times();
  auto& discrete_update_times = sys.get_discrete_update_times();
  auto& unrestricted_update_times = sys.get_unrestricted_update_times();
  ASSERT_EQ(publish_times.size(), N + 1);  // Once at end of Initialize().
  ASSERT_EQ(sys.get_discrete_update_times().size(), N);
  ASSERT_EQ(sys.get_unrestricted_update_times().size(), N);
  for (int i = 0; i < N; ++i) {
    // Publish happens at the end of a step (including end of Initialize());
    // unrestricted and discrete updates happen at the beginning of a step.
    EXPECT_NEAR(publish_times[i], i * h, 1e-12);
    EXPECT_NEAR(discrete_update_times[i], i * h, 1e-12);
    EXPECT_NEAR(unrestricted_update_times[i], i * h, 1e-12);
  }
  // There is a final end-of-step publish, but no final updates.
  EXPECT_NEAR(publish_times[N], N * h, 1e-12);
}

// Tests initialization from the simulator.
GTEST_TEST(SimulatorTest, Initialization) {
  class InitializationTestSystem : public LeafSystem<double> {
   public:
    InitializationTestSystem() {
      PublishEvent<double> pub_event(
          TriggerType::kInitialization,
          std::bind(&InitializationTestSystem::InitPublish, this,
                    std::placeholders::_1, std::placeholders::_2));
      DeclareInitializationEvent(pub_event);

      DeclareInitializationEvent(DiscreteUpdateEvent<double>(
          TriggerType::kInitialization));
      DeclareInitializationEvent(UnrestrictedUpdateEvent<double>(
          TriggerType::kInitialization));

      DeclarePeriodicDiscreteUpdate(0.1);
      DeclarePerStepEvent<UnrestrictedUpdateEvent<double>>(
          UnrestrictedUpdateEvent<double>(
              TriggerType::kPerStep));
    }

    bool get_pub_init() const { return pub_init_; }
    bool get_dis_update_init() const { return dis_update_init_; }
    bool get_unres_update_init() const { return unres_update_init_; }
    void reset() const {
      pub_init_ = false;
      dis_update_init_ = false;
      unres_update_init_ = false;
    }

   private:
    void InitPublish(const Context<double>& context,
                     const PublishEvent<double>& event) const {
      EXPECT_EQ(context.get_time(), 0);
      EXPECT_EQ(event.get_trigger_type(),
                TriggerType::kInitialization);
      pub_init_ = true;
    }

    void DoCalcDiscreteVariableUpdates(
        const Context<double>& context,
        const std::vector<const DiscreteUpdateEvent<double>*>& events,
        DiscreteValues<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      if (events.front()->get_trigger_type() ==
          TriggerType::kInitialization) {
        EXPECT_EQ(context.get_time(), 0);
        dis_update_init_ = true;
      }
    }

    void DoCalcUnrestrictedUpdate(
        const Context<double>& context,
        const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
        State<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      if (events.front()->get_trigger_type() ==
          TriggerType::kInitialization) {
        EXPECT_EQ(context.get_time(), 0);
        unres_update_init_ = true;
      }
    }

    mutable bool pub_init_{false};
    mutable bool dis_update_init_{false};
    mutable bool unres_update_init_{false};
  };

  InitializationTestSystem sys;
  Simulator<double> simulator(sys);
  simulator.AdvanceTo(1);

  EXPECT_TRUE(sys.get_pub_init());
  EXPECT_TRUE(sys.get_dis_update_init());
  EXPECT_TRUE(sys.get_unres_update_init());

  sys.reset();
  simulator.get_mutable_context().SetTime(0);
  simulator.Initialize({.suppress_initialization_events = true});
  simulator.AdvanceTo(1);

  EXPECT_FALSE(sys.get_pub_init());
  EXPECT_FALSE(sys.get_dis_update_init());
  EXPECT_FALSE(sys.get_unres_update_init());
}

GTEST_TEST(SimulatorTest, OwnedSystemTest) {
  const double offset = 0.1;
  Simulator<double> simulator_w_system(
      std::make_unique<ExampleDiagram>(offset));

  // Check that my System reference is still valid.
  EXPECT_NE(
      dynamic_cast<const ExampleDiagram*>(&simulator_w_system.get_system()),
      nullptr);
}

// This integrator is just explicit Euler with an extra unnecessary derivative
// calculation thrown in to test that the derivative counter isn't fooled.
class WastefulIntegrator final : public IntegratorBase<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WastefulIntegrator)
  ~WastefulIntegrator() override = default;

  WastefulIntegrator(const System<double>& system, double max_step_size,
                     Context<double>* context = nullptr)
      : IntegratorBase<double>(system, context) {
    IntegratorBase<double>::set_maximum_step_size(max_step_size);
  }

  bool supports_error_estimation() const final { return false; }
  int get_error_estimate_order() const final { return 0; }

 private:
  bool DoStep(const double& h) final {
    Context<double>& context = *this->get_mutable_context();
    this->EvalTimeDerivatives(context);  // Unused, but now up-to-date.

    // The rest of this is copied from ExplicitEuler.
    const ContinuousState<double>& xc_deriv =
        this->EvalTimeDerivatives(context);
    const VectorBase<double>& xcdot0 = xc_deriv.get_vector();
    VectorBase<double>& xc = context.SetTimeAndGetMutableContinuousStateVector(
        context.get_time() + h);
    xc.PlusEqScaled(h, xcdot0);
    return true;
  }
};

// The integrators are supposed to keep count of how many _actual_ derivative
// evaluations are performed. Requests to evaluate that just return an
// already-cached value don't count. For this test we use the fake "integrator"
// above whose only "virtue" is that it makes multiple calls to derivatives
// without changing the context so only the first of those should count.
GTEST_TEST(SimulatorTest, EvalDerivativesCounter) {
  SpringMassSystem<double> spring_mass(1., 1., 0.);

  Simulator<double> simulator(spring_mass);
  Context<double>& context = simulator.get_mutable_context();
  context.DisableCaching();
  simulator.reset_integrator<WastefulIntegrator>(0.125);
  simulator.AdvanceTo(1.);  // 8 steps, but 16 evaluations since no caching.
  EXPECT_EQ(simulator.get_integrator().get_num_steps_taken(), 8);
  EXPECT_EQ(simulator.get_integrator().get_num_derivative_evaluations(), 16);

  context.EnableCaching();
  simulator.AdvanceTo(2.);  // 8 more steps, but only 8 more evaluations.
  EXPECT_EQ(simulator.get_integrator().get_num_steps_taken(), 16);
  EXPECT_EQ(simulator.get_integrator().get_num_derivative_evaluations(), 24);
}

// Verify correct functioning of the monitor() API and runtime monitor
// behavior, including correct monitor status reporting from the Simulator.
GTEST_TEST(SimulatorTest, MonitorFunctionAndStatusReturn) {
  SpringMassSystem<double> spring_mass(1., 1., 0.);
  spring_mass.set_name("my_spring_mass");
  Simulator<double> simulator(spring_mass);
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(0.125);
  std::vector<Eigen::VectorXd> states;
  const auto monitor = [&states](const Context<double>& root_context) {
    Eigen::VectorXd vector(1 + root_context.num_continuous_states());
    vector << root_context.get_time(),
        root_context.get_continuous_state_vector().CopyToVector();
    states.push_back(vector);
    return EventStatus::Succeeded();
  };

  simulator.set_monitor(monitor);
  EXPECT_TRUE(simulator.get_monitor());

  SimulatorStatus status = simulator.AdvanceTo(1.);  // Initialize + 8 steps.
  EXPECT_EQ(status.reason(), SimulatorStatus::kReachedBoundaryTime);
  EXPECT_EQ(simulator.get_num_steps_taken(), 8);
  EXPECT_EQ(states.size(), 9u);

  EXPECT_THAT(status.FormatMessage(), ::testing::MatchesRegex(
      "Simulator successfully reached the boundary time.*1.*"));

  // Check that some of the timestamps are right.
  EXPECT_EQ(states[0](0), 0.);
  EXPECT_EQ(states[1](0), 0.125);
  EXPECT_EQ(states[8](0), 1.);

  simulator.clear_monitor();
  EXPECT_FALSE(simulator.get_monitor());
  simulator.AdvanceTo(2.);
  EXPECT_EQ(simulator.get_num_steps_taken(), 16);
  EXPECT_EQ(states.size(), 9u);

  simulator.set_monitor(monitor);
  simulator.AdvanceTo(3.);  // 8 more steps.
  EXPECT_EQ(simulator.get_num_steps_taken(), 24);
  EXPECT_EQ(states.size(), 17u);
  EXPECT_EQ(states[16](0), 3.);

  // This monitor should provide a clean termination that is properly
  // reported through the Simulator's status return.
  const auto good_monitor = [](const Context<double>& root_context) {
    const double time = root_context.get_time();
    // Don't pass a System to blame for termination.
    return time < 3.49 ? EventStatus::Succeeded()
                       : EventStatus::ReachedTermination(nullptr, "All done.");
  };

  simulator.set_monitor(good_monitor);
  status = simulator.AdvanceTo(10.);
  EXPECT_EQ(status.reason(), SimulatorStatus::kReachedTerminationCondition);
  // Time should be exactly 3.5 due to our choice of step size.
  EXPECT_EQ(simulator.get_context().get_time(), 3.5);
  EXPECT_THAT(
      status.FormatMessage(),
      ::testing::MatchesRegex(
          "Simulator returned early.*3\\.5.*because.*requested termination.*"
          "with message.*All done\\..*"));

  // This monitor produces a hard failure that should cause the Simulator
  // to throw with a helpful message.
  const auto bad_monitor = [&spring_mass](const Context<double>& root_context) {
    const double time = root_context.get_time();
    // Blame spring_mass for the error.
    return time < 5.99 ? EventStatus::Succeeded()
                       : EventStatus::Failed(&spring_mass,
                           "Something terrible happened.");
  };
  simulator.set_monitor(bad_monitor);
  DRAKE_EXPECT_THROWS_MESSAGE(
      simulator.AdvanceTo(10.), std::runtime_error,
      ".*Simulator stopped at time 6.*because.*"
      "SpringMassSystem.*my_spring_mass.*"
      "failed with message.*Something terrible happened.*");
}

// Simulator::Initialize() called at time t temporarily moves time back to
// t̅ = t-δ prior to calling CalcNextUpdateTime() so that timed events scheduled
// for time t will trigger. (t̅ is the next floating point value below t.)
// This causes trouble for DoCalcNextUpdateTime() overloads that want an
// event "right now", since those return contact.get_time(), which in this
// case will be t̅ when it should have been t. When a Diagram runs through
// all its subsystems looking for the "next" update time, it keeps only the
// ones that occur at the earliest of all times reported, and forgets any
// later ones (since obviously those are not "next"). Without special handling,
// the "right now" events at t̅ would prevent other time t events from being
// seen. (This was not done originally, see Drake issue #13296.)
//
// The case here creates a two-subsystem Diagram in which one of the subsystems
// specifies a "right now" update time while the other has an event that occurs
// at a specified time t. We'll verify that they play well together under
// various circumstances that can occur during Simulator::Initialize().
GTEST_TEST(SimulatorTest, MissedPublishEventIssue13296) {
  // This models systems like LcmSubscriberSystem that want to generate
  // an event as soon as possible after an external message arrives. Here we
  // just set a flag to indicate that a "message" is waiting and generate an
  // event whenever that flag is set.
  class RightNowEventSystem : public LeafSystem<double> {
   public:
    int publish_count() const { return publish_counter_; }
    void reset_count() { publish_counter_ = 0; }
    void set_message_is_waiting(bool message_is_waiting) {
      message_is_waiting_ = message_is_waiting;
    }

   private:
    void DoCalcNextUpdateTime(const Context<double>& context,
                              CompositeEventCollection<double>* event_info,
                              double* next_update_time) const final {
      const double inf = std::numeric_limits<double>::infinity();
      *next_update_time = message_is_waiting_ ? context.get_time() : inf;
      PublishEvent<double> event(
          TriggerType::kTimed,
          [this](const Context<double>& handler_context,
                 const PublishEvent<double>& publish_event) {
            this->MyPublishHandler(handler_context, publish_event);
          });
      event.AddToComposite(event_info);
    }

    void MyPublishHandler(const Context<double>& context,
                          const PublishEvent<double>& publish_event) const {
      ++publish_counter_;
    }

    bool message_is_waiting_{false};
    mutable int publish_counter_{0};
  };

  // Just an ordinary system that has a periodic event with period 0.25. It
  // should play nicely with simultaneous events from the
  // RightNowEventSystem above.
  class PeriodicEventSystem : public LeafSystem<double> {
   public:
    PeriodicEventSystem() {
      this->DeclarePeriodicPublishEvent(0.25, 0.,
                                        &PeriodicEventSystem::MakeItCount);
    }

    int publish_count() const { return publish_counter_; }
    void reset_count() { publish_counter_ = 0; }

   private:
    void MakeItCount(const Context<double>&) const {
      ++publish_counter_;
    }
    mutable int publish_counter_{0};
  };

  DiagramBuilder<double> builder;
  auto right_now_system = builder.AddSystem<RightNowEventSystem>();
  auto periodic_system = builder.AddSystem<PeriodicEventSystem>();
  Simulator<double> simulator(builder.Build());

  // Verify that CalcNextUpdateTime() returns "true time" rather than
  // current time when time is perturbed but we return "right now".
  // Note that the times are chosen to trigger only the "right now" event.
  const double true_time = .125;
  right_now_system->set_message_is_waiting(true);  // Activate event.
  simulator.get_mutable_context().SetTime(true_time);
  auto events = simulator.get_system().AllocateCompositeEventCollection();
  double next_update_time = simulator.get_system().CalcNextUpdateTime
      (simulator.get_context(), events.get());
  EXPECT_EQ(simulator.get_context().get_time(), true_time);
  EXPECT_EQ(next_update_time, true_time);  // Normal behavior.

  // Tweak the time. (Simulator::Initialize() does this more carefully.)
  const double perturbed_time = true_time - 1e-14;
  ASSERT_NE(perturbed_time, true_time);  // Watch for precision loss.
  simulator.get_mutable_context().PerturbTime(perturbed_time, true_time);
  next_update_time = simulator.get_system().CalcNextUpdateTime
      (simulator.get_context(), events.get());
  EXPECT_EQ(simulator.get_context().get_time(), perturbed_time);
  EXPECT_EQ(next_update_time, true_time);  // Return true time, not current.

  // With no "message" waiting, the "right now" event won't trigger. At
  // the offset time 0, we should get just the periodic event. This has always
  // worked properly.
  right_now_system->set_message_is_waiting(false);  // Deactivate event.
  simulator.get_mutable_context().SetTime(0.);
  simulator.Initialize();
  EXPECT_EQ(right_now_system->publish_count(), 0);
  EXPECT_EQ(periodic_system->publish_count(), 1);
  periodic_system->reset_count();

  // With a message waiting, the "right now" event should trigger,
  // but the periodic event won't issue until its offset time (0) is reached.
  // Although this wasn't reported in #13296, it did not work correctly before
  // the fix in PR #13438.
  right_now_system->set_message_is_waiting(true);
  simulator.get_mutable_context().SetTime(-1.);
  simulator.Initialize();
  EXPECT_EQ(right_now_system->publish_count(), 1);
  EXPECT_EQ(periodic_system->publish_count(), 0);
  right_now_system->reset_count();

  // With a message present and t=0 both should trigger. This is the case
  // that failed in #13296: the "right now" event would come back at the
  // perturbed time (slightly before zero), hiding the periodic event, and
  // its handler would not get called either since that isn't the current time.
  right_now_system->set_message_is_waiting(true);
  simulator.get_mutable_context().SetTime(0.);
  simulator.Initialize();
  EXPECT_EQ(right_now_system->publish_count(), 1);
  EXPECT_EQ(periodic_system->publish_count(), 1);
}

}  // namespace
}  // namespace systems
}  // namespace drake
