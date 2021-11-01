#include "drake/systems/framework/leaf_system.h"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/random.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace {

// A shell System to test the event dispatchers are sent at least one event upon
// a forced call.
class ForcedDispatchOverrideSystem : public LeafSystem<double> {
 public:
  bool got_publish_event() const { return got_publish_event_; }
  bool got_discrete_update_event() const {
    return got_discrete_update_event_;
  }
  bool got_unrestricted_update_event() const {
    return got_unrestricted_update_event_;
  }
  TriggerType get_publish_event_trigger_type() const {
    return publish_event_trigger_type_;
  }
  TriggerType get_discrete_update_event_trigger_type() const {
    return discrete_update_event_trigger_type_;
  }
  TriggerType get_unrestricted_update_event_trigger_type() const {
    return unrestricted_update_event_trigger_type_;
  }

 private:
  void DoPublish(
      const Context<double>&,
      const std::vector<const PublishEvent<double>*>& events) const final {
    got_publish_event_ = (events.size() == 1);
    if (got_publish_event_)
      publish_event_trigger_type_ = events.front()->get_trigger_type();
  }

  void DoCalcDiscreteVariableUpdates(
      const Context<double>&,
      const std::vector<const DiscreteUpdateEvent<double>*>& events,
      DiscreteValues<double>*) const final {
    got_discrete_update_event_ = (events.size() == 1);
    if (got_discrete_update_event_)
      discrete_update_event_trigger_type_ = events.front()->get_trigger_type();
  }

  void DoCalcUnrestrictedUpdate(
      const Context<double>&,
      const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
      State<double>*) const final {
    got_unrestricted_update_event_ = (events.size() == 1);
    if (got_unrestricted_update_event_) {
      unrestricted_update_event_trigger_type_ =
          events.front()->get_trigger_type();
    }
  }

  // Variables used to determine whether the handlers have been called with
  // forced update events. Note: updating method variables in event handlers
  // is an anti-pattern, but we do it here to simplify test code.
  mutable bool got_publish_event_{false};
  mutable bool got_discrete_update_event_{false};
  mutable bool got_unrestricted_update_event_{false};
  mutable TriggerType publish_event_trigger_type_{TriggerType::kUnknown};
  mutable TriggerType discrete_update_event_trigger_type_{
    TriggerType::kUnknown
  };
  mutable TriggerType unrestricted_update_event_trigger_type_{
    TriggerType::kUnknown
  };
};

GTEST_TEST(ForcedDispatchOverrideSystemTest, Dispatchers) {
  ForcedDispatchOverrideSystem system;
  auto context = system.CreateDefaultContext();
  auto discrete_values = system.AllocateDiscreteVariables();
  EXPECT_EQ(discrete_values->get_system_id(), context->get_system_id());
  auto state = context->CloneState();
  system.Publish(*context);
  system.CalcDiscreteVariableUpdates(*context, discrete_values.get());
  system.CalcUnrestrictedUpdate(*context, state.get());
  ASSERT_TRUE(system.got_publish_event());
  ASSERT_TRUE(system.got_discrete_update_event());
  ASSERT_TRUE(system.got_unrestricted_update_event());
  EXPECT_TRUE(system.get_publish_event_trigger_type() == TriggerType::kForced);
  EXPECT_TRUE(system.get_discrete_update_event_trigger_type() ==
      TriggerType::kForced);
  EXPECT_TRUE(system.get_unrestricted_update_event_trigger_type() ==
      TriggerType::kForced);
}

// A shell System to test the default implementations.
template <typename T>
class TestSystem : public LeafSystem<T> {
 public:
  TestSystem() {
    this->set_name("TestSystem");
    this->DeclareNumericParameter(BasicVector<T>{13.0, 7.0});
    this->DeclareAbstractParameter(Value<std::string>("parameter value"));

    this->DeclareDiscreteState(3);
    this->DeclareAbstractState(Value<int>(5));
    this->DeclareAbstractState(Value<std::string>("second abstract state"));
  }
  ~TestSystem() override {}

  using LeafSystem<T>::DeclareContinuousState;
  using LeafSystem<T>::DeclareDiscreteState;
  using LeafSystem<T>::DeclareStateOutputPort;
  using LeafSystem<T>::DeclareNumericParameter;
  using LeafSystem<T>::DeclareVectorInputPort;
  using LeafSystem<T>::DeclareAbstractInputPort;
  using LeafSystem<T>::DeclareVectorOutputPort;
  using LeafSystem<T>::DeclareAbstractOutputPort;
  using LeafSystem<T>::DeclarePerStepEvent;

  void AddPeriodicUpdate() {
    const double period = 10.0;
    const double offset = 5.0;
    this->DeclarePeriodicDiscreteUpdate(period, offset);
    std::optional<PeriodicEventData> periodic_attr =
        this->GetUniquePeriodicDiscreteUpdateAttribute();
    ASSERT_TRUE(periodic_attr);
    EXPECT_EQ(periodic_attr.value().period_sec(), period);
    EXPECT_EQ(periodic_attr.value().offset_sec(), offset);
  }

  void AddPeriodicUpdate(double period) {
    const double offset = 0.0;
    this->DeclarePeriodicDiscreteUpdate(period, offset);
    std::optional<PeriodicEventData> periodic_attr =
       this->GetUniquePeriodicDiscreteUpdateAttribute();
    ASSERT_TRUE(periodic_attr);
    EXPECT_EQ(periodic_attr.value().period_sec(), period);
    EXPECT_EQ(periodic_attr.value().offset_sec(), offset);
  }

  void AddPeriodicUpdate(double period, double offset) {
    this->DeclarePeriodicDiscreteUpdate(period, offset);
  }

  void AddPeriodicUnrestrictedUpdate(double period, double offset) {
    this->DeclarePeriodicUnrestrictedUpdate(period, offset);
  }

  void AddPublish(double period) { this->DeclarePeriodicPublish(period); }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {}

  void CalcOutput(const Context<T>& context, BasicVector<T>* output) const {}

  const BasicVector<T>& GetVanillaNumericParameters(
      const Context<T>& context) const {
    return this->GetNumericParameter(context, 0 /* index */);
  }

  BasicVector<T>& GetVanillaMutableNumericParameters(
      Context<T>* context) const {
    return this->GetMutableNumericParameter(context, 0 /* index */);
  }

  // First testing type: no event specified.
  std::unique_ptr<WitnessFunction<T>> MakeWitnessWithoutEvent() const {
    return this->MakeWitnessFunction(
        "dummy1", WitnessFunctionDirection::kCrossesZero,
        &TestSystem<double>::DummyWitnessFunction);
  }

  // Second testing type: event specified.
  std::unique_ptr<WitnessFunction<T>> MakeWitnessWithEvent() const {
    return this->MakeWitnessFunction(
        "dummy2", WitnessFunctionDirection::kNone,
        &TestSystem<double>::DummyWitnessFunction,
        PublishEvent<double>());
  }

  // Third testing type: publish callback specified.
  std::unique_ptr<WitnessFunction<T>> MakeWitnessWithPublish() const {
    return this->MakeWitnessFunction(
        "dummy3", WitnessFunctionDirection::kNone,
        &TestSystem<double>::DummyWitnessFunction,
        &TestSystem<double>::PublishCallback);
  }

  // Fourth testing type: discrete update callback specified.
  std::unique_ptr<WitnessFunction<T>> MakeWitnessWithDiscreteUpdate() const {
    return this->MakeWitnessFunction(
        "dummy4", WitnessFunctionDirection::kNone,
        &TestSystem<double>::DummyWitnessFunction,
        &TestSystem<double>::DiscreteUpdateCallback);
  }

  // Fifth testing type: unrestricted update callback specified.
  std::unique_ptr<WitnessFunction<T>>
      MakeWitnessWithUnrestrictedUpdate() const {
    return this->MakeWitnessFunction(
        "dummy5", WitnessFunctionDirection::kNone,
        &TestSystem<double>::DummyWitnessFunction,
        &TestSystem<double>::UnrestrictedUpdateCallback);
  }

  // Sixth testing type: lambda function with no event specified.
  std::unique_ptr<WitnessFunction<T>> DeclareLambdaWitnessWithoutEvent() const {
    return this->MakeWitnessFunction(
        "dummy6", WitnessFunctionDirection::kCrossesZero,
        [](const Context<double>&) -> double { return 7.0; });
  }

  // Seventh testing type: lambda function with event specified.
  std::unique_ptr<WitnessFunction<T>>
      DeclareLambdaWitnessWithUnrestrictedUpdate() const {
    return this->MakeWitnessFunction(
        "dummy7", WitnessFunctionDirection::kPositiveThenNonPositive,
        [](const Context<double>&) -> double { return 11.0; },
        UnrestrictedUpdateEvent<double>());
  }

  // Indicates whether various callbacks have been called.
  bool publish_callback_called() const { return publish_callback_called_; }
  bool discrete_update_callback_called() const {
      return discrete_update_callback_called_;
  }
  bool unrestricted_update_callback_called() const {
      return unrestricted_update_callback_called_;
  }

  // Verifies that the forced publish events collection has been allocated.
  bool forced_publish_events_collection_allocated() const {
    return this->forced_publish_events_exist();
  }

  // Verifies that the forced discrete update events collection has been
  // allocated.
  bool forced_discrete_update_events_collection_allocated() const {
    return this->forced_discrete_update_events_exist();
  }

  // Verifies that the forced unrestricted update events collection has been=
  // allocated.
  bool forced_unrestricted_update_events_collection_allocated() const {
    return this->forced_unrestricted_update_events_exist();
  }

  // Gets the forced publish events collection.
  const EventCollection<PublishEvent<double>>&
      get_forced_publish_events_collection() const {
    return this->get_forced_publish_events();
  }

  // Gets the forced discrete update events collection.
  const EventCollection<DiscreteUpdateEvent<double>>&
      get_forced_discrete_update_events_collection() const {
    return this->get_forced_discrete_update_events();
  }

  // Gets the forced unrestricted update events collection.
  const EventCollection<UnrestrictedUpdateEvent<double>>&
      get_forced_unrestricted_update_events_collection() const {
    return this->get_forced_unrestricted_update_events();
  }

  // Sets the forced publish events collection.
  void set_forced_publish_events_collection(
      std::unique_ptr<EventCollection<PublishEvent<double>>>
          publish_events) {
    this->set_forced_publish_events(std::move(publish_events));
  }

  // Sets the forced discrete_update events collection.
  void set_forced_discrete_update_events_collection(
      std::unique_ptr<EventCollection<DiscreteUpdateEvent<double>>>
          discrete_update_events) {
    this->set_forced_discrete_update_events(std::move(discrete_update_events));
  }

  // Sets the forced unrestricted_update events collection.
  void set_forced_unrestricted_update_events_collection(
      std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<double>>>
          unrestricted_update_events) {
    this->set_forced_unrestricted_update_events(
        std::move(unrestricted_update_events));
  }

  // Gets the mutable version of the forced publish events collection.
  EventCollection<PublishEvent<double>>&
      get_mutable_forced_publish_events_collection() {
    return this->get_mutable_forced_publish_events();
  }

 private:
  // This dummy witness function exists only to test that the
  // MakeWitnessFunction() interface works as promised.
  T DummyWitnessFunction(const Context<T>& context) const {
    static int call_counter = 0;
    return static_cast<T>(++call_counter);
  }

  // Publish callback function, which serves to test whether the appropriate
  // MakeWitnessFunction() interface works as promised.
  void PublishCallback(const Context<T>&, const PublishEvent<T>&) const {
    publish_callback_called_ = true;
  }

  // Discrete update callback function, which serves to test whether the
  // appropriate MakeWitnessFunction() interface works as promised.
  void DiscreteUpdateCallback(const Context<T>&,
      const DiscreteUpdateEvent<T>&, DiscreteValues<T>*) const {
    discrete_update_callback_called_ = true;
  }

  // Unrestricted update callback function, which serves to test whether the
  // appropriate MakeWitnessFunction() interface works as promised.
  void UnrestrictedUpdateCallback(const Context<T>&,
      const UnrestrictedUpdateEvent<T>&, State<T>*) const {
    unrestricted_update_callback_called_ = true;
  }

  // Indicators for which callbacks have been called, made mutable as
  // PublishCallback() cannot alter any state (the others are mutable for
  // consistency with PublishCallback() and to avoid machinations with state
  // that would otherwise be required).
  mutable bool publish_callback_called_{false};
  mutable bool discrete_update_callback_called_{false};
  mutable bool unrestricted_update_callback_called_{false};
};

class LeafSystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    event_info_ = system_.AllocateCompositeEventCollection();
    leaf_info_ = dynamic_cast<const LeafCompositeEventCollection<double>*>(
        event_info_.get());

    // Make sure caching tests will work properly even if caching is off
    // by default.
    context_.EnableCaching();
  }

  TestSystem<double> system_;
  std::unique_ptr<LeafContext<double>> context_ptr_ = system_.AllocateContext();
  LeafContext<double>& context_ = *context_ptr_;

  std::unique_ptr<CompositeEventCollection<double>> event_info_;
  const LeafCompositeEventCollection<double>* leaf_info_;
};

TEST_F(LeafSystemTest, ForcedEventCollectionsTest) {
  // Verify that all collections are allocated.
  EXPECT_TRUE(system_.forced_publish_events_collection_allocated());
  EXPECT_TRUE(system_.forced_discrete_update_events_collection_allocated());
  EXPECT_TRUE(system_.forced_unrestricted_update_events_collection_allocated());

  // Verify that we can set the publish collection.
  auto forced_publishes =
      std::make_unique<LeafEventCollection<PublishEvent<double>>>();
  auto* forced_publishes_pointer = forced_publishes.get();
  system_.set_forced_publish_events_collection(std::move(forced_publishes));
  EXPECT_EQ(&system_.get_forced_publish_events_collection(),
      forced_publishes_pointer);
  EXPECT_EQ(&system_.get_mutable_forced_publish_events_collection(),
      forced_publishes_pointer);

  // Verify that we can set the discrete update collection.
  auto forced_discrete_updates =
      std::make_unique<LeafEventCollection<DiscreteUpdateEvent<double>>>();
  auto* forced_discrete_updates_pointer = forced_discrete_updates.get();
  system_.set_forced_discrete_update_events_collection(
      std::move(forced_discrete_updates));
  EXPECT_EQ(&system_.get_forced_discrete_update_events_collection(),
      forced_discrete_updates_pointer);

  // Verify that we can set the forced unrestricted update collection.
  auto forced_unrestricted_updates =
      std::make_unique<LeafEventCollection<UnrestrictedUpdateEvent<double>>>();
  auto* forced_unrestricted_updates_pointer = forced_unrestricted_updates.get();
  system_.set_forced_unrestricted_update_events_collection(
      std::move(forced_unrestricted_updates));
  EXPECT_EQ(&system_.get_forced_unrestricted_update_events_collection(),
      forced_unrestricted_updates_pointer);
}

TEST_F(LeafSystemTest, DefaultPortNameTest) {
  EXPECT_EQ(
      system_.DeclareVectorInputPort(kUseDefaultName, BasicVector<double>(2))
          .get_name(),
      "u0");
  EXPECT_EQ(
      system_.DeclareAbstractInputPort(kUseDefaultName, Value<int>(1))
          .get_name(),
      "u1");

  EXPECT_EQ(
      system_.DeclareVectorOutputPort(kUseDefaultName,
                                      &TestSystem<double>::CalcOutput)
          .get_name(),
      "y0");
  EXPECT_EQ(
      system_
          .DeclareAbstractOutputPort(kUseDefaultName, BasicVector<double>(2),
                                     &TestSystem<double>::CalcOutput)
          .get_name(),
      "y1");
}

TEST_F(LeafSystemTest, DeclareVectorPortsBySizeTest) {
  const InputPort<double>& input =
      system_.DeclareVectorInputPort("my_input", 2);
  EXPECT_EQ(input.get_name(), "my_input");
  EXPECT_EQ(input.size(), 2);
  EXPECT_FALSE(input.is_random());
  EXPECT_TRUE(system_
                  .DeclareVectorInputPort(kUseDefaultName, 3,
                                          RandomDistribution::kUniform)
                  .is_random());

  const OutputPort<double>& output = system_.DeclareVectorOutputPort(
      "my_output", 3, &TestSystem<double>::CalcOutput);
  EXPECT_EQ(output.get_name(), "my_output");
  EXPECT_EQ(output.size(), 3);

  const OutputPort<double>& output2 = system_.DeclareVectorOutputPort(
      "my_output2", 2, [](const Context<double>&, BasicVector<double>*) {});
  EXPECT_EQ(output2.get_name(), "my_output2");
  EXPECT_EQ(output2.size(), 2);
}

// Tests that witness functions can be declared. Tests that witness functions
// stop Simulator at desired points (i.e., the raison d'être of a witness
// function) are done in diagram_test.cc and
// drake/systems/analysis/test/simulator_test.cc.
TEST_F(LeafSystemTest, WitnessDeclarations) {
  auto witness1 = system_.MakeWitnessWithoutEvent();
  ASSERT_TRUE(witness1);
  EXPECT_EQ(witness1->description(), "dummy1");
  EXPECT_EQ(witness1->direction_type(),
      WitnessFunctionDirection::kCrossesZero);
  EXPECT_FALSE(witness1->get_event());
  EXPECT_EQ(witness1->CalcWitnessValue(context_), 1.0);

  auto witness2 = system_.MakeWitnessWithEvent();
  ASSERT_TRUE(witness2);
  EXPECT_EQ(witness2->description(), "dummy2");
  EXPECT_EQ(witness2->direction_type(), WitnessFunctionDirection::kNone);
  EXPECT_TRUE(witness2->get_event());
  EXPECT_EQ(witness2->get_event()->get_trigger_type(), TriggerType::kWitness);
  EXPECT_EQ(witness2->CalcWitnessValue(context_), 2.0);

  auto witness3 = system_.MakeWitnessWithPublish();
  ASSERT_TRUE(witness3);
  EXPECT_EQ(witness3->description(), "dummy3");
  EXPECT_EQ(witness3->direction_type(),
      WitnessFunctionDirection::kNone);
  EXPECT_TRUE(witness3->get_event());
  EXPECT_EQ(witness3->get_event()->get_trigger_type(), TriggerType::kWitness);
  EXPECT_EQ(witness3->CalcWitnessValue(context_), 3.0);
  auto pe = dynamic_cast<const PublishEvent<double>*>(witness3->get_event());
  ASSERT_TRUE(pe);
  pe->handle(system_, context_);
  EXPECT_TRUE(system_.publish_callback_called());

  auto witness4 = system_.MakeWitnessWithDiscreteUpdate();
  ASSERT_TRUE(witness4);
  EXPECT_EQ(witness4->description(), "dummy4");
  EXPECT_EQ(witness4->direction_type(),
      WitnessFunctionDirection::kNone);
  EXPECT_TRUE(witness4->get_event());
  EXPECT_EQ(witness4->get_event()->get_trigger_type(), TriggerType::kWitness);
  EXPECT_EQ(witness4->CalcWitnessValue(context_), 4.0);
  auto de = dynamic_cast<const DiscreteUpdateEvent<double>*>(
      witness4->get_event());
  ASSERT_TRUE(de);
  de->handle(system_, context_, nullptr);
  EXPECT_TRUE(system_.discrete_update_callback_called());

  auto witness5 = system_.MakeWitnessWithUnrestrictedUpdate();
  ASSERT_TRUE(witness5);
  EXPECT_EQ(witness5->description(), "dummy5");
  EXPECT_EQ(witness5->direction_type(),
      WitnessFunctionDirection::kNone);
  EXPECT_TRUE(witness5->get_event());
  EXPECT_EQ(witness5->get_event()->get_trigger_type(), TriggerType::kWitness);
  EXPECT_EQ(witness5->CalcWitnessValue(context_), 5.0);
  auto ue = dynamic_cast<const UnrestrictedUpdateEvent<double>*>(
      witness5->get_event());
  ASSERT_TRUE(ue);
  ue->handle(system_, context_, nullptr);
  EXPECT_TRUE(system_.unrestricted_update_callback_called());

  auto witness6 = system_.DeclareLambdaWitnessWithoutEvent();
  ASSERT_TRUE(witness6);
  EXPECT_EQ(witness6->description(), "dummy6");
  EXPECT_EQ(witness6->direction_type(),
            WitnessFunctionDirection::kCrossesZero);
  EXPECT_EQ(witness6->CalcWitnessValue(context_), 7.0);

  auto witness7 = system_.DeclareLambdaWitnessWithUnrestrictedUpdate();
  ASSERT_TRUE(witness7);
  EXPECT_EQ(witness7->description(), "dummy7");
  EXPECT_EQ(witness7->direction_type(),
            WitnessFunctionDirection::kPositiveThenNonPositive);
  EXPECT_TRUE(witness7->get_event());
  EXPECT_EQ(witness7->get_event()->get_trigger_type(), TriggerType::kWitness);
  EXPECT_EQ(witness7->CalcWitnessValue(context_), 11.0);
  ue = dynamic_cast<const UnrestrictedUpdateEvent<double>*>(
      witness7->get_event());
  ASSERT_TRUE(ue);
}

// Tests that if no update events are configured, none are reported.
TEST_F(LeafSystemTest, NoUpdateEvents) {
  context_.SetTime(25.0);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(std::numeric_limits<double>::infinity(), time);
  EXPECT_TRUE(!leaf_info_->HasEvents());
}

// Tests that multiple periodic updates with the same periodic attribute are
// identified as unique.
TEST_F(LeafSystemTest, MultipleUniquePeriods) {
  system_.AddPeriodicUpdate();
  system_.AddPeriodicUpdate();

  // Verify the size of the periodic events mapping.
  auto mapping = system_.GetPeriodicEvents();
  ASSERT_EQ(mapping.size(), 1);
  EXPECT_EQ(mapping.begin()->second.size(), 2);
  EXPECT_TRUE(system_.GetUniquePeriodicDiscreteUpdateAttribute());
}

// Tests that periodic updates with different periodic attributes are
// identified as non-unique.
TEST_F(LeafSystemTest, MultipleNonUniquePeriods) {
  system_.AddPeriodicUpdate(1.0, 2.0);
  system_.AddPeriodicUpdate(2.0, 3.0);

  // Verify the size of the periodic events mapping.
  auto mapping = system_.GetPeriodicEvents();
  ASSERT_EQ(mapping.size(), 2);
  EXPECT_FALSE(system_.GetUniquePeriodicDiscreteUpdateAttribute());
  EXPECT_FALSE(system_.IsDifferenceEquationSystem());
}

// Tests that if the current time is smaller than the offset, the next
// update time is the offset.
TEST_F(LeafSystemTest, OffsetHasNotArrivedYet) {
  context_.SetTime(2.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(5.0, time);
  const auto& events = leaf_info_->get_discrete_update_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
}

// Tests that if the current time is smaller than the offset, the next
// update time is the offset, DiscreteUpdate and UnrestrictedUpdate happen
// at the same time.
TEST_F(LeafSystemTest, EventsAtTheSameTime) {
  context_.SetTime(2.0);
  // Both actions happen at t = 5.
  system_.AddPeriodicUpdate();
  system_.AddPeriodicUnrestrictedUpdate(3, 5);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(5.0, time);
  {
    const auto& events = leaf_info_->get_discrete_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
  }
  {
    const auto& events =
        leaf_info_->get_unrestricted_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
  }
}

// Tests that if the current time is exactly the offset, the next
// update time is in the future.
TEST_F(LeafSystemTest, ExactlyAtOffset) {
  context_.SetTime(5.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(15.0, time);
  const auto& events = leaf_info_->get_discrete_update_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
}

// Tests that if the current time is larger than the offset, the next
// update time is determined by the period.
TEST_F(LeafSystemTest, OffsetIsInThePast) {
  context_.SetTime(23.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(25.0, time);
  const auto& events = leaf_info_->get_discrete_update_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
}

// Tests that if the current time is exactly an update time, the next update
// time is in the future.
TEST_F(LeafSystemTest, ExactlyOnUpdateTime) {
  context_.SetTime(25.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(35.0, time);
  const auto& events = leaf_info_->get_discrete_update_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
}

// Tests periodic events' scheduling when its offset is zero.
TEST_F(LeafSystemTest, PeriodicUpdateZeroOffset) {
  system_.AddPeriodicUpdate(2.0);

  context_.SetTime(0.0);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(2.0, time);

  context_.SetTime(1.0);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(2.0, time);

  context_.SetTime(2.1);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(4.0, time);
}

// Tests that if a LeafSystem has both a discrete update and a periodic Publish,
// the update actions are computed appropriately.
TEST_F(LeafSystemTest, UpdateAndPublish) {
  system_.AddPeriodicUpdate(15.0);
  system_.AddPublish(12.0);

  // The publish event fires at 12sec.
  context_.SetTime(9.0);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(12.0, time);
  {
    const auto& events = leaf_info_->get_publish_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
  }

  // The update event fires at 15sec.
  context_.SetTime(14.0);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(15.0, time);
  {
    const auto& events = leaf_info_->get_discrete_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
  }

  // Both events fire at 60sec.
  context_.SetTime(59.0);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(60.0, time);
  {
    const auto& events = leaf_info_->get_discrete_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
  }
  {
    const auto& events = leaf_info_->get_publish_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
  }
}

// Tests that if the integrator has stopped on the k-th sample, and the current
// time for that sample is slightly less than k * period due to floating point
// rounding, the next sample time is (k + 1) * period.
TEST_F(LeafSystemTest, FloatingPointRoundingZeroPointZeroOneFive) {
  context_.SetTime(0.015 * 11);  // Slightly less than 0.165.
  system_.AddPeriodicUpdate(0.015);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  // 0.015 * 12 = 0.18.
  EXPECT_NEAR(0.18, time, 1e-8);
}

// Tests that if the integrator has stopped on the k-th sample, and the current
// time for that sample is slightly less than k * period due to floating point
// rounding, the next sample time is (k + 1) * period.
TEST_F(LeafSystemTest, FloatingPointRoundingZeroPointZeroZeroTwoFive) {
  context_.SetTime(0.0025 * 977);  // Slightly less than 2.4425
  system_.AddPeriodicUpdate(0.0025);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_NEAR(2.445, time, 1e-8);
}

// Tests that discrete and abstract state dependency wiring is set up
// correctly.
TEST_F(LeafSystemTest, DiscreteAndAbstractStateTrackers) {
  EXPECT_EQ(system_.num_discrete_state_groups(), 1);
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const DependencyTracker& xd_tracker =
      context->get_tracker(system_.xd_ticket());
  for (DiscreteStateIndex i(0); i < system_.num_discrete_state_groups(); ++i) {
    const DependencyTracker& tracker =
        context->get_tracker(system_.discrete_state_ticket(i));
    EXPECT_TRUE(xd_tracker.HasPrerequisite(tracker));
    EXPECT_TRUE(tracker.HasSubscriber(xd_tracker));
  }

  EXPECT_EQ(system_.num_abstract_states(), 2);
  const DependencyTracker& xa_tracker =
      context->get_tracker(system_.xa_ticket());
  for (AbstractStateIndex i(0); i < system_.num_abstract_states(); ++i) {
    const DependencyTracker& tracker =
        context->get_tracker(system_.abstract_state_ticket(i));
    EXPECT_TRUE(xa_tracker.HasPrerequisite(tracker));
    EXPECT_TRUE(tracker.HasSubscriber(xa_tracker));
  }
}

// Tests that the leaf system reserved the declared Parameters with default
// values, that they are modifiable, and that dependency wiring is set up
// correctly for them.
TEST_F(LeafSystemTest, NumericParameters) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const BasicVector<double>& vec =
      system_.GetVanillaNumericParameters(*context);
  EXPECT_EQ(13.0, vec[0]);
  EXPECT_EQ(7.0, vec[1]);
  BasicVector<double>& mutable_vec =
      system_.GetVanillaMutableNumericParameters(context.get());
  mutable_vec[1] = 42.0;
  EXPECT_EQ(42.0, vec[1]);

  EXPECT_EQ(system_.num_numeric_parameter_groups(), 1);
  const DependencyTracker& pn_tracker =
      context->get_tracker(system_.pn_ticket());
  for (NumericParameterIndex i(0);
       i < system_.num_numeric_parameter_groups(); ++i) {
    const DependencyTracker& tracker =
        context->get_tracker(system_.numeric_parameter_ticket(i));
    EXPECT_TRUE(pn_tracker.HasPrerequisite(tracker));
    EXPECT_TRUE(tracker.HasSubscriber(pn_tracker));
  }
}

// Tests that the leaf system reserved the declared abstract Parameters with
// default values, that they are modifiable, and that dependency wiring is set
// up correctly for them.
TEST_F(LeafSystemTest, AbstractParameters) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const std::string& param = context->get_abstract_parameter(0 /*index*/)
                                 .get_value<std::string>();
  EXPECT_EQ(param, "parameter value");
  std::string& mutable_param =
      context->get_mutable_abstract_parameter(0 /*index*/)
          .get_mutable_value<std::string>();
  mutable_param = "modified parameter value";
  EXPECT_EQ("modified parameter value", param);

  EXPECT_EQ(system_.num_abstract_parameters(), 1);
  const DependencyTracker& pa_tracker =
      context->get_tracker(system_.pa_ticket());
  for (AbstractParameterIndex i(0); i < system_.num_abstract_parameters();
       ++i) {
    const DependencyTracker& tracker =
        context->get_tracker(system_.abstract_parameter_ticket(i));
    EXPECT_TRUE(pa_tracker.HasPrerequisite(tracker));
    EXPECT_TRUE(tracker.HasSubscriber(pa_tracker));
  }
}

// Tests that the leaf system reserved the declared misc continuous state.
TEST_F(LeafSystemTest, DeclareVanillaMiscContinuousState) {
  system_.DeclareContinuousState(2);

  // Tests num_continuous_states without a context.
  EXPECT_EQ(2, system_.num_continuous_states());

  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>& xc = context->get_continuous_state();
  EXPECT_EQ(2, xc.size());
  EXPECT_EQ(0, xc.get_generalized_position().size());
  EXPECT_EQ(0, xc.get_generalized_velocity().size());
  EXPECT_EQ(2, xc.get_misc_continuous_state().size());
}

// Tests that the leaf system reserved the declared misc continuous state of
// interesting custom type.
TEST_F(LeafSystemTest, DeclareTypedMiscContinuousState) {
  system_.DeclareContinuousState(MyVector2d());

  // Tests num_continuous_states without a context.
  EXPECT_EQ(2, system_.num_continuous_states());

  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>& xc = context->get_continuous_state();
  // Check that type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<MyVector2d*>(
                         &context->get_mutable_continuous_state_vector()));
  EXPECT_EQ(2, xc.size());
  EXPECT_EQ(0, xc.get_generalized_position().size());
  EXPECT_EQ(0, xc.get_generalized_velocity().size());
  EXPECT_EQ(2, xc.get_misc_continuous_state().size());
}

// Tests that the leaf system reserved the declared continuous state with
// second-order structure.
TEST_F(LeafSystemTest, DeclareVanillaContinuousState) {
  system_.DeclareContinuousState(4, 3, 2);

  // Tests num_continuous_states without a context.
  EXPECT_EQ(4 + 3 + 2, system_.num_continuous_states());

  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>& xc = context->get_continuous_state();
  EXPECT_EQ(4 + 3 + 2, xc.size());
  EXPECT_EQ(4, xc.get_generalized_position().size());
  EXPECT_EQ(3, xc.get_generalized_velocity().size());
  EXPECT_EQ(2, xc.get_misc_continuous_state().size());
}

// Tests that the leaf system reserved the declared continuous state with
// second-order structure of interesting custom type.
TEST_F(LeafSystemTest, DeclareTypedContinuousState) {
  using MyVector9d = MyVector<double, 4 + 3 + 2>;
  auto state_index = system_.DeclareContinuousState(MyVector9d(), 4, 3, 2);

  // Tests num_continuous_states without a context.
  EXPECT_EQ(4 + 3 + 2, system_.num_continuous_states());

  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>& xc = context->get_continuous_state();
  // Check that type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<MyVector9d*>(
                         &context->get_mutable_continuous_state_vector()));
  // Check that dimensions were preserved.
  EXPECT_EQ(4 + 3 + 2, xc.size());
  EXPECT_EQ(4, xc.get_generalized_position().size());
  EXPECT_EQ(3, xc.get_generalized_velocity().size());
  EXPECT_EQ(2, xc.get_misc_continuous_state().size());

  // Check that the state retains its type when placed on an output port.
  const auto& state_output_port = system_.DeclareStateOutputPort(
      "state", state_index);
  context = system_.CreateDefaultContext();
  const Eigen::VectorXd ones = VectorXd::Ones(9);
  context->SetContinuousState(ones);
  EXPECT_EQ(state_output_port.Eval<MyVector9d>(*context).get_value(), ones);
}

TEST_F(LeafSystemTest, ContinuousStateBelongsWithSystem) {
  // Successfully calc using a storage that was created by the system.
  std::unique_ptr<ContinuousState<double>> derivatives =
      system_.AllocateTimeDerivatives();
  EXPECT_EQ(derivatives->get_system_id(), context_.get_system_id());
  DRAKE_EXPECT_NO_THROW(
      system_.CalcTimeDerivatives(context_, derivatives.get()));

  // Successfully calc using storage that was indirectly created by the system.
  auto temp_context = system_.AllocateContext();
  ContinuousState<double>& temp_xc =
      temp_context->get_mutable_continuous_state();
  DRAKE_EXPECT_NO_THROW(
      system_.CalcTimeDerivatives(context_, &temp_xc));

  // Cannot ask other_system to calc into storage that was created by the
  // original system.
  TestSystem<double> other_system;
  auto other_context = other_system.AllocateContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      other_system.CalcTimeDerivatives(*other_context, derivatives.get()),
      std::logic_error,
      ".*::ContinuousState<double> was not created for.*::TestSystem.*");
}

TEST_F(LeafSystemTest, DeclarePerStepEvents) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();

  system_.DeclarePerStepEvent(PublishEvent<double>());
  system_.DeclarePerStepEvent(DiscreteUpdateEvent<double>());
  system_.DeclarePerStepEvent(UnrestrictedUpdateEvent<double>());

  system_.GetPerStepEvents(*context, event_info_.get());

  {
    const auto& events = leaf_info_->get_publish_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPerStep);
  }
  {
    const auto& events = leaf_info_->get_discrete_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPerStep);
  }
  {
    const auto& events =
        leaf_info_->get_unrestricted_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPerStep);
  }
}

// A system that exercises the model_value-based input and output ports,
// as well as model-declared params.
class DeclaredModelPortsSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeclaredModelPortsSystem);

  DeclaredModelPortsSystem() {
    // Use these to validate the expected return type from each method.
    InputPort<double>* in_port{nullptr};
    LeafOutputPort<double>* out_port{nullptr};

    in_port = &DeclareInputPort("input", kVectorValued, 1);
    unused(in_port);
    in_port = &DeclareVectorInputPort("vector_input", MyVector2d());
    unused(in_port);
    in_port = &DeclareAbstractInputPort("abstract_input", Value<int>(22));
    unused(in_port);
    in_port = &DeclareVectorInputPort("uniform", MyVector2d(),
                                      RandomDistribution::kUniform);
    unused(in_port);
    in_port = &DeclareVectorInputPort("gaussian", MyVector2d(),
                                      RandomDistribution::kGaussian);
    unused(in_port);

    // Output port 0 uses a BasicVector base class model.
    out_port =
        &DeclareVectorOutputPort("basic_vector", BasicVector<double>(3),
                                 &DeclaredModelPortsSystem::CalcBasicVector3);
    unused(out_port);
    // Output port 1 uses a class derived from BasicVector.
    out_port = &DeclareVectorOutputPort(
        "my_vector", MyVector4d(), &DeclaredModelPortsSystem::CalcMyVector4d);
    unused(out_port);

    // Output port 2 uses a concrete string model.
    out_port = &DeclareAbstractOutputPort(
        "string", std::string("45"), &DeclaredModelPortsSystem::CalcString);
    unused(out_port);

    // Output port 3 uses the "Advanced" methods that take a model
    // and a general calc function rather than a calc method.
    out_port = &DeclareVectorOutputPort(
        "advanced", BasicVector<double>(2),
        [](const Context<double>&, BasicVector<double>* out) {
          ASSERT_NE(out, nullptr);
          EXPECT_EQ(out->size(), 2);
          out->SetAtIndex(0, 10.);
          out->SetAtIndex(1, 20.);
        });
    unused(out_port);

    DeclareNumericParameter(*MyVector2d::Make(1.1, 2.2));
  }

  const BasicVector<double>& expected_basic() const { return expected_basic_; }
  const MyVector4d& expected_myvector() const { return *expected_myvector_; }

 private:
  void CalcBasicVector3(const Context<double>&,
                        BasicVector<double>* out) const {
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->size(), 3);
    out->get_mutable_value() = expected_basic().get_value();
  }

  void CalcMyVector4d(const Context<double>&, MyVector4d* out) const {
    ASSERT_NE(out, nullptr);
    out->get_mutable_value() = expected_myvector().get_value();
  }

  void CalcAbstractString(const Context<double>&, AbstractValue* out) const {
    ASSERT_NE(out, nullptr);
    out->get_mutable_value<std::string>() = "abstract string";
  }

  void CalcString(const Context<double>&, std::string* out) const {
    ASSERT_NE(out, nullptr);
    *out = "concrete string";
  }

  const BasicVector<double> expected_basic_{1., .5, .25};
  std::unique_ptr<MyVector4d> expected_myvector_{
      MyVector4d::Make(4., 3., 2., 1.)};
};

// Tests that Declare{Vector,Abstract}{Input,Output}Port end up with the
// correct topology, and that the all_input_ports dependency tracker is
// properly subscribed to the input ports in an allocated context.
GTEST_TEST(ModelLeafSystemTest, ModelPortsTopology) {
  DeclaredModelPortsSystem dut;

  ASSERT_EQ(dut.num_input_ports(), 5);
  ASSERT_EQ(dut.num_output_ports(), 4);

  // Check that SystemBase port APIs work properly.
  const InputPortBase& in3_base = dut.get_input_port_base(InputPortIndex(3));
  EXPECT_EQ(in3_base.get_index(), 3);
  const DependencyTicket in3_ticket = dut.input_port_ticket(InputPortIndex(3));
  const DependencyTicket in4_ticket = dut.input_port_ticket(InputPortIndex(4));
  EXPECT_TRUE(in3_ticket.is_valid() && in4_ticket.is_valid());
  EXPECT_NE(in3_ticket, in4_ticket);  // We don't know the actual values.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.get_input_port_base(InputPortIndex(10)), std::out_of_range,
      "System.*get_input_port_base().*no input port.*10.*only 5.*");

  // Check DependencyTracker setup for input ports.
  auto context = dut.AllocateContext();
  const DependencyTracker& all_inputs_tracker =
      context->get_tracker(dut.all_input_ports_ticket());
  for (InputPortIndex i(0); i < dut.num_input_ports(); ++i) {
    const DependencyTracker& tracker =
        context->get_tracker(dut.input_port_ticket(i));
    EXPECT_TRUE(all_inputs_tracker.HasPrerequisite(tracker));
    EXPECT_TRUE(tracker.HasSubscriber(all_inputs_tracker));
  }

  const OutputPortBase& out2_base =
      dut.get_output_port_base(OutputPortIndex(2));
  EXPECT_EQ(out2_base.get_index(), 2);
  const DependencyTicket out2_ticket =
      dut.output_port_ticket(OutputPortIndex(2));
  const DependencyTicket out3_ticket =
      dut.output_port_ticket(OutputPortIndex(3));
  EXPECT_TRUE(out2_ticket.is_valid() && out3_ticket.is_valid());
  EXPECT_NE(out2_ticket, out3_ticket);  // We don't know the actual values.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.get_output_port_base(OutputPortIndex(7)), std::out_of_range,
      "System.*get_output_port_base().*no output port.*7.*only 4.*");

  const InputPort<double>& in0 = dut.get_input_port(0);
  const InputPort<double>& in1 = dut.get_input_port(1);
  const InputPort<double>& in2 = dut.get_input_port(2);
  const InputPort<double>& in3 = dut.get_input_port(3);
  const InputPort<double>& in4 = dut.get_input_port(4);

  const OutputPort<double>& out0 = dut.get_output_port(0);
  const OutputPort<double>& out1 = dut.get_output_port(1);
  const OutputPort<double>& out2 = dut.get_output_port(2);
  const OutputPort<double>& out3 = dut.get_output_port(3);

  EXPECT_EQ(in0.get_data_type(), kVectorValued);
  EXPECT_EQ(in1.get_data_type(), kVectorValued);
  EXPECT_EQ(in2.get_data_type(), kAbstractValued);
  EXPECT_EQ(in3.get_data_type(), kVectorValued);
  EXPECT_EQ(in4.get_data_type(), kVectorValued);

  EXPECT_EQ(out0.get_data_type(), kVectorValued);
  EXPECT_EQ(out1.get_data_type(), kVectorValued);
  EXPECT_EQ(out2.get_data_type(), kAbstractValued);
  EXPECT_EQ(out3.get_data_type(), kVectorValued);

  EXPECT_EQ(in0.size(), 1);
  EXPECT_EQ(in1.size(), 2);
  EXPECT_EQ(in3.size(), 2);
  EXPECT_EQ(in4.size(), 2);

  EXPECT_EQ(out0.size(), 3);
  EXPECT_EQ(out1.size(), 4);
  EXPECT_EQ(out3.size(), 2);

  EXPECT_FALSE(in0.is_random());
  EXPECT_FALSE(in1.is_random());
  EXPECT_FALSE(in2.is_random());
  EXPECT_TRUE(in3.is_random());
  EXPECT_TRUE(in4.is_random());

  EXPECT_FALSE(in0.get_random_type());
  EXPECT_FALSE(in1.get_random_type());
  EXPECT_FALSE(in2.get_random_type());
  EXPECT_EQ(in3.get_random_type(), RandomDistribution::kUniform);
  EXPECT_EQ(in4.get_random_type(), RandomDistribution::kGaussian);
}

// A system that incorrectly declares an input port.
class MissingModelAbstractInputSystem : public LeafSystem<double> {
 public:
  MissingModelAbstractInputSystem() {
    this->DeclareInputPort("no_model_input", kAbstractValued, 0);
  }
};

GTEST_TEST(ModelLeafSystemTest, MissingModelAbstractInput) {
  MissingModelAbstractInputSystem dut;
  dut.set_name("dut");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.AllocateInputAbstract(dut.get_input_port(0)),
      "System::AllocateInputAbstract\\(\\): a System with abstract input "
      "ports must pass a model_value to DeclareAbstractInputPort; the "
      "port\\[0\\] named 'no_model_input' did not do so \\(System ::dut\\)");
}

// Check that model inputs place validity checks on FixInput calls.  (This is
// more of an acceptance test than a unit test.  The relevant code is sprinkled
// across a few files.)  Note that even the debug builds might not detect the
// kind of use-after-free errors that this test tries to expose; the dynamic
// analysis build configurations such as valgrind or msan might be needed in
// order to detect the errors.
GTEST_TEST(ModelLeafSystemTest, ModelInputGovernsFixedInput) {
  // The Context checks must be able to outlive the System that created them.
  auto dut = std::make_unique<DeclaredModelPortsSystem>();
  dut->set_name("dut");
  auto context = dut->CreateDefaultContext();
  dut.reset();

  // The first port should only accept a 1d vector.
  context->FixInputPort(0, Value<BasicVector<double>>(
                               VectorXd::Constant(1, 0.0)));
  DRAKE_EXPECT_THROWS_MESSAGE(
      context->FixInputPort(0, Value<BasicVector<double>>(
                                   VectorXd::Constant(2, 0.0))),
      "System::FixInputPortTypeCheck\\(\\): expected value of type "
      "drake::systems::BasicVector<double> with size=1 "
      "for input port 'input' \\(index 0\\) but the actual type was "
      "drake::systems::BasicVector<double> with size=2. "
      "\\(System ::dut\\)");
  DRAKE_EXPECT_THROWS_MESSAGE(
      context->FixInputPort(0, Value<std::string>()),
      "System::FixInputPortTypeCheck\\(\\): expected value of type "
      "drake::Value<drake::systems::BasicVector<double>> "
      "for input port 'input' \\(index 0\\) but the actual type was "
      "drake::Value<std::string>. "
      "\\(System ::dut\\)");

  // The second port should only accept ints.
  context->FixInputPort(2, Value<int>(11));
  DRAKE_EXPECT_THROWS_MESSAGE(
      context->FixInputPort(2, Value<std::string>()),
      "System::FixInputPortTypeCheck\\(\\): expected value of type "
      "int "
      "for input port 'abstract_input' \\(index 2\\) but the actual type was "
      "std::string. "
      "\\(System ::dut\\)");
}

// Check that names can be assigned to the ports through all of the various
// APIs.
GTEST_TEST(ModelLeafSystemTest, ModelPortNames) {
  DeclaredModelPortsSystem dut;

  EXPECT_EQ(dut.get_input_port(0).get_name(), "input");
  EXPECT_EQ(dut.get_input_port(1).get_name(), "vector_input");
  EXPECT_EQ(dut.get_input_port(2).get_name(), "abstract_input");

  EXPECT_EQ(dut.get_output_port(0).get_name(), "basic_vector");
  EXPECT_EQ(dut.get_output_port(1).get_name(), "my_vector");
  EXPECT_EQ(dut.get_output_port(2).get_name(), "string");
  EXPECT_EQ(dut.get_output_port(3).get_name(), "advanced");
}

// Tests that the model values specified in Declare{...} are actually used by
// the corresponding Allocate{...} methods to yield correct types and values.
GTEST_TEST(ModelLeafSystemTest, ModelPortsInput) {
  DeclaredModelPortsSystem dut;

  // Check that BasicVector<double>(1) came out.
  auto input0 = dut.AllocateInputVector(dut.get_input_port(0));
  ASSERT_NE(input0, nullptr);
  EXPECT_EQ(input0->size(), 1);

  // Check that MyVector2d came out.
  auto input1 = dut.AllocateInputVector(dut.get_input_port(1));
  ASSERT_NE(input1, nullptr);
  MyVector2d* downcast_input1 = dynamic_cast<MyVector2d*>(input1.get());
  ASSERT_NE(downcast_input1, nullptr);

  // Check that Value<int>(22) came out.
  auto input2 = dut.AllocateInputAbstract(dut.get_input_port(2));
  ASSERT_NE(input2, nullptr);
  int downcast_input2{};
  DRAKE_EXPECT_NO_THROW(downcast_input2 = input2->get_value<int>());
  EXPECT_EQ(downcast_input2, 22);
}

// Tests that Declare{Vector,Abstract}OutputPort flow through to allocating the
// correct values.
GTEST_TEST(ModelLeafSystemTest, ModelPortsAllocOutput) {
  DeclaredModelPortsSystem dut;
  auto system_output = dut.AllocateOutput();

  // Check that BasicVector<double>(3) came out.
  auto output0 = system_output->get_vector_data(0);
  ASSERT_NE(output0, nullptr);
  EXPECT_EQ(output0->size(), 3);

  // Check that MyVector4d came out.
  auto output1 = system_output->GetMutableVectorData(1);
  ASSERT_NE(output1, nullptr);
  MyVector4d* downcast_output1 = dynamic_cast<MyVector4d*>(output1);
  ASSERT_NE(downcast_output1, nullptr);

  // Check that Value<string>("45") came out (even though we only specified
  // a concrete string.
  auto output2 = system_output->get_data(2);
  ASSERT_NE(output2, nullptr);
  std::string downcast_output2{};
  DRAKE_EXPECT_NO_THROW(downcast_output2 = output2->get_value<std::string>());
  EXPECT_EQ(downcast_output2, "45");

  // Check that BasicVector<double>(2) came out.
  auto output3 = system_output->get_vector_data(3);
  ASSERT_NE(output3, nullptr);
  EXPECT_EQ(output3->size(), 2);
}

// Tests that calculator functions were generated correctly for the
// model-based output ports.
GTEST_TEST(ModelLeafSystemTest, ModelPortsCalcOutput) {
  DeclaredModelPortsSystem dut;
  auto context = dut.CreateDefaultContext();

  // Make sure caching is on locally, even if it is off by default.
  context->EnableCaching();

  // Calculate values for each output port and save copies of those values.
  std::vector<std::unique_ptr<AbstractValue>> values;
  for (OutputPortIndex i(0); i < 4; ++i) {
    const OutputPort<double>& out = dut.get_output_port(i);
    values.emplace_back(out.Allocate());
    out.Calc(*context, values.back().get());
  }

  const auto& port2 = dut.get_output_port(OutputPortIndex(2));
  const auto& cache2 =
      dynamic_cast<const LeafOutputPort<double>&>(port2).cache_entry();
  const auto& cacheval2 = cache2.get_cache_entry_value(*context);
  EXPECT_EQ(cacheval2.serial_number(), 1);
  EXPECT_TRUE(cache2.is_out_of_date(*context));
  EXPECT_THROW(cache2.GetKnownUpToDate<std::string>(*context),
               std::logic_error);
  const std::string& str2_cached = port2.Eval<std::string>(*context);
  EXPECT_EQ(str2_cached, "concrete string");
  EXPECT_EQ(cacheval2.serial_number(), 2);
  EXPECT_FALSE(cache2.is_out_of_date(*context));
  EXPECT_EQ(cache2.GetKnownUpToDate<std::string>(*context),
            "concrete string");  // Doesn't throw now.

  // Check that setting time invalidates correctly. Note that the method
  // *may* avoid invalidation if the time hasn't actually changed.

  // Should invalidate time- and everything-dependents.
  context->SetTime(context->get_time() + 1.);
  EXPECT_TRUE(cache2.is_out_of_date(*context));
  EXPECT_EQ(cacheval2.serial_number(), 2);  // Unchanged since invalid.
  (void)port2.template Eval<AbstractValue>(*context);  // Recalculate.
  EXPECT_FALSE(cache2.is_out_of_date(*context));
  EXPECT_EQ(cacheval2.serial_number(), 3);
  (void)port2.template Eval<AbstractValue>(*context);  // Should do nothing.
  EXPECT_EQ(cacheval2.serial_number(), 3);

  // Should invalidate accuracy- and everything-dependents. Note that the
  // method *may* avoid invalidation if the accuracy hasn't actually changed.
  EXPECT_FALSE(context->get_accuracy());  // None set initially.
  context->SetAccuracy(.000025);  // This is a change.
  EXPECT_TRUE(cache2.is_out_of_date(*context));
  (void)port2.template Eval<AbstractValue>(*context);  // Recalculate.
  EXPECT_FALSE(cache2.is_out_of_date(*context));
  EXPECT_EQ(cacheval2.serial_number(), 4);

  // Downcast to concrete types.
  const BasicVector<double>* vec0{};
  const MyVector4d* vec1{};
  const std::string* str2{};
  const BasicVector<double>* vec3{};
  DRAKE_EXPECT_NO_THROW(vec0 = &values[0]->get_value<BasicVector<double>>());
  DRAKE_EXPECT_NO_THROW(vec1 = dynamic_cast<const MyVector4d*>(
                      &values[1]->get_value<BasicVector<double>>()));
  DRAKE_EXPECT_NO_THROW(str2 = &values[2]->get_value<std::string>());
  DRAKE_EXPECT_NO_THROW(vec3 = &values[3]->get_value<BasicVector<double>>());

  // Check the calculated values.
  EXPECT_EQ(vec0->get_value(), dut.expected_basic().get_value());
  EXPECT_EQ(vec1->get_value(), dut.expected_myvector().get_value());
  EXPECT_EQ(*str2, "concrete string");
  EXPECT_EQ(vec3->get_value(), Vector2d(10., 20.));
}

// Tests that the leaf system reserved the declared parameters of interesting
// custom type.
GTEST_TEST(ModelLeafSystemTest, ModelNumericParams) {
  DeclaredModelPortsSystem dut;
  auto context = dut.CreateDefaultContext();
  ASSERT_EQ(context->num_numeric_parameter_groups(), 1);
  const BasicVector<double>& param = context->get_numeric_parameter(0);
  // Check that type was preserved.
  ASSERT_TRUE(is_dynamic_castable<const MyVector2d>(&param));
  EXPECT_EQ(2, param.size());
  EXPECT_EQ(1.1, param[0]);
  EXPECT_EQ(2.2, param[1]);
}

// Tests that various DeclareDiscreteState() signatures work correctly and
// that the model values get used in SetDefaultContext().
GTEST_TEST(ModelLeafSystemTest, ModelDiscreteState) {
  class DeclaredModelDiscreteStateSystem : public LeafSystem<double> {
   public:
    // This should produce three discrete variable groups.
    DeclaredModelDiscreteStateSystem() {
      // Takes a BasicVector.
      indexes_.push_back(
          DeclareDiscreteState(MyVector2d(Vector2d(1., 2.))));
      DeclareStateOutputPort("state", indexes_.back());
      // Takes an Eigen vector.
      indexes_.push_back(DeclareDiscreteState(Vector3d(3., 4., 5.)));
      // Four state variables, initialized to zero.
      indexes_.push_back(DeclareDiscreteState(4));
    }
    std::vector<DiscreteStateIndex> indexes_;
  };

  DeclaredModelDiscreteStateSystem dut;
  EXPECT_EQ(dut.num_discrete_state_groups(), 3);
  for (int i=0; i < static_cast<int>(dut.indexes_.size()); ++i)
    EXPECT_TRUE(dut.indexes_[i] == i);

  auto context = dut.CreateDefaultContext();
  DiscreteValues<double>& xd = context->get_mutable_discrete_state();
  EXPECT_EQ(xd.num_groups(), 3);

  // Concrete type and value should have been preserved.
  BasicVector<double>& xd0 = xd.get_mutable_vector(0);
  EXPECT_TRUE(is_dynamic_castable<const MyVector2d>(&xd0));
  EXPECT_EQ(xd0.get_value(), Vector2d(1., 2.));
  EXPECT_EQ(dut.get_output_port().Eval<MyVector2d>(*context).get_value(),
            Vector2d(1., 2.));

  // Eigen vector should have been stored in a BasicVector-type object.
  BasicVector<double>& xd1 = xd.get_mutable_vector(1);
  EXPECT_EQ(typeid(xd1), typeid(BasicVector<double>));
  EXPECT_EQ(xd1.get_value(), Vector3d(3., 4., 5.));

  // Discrete state with no model should act as though it were given an
  // all-zero Eigen vector model.
  BasicVector<double>& xd2 = xd.get_mutable_vector(2);
  EXPECT_EQ(typeid(xd2), typeid(BasicVector<double>));
  EXPECT_EQ(xd2.get_value(), Vector4d(0., 0., 0., 0.));

  // Now make changes, then see if SetDefaultContext() puts them back.
  xd0.SetFromVector(Vector2d(9., 10.));
  xd1.SetFromVector(Vector3d(11., 12., 13.));
  xd2.SetFromVector(Vector4d(1., 2., 3., 4.));
  // Ensure that the cache knows that the values have changed.
  context->get_mutable_discrete_state();

  // Of course that had to work, but let's just prove it ...
  EXPECT_EQ(xd0.get_value(), Vector2d(9., 10.));
  EXPECT_EQ(xd1.get_value(), Vector3d(11., 12., 13.));
  EXPECT_EQ(xd2.get_value(), Vector4d(1., 2., 3., 4.));
  EXPECT_EQ(dut.get_output_port().Eval<MyVector2d>(*context).get_value(),
            Vector2d(9., 10.));

  dut.SetDefaultContext(&*context);
  EXPECT_EQ(xd0.get_value(), Vector2d(1., 2.));
  EXPECT_EQ(xd1.get_value(), Vector3d(3., 4., 5.));
  EXPECT_EQ(xd2.get_value(), Vector4d(0., 0., 0., 0.));
}

// Tests that DeclareAbstractState works expectedly.
GTEST_TEST(ModelLeafSystemTest, ModelAbstractState) {
  class DeclaredModelAbstractStateSystem : public LeafSystem<double> {
   public:
    DeclaredModelAbstractStateSystem() {
      DeclareAbstractState(Value<int>(1));
      DeclareAbstractState(Value<std::string>("wow"));
      DeclareStateOutputPort("state", AbstractStateIndex{1});
    }
  };

  DeclaredModelAbstractStateSystem dut;

  // Allocate the resources that were created on system construction.
  auto context = dut.AllocateContext();

  // Check that the allocations were made and with the correct type
  DRAKE_EXPECT_NO_THROW(context->get_abstract_state<int>(0));
  DRAKE_EXPECT_NO_THROW(context->get_abstract_state<std::string>(1));
  EXPECT_EQ(dut.get_output_port().Eval<std::string>(*context), "wow");

  // Mess with the abstract values on the context.
  AbstractValues& values = context->get_mutable_abstract_state();
  AbstractValue& value = values.get_mutable_value(1);
  DRAKE_EXPECT_NO_THROW(value.set_value<std::string>("whoops"));
  EXPECT_EQ(context->get_abstract_state<std::string>(1), "whoops");
  EXPECT_EQ(dut.get_output_port().Eval<std::string>(*context), "whoops");

  // Ask it to reset to the defaults specified on system construction.
  dut.SetDefaultContext(context.get());
  EXPECT_EQ(context->get_abstract_state<int>(0), 1);
  EXPECT_EQ(context->get_abstract_state<std::string>(1), "wow");

  // Just create a default context directly.
  auto default_context = dut.CreateDefaultContext();
  EXPECT_EQ(default_context->get_abstract_state<int>(0), 1);
  EXPECT_EQ(default_context->get_abstract_state<std::string>(1), "wow");
}


// A system that exercises the non-model based output port declarations.
// These include declarations that take only a calculator function and use
// default construction for allocation, and those that take an explicit
// allocator as a class method or general function.

// A BasicVector subclass with an initializing default constructor that
// sets it to (100,200).
class DummyVec2 : public BasicVector<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyVec2)

  DummyVec2(double e0, double e1) : BasicVector<double>(2) {
    SetAtIndex(0, e0);
    SetAtIndex(1, e1);
  }
  DummyVec2() : DummyVec2(100., 200.) {}

 private:
  // Note that the actual data is copied by the BasicVector base class.
  DummyVec2* DoClone() const override { return new DummyVec2; }
};

// This bare struct is used to verify that we value-initialize output ports.
struct SomePOD {
  int some_int;
  double some_double;
};

class DeclaredNonModelOutputSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeclaredNonModelOutputSystem);

  DeclaredNonModelOutputSystem() {
    // Use this to validate the expected return type from each method.
    LeafOutputPort<double>* port{nullptr};

    // Output port 0 default-constructs a class derived from BasicVector as
    // its allocator.
    port = &DeclareVectorOutputPort(
        kUseDefaultName, &DeclaredNonModelOutputSystem::CalcDummyVec2);
    unused(port);
    // Output port 1 default-constructs a string as its allocator.
    port = &DeclareAbstractOutputPort(
        kUseDefaultName, &DeclaredNonModelOutputSystem::CalcString);
    unused(port);

    // Output port 2 uses the "Advanced" method for abstract ports, providing
    // explicit non-member functors for allocator and calculator.
    port = &DeclareAbstractOutputPort(
        kUseDefaultName,
        []() { return AbstractValue::Make<int>(-2); },
        [](const Context<double>&, AbstractValue* out) {
          ASSERT_NE(out, nullptr);
          int* int_out{};
          DRAKE_EXPECT_NO_THROW(int_out = &out->get_mutable_value<int>());
          *int_out = 321;
        });
    unused(port);

    // Output port 3 uses a default-constructed bare struct which should be
    // value-initialized.
    port = &DeclareAbstractOutputPort(
        kUseDefaultName, &DeclaredNonModelOutputSystem::CalcPOD);
    unused(port);
  }

  int calc_dummy_vec2_calls() const { return count_calc_dummy_vec2_; }
  int calc_string_calls() const { return count_calc_string_; }
  int calc_POD_calls() const { return count_calc_POD_; }

 private:
  void CalcDummyVec2(const Context<double>&, DummyVec2* out) const {
    ++count_calc_dummy_vec2_;
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->size(), 2);
    out->get_mutable_value() = Vector2d(-100., -200);
  }

  // Explicit allocator method.
  std::string MakeString() const {
    return std::string("freshly made");
  }

  void CalcString(const Context<double>&, std::string* out) const {
    ++count_calc_string_;
    ASSERT_NE(out, nullptr);
    *out = "calc'ed string";
  }

  void CalcPOD(const Context<double>&, SomePOD* out) const {
    ++count_calc_POD_;
    ASSERT_NE(out, nullptr);
    *out = {-10, 3.25};
  }

  // Call counters for caching checks.
  mutable int count_calc_dummy_vec2_{0};
  mutable int count_calc_string_{0};
  mutable int count_calc_POD_{0};
};

// Tests that non-model based Declare{Vector,Abstract}OutputPort generate the
// expected output port allocators, and that their Calc and Eval functions work.
GTEST_TEST(NonModelLeafSystemTest, NonModelPortsOutput) {
  DeclaredNonModelOutputSystem dut;
  auto context = dut.CreateDefaultContext();
  auto system_output = dut.AllocateOutput();  // Invokes all allocators.

  // Make sure caching is on locally, even if it is off by default.
  context->EnableCaching();

  // Check topology.
  EXPECT_EQ(dut.num_input_ports(), 0);
  ASSERT_EQ(dut.num_output_ports(), 4);

  auto& out0 = dut.get_output_port(0);
  auto& out1 = dut.get_output_port(1);
  auto& out2 = dut.get_output_port(2);
  auto& out3 = dut.get_output_port(3);
  EXPECT_EQ(out0.get_data_type(), kVectorValued);
  EXPECT_EQ(out1.get_data_type(), kAbstractValued);
  EXPECT_EQ(out2.get_data_type(), kAbstractValued);
  EXPECT_EQ(out3.get_data_type(), kAbstractValued);

  // Sanity check output port prerequisites. Leaf ports should not designate
  // a subsystem since they are resolved internally. We don't know the right
  // dependency ticket, but at least it should be valid.
  for (OutputPortIndex i(0); i < dut.num_output_ports(); ++i) {
    internal::OutputPortPrerequisite prereq =
        dut.get_output_port(i).GetPrerequisite();
    EXPECT_FALSE(prereq.child_subsystem.has_value());
    EXPECT_TRUE(prereq.dependency.is_valid());
  }

  // Check that DummyVec2 came out, default constructed to (100,200).
  auto output0 = system_output->GetMutableVectorData(0);
  ASSERT_NE(output0, nullptr);
  EXPECT_EQ(output0->size(), 2);
  auto out0_dummy = dynamic_cast<DummyVec2*>(output0);
  EXPECT_NE(out0_dummy, nullptr);
  EXPECT_EQ(out0_dummy->get_value(), Vector2d(100., 200.));
  out0.Calc(*context, system_output->GetMutableData(0));
  EXPECT_EQ(out0_dummy->get_value(), Vector2d(-100., -200.));

  EXPECT_EQ(dut.calc_dummy_vec2_calls(), 1);
  EXPECT_EQ(out0.Eval<BasicVector<double>>(*context).get_value(),
            out0_dummy->get_value());
  EXPECT_EQ(dut.calc_dummy_vec2_calls(), 2);
  out0.Eval<BasicVector<double>>(*context);
  EXPECT_EQ(dut.calc_dummy_vec2_calls(), 2);  // Should have been cached.

  // Check that Value<string>() came out, default initialized to empty.
  auto output1 = system_output->GetMutableData(1);
  ASSERT_NE(output1, nullptr);
  const std::string* downcast_output1{};
  DRAKE_EXPECT_NO_THROW(downcast_output1 = &output1->get_value<std::string>());
  EXPECT_TRUE(downcast_output1->empty());
  out1.Calc(*context, output1);
  EXPECT_EQ(*downcast_output1, "calc'ed string");

  EXPECT_EQ(dut.calc_string_calls(), 1);
  EXPECT_EQ(out1.Eval<std::string>(*context), *downcast_output1);
  EXPECT_EQ(dut.calc_string_calls(), 2);
  out1.Eval<std::string>(*context);
  EXPECT_EQ(dut.calc_string_calls(), 2);  // Should have been cached.

  // Check that Value<int> came out, default initialized to -2.
  auto output2 = system_output->GetMutableData(2);
  ASSERT_NE(output2, nullptr);
  const int* downcast_output2{};
  DRAKE_EXPECT_NO_THROW(downcast_output2 = &output2->get_value<int>());
  EXPECT_EQ(*downcast_output2, -2);
  out2.Calc(*context, output2);
  EXPECT_EQ(*downcast_output2, 321);

  // Check that Value<SomePOD>{} came out, value initialized. Note that this
  // is not a perfect test since the values *could* come out zero by accident
  // even if the value initializer had not been called. Better than nothing!
  auto output3 = system_output->GetMutableData(3);
  ASSERT_NE(output3, nullptr);
  const SomePOD* downcast_output3{};
  DRAKE_EXPECT_NO_THROW(downcast_output3 = &output3->get_value<SomePOD>());
  EXPECT_EQ(downcast_output3->some_int, 0);
  EXPECT_EQ(downcast_output3->some_double, 0.0);
  out3.Calc(*context, output3);
  EXPECT_EQ(downcast_output3->some_int, -10);
  EXPECT_EQ(downcast_output3->some_double, 3.25);

  EXPECT_EQ(dut.calc_POD_calls(), 1);
  const auto& eval_out = out3.Eval<SomePOD>(*context);
  EXPECT_EQ(eval_out.some_int, -10);
  EXPECT_EQ(eval_out.some_double, 3.25);
  EXPECT_EQ(dut.calc_POD_calls(), 2);
  out3.Eval<SomePOD>(*context);
  EXPECT_EQ(dut.calc_POD_calls(), 2);  // Should have been cached.
}

// Tests that zero-sized vectors can be declared and used.
GTEST_TEST(ZeroSizeSystemTest, AcceptanceTest) {
  TestSystem<double> dut;

  // Input.
  auto& in0 = dut.DeclareVectorInputPort(
      kUseDefaultName, BasicVector<double>(0));
  EXPECT_EQ(in0.get_data_type(), kVectorValued);
  EXPECT_EQ(in0.size(), 0);

  // Output.
  auto& out0 = dut.DeclareVectorOutputPort(
      kUseDefaultName, BasicVector<double>(0),
      [](const Context<double>&, BasicVector<double>*) {});
  EXPECT_EQ(out0.get_data_type(), kVectorValued);
  EXPECT_EQ(out0.size(), 0);

  // State.
  dut.DeclareContinuousState(0);
  const auto& disc0 = dut.DeclareDiscreteState(0);

  // Parameters.
  const auto& param0 = dut.DeclareNumericParameter(BasicVector<double>(0));

  auto context = dut.CreateDefaultContext();
  EXPECT_EQ(context->get_continuous_state_vector().size(), 0);
  EXPECT_EQ(context->get_discrete_state(disc0).size(), 0);
  EXPECT_EQ(context->get_numeric_parameter(param0).size(), 0);
}

// Tests both that an unrestricted update callback is called and that
// modifications to state dimension are caught.
TEST_F(LeafSystemTest, CallbackAndInvalidUpdates) {
  // Create 9, 1, and 3 dimensional continuous, discrete, and abstract state
  // vectors.

  // This needs to be a LeafContext for access to init_ methods.
  auto context = dynamic_pointer_cast_or_throw<LeafContext<double>>(
      system_.CreateDefaultContext());

  context->init_continuous_state(std::make_unique<ContinuousState<double>>(
      std::make_unique<BasicVector<double>>(9), 3, 3, 3));
  context->init_discrete_state(std::make_unique<DiscreteValues<double>>(
      std::make_unique<BasicVector<double>>(1)));
  std::vector<std::unique_ptr<AbstractValue>> abstract_data;
  abstract_data.push_back(PackValue(3));
  abstract_data.push_back(PackValue(5));
  abstract_data.push_back(PackValue(7));
  context->init_abstract_state(
      std::make_unique<AbstractValues>(std::move(abstract_data)));

  // Copy the state.
  std::unique_ptr<State<double>> x = context->CloneState();

  // Create an unrestricted update callback that just copies the state.
  LeafCompositeEventCollection<double> leaf_events;
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback = [](
        const Context<double>& c, const Event<double>&, State<double>* s) {
      s->SetFrom(*c.CloneState());
    };

    UnrestrictedUpdateEvent<double> event(TriggerType::kPeriodic, callback);
    event.AddToComposite(&leaf_events);
  }

  // Verify no exception is thrown.
  DRAKE_EXPECT_NO_THROW(system_.CalcUnrestrictedUpdate(
      *context, leaf_events.get_unrestricted_update_events(), x.get()));

  // Change the function to change the continuous state dimension.
  // Call the unrestricted update function again, now verifying that an
  // exception is thrown.
  leaf_events.Clear();
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback = [](
        const Context<double>& c, const Event<double>&, State<double>* s) {
      s->SetFrom(*c.CloneState());
      s->set_continuous_state(std::make_unique<ContinuousState<double>>(
          std::make_unique<BasicVector<double>>(4), 4, 0, 0));
    };

    UnrestrictedUpdateEvent<double> event(TriggerType::kPeriodic, callback);
    event.AddToComposite(&leaf_events);
  }

  // Call the unrestricted update function, verifying that an exception
  // is thrown.
  EXPECT_THROW(
      system_.CalcUnrestrictedUpdate(
          *context, leaf_events.get_unrestricted_update_events(), x.get()),
      std::logic_error);

  // Restore the continuous state (size).
  x->set_continuous_state(std::make_unique<ContinuousState<double>>(
      std::make_unique<BasicVector<double>>(9), 3, 3, 3));

  // Change the event to indicate to change the discrete state dimension.
  leaf_events.Clear();
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback = [](
        const Context<double>& c, const Event<double>&, State<double>* s) {
      std::vector<std::unique_ptr<BasicVector<double>>> disc_data;
      s->SetFrom(*c.CloneState());
      disc_data.push_back(std::make_unique<BasicVector<double>>(1));
      disc_data.push_back(std::make_unique<BasicVector<double>>(1));
      s->set_discrete_state(
          std::make_unique<DiscreteValues<double>>(std::move(disc_data)));
    };

    UnrestrictedUpdateEvent<double> event(TriggerType::kPeriodic, callback);
    event.AddToComposite(&leaf_events);
  }

  // Call the unrestricted update function again, again verifying that an
  // exception is thrown.
  EXPECT_THROW(
      system_.CalcUnrestrictedUpdate(
          *context, leaf_events.get_unrestricted_update_events(), x.get()),
      std::logic_error);

  // Restore the discrete state (size).
  x->set_discrete_state(std::make_unique<DiscreteValues<double>>(
      std::make_unique<BasicVector<double>>(1)));

  // Change the event to indicate to change the abstract state dimension.
  leaf_events.Clear();
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback = [](
        const Context<double>& c, const Event<double>&, State<double>* s) {
      s->SetFrom(*c.CloneState());
      s->set_abstract_state(std::make_unique<AbstractValues>());
    };

    UnrestrictedUpdateEvent<double> event(TriggerType::kPeriodic, callback);
    event.AddToComposite(&leaf_events);
  }

  // Call the unrestricted update function again, again verifying that an
  // exception is thrown.
  EXPECT_THROW(
      system_.CalcUnrestrictedUpdate(
          *context, leaf_events.get_unrestricted_update_events(), x.get()),
      std::logic_error);
}

// Tests that the next update time is computed correctly for LeafSystems
// templated on AutoDiffXd. Protects against regression on #4431.
GTEST_TEST(AutodiffLeafSystemTest, NextUpdateTimeAutodiff) {
  TestSystem<AutoDiffXd> system;
  std::unique_ptr<Context<AutoDiffXd>> context = system.CreateDefaultContext();

  context->SetTime(21.0);
  system.AddPeriodicUpdate();

  auto event_info = system.AllocateCompositeEventCollection();
  auto time = system.CalcNextUpdateTime(*context, event_info.get());

  EXPECT_EQ(25.0, time);
}

// A LeafSystem that uses the default direct-feedthrough implementation without
// symbolic sparsity analysis.  The only sparsity that is (optionally) used is
// the output port cache prerequisites.  (The particular unit that employs this
// system chooses those prerequisites.)
class DefaultFeedthroughSystem : public LeafSystem<double> {
 public:
  DefaultFeedthroughSystem() {}

  ~DefaultFeedthroughSystem() override {}

  InputPortIndex AddAbstractInputPort() {
    return this->DeclareAbstractInputPort(
        kUseDefaultName, Value<std::string>{}).get_index();
  }

  OutputPortIndex AddAbstractOutputPort(
      std::optional<std::set<DependencyTicket>> prerequisites_of_calc = {}) {
    // Dummies.
    auto alloc = []() { return AbstractValue::Make<int>(); };
    auto calc = [](const ContextBase&, AbstractValue*) {};
    if (prerequisites_of_calc) {
      return this->DeclareAbstractOutputPort(
          kUseDefaultName, alloc, calc, *prerequisites_of_calc).get_index();
    } else {
      // The DeclareAbstractOutputPort API's default value for the
      // prerequisites_of_calc is everything (i.e., "all_sources_ticket()").
      return this->DeclareAbstractOutputPort(
          kUseDefaultName, alloc, calc).get_index();
    }
  }

  // Elevate helper methods to be public.
  using LeafSystem<double>::accuracy_ticket;
  using LeafSystem<double>::all_input_ports_ticket;
  using LeafSystem<double>::all_parameters_ticket;
  using LeafSystem<double>::all_sources_ticket;
  using LeafSystem<double>::all_state_ticket;
  using LeafSystem<double>::configuration_ticket;
  using LeafSystem<double>::input_port_ticket;
  using LeafSystem<double>::kinematics_ticket;
  using LeafSystem<double>::nothing_ticket;
  using LeafSystem<double>::time_ticket;
};

GTEST_TEST(FeedthroughTest, DefaultWithNoInputsOrOutputs) {
  DefaultFeedthroughSystem system;
  EXPECT_FALSE(system.HasAnyDirectFeedthrough());
  // No ports implies no pairs reported for direct feedthrough.
  const std::multimap<int, int> expected;
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);
}

GTEST_TEST(FeedthroughTest, DefaultWithBothInputsAndOutputs) {
  DefaultFeedthroughSystem system;
  system.AddAbstractInputPort();
  system.AddAbstractOutputPort();
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  EXPECT_TRUE(system.HasDirectFeedthrough(0));
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 0));
  // Confirm all pairs are returned.
  const std::multimap<int, int> expected{{0, 0}};
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);
}

GTEST_TEST(FeedthroughTest, DefaultWithInputsOnly) {
  DefaultFeedthroughSystem system;
  system.AddAbstractInputPort();
  EXPECT_FALSE(system.HasAnyDirectFeedthrough());
  // No output ports implies no pairs reported for direct feedthrough.
  const std::multimap<int, int> expected;
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);
}

GTEST_TEST(FeedthroughTest, DefaultWithOutputsOnly) {
  DefaultFeedthroughSystem system;
  system.AddAbstractOutputPort();
  EXPECT_FALSE(system.HasAnyDirectFeedthrough());
  EXPECT_FALSE(system.HasDirectFeedthrough(0));
  // No input ports implies no pairs reported for direct feedthrough.
  const std::multimap<int, int> expected;
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);
}

GTEST_TEST(FeedthroughTest, DefaultWithPrerequisites) {
  DefaultFeedthroughSystem system;
  const auto input_port_index0 = system.AddAbstractInputPort();
  const auto input_port_index1 = system.AddAbstractInputPort();

  const std::vector<DependencyTicket> feedthrough_tickets{
    system.input_port_ticket(input_port_index0),
    system.input_port_ticket(input_port_index1),
    system.all_input_ports_ticket(),
    system.all_sources_ticket(),
  };
  const std::vector<DependencyTicket> non_feedthrough_tickets{
    system.nothing_ticket(),
    system.time_ticket(),
    system.accuracy_ticket(),
    system.all_state_ticket(),
    system.all_parameters_ticket(),
    system.configuration_ticket(),
    system.kinematics_ticket(),
  };

  for (const auto& ticket : non_feedthrough_tickets) {
    const auto output_port_index = system.AddAbstractOutputPort({{ ticket }});
    EXPECT_FALSE(system.HasDirectFeedthrough(output_port_index));
  }
  for (const auto& ticket : feedthrough_tickets) {
    const auto output_port_index = system.AddAbstractOutputPort({{ ticket }});
    EXPECT_TRUE(system.HasDirectFeedthrough(output_port_index));
  }
}

// With multiple input and output ports, all input ports are thought to feed
// through to all output ports.
GTEST_TEST(FeedthroughTest, DefaultWithMultipleIoPorts) {
  const int input_count = 3;
  const int output_count = 4;
  std::multimap<int, int> expected;
  DefaultFeedthroughSystem system;
  for (int i = 0; i < input_count; ++i) {
    system.AddAbstractInputPort();
    for (int o = 0; o < output_count; ++o) {
      if (i == 0) system.AddAbstractOutputPort();
      expected.emplace(i, o);
    }
  }
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  for (int o = 0; o < output_count; ++o) {
    EXPECT_TRUE(system.HasDirectFeedthrough(o));
    for (int i = 0; i < input_count; ++i) {
      EXPECT_TRUE(system.HasDirectFeedthrough(i, o));
    }
  }
  // No sparsity matrix means all inputs feedthrough to all outputs.
  auto feedthrough_pairs = system.GetDirectFeedthroughs();
  EXPECT_EQ(feedthrough_pairs, expected);
}

// SymbolicSparsitySystem has the same sparsity matrix as ManualSparsitySystem,
// but the matrix can be inferred from the symbolic form.
template <typename T>
class SymbolicSparsitySystem : public LeafSystem<T> {
 public:
  explicit SymbolicSparsitySystem(bool use_default_prereqs = true)
      : SymbolicSparsitySystem(SystemTypeTag<SymbolicSparsitySystem>{},
                               use_default_prereqs) {}

  // Scalar-converting copy constructor.
  template <typename U>
  SymbolicSparsitySystem(const SymbolicSparsitySystem<U>& source)
      : SymbolicSparsitySystem<T>(source.is_using_default_prereqs()) {
    source.count_conversion();
  }

  // Note that this object was used as the source for a scalar conversion.
  void count_conversion() const { ++num_conversions_; }

  int num_conversions() const { return num_conversions_; }

  bool is_using_default_prereqs() const { return use_default_prereqs_; }

 protected:
  explicit SymbolicSparsitySystem(SystemScalarConverter converter,
                                  bool use_default_prereqs = true)
      : LeafSystem<T>(std::move(converter)),
        use_default_prereqs_(use_default_prereqs) {
    const int kSize = 1;

    this->DeclareInputPort(kUseDefaultName, kVectorValued, kSize);
    this->DeclareInputPort(kUseDefaultName, kVectorValued, kSize);

    if (is_using_default_prereqs()) {
      // Don't specify prerequisites; we'll have to perform symbolic analysis
      // to determine whether there is feedthrough.
      this->DeclareVectorOutputPort(kUseDefaultName, BasicVector<T>(kSize),
                                    &SymbolicSparsitySystem::CalcY0);
      this->DeclareVectorOutputPort(kUseDefaultName, BasicVector<T>(kSize),
                                    &SymbolicSparsitySystem::CalcY1);
    } else {
      // Explicitly specify the prerequisites for the code in CalcY0() and
      // CalcY1() below. No need for further analysis to determine feedthrough.
      this->DeclareVectorOutputPort(
          kUseDefaultName, BasicVector<T>(kSize),
          &SymbolicSparsitySystem::CalcY0,
          {this->input_port_ticket(InputPortIndex(1))});
      this->DeclareVectorOutputPort(
          kUseDefaultName, BasicVector<T>(kSize),
          &SymbolicSparsitySystem::CalcY1,
          {this->input_port_ticket(InputPortIndex(0))});
    }
  }

 private:
  void CalcY0(const Context<T>& context,
                    BasicVector<T>* y0) const {
    const auto& u1 = this->get_input_port(1).Eval(context);
    y0->set_value(u1);
  }

  void CalcY1(const Context<T>& context,
              BasicVector<T>* y1) const {
    const auto& u0 = this->get_input_port(0).Eval(context);
    y1->set_value(u0);
  }

  const bool use_default_prereqs_;

  // Count how many times this object was used as the _source_ for the
  // conversion constructor.
  mutable int num_conversions_{0};
};

// The sparsity reporting should be the same no matter which scalar type the
// original system has been instantiated with.
using FeedthroughTestScalars = ::testing::Types<
  double,
  AutoDiffXd,
  symbolic::Expression>;

template <typename T>
class FeedthroughTypedTest : public ::testing::Test {};
TYPED_TEST_SUITE(FeedthroughTypedTest, FeedthroughTestScalars);

// The sparsity of a System should be inferred from its symbolic form.
TYPED_TEST(FeedthroughTypedTest, SymbolicSparsityDefaultPrereqs) {
  using T = TypeParam;
  const SymbolicSparsitySystem<T> system;

  // Both the output ports have direct feedthrough from some input.
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  EXPECT_TRUE(system.HasDirectFeedthrough(0));
  EXPECT_TRUE(system.HasDirectFeedthrough(1));
  // Check the entire matrix.
  EXPECT_FALSE(system.HasDirectFeedthrough(0, 0));
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 1));
  EXPECT_TRUE(system.HasDirectFeedthrough(1, 0));
  EXPECT_FALSE(system.HasDirectFeedthrough(1, 1));
  // Confirm the exact set of desired pairs are returned.
  std::multimap<int, int> expected;
  expected.emplace(1, 0);
  expected.emplace(0, 1);
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);

  // Since we didn't provide prerequisites for the output ports, each of the 8
  // calls above should have required a scalar conversion to symbolic unless
  // T was already symbolic.
  int expected_conversions = 8;
  if constexpr (std::is_same_v<T, symbolic::Expression>) {
    expected_conversions = 0;
  }

  EXPECT_EQ(system.num_conversions(), expected_conversions);
}

// Repeat the above test using explicitly-specified prerequisites to avoid
// having to convert to symbolic form.
TYPED_TEST(FeedthroughTypedTest, SymbolicSparsityExplicitPrereqs) {
  using T = TypeParam;
  const SymbolicSparsitySystem<T> system(false);  // Use explicit prereqs.

  // Both the output ports have direct feedthrough from some input.
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  EXPECT_TRUE(system.HasDirectFeedthrough(0));
  EXPECT_TRUE(system.HasDirectFeedthrough(1));
  // Check the entire matrix.
  EXPECT_FALSE(system.HasDirectFeedthrough(0, 0));
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 1));
  EXPECT_TRUE(system.HasDirectFeedthrough(1, 0));
  EXPECT_FALSE(system.HasDirectFeedthrough(1, 1));
  // Confirm the exact set of desired pairs are returned.
  std::multimap<int, int> expected;
  expected.emplace(1, 0);
  expected.emplace(0, 1);
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);

  // Shouldn't have been any conversions required above.
  const int expected_conversions = 0;
  EXPECT_EQ(system.num_conversions(), expected_conversions);
}

// This system only supports T = symbolic::Expression; it does not support
// scalar conversion.
class NoScalarConversionSymbolicSparsitySystem
    : public SymbolicSparsitySystem<symbolic::Expression> {
 public:
  NoScalarConversionSymbolicSparsitySystem()
      : SymbolicSparsitySystem<symbolic::Expression>(
            SystemScalarConverter{}) {}
};

// The sparsity of a System should be inferred from its symbolic form, even
// when the system does not support scalar conversion.
GTEST_TEST(FeedthroughTest, SymbolicSparsityWithoutScalarConversion) {
  const NoScalarConversionSymbolicSparsitySystem system;

  // Confirm the exact set of desired pairs are returned.
  std::multimap<int, int> expected;
  expected.emplace(1, 0);
  expected.emplace(0, 1);
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);
}

// Sanity check ToScalarTypeMaybe, to show which type conversions
// should or should not succeed.
GTEST_TEST(LeafSystemScalarConverterTest, TemplateSupportedConversions) {
  SymbolicSparsitySystem<double> dut;

  EXPECT_EQ(dut.ToScalarTypeMaybe<double>(), nullptr);
  EXPECT_NE(dut.ToScalarTypeMaybe<AutoDiffXd>(), nullptr);
  EXPECT_NE(dut.ToScalarTypeMaybe<symbolic::Expression>(), nullptr);

  auto autodiff = dut.ToScalarTypeMaybe<AutoDiffXd>();
  ASSERT_NE(autodiff, nullptr);

  EXPECT_NE(autodiff->ToScalarTypeMaybe<double>(), nullptr);
  EXPECT_EQ(autodiff->ToScalarTypeMaybe<AutoDiffXd>(), nullptr);
  EXPECT_NE(autodiff->ToScalarTypeMaybe<symbolic::Expression>(), nullptr);

  auto symbolic = dut.ToScalarTypeMaybe<symbolic::Expression>();
  ASSERT_NE(symbolic, nullptr);

  EXPECT_NE(symbolic->ToScalarTypeMaybe<double>(), nullptr);
  EXPECT_NE(symbolic->ToScalarTypeMaybe<AutoDiffXd>(), nullptr);
  EXPECT_EQ(symbolic->ToScalarTypeMaybe<symbolic::Expression>(), nullptr);
}

// Sanity check the default implementation of ToAutoDiffXd, for cases that
// should succeed.
GTEST_TEST(LeafSystemScalarConverterTest, AutoDiffYes) {
  SymbolicSparsitySystem<double> dut;
  dut.set_name("special_name");

  // Static method automatically downcasts.
  std::unique_ptr<SymbolicSparsitySystem<AutoDiffXd>> clone =
      System<double>::ToAutoDiffXd(dut);
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->get_name(), "special_name");

  clone = System<double>::ToScalarType<AutoDiffXd>(dut);
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->get_name(), "special_name");

  // Instance method that reports failures via exception.
  EXPECT_NE(dut.ToAutoDiffXd(), nullptr);
  EXPECT_NE(dut.ToScalarType<AutoDiffXd>(), nullptr);

  // Instance method that reports failures via nullptr.
  auto maybe = dut.ToAutoDiffXdMaybe();
  ASSERT_NE(maybe, nullptr);
  EXPECT_EQ(maybe->get_name(), "special_name");

  maybe = dut.ToScalarTypeMaybe<AutoDiffXd>();
  ASSERT_NE(maybe, nullptr);
  EXPECT_EQ(maybe->get_name(), "special_name");

  // Spot check the specific converter object.
  EXPECT_TRUE((
      dut.get_system_scalar_converter().IsConvertible<AutoDiffXd, double>()));
  EXPECT_FALSE((
      dut.get_system_scalar_converter().IsConvertible<double, double>()));
}

// Sanity check the default implementation of ToAutoDiffXd, for cases that
// should fail.
GTEST_TEST(LeafSystemScalarConverterTest, AutoDiffNo) {
  TestSystem<double> dut;

  // Static method.
  EXPECT_THROW(System<double>::ToAutoDiffXd(dut), std::exception);
  EXPECT_THROW(System<double>::ToScalarType<AutoDiffXd>(dut), std::exception);

  // Instance method that reports failures via exception.
  EXPECT_THROW(dut.ToAutoDiffXd(), std::exception);
  EXPECT_THROW(dut.ToScalarType<AutoDiffXd>(), std::exception);

  // Instance method that reports failures via nullptr.
  EXPECT_EQ(dut.ToAutoDiffXdMaybe(), nullptr);
  EXPECT_EQ(dut.ToScalarTypeMaybe<AutoDiffXd>(), nullptr);
}

// Sanity check the default implementation of ToSymbolic, for cases that
// should succeed.
GTEST_TEST(LeafSystemScalarConverterTest, SymbolicYes) {
  SymbolicSparsitySystem<double> dut;
  dut.set_name("special_name");

  // Static method automatically downcasts.
  std::unique_ptr<SymbolicSparsitySystem<symbolic::Expression>> clone =
      System<double>::ToSymbolic(dut);
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->get_name(), "special_name");

  clone = System<double>::ToScalarType<symbolic::Expression>(dut);
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->get_name(), "special_name");

  // Instance method that reports failures via exception.
  EXPECT_NE(dut.ToSymbolic(), nullptr);
  EXPECT_NE(dut.ToScalarType<symbolic::Expression>(), nullptr);

  // Instance method that reports failures via nullptr.
  auto maybe = dut.ToSymbolicMaybe();
  ASSERT_NE(maybe, nullptr);
  EXPECT_EQ(maybe->get_name(), "special_name");

  maybe = dut.ToScalarTypeMaybe<symbolic::Expression>();
  ASSERT_NE(maybe, nullptr);
  EXPECT_EQ(maybe->get_name(), "special_name");
}

// Sanity check the default implementation of ToSymbolic, for cases that
// should fail.
GTEST_TEST(LeafSystemScalarConverterTest, SymbolicNo) {
  TestSystem<double> dut;

  // Static method.
  EXPECT_THROW(System<double>::ToSymbolic(dut), std::exception);
  EXPECT_THROW(System<double>::ToScalarType<symbolic::Expression>(dut),
               std::exception);

  // Instance method that reports failures via exception.
  EXPECT_THROW(dut.ToSymbolic(), std::exception);
  EXPECT_THROW(dut.ToScalarType<symbolic::Expression>(), std::exception);

  // Instance method that reports failures via nullptr.
  EXPECT_EQ(dut.ToSymbolicMaybe(), nullptr);
  EXPECT_EQ(dut.ToScalarTypeMaybe<symbolic::Expression>(), nullptr);
}

GTEST_TEST(GraphvizTest, Attributes) {
  DefaultFeedthroughSystem system;
  // Check that the ID is the memory address.
  ASSERT_EQ(reinterpret_cast<int64_t>(&system), system.GetGraphvizId());
  const std::string dot = system.GetGraphvizString();
  // Check that left-to-right ranking is imposed.
  EXPECT_THAT(dot, ::testing::HasSubstr("rankdir=LR"));
  // Check that NiceTypeName is used to compute the label.
  EXPECT_THAT(
      dot, ::testing::HasSubstr(
               "label=\"drake/systems/(anonymous)/DefaultFeedthroughSystem@"));
}

GTEST_TEST(GraphvizTest, Ports) {
  DefaultFeedthroughSystem system;
  system.AddAbstractInputPort();
  system.AddAbstractInputPort();
  system.AddAbstractOutputPort();
  const std::string dot = system.GetGraphvizString();
  EXPECT_THAT(dot, ::testing::HasSubstr("{{<u0>u0|<u1>u1} | {<y0>y0}}"));
}

// This class serves as the mechanism by which we confirm that LeafSystem's
// implementation of DispatchPublishHandler() actually calls DoPublish() and,
// furthermore, the per-event dispatch implemented in DoPublish is completely
// replaced by an overridden implementation.
//
// The system has to be able to detect when its DoPublish() is called but also
// has to allow us to detect the difference between the built-in LeafSystem
// publish event dispatching and the overridden behavior.
// We do this in the following way:
//
//   1. We create a system that overrides DoPublish().
//     a. The system can be configured to run in one of two modes:
//        i. Ignore publish events.
//        ii. Use default publication dispatch.
//        iii. In either mode, we increment a counter that will allow us to
//             recognize that our DoPublish override got exercised.
//     b. The system declares one force publish event.
//        i. The force event allows us to test both APIs
//           System::Publish(context) and System::Publish(context, events). As
//           all other trigger types exercise the two apis, we'll ignore other
//           trigger types.
//        ii. The event handler increments a counter allowing us to recognize
//            when the *event* has been handled.
//   2. We instantiate the system and evaluate the force events on it.
//      i. In the "default" mode, we should see DoPublish() and the event
//         handler invoked. This allows us to observe the "default" behavior and
//         see that it changes.
//      ii. In the "ignore events" mode, we expect DoPublish() to be called but
//         not the event handler - we'll have completely supplanted the
//         LeafSystem::DoPublish implementation).
class DoPublishOverrideSystem : public LeafSystem<double> {
 public:
  /* Constructs the system to *default* event handling -- events will *not*
   be ignored. */
  DoPublishOverrideSystem() : LeafSystem<double>() {
    DeclareForcedPublishEvent(&DoPublishOverrideSystem::HandleEvent);
  }

  bool ignore_events() const { return ignore_events_; }
  void set_ignore_events(bool ignore_events) { ignore_events_ = ignore_events; }
  int do_publish_count() const { return do_publish_count_; }
  int event_handle_count() const { return event_handle_count_; }

  EventStatus HandleEvent(const Context<double>&) const {
    ++event_handle_count_;
    return EventStatus::Succeeded();
  }

  // We need public access to the event collection to call
  // Publish(context, events).
  const EventCollection<PublishEvent<double>>&
  get_forced_publish_events_collection() const {
    return get_forced_publish_events();
  }

 private:
  void DoPublish(
      const Context<double>& context,
      const std::vector<const PublishEvent<double>*>& events) const override {
    ++do_publish_count_;
    if (!ignore_events_) LeafSystem<double>::DoPublish(context, events);
  }

  // If true, DoPublish ignores events, calls LeafSystem::DoPublish() if false.
  bool ignore_events_{false};

  // These mutable system members are an anti-pattern only acceptable as part of
  // a unit test.

  // The number of times DoPublish() has been called on this instance.
  mutable int do_publish_count_{0};

  // The number of times the force publish event handler has been called on this
  // instance.
  mutable int event_handle_count_{0};
};

GTEST_TEST(DoPublishOverrideTest, ConfirmOverride) {
  DoPublishOverrideSystem system;
  std::unique_ptr<Context<double>> context = system.CreateDefaultContext();
  const EventCollection<PublishEvent<double>>& events =
      system.get_forced_publish_events_collection();

  // First test default behaviors - DoPublish gets called and event handler
  // get called.
  ASSERT_FALSE(system.ignore_events());
  ASSERT_EQ(system.do_publish_count(), 0);
  ASSERT_EQ(system.event_handle_count(), 0);

  system.Publish(*context);
  EXPECT_EQ(system.do_publish_count(), 1);
  EXPECT_EQ(system.event_handle_count(), 1);
  system.Publish(*context, events);
  EXPECT_EQ(system.do_publish_count(), 2);
  EXPECT_EQ(system.event_handle_count(), 2);

  // Now ignore default behaviors. This confirms *our* DoPublish completely
  // supplants the default behavior.
  system.set_ignore_events(true);
  ASSERT_TRUE(system.ignore_events());

  system.Publish(*context);
  EXPECT_EQ(system.do_publish_count(), 3);
  EXPECT_EQ(system.event_handle_count(), 2);
  system.Publish(*context, events);
  EXPECT_EQ(system.do_publish_count(), 4);
  EXPECT_EQ(system.event_handle_count(), 2);
}

// The custom context type for the CustomContextSystem.
template <typename T>
class CustomContext : public LeafContext<T> {};

// CustomContextSystem has a LeafContext-derived custom context type. This
// confirms that the appropriate context type is generated..
template <typename T>
class CustomContextSystem : public LeafSystem<T> {
 protected:
  std::unique_ptr<LeafContext<T>> DoMakeLeafContext() const override {
    return std::make_unique<CustomContext<T>>();
  }
};

GTEST_TEST(CustomContextTest, AllocatedContext) {
  CustomContextSystem<double> system;
  auto allocated = system.AllocateContext();
  ASSERT_TRUE(is_dynamic_castable<CustomContext<double>>(allocated.get()));
  auto defaulted = system.CreateDefaultContext();
  ASSERT_TRUE(is_dynamic_castable<CustomContext<double>>(defaulted.get()));
}

// Specializes BasicVector to add inequality constraints.
template <typename T, int bias>
class ConstraintBasicVector final : public BasicVector<T> {
 public:
  static constexpr int kSize = 3;
  ConstraintBasicVector() : BasicVector<T>(VectorX<T>::Zero(kSize)) {}
  BasicVector<T>* DoClone() const override { return new ConstraintBasicVector; }

  void GetElementBounds(VectorXd* lower,
                        VectorXd* upper) const override {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Vector3d(bias, -kInf, -kInf);
    *upper = Vector3d::Constant(kInf);
  }
};

class ConstraintTestSystem : public LeafSystem<double> {
 public:
  ConstraintTestSystem() { DeclareContinuousState(2); }

  // Expose some protected methods for testing.
  using LeafSystem<double>::DeclareContinuousState;
  using LeafSystem<double>::DeclareDiscreteState;
  using LeafSystem<double>::DeclareEqualityConstraint;
  using LeafSystem<double>::DeclareInequalityConstraint;
  using LeafSystem<double>::DeclareNumericParameter;
  using LeafSystem<double>::DeclareVectorInputPort;
  using LeafSystem<double>::DeclareVectorOutputPort;

  void CalcState0Constraint(const Context<double>& context,
                            VectorXd* value) const {
    *value = Vector1d(context.get_continuous_state_vector()[0]);
  }
  void CalcStateConstraint(const Context<double>& context,
                           VectorXd* value) const {
    *value = context.get_continuous_state_vector().CopyToVector();
  }

  void CalcOutput(
      const Context<double>& context,
      ConstraintBasicVector<double, 44>* output) const {
    output->SetFromVector(VectorXd::Constant(output->size(), 4.0));
  }

 private:
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    // xdot = -x.
    derivatives->SetFromVector(-dynamic_cast<const BasicVector<double>&>(
                                    context.get_continuous_state_vector())
                                    .get_value());
  }
};

// Tests adding constraints implemented as methods inside the System class.
GTEST_TEST(SystemConstraintTest, ClassMethodTest) {
  ConstraintTestSystem dut;
  EXPECT_EQ(dut.num_constraints(), 0);

  EXPECT_EQ(dut.DeclareEqualityConstraint(
                &ConstraintTestSystem::CalcState0Constraint, 1, "x0"),
            0);
  EXPECT_EQ(dut.num_constraints(), 1);

  EXPECT_EQ(
      dut.DeclareInequalityConstraint(
          &ConstraintTestSystem::CalcStateConstraint,
          { Vector2d::Zero(), std::nullopt },
          "x"),
      1);
  EXPECT_EQ(dut.num_constraints(), 2);

  auto context = dut.CreateDefaultContext();
  context->get_mutable_continuous_state_vector().SetFromVector(
      Vector2d(5.0, 7.0));

  EXPECT_EQ(dut.get_constraint(SystemConstraintIndex(0)).size(), 1);
  EXPECT_EQ(dut.get_constraint(SystemConstraintIndex(1)).size(), 2);

  VectorXd value;
  dut.get_constraint(SystemConstraintIndex(0)).Calc(*context, &value);
  EXPECT_EQ(value.rows(), 1);
  EXPECT_EQ(value[0], 5.0);

  dut.get_constraint(SystemConstraintIndex(1)).Calc(*context, &value);
  EXPECT_EQ(value.rows(), 2);
  EXPECT_EQ(value[0], 5.0);
  EXPECT_EQ(value[1], 7.0);

  EXPECT_TRUE(
      dut.get_constraint(SystemConstraintIndex(0)).is_equality_constraint());
  EXPECT_EQ(dut.get_constraint(SystemConstraintIndex(0)).description(), "x0");

  EXPECT_FALSE(
      dut.get_constraint(SystemConstraintIndex(1)).is_equality_constraint());
  EXPECT_EQ(dut.get_constraint(SystemConstraintIndex(1)).description(), "x");
}

// Tests adding constraints implemented as function handles (lambda functions).
GTEST_TEST(SystemConstraintTest, FunctionHandleTest) {
  ConstraintTestSystem dut;
  EXPECT_EQ(dut.num_constraints(), 0);

  ContextConstraintCalc<double> calc0 = [](
      const Context<double>& context, VectorXd* value) {
    *value = Vector1d(context.get_continuous_state_vector()[1]);
  };
  EXPECT_EQ(dut.DeclareInequalityConstraint(calc0,
                                            { Vector1d::Zero(), std::nullopt },
                                            "x1_lower"),
            0);
  EXPECT_EQ(dut.num_constraints(), 1);

  ContextConstraintCalc<double> calc1 = [](
      const Context<double>& context, VectorXd* value) {
    *value =
        Vector2d(context.get_continuous_state_vector()[1],
                        context.get_continuous_state_vector()[0]);
  };
  EXPECT_EQ(dut.DeclareInequalityConstraint(calc1,
      { std::nullopt, Vector2d(2, 3) }, "x_upper"), 1);

  auto context = dut.CreateDefaultContext();
  context->get_mutable_continuous_state_vector().SetFromVector(
      Vector2d(5.0, 7.0));

  VectorXd value;
  const SystemConstraint<double>& inequality_constraint0 =
      dut.get_constraint(SystemConstraintIndex(0));
  inequality_constraint0.Calc(*context, &value);
  EXPECT_EQ(value.rows(), 1);
  EXPECT_EQ(value[0], 7.0);
  EXPECT_FALSE(inequality_constraint0.is_equality_constraint());
  EXPECT_EQ(inequality_constraint0.description(), "x1_lower");

  const SystemConstraint<double>& inequality_constraint1 =
      dut.get_constraint(SystemConstraintIndex(1));
  inequality_constraint1.Calc(*context, &value);
  EXPECT_EQ(value.rows(), 2);
  EXPECT_EQ(value[0], 7.0);
  EXPECT_EQ(value[1], 5.0);
  EXPECT_FALSE(inequality_constraint1.is_equality_constraint());
  EXPECT_EQ(inequality_constraint1.description(), "x_upper");

  EXPECT_EQ(dut.DeclareEqualityConstraint(calc0, 1, "x1eq"), 2);
  EXPECT_EQ(dut.num_constraints(), 3);

  const SystemConstraint<double>& equality_constraint =
      dut.get_constraint(SystemConstraintIndex(2));
  equality_constraint.Calc(*context, &value);
  EXPECT_EQ(value.rows(), 1);
  EXPECT_EQ(value[0], 7.0);
  EXPECT_TRUE(equality_constraint.is_equality_constraint());
  EXPECT_EQ(equality_constraint.description(), "x1eq");
}

// Tests constraints implied by BasicVector subtypes.
GTEST_TEST(SystemConstraintTest, ModelVectorTest) {
  ConstraintTestSystem dut;
  EXPECT_EQ(dut.num_constraints(), 0);

  // Declaring a constrained model vector parameter should add constraints.
  // We want `vec[0] >= 11` on the parameter vector.
  using ParameterVector = ConstraintBasicVector<double, 11>;
  dut.DeclareNumericParameter(ParameterVector{});
  ASSERT_EQ(dut.num_constraints(), 1);
  using Index = SystemConstraintIndex;
  const SystemConstraint<double>& constraint0 = dut.get_constraint(Index{0});
  const double kInf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(CompareMatrices(constraint0.lower_bound(), Vector1d(11)));
  EXPECT_TRUE(CompareMatrices(constraint0.upper_bound(), Vector1d(kInf)));
  EXPECT_FALSE(constraint0.is_equality_constraint());
  EXPECT_THAT(constraint0.description(), ::testing::ContainsRegex(
      "^parameter 0 of type .*ConstraintBasicVector<double,11>$"));

  // Declaring constrained model continuous state should add constraints.
  // We want `vec[0] >= 22` on the state vector.
  using StateVector = ConstraintBasicVector<double, 22>;
  dut.DeclareContinuousState(StateVector{}, 0, 0, StateVector::kSize);
  EXPECT_EQ(dut.num_constraints(), 2);
  const SystemConstraint<double>& constraint1 = dut.get_constraint(Index{1});
  EXPECT_TRUE(CompareMatrices(constraint1.lower_bound(), Vector1d(22)));
  EXPECT_TRUE(CompareMatrices(constraint1.upper_bound(), Vector1d(kInf)));
  EXPECT_FALSE(constraint1.is_equality_constraint());
  EXPECT_THAT(constraint1.description(), ::testing::ContainsRegex(
      "^continuous state of type .*ConstraintBasicVector<double,22>$"));

  // Declaring a constrained model vector input should add constraints.
  // We want `vec[0] >= 33` on the input vector.
  using InputVector = ConstraintBasicVector<double, 33>;
  dut.DeclareVectorInputPort(kUseDefaultName, InputVector{});
  EXPECT_EQ(dut.num_constraints(), 3);
  const SystemConstraint<double>& constraint2 = dut.get_constraint(Index{2});
  EXPECT_TRUE(CompareMatrices(constraint2.lower_bound(), Vector1d(33)));
  EXPECT_TRUE(CompareMatrices(constraint2.upper_bound(), Vector1d(kInf)));
  EXPECT_FALSE(constraint2.is_equality_constraint());
  EXPECT_THAT(constraint2.description(), ::testing::ContainsRegex(
      "^input 0 of type .*ConstraintBasicVector<double,33>$"));

  // Declaring a constrained model vector output should add constraints.
  // We want `vec[0] >= 44` on the output vector.
  dut.DeclareVectorOutputPort(
        kUseDefaultName, &ConstraintTestSystem::CalcOutput);
  EXPECT_EQ(dut.num_constraints(), 4);
  const SystemConstraint<double>& constraint3 = dut.get_constraint(Index{3});
  EXPECT_TRUE(CompareMatrices(constraint3.lower_bound(), Vector1d(44)));
  EXPECT_TRUE(CompareMatrices(constraint3.upper_bound(), Vector1d(kInf)));
  EXPECT_FALSE(constraint3.is_equality_constraint());
  EXPECT_THAT(constraint3.description(), ::testing::ContainsRegex(
      "^output 0 of type .*ConstraintBasicVector<double,44>$"));

  // Declaring constrained model discrete state should add constraints.
  // We want `vec[0] >= 55` on the state vector.
  using DiscreteStateVector = ConstraintBasicVector<double, 55>;
  dut.DeclareDiscreteState(DiscreteStateVector{});
  EXPECT_EQ(dut.num_constraints(), 5);
  const SystemConstraint<double>& constraint4 = dut.get_constraint(Index{4});
  EXPECT_TRUE(CompareMatrices(constraint4.lower_bound(), Vector1d(55)));
  EXPECT_TRUE(CompareMatrices(constraint4.upper_bound(), Vector1d(kInf)));
  EXPECT_FALSE(constraint4.is_equality_constraint());
  EXPECT_THAT(constraint4.description(), ::testing::ContainsRegex(
      "^discrete state of type .*ConstraintBasicVector<double,55>$"));

  // We'll work through the Calc results all at the end, so that we don't
  // change the shape of the System and Context while we're Calc'ing.
  auto context = dut.CreateDefaultContext();

  // `param0[0] >= 11.0` with `param0[0] == 1.0` produces `1.0 >= 11.0`.
  context->get_mutable_numeric_parameter(0)[0] = 1.0;
  VectorXd value0;
  constraint0.Calc(*context, &value0);
  EXPECT_TRUE(CompareMatrices(value0, Vector1<double>::Constant(1.0)));

  // `xc[0] >= 22.0` with `xc[0] == 2.0` produces `2.0 >= 22.0`.
  context->get_mutable_continuous_state_vector()[0] = 2.0;
  VectorXd value1;
  constraint1.Calc(*context, &value1);
  EXPECT_TRUE(CompareMatrices(value1, Vector1<double>::Constant(2.0)));

  // `u0[0] >= 33.0` with `u0[0] == 3.0` produces `3.0 >= 33.0`.
  InputVector input;
  input[0] = 3.0;
  dut.get_input_port(0).FixValue(&*context, input);
  VectorXd value2;
  constraint2.Calc(*context, &value2);
  EXPECT_TRUE(CompareMatrices(value2, Vector1<double>::Constant(3.0)));

  // `y0[0] >= 44.0` with `y0[0] == 4.0` produces `4.0 >= 44.0`.
  VectorXd value3;
  constraint3.Calc(*context, &value3);
  EXPECT_TRUE(CompareMatrices(value3, Vector1<double>::Constant(4.0)));
}

// Note: this class is duplicated in diagram_test.
class RandomContextTestSystem : public LeafSystem<double> {
 public:
  RandomContextTestSystem() {
    this->DeclareContinuousState(
        BasicVector<double>(Vector2d(-1.0, -2.0)));
    this->DeclareNumericParameter(
        BasicVector<double>(Vector3d(1.0, 2.0, 3.0)));
  }

  void SetRandomState(const Context<double>& context, State<double>* state,
                      RandomGenerator* generator) const override {
    std::normal_distribution<double> normal;
    for (int i = 0; i < context.get_continuous_state_vector().size(); i++) {
      state->get_mutable_continuous_state().get_mutable_vector().SetAtIndex(
          i, normal(*generator));
    }
  }
  void SetRandomParameters(const Context<double>& context,
                           Parameters<double>* params,
                           RandomGenerator* generator) const override {
    std::uniform_real_distribution<double> uniform;
    for (int i = 0; i < context.get_numeric_parameter(0).size(); i++) {
      params->get_mutable_numeric_parameter(0).SetAtIndex(
          i, uniform(*generator));
    }
  }
};

GTEST_TEST(RandomContextTest, SetRandomTest) {
  RandomContextTestSystem system;

  auto context = system.CreateDefaultContext();

  // Back-up the numeric context values.
  Vector2d state = context->get_continuous_state_vector().CopyToVector();
  Vector3d params = context->get_numeric_parameter(0).CopyToVector();

  // Should return the (same) original values.
  system.SetDefaultContext(context.get());
  EXPECT_TRUE((state.array() ==
               context->get_continuous_state_vector().CopyToVector().array())
                  .all());
  EXPECT_TRUE(
      (params.array() == context->get_numeric_parameter(0).get_value().array())
          .all());

  RandomGenerator generator;

  // Should return different values.
  system.SetRandomContext(context.get(), &generator);
  EXPECT_TRUE((state.array() !=
               context->get_continuous_state_vector().CopyToVector().array())
                  .all());
  EXPECT_TRUE(
      (params.array() != context->get_numeric_parameter(0).get_value().array())
          .all());

  // Update backup.
  state = context->get_continuous_state_vector().CopyToVector();
  params = context->get_numeric_parameter(0).CopyToVector();

  // Should return different values (again).
  system.SetRandomContext(context.get(), &generator);
  EXPECT_TRUE((state.array() !=
               context->get_continuous_state_vector().CopyToVector().array())
                  .all());
  EXPECT_TRUE(
      (params.array() != context->get_numeric_parameter(0).get_value().array())
          .all());
}

// Tests initialization works properly for a leaf system.
GTEST_TEST(InitializationTest, InitializationTest) {
  class InitializationTestSystem : public LeafSystem<double> {
   public:
    InitializationTestSystem() {
      PublishEvent<double> pub_event(
          std::bind(&InitializationTestSystem::InitPublish, this,
                    std::placeholders::_1, std::placeholders::_2));
      DeclareInitializationEvent(pub_event);

      DeclareInitializationEvent(DiscreteUpdateEvent<double>());
      DeclareInitializationEvent(UnrestrictedUpdateEvent<double>());
    }

    bool get_pub_init() const { return pub_init_; }
    bool get_dis_update_init() const { return dis_update_init_; }
    bool get_unres_update_init() const { return unres_update_init_; }

   private:
    void InitPublish(const Context<double>&,
                     const PublishEvent<double>& event) const {
      EXPECT_EQ(event.get_trigger_type(), TriggerType::kInitialization);
      pub_init_ = true;
    }

    void DoCalcDiscreteVariableUpdates(
        const Context<double>&,
        const std::vector<const DiscreteUpdateEvent<double>*>& events,
        DiscreteValues<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      EXPECT_EQ(events.front()->get_trigger_type(),
                TriggerType::kInitialization);
      dis_update_init_ = true;
    }

    void DoCalcUnrestrictedUpdate(
        const Context<double>&,
        const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
        State<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      EXPECT_EQ(events.front()->get_trigger_type(),
                TriggerType::kInitialization);
      unres_update_init_ = true;
    }

    mutable bool pub_init_{false};
    mutable bool dis_update_init_{false};
    mutable bool unres_update_init_{false};
  };

  InitializationTestSystem dut;
  auto context = dut.CreateDefaultContext();
  auto discrete_updates = dut.AllocateDiscreteVariables();
  auto state = context->CloneState();
  auto init_events = dut.AllocateCompositeEventCollection();
  EXPECT_EQ(init_events->get_system_id(), context->get_system_id());
  dut.GetInitializationEvents(*context, init_events.get());

  dut.Publish(*context, init_events->get_publish_events());
  dut.CalcDiscreteVariableUpdates(*context,
                                  init_events->get_discrete_update_events(),
                                  discrete_updates.get());
  dut.CalcUnrestrictedUpdate(
      *context, init_events->get_unrestricted_update_events(), state.get());

  EXPECT_TRUE(dut.get_pub_init());
  EXPECT_TRUE(dut.get_dis_update_init());
  EXPECT_TRUE(dut.get_unres_update_init());
}

// Although many of the tests above validate behavior of events when the
// event dispatchers DoPublish() etc are overridden, the preferred method
// for users is to provide individual handler functions for each event. There
// is a set of sugar methods to facilitate that; this class uses them all.
class EventSugarTestSystem : public LeafSystem<double> {
 public:
  EventSugarTestSystem() {
    DeclareInitializationPublishEvent(
        &EventSugarTestSystem::MyPublishHandler);
    DeclareInitializationDiscreteUpdateEvent(
        &EventSugarTestSystem::MyDiscreteUpdateHandler);
    DeclareInitializationUnrestrictedUpdateEvent(
        &EventSugarTestSystem::MyUnrestrictedUpdateHandler);

    DeclarePerStepPublishEvent(
        &EventSugarTestSystem::MyPublishHandler);
    DeclarePerStepDiscreteUpdateEvent(
        &EventSugarTestSystem::MyDiscreteUpdateHandler);
    DeclarePerStepUnrestrictedUpdateEvent(
        &EventSugarTestSystem::MyUnrestrictedUpdateHandler);

    DeclarePeriodicPublishEvent(kPeriod, kOffset,
        &EventSugarTestSystem::MyPublishHandler);
    DeclarePeriodicDiscreteUpdateEvent(kPeriod, kOffset,
        &EventSugarTestSystem::MyDiscreteUpdateHandler);
    DeclarePeriodicUnrestrictedUpdateEvent(kPeriod, kOffset,
        &EventSugarTestSystem::MyUnrestrictedUpdateHandler);

    // Two forced publish callbacks (to ensure that they are additive).
    DeclareForcedPublishEvent(&EventSugarTestSystem::MyPublishHandler);
    DeclareForcedPublishEvent(&EventSugarTestSystem::MySecondPublishHandler);

    // Two forced discrete update callbacks (to ensure that they are
    // additive).
    DeclareForcedDiscreteUpdateEvent(
        &EventSugarTestSystem::MyDiscreteUpdateHandler);
    DeclareForcedDiscreteUpdateEvent(
        &EventSugarTestSystem::MySecondDiscreteUpdateHandler);

    // Two forced unrestricted update callbacks (to ensure that they are
    // additive).
    DeclareForcedUnrestrictedUpdateEvent(
        &EventSugarTestSystem::MyUnrestrictedUpdateHandler);
    DeclareForcedUnrestrictedUpdateEvent(
        &EventSugarTestSystem::MySecondUnrestrictedUpdateHandler);

    // These variants don't require an EventStatus return.
    DeclarePeriodicPublishEvent(kPeriod, kOffset,
        &EventSugarTestSystem::MySuccessfulPublishHandler);
    DeclarePeriodicDiscreteUpdateEvent(kPeriod, kOffset,
        &EventSugarTestSystem::MySuccessfulDiscreteUpdateHandler);
    DeclarePeriodicUnrestrictedUpdateEvent(kPeriod, kOffset,
        &EventSugarTestSystem::MySuccessfulUnrestrictedUpdateHandler);
  }

  const double kPeriod = 0.125;
  const double kOffset = 0.25;

  int num_publish() const {return num_publish_;}
  int num_second_publish_handler_publishes() const {
    return num_second_publish_handler_publishes_;
  }
  int num_discrete_update() const {return num_discrete_update_;}
  int num_second_discrete_update() const {return num_second_discrete_update_;}
  int num_unrestricted_update() const {return num_unrestricted_update_;}
  int num_second_unrestricted_update() const {
    return num_second_unrestricted_update_;
  }

 private:
  EventStatus MyPublishHandler(const Context<double>& context) const {
    MySuccessfulPublishHandler(context);
    return EventStatus::Succeeded();
  }

  EventStatus MySecondPublishHandler(const Context<double>& context) const {
    ++num_second_publish_handler_publishes_;
    return EventStatus::Succeeded();
  }

  EventStatus MyDiscreteUpdateHandler(
      const Context<double>& context,
      DiscreteValues<double>* discrete_state) const {
    MySuccessfulDiscreteUpdateHandler(context, &*discrete_state);
    return EventStatus::Succeeded();
  }

  EventStatus MySecondDiscreteUpdateHandler(
      const Context<double>& context,
      DiscreteValues<double>* discrete_state) const {
    ++num_second_discrete_update_;
    return EventStatus::Succeeded();
  }

  EventStatus MyUnrestrictedUpdateHandler(const Context<double>& context,
                                          State<double>* state) const {
    MySuccessfulUnrestrictedUpdateHandler(context, &*state);
    return EventStatus::Succeeded();
  }

  EventStatus MySecondUnrestrictedUpdateHandler(const Context<double>& context,
                                                State<double>* state) const {
    ++num_second_unrestricted_update_;
    return EventStatus::Succeeded();
  }

  void MySuccessfulPublishHandler(const Context<double>&) const {
    ++num_publish_;
  }

  void MySuccessfulDiscreteUpdateHandler(const Context<double>&,
                                         DiscreteValues<double>*) const {
    ++num_discrete_update_;
  }

  void MySuccessfulUnrestrictedUpdateHandler(const Context<double>&,
                                             State<double>*) const {
    ++num_unrestricted_update_;
  }

  mutable int num_publish_{0};
  mutable int num_second_publish_handler_publishes_{0};
  mutable int num_discrete_update_{0};
  mutable int num_second_discrete_update_{0};
  mutable int num_unrestricted_update_{0};
  mutable int num_second_unrestricted_update_{0};
};

GTEST_TEST(EventSugarTest, EventsAreRegistered) {
  EventSugarTestSystem dut;
  auto context = dut.CreateDefaultContext();

  auto init_events = dut.AllocateCompositeEventCollection();
  dut.GetInitializationEvents(*context, &*init_events);
  EXPECT_TRUE(init_events->HasPublishEvents());
  EXPECT_TRUE(init_events->HasDiscreteUpdateEvents());
  EXPECT_TRUE(init_events->HasUnrestrictedUpdateEvents());

  auto per_step_events = dut.AllocateCompositeEventCollection();
  dut.GetPerStepEvents(*context, &*per_step_events);
  EXPECT_TRUE(per_step_events->HasPublishEvents());
  EXPECT_TRUE(per_step_events->HasDiscreteUpdateEvents());
  EXPECT_TRUE(per_step_events->HasUnrestrictedUpdateEvents());

  auto timed_events = dut.AllocateCompositeEventCollection();
  double next_event_time = dut.CalcNextUpdateTime(*context, &*timed_events);
  EXPECT_EQ(next_event_time, 0.25);
  EXPECT_TRUE(timed_events->HasPublishEvents());
  EXPECT_TRUE(timed_events->HasDiscreteUpdateEvents());
  EXPECT_TRUE(timed_events->HasUnrestrictedUpdateEvents());
}

GTEST_TEST(EventSugarTest, HandlersGetCalled) {
  EventSugarTestSystem dut;
  auto context = dut.CreateDefaultContext();
  auto discrete_state = dut.AllocateDiscreteVariables();
  auto state = context->CloneState();

  auto all_events = dut.AllocateCompositeEventCollection();
  dut.GetInitializationEvents(*context, &*all_events);

  auto per_step_events = dut.AllocateCompositeEventCollection();
  dut.GetPerStepEvents(*context, &*per_step_events);
  all_events->AddToEnd(*per_step_events);

  auto timed_events = dut.AllocateCompositeEventCollection();
  dut.CalcNextUpdateTime(*context, &*timed_events);
  all_events->AddToEnd(*timed_events);

  dut.CalcUnrestrictedUpdate(
      *context, all_events->get_unrestricted_update_events(), &*state);
  dut.CalcUnrestrictedUpdate(*context, &*state);
  dut.CalcDiscreteVariableUpdates(
      *context, all_events->get_discrete_update_events(), &*discrete_state);
  dut.CalcDiscreteVariableUpdates(*context, &*discrete_state);
  dut.Publish(*context, all_events->get_publish_events());
  dut.Publish(*context);

  EXPECT_EQ(dut.num_publish(), 5);
  EXPECT_EQ(dut.num_second_publish_handler_publishes(), 1);
  EXPECT_EQ(dut.num_discrete_update(), 5);
  EXPECT_EQ(dut.num_second_discrete_update(), 1);
  EXPECT_EQ(dut.num_unrestricted_update(), 5);
  EXPECT_EQ(dut.num_second_unrestricted_update(), 1);
}

// A System that does not override the default implicit time derivatives
// implementation.
class DefaultExplicitSystem : public LeafSystem<double> {
 public:
  DefaultExplicitSystem() { DeclareContinuousState(3); }

  void RedeclareResidualSize(int n) {
    DeclareImplicitTimeDerivativesResidualSize(n);
  }

  static Vector3d fixed_derivative() { return {1., 2., 3.}; }

 private:
  void DoCalcTimeDerivatives(const Context<double>& context,
                             ContinuousState<double>* derivatives) const final {
    derivatives->SetFromVector(fixed_derivative());
  }

  // No override for the implicit derivatives method.
};

// A System that _does_ override the default implicit time derivatives
// implementation, and also changes the residual size from its default.
class OverrideImplicitSystem : public DefaultExplicitSystem {
 public:
  OverrideImplicitSystem() {
    DeclareImplicitTimeDerivativesResidualSize(1);
  }

  void RedeclareResidualSize(int n) {
    DeclareImplicitTimeDerivativesResidualSize(n);
  }

 private:
  void DoCalcImplicitTimeDerivativesResidual(
    const systems::Context<double>& context,
    const systems::ContinuousState<double>& proposed_derivatives,
    EigenPtr<VectorX<double>> residual) const final {
    EXPECT_EQ(residual->size(), 1);
    (*residual)[0] = proposed_derivatives.CopyToVector().sum();
  }
};

GTEST_TEST(ImplicitTimeDerivatives, DefaultImplementation) {
  const Vector3d derivs = DefaultExplicitSystem::fixed_derivative();

  DefaultExplicitSystem dut;
  auto context = dut.CreateDefaultContext();
  auto xdot = dut.AllocateTimeDerivatives();

  // Check that SystemBase method returns the default size.
  EXPECT_EQ(dut.implicit_time_derivatives_residual_size(), 3);

  // Proposing the actual derivative should yield a zero residual.
  xdot->SetFromVector(derivs);

  // Make sure the vector type returned by the allocator works.
  VectorXd residual = dut.AllocateImplicitTimeDerivativesResidual();
  EXPECT_EQ(residual.size(), 3);
  dut.CalcImplicitTimeDerivativesResidual(*context, *xdot, &residual);
  EXPECT_EQ(residual, Vector3d::Zero());  // Yes, exactly.

  // To make sure the method isn't just returning zero, try the reverse
  // case which should have the actual derivative as a residual.
  xdot->SetFromVector(Vector3d::Zero());
  dut.CalcImplicitTimeDerivativesResidual(*context, *xdot, &residual);
  EXPECT_EQ(residual, -derivs);  // Exact.

  // The residual should be acceptable as long as it is some mutable
  // Eigen object of the right shape.

  // A fixed-size residual should work.
  Vector3d residual3;
  dut.CalcImplicitTimeDerivativesResidual(*context, *xdot, &residual3);
  EXPECT_EQ(residual3, -derivs);

  // Some complicated Eigen mutable object should also work.
  VectorXd buffer(VectorXd::Zero(5));
  auto segment = buffer.segment(1, 3);  // Some kind of block object.
  VectorXd expected_residual(VectorXd::Zero(5));
  expected_residual.segment(1, 3) = -derivs;
  dut.CalcImplicitTimeDerivativesResidual(*context, *xdot, &segment);
  EXPECT_EQ(buffer, expected_residual);

  // Let's change the declared residual size, pass in a matching vector,
  // and verify that the default implementation complains properly.
  dut.RedeclareResidualSize(4);
  Vector4d residual4;
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.CalcImplicitTimeDerivativesResidual(*context, *xdot, &residual4),
      std::logic_error,
      "System::DoCalcImplicitTimeDerivativesResidual.*"
      "default implementation requires.*residual size.*4.*"
      "matches.*state variables.*3.*must override.*");

  // And if the residual matches the state size but not the declared size,
  // we should get stopped by the NVI public method.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.CalcImplicitTimeDerivativesResidual(*context, *xdot, &residual3),
      std::logic_error,
      ".*CalcImplicitTimeDerivativesResidual.*"
      "expected residual.*size 4 but got.*size 3.*\n"
      "Use AllocateImplicitTimeDerivativesResidual.*");
}

GTEST_TEST(ImplicitTimeDerivatives, OverrideImplementation) {
  OverrideImplicitSystem dut;
  auto context = dut.CreateDefaultContext();
  auto xdot = dut.AllocateTimeDerivatives();
  EXPECT_EQ(xdot->size(), 3);

  // Check that SystemBase method returns the adjusted size.
  EXPECT_EQ(dut.implicit_time_derivatives_residual_size(), 1);

  // Make sure the vector size returned by the allocator reflects the change.
  VectorXd residual = dut.AllocateImplicitTimeDerivativesResidual();
  EXPECT_EQ(residual.size(), 1);
  residual[0] = 99.;  // So we can make sure it gets changed.

  // "Residual" just sums the proposed derivative xdot.
  xdot->SetFromVector(Vector3d(10., 20., 30.));
  dut.CalcImplicitTimeDerivativesResidual(*context, *xdot, &residual);
  EXPECT_EQ(residual[0], 60.);

  // (No need to re-test the acceptable Eigen output types as we did in
  // the previous test since both go through the same public NVI method.)

  // Check that a wrong-sized residual gets a nice message.
  Vector3d bad_residual;
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.CalcImplicitTimeDerivativesResidual(*context, *xdot, &bad_residual),
      std::logic_error,
      ".*CalcImplicitTimeDerivativesResidual.*"
      "expected residual.*size 1 but got.*size 3.*\n"
      "Use AllocateImplicitTimeDerivativesResidual.*");
}

// Check that declaring an illegal residual size just resets size back to the
// default (that is, the same size as the time derivatives).
GTEST_TEST(ImplicitTimeDerivatives, ResetToDefaultResidualSize) {
  OverrideImplicitSystem dut;
  EXPECT_EQ(dut.implicit_time_derivatives_residual_size(), 1);

  dut.RedeclareResidualSize(0);
  EXPECT_EQ(dut.implicit_time_derivatives_residual_size(), 3);

  dut.RedeclareResidualSize(1);  // Non-default.
  EXPECT_EQ(dut.implicit_time_derivatives_residual_size(), 1);

  dut.RedeclareResidualSize(-29);
  EXPECT_EQ(dut.implicit_time_derivatives_residual_size(), 3);
}

// Simulator::AdvanceTo() could miss an event that was triggered by a
// DoCalcNextUpdateTime() override that returned a finite next-trigger time but
// no Event object to handle the event. See issues #12620 and #14644.
//
// PR #14663 redefined this as an error -- an Event must be returned if there
// is a finite trigger time. That PR also replaced an assert with a real error
// message in the case the time is returned NaN; we check that here also.
//
// Note that this is really just a System test, but we need a LeafSystem
// to satisfy all the uninteresting pure virtuals.
GTEST_TEST(SystemTest, MissedEventIssue12620) {
  class TriggerTimeButNoEventSystem : public LeafSystem<double> {
   public:
    explicit TriggerTimeButNoEventSystem(double trigger_time)
        : trigger_time_(trigger_time) {
      this->set_name("MyTriggerSystem");
    }

   private:
    void DoCalcNextUpdateTime(const Context<double>& context,
                              CompositeEventCollection<double>*,
                              double* next_update_time) const final {
      *next_update_time = trigger_time_;
      // Don't push anything to the EventCollection.
    }

    const double trigger_time_;
  };

  // First test returns NaN, which should be detected.
  TriggerTimeButNoEventSystem nan_system(NAN);
  auto nan_context = nan_system.AllocateContext();
  auto events = nan_system.AllocateCompositeEventCollection();
  nan_context->SetTime(0.25);
  DRAKE_EXPECT_THROWS_MESSAGE(
      nan_system.CalcNextUpdateTime(*nan_context, events.get()),
      ".*CalcNextUpdateTime.*TriggerTimeButNoEventSystem.*MyTriggerSystem.*"
      "time=0.25.*no update time.*NaN.*Return infinity.*");

  // Second test returns a trigger time but no Event object.
  TriggerTimeButNoEventSystem trigger_system(0.375);
  auto trigger_context = trigger_system.AllocateContext();
  events = trigger_system.AllocateCompositeEventCollection();
  trigger_context->SetTime(0.25);
  DRAKE_EXPECT_THROWS_MESSAGE(
      trigger_system.CalcNextUpdateTime(*trigger_context, events.get()),
      ".*CalcNextUpdateTime.*TriggerTimeButNoEventSystem.*MyTriggerSystem.*"
      "time=0.25.*update time 0.375.*empty Event collection.*"
      "at least one Event object must be provided.*");
}

// Check that a DoCalcNextUpdateTime() override that fails to set the update
// time at all gets an error message. (This is detected as a returned time of
// NaN because calling code initializes the time that way.)
GTEST_TEST(SystemTest, ForgotToSetTheUpdateTime) {
  class ForgotToSetTimeSystem : public LeafSystem<double> {
   public:
    ForgotToSetTimeSystem() { this->set_name("MyForgetfulSystem"); }

   private:
    void DoCalcNextUpdateTime(const Context<double>& context,
                              CompositeEventCollection<double>*,
                              double* next_update_time) const final {
      // Oops -- forgot to set the time.
    }
  };

  ForgotToSetTimeSystem forgot_system;
  auto forgot_context = forgot_system.AllocateContext();
  auto events = forgot_system.AllocateCompositeEventCollection();
  forgot_context->SetTime(0.25);
  DRAKE_EXPECT_THROWS_MESSAGE(
      forgot_system.CalcNextUpdateTime(*forgot_context, events.get()),
      ".*CalcNextUpdateTime.*ForgotToSetTimeSystem.*MyForgetfulSystem.*"
      "time=0.25.*no update time.*NaN.*Return infinity.*");
}

}  // namespace
}  // namespace systems
}  // namespace drake
