#include "drake/systems/framework/leaf_system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

namespace drake {
namespace systems {
namespace {

// A shell System to test the default implementations.
template <typename T>
class TestSystem : public LeafSystem<T> {
 public:
  TestSystem() {
    this->set_name("TestSystem");
    this->DeclareNumericParameter(BasicVector<T>{13.0, 7.0});
    this->DeclareAbstractParameter(Value<std::string>("parameter value"));
  }
  ~TestSystem() override {}

  using LeafSystem<T>::DeclareContinuousState;

  void AddPeriodicUpdate() {
    const double period = 10.0;
    const double offset = 5.0;
    this->DeclarePeriodicDiscreteUpdate(period, offset);
    optional<PeriodicEventData> periodic_attr =
        this->GetUniquePeriodicDiscreteUpdateAttribute();
    ASSERT_TRUE(periodic_attr);
    EXPECT_EQ(periodic_attr.value().period_sec(), period);
    EXPECT_EQ(periodic_attr.value().offset_sec(), offset);
  }

  void AddPeriodicUpdate(double period) {
    const double offset = 0.0;
    this->DeclarePeriodicDiscreteUpdate(period, offset);
    optional<PeriodicEventData> periodic_attr =
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

  template <typename EventType>
  void AddPerStepEvent() {
    EventType event(Event<T>::TriggerType::kPerStep);
    this->DeclarePerStepEvent(event);
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {}

  const BasicVector<T>& GetVanillaNumericParameters(
      const Context<T>& context) const {
    return this->GetNumericParameter(context, 0 /* index */);
  }

  BasicVector<T>& GetVanillaMutableNumericParameters(
      Context<T>* context) const {
    return this->GetMutableNumericParameter(context, 0 /* index */);
  }

  // First testing type: no event specified.
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessWithoutEvent() const {
    return this->DeclareWitnessFunction(
        "dummy1", WitnessFunctionDirection::kCrossesZero,
        &TestSystem<double>::DummyWitnessFunction);
  }

  // Second testing type: event specified.
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessWithEvent() const {
    return this->DeclareWitnessFunction(
        "dummy2", WitnessFunctionDirection::kNone,
        &TestSystem<double>::DummyWitnessFunction,
        PublishEvent<double>());
  }

  // Third testing type: publish callback specified.
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessWithPublish() const {
    return this->DeclareWitnessFunction(
        "dummy3", WitnessFunctionDirection::kNone,
        &TestSystem<double>::DummyWitnessFunction,
        &TestSystem<double>::PublishCallback);
  }

  // Fourth testing type: discrete update callback specified.
  std::unique_ptr<WitnessFunction<T>> DeclareWitnessWithDiscreteUpdate() const {
    return this->DeclareWitnessFunction(
        "dummy4", WitnessFunctionDirection::kNone,
        &TestSystem<double>::DummyWitnessFunction,
        &TestSystem<double>::DiscreteUpdateCallback);
  }

  // Fifth testing type: unrestricted update callback specified.
  std::unique_ptr<WitnessFunction<T>>
      DeclareWitnessWithUnrestrictedUpdate() const {
    return this->DeclareWitnessFunction(
        "dummy5", WitnessFunctionDirection::kNone,
        &TestSystem<double>::DummyWitnessFunction,
        &TestSystem<double>::UnrestrictedUpdateCallback);
  }

  // Sixth testing type: lambda function with no event specified.
  std::unique_ptr<WitnessFunction<T>> DeclareLambdaWitnessWithoutEvent() const {
    return this->DeclareWitnessFunction(
        "dummy6", WitnessFunctionDirection::kCrossesZero,
        [](const Context<double>&) -> double { return 7.0; });
  }

  // Seventh testing type: lambda function with event specified.
  std::unique_ptr<WitnessFunction<T>>
      DeclareLambdaWitnessWithUnrestrictedUpdate() const {
    return this->DeclareWitnessFunction(
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

 private:
  // This dummy witness function exists only to test that the
  // DeclareWitnessFunction() interface works as promised.
  T DummyWitnessFunction(const Context<T>& context) const {
    static int call_counter = 0;
    return static_cast<T>(++call_counter);
  }

  // Publish callback function, which serves to test whether the appropriate
  // DeclareWitnessFunction() interface works as promised.
  void PublishCallback(const Context<T>&, const PublishEvent<T>&) const {
    publish_callback_called_ = true;
  }

  // Discrete update callback function, which serves to test whether the
  // appropriate DeclareWitnessFunction() interface works as promised.
  void DiscreteUpdateCallback(const Context<T>&,
      const DiscreteUpdateEvent<T>&, DiscreteValues<T>*) const {
    discrete_update_callback_called_ = true;
  }

  // Unrestricted update callback function, which serves to test whether the
  // appropriate DeclareWitnessFunction() interface works as promised.
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
  }

  TestSystem<double> system_;
  LeafContext<double> context_;

  std::unique_ptr<CompositeEventCollection<double>> event_info_;
  const LeafCompositeEventCollection<double>* leaf_info_;
};

// Tests that witness functions can be declared. Tests that witness functions
// stop Simulator at desired points (i.e., the raison d'etre of a witness
// function) are done in diagram_test.cc and
// drake/systems/analysis/test/simulator_test.cc.
TEST_F(LeafSystemTest, WitnessDeclarations) {
  auto witness1 = system_.DeclareWitnessWithoutEvent();
  ASSERT_TRUE(witness1);
  EXPECT_EQ(witness1->description(), "dummy1");
  EXPECT_EQ(witness1->direction_type(),
      WitnessFunctionDirection::kCrossesZero);
  EXPECT_FALSE(witness1->get_event());
  EXPECT_EQ(witness1->CalcWitnessValue(context_), 1.0);

  auto witness2 = system_.DeclareWitnessWithEvent();
  ASSERT_TRUE(witness2);
  EXPECT_EQ(witness2->description(), "dummy2");
  EXPECT_EQ(witness2->direction_type(), WitnessFunctionDirection::kNone);
  EXPECT_TRUE(witness2->get_event());
  EXPECT_EQ(witness2->CalcWitnessValue(context_), 2.0);

  auto witness3 = system_.DeclareWitnessWithPublish();
  ASSERT_TRUE(witness3);
  EXPECT_EQ(witness3->description(), "dummy3");
  EXPECT_EQ(witness3->direction_type(),
      WitnessFunctionDirection::kNone);
  EXPECT_TRUE(witness3->get_event());
  EXPECT_EQ(witness3->CalcWitnessValue(context_), 3.0);
  auto pe = dynamic_cast<const PublishEvent<double>*>(witness3->get_event());
  ASSERT_TRUE(pe);
  pe->handle(context_);
  EXPECT_TRUE(system_.publish_callback_called());

  auto witness4 = system_.DeclareWitnessWithDiscreteUpdate();
  ASSERT_TRUE(witness4);
  EXPECT_EQ(witness4->description(), "dummy4");
  EXPECT_EQ(witness4->direction_type(),
      WitnessFunctionDirection::kNone);
  EXPECT_TRUE(witness4->get_event());
  EXPECT_EQ(witness4->CalcWitnessValue(context_), 4.0);
  auto de = dynamic_cast<const DiscreteUpdateEvent<double>*>(
      witness4->get_event());
  ASSERT_TRUE(de);
  de->handle(context_, nullptr);
  EXPECT_TRUE(system_.discrete_update_callback_called());

  auto witness5 = system_.DeclareWitnessWithUnrestrictedUpdate();
  ASSERT_TRUE(witness5);
  EXPECT_EQ(witness5->description(), "dummy5");
  EXPECT_EQ(witness5->direction_type(),
      WitnessFunctionDirection::kNone);
  EXPECT_TRUE(witness5->get_event());
  EXPECT_EQ(witness5->CalcWitnessValue(context_), 5.0);
  auto ue = dynamic_cast<const UnrestrictedUpdateEvent<double>*>(
      witness5->get_event());
  ASSERT_TRUE(ue);
  ue->handle(context_, nullptr);
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
  EXPECT_EQ(witness7->CalcWitnessValue(context_), 11.0);
  ue = dynamic_cast<const UnrestrictedUpdateEvent<double>*>(
      witness7->get_event());
  ASSERT_TRUE(ue);
}

// Tests that if no update events are configured, none are reported.
TEST_F(LeafSystemTest, NoUpdateEvents) {
  context_.set_time(25.0);
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
}


// Tests that if the current time is smaller than the offset, the next
// update time is the offset.
TEST_F(LeafSystemTest, OffsetHasNotArrivedYet) {
  context_.set_time(2.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(5.0, time);
  const auto& events = leaf_info_->get_discrete_update_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(),
            Event<double>::TriggerType::kPeriodic);
}

// Tests that if the current time is smaller than the offset, the next
// update time is the offset, DiscreteUpdate and UnrestrictedUpdate happen
// at the same time.
TEST_F(LeafSystemTest, EventsAtTheSameTime) {
  context_.set_time(2.0);
  // Both actions happen at t = 5.
  system_.AddPeriodicUpdate();
  system_.AddPeriodicUnrestrictedUpdate(3, 5);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(5.0, time);
  {
    const auto& events = leaf_info_->get_discrete_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(),
              Event<double>::TriggerType::kPeriodic);
  }
  {
    const auto& events =
        leaf_info_->get_unrestricted_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(),
              Event<double>::TriggerType::kPeriodic);
  }
}

// Tests that if the current time is exactly the offset, the next
// update time is in the future.
TEST_F(LeafSystemTest, ExactlyAtOffset) {
  context_.set_time(5.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(15.0, time);
  const auto& events = leaf_info_->get_discrete_update_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(),
            Event<double>::TriggerType::kPeriodic);
}

// Tests that if the current time is larger than the offset, the next
// update time is determined by the period.
TEST_F(LeafSystemTest, OffsetIsInThePast) {
  context_.set_time(23.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(25.0, time);
  const auto& events = leaf_info_->get_discrete_update_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(),
            Event<double>::TriggerType::kPeriodic);
}

// Tests that if the current time is exactly an update time, the next update
// time is in the future.
TEST_F(LeafSystemTest, ExactlyOnUpdateTime) {
  context_.set_time(25.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(35.0, time);
  const auto& events = leaf_info_->get_discrete_update_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(),
            Event<double>::TriggerType::kPeriodic);
}

// Tests periodic events' scheduling when its offset is zero.
TEST_F(LeafSystemTest, PeriodicUpdateZeroOffset) {
  system_.AddPeriodicUpdate(2.0);

  context_.set_time(0.0);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(2.0, time);

  context_.set_time(1.0);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(2.0, time);

  context_.set_time(2.1);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(4.0, time);
}

// Tests that if a LeafSystem has both a discrete update and a periodic Publish,
// the update actions are computed appropriately.
TEST_F(LeafSystemTest, UpdateAndPublish) {
  system_.AddPeriodicUpdate(15.0);
  system_.AddPublish(12.0);

  // The publish event fires at 12sec.
  context_.set_time(9.0);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(12.0, time);
  {
    const auto& events = leaf_info_->get_publish_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(),
              Event<double>::TriggerType::kPeriodic);
  }

  // The update event fires at 15sec.
  context_.set_time(14.0);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(15.0, time);
  {
    const auto& events = leaf_info_->get_discrete_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(),
              Event<double>::TriggerType::kPeriodic);
  }

  // Both events fire at 60sec.
  context_.set_time(59.0);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(60.0, time);
  {
    const auto& events = leaf_info_->get_discrete_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(),
              Event<double>::TriggerType::kPeriodic);
  }
  {
    const auto& events = leaf_info_->get_publish_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(),
              Event<double>::TriggerType::kPeriodic);
  }
}

// Tests that if the integrator has stopped on the k-th sample, and the current
// time for that sample is slightly less than k * period due to floating point
// rounding, the next sample time is (k + 1) * period.
TEST_F(LeafSystemTest, FloatingPointRoundingZeroPointZeroOneFive) {
  context_.set_time(0.015 * 11);  // Slightly less than 0.165.
  system_.AddPeriodicUpdate(0.015);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  // 0.015 * 12 = 0.18.
  EXPECT_NEAR(0.18, time, 1e-8);
}

// Tests that if the integrator has stopped on the k-th sample, and the current
// time for that sample is slightly less than k * period due to floating point
// rounding, the next sample time is (k + 1) * period.
TEST_F(LeafSystemTest, FloatingPointRoundingZeroPointZeroZeroTwoFive) {
  context_.set_time(0.0025 * 977);  // Slightly less than 2.4425
  system_.AddPeriodicUpdate(0.0025);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_NEAR(2.445, time, 1e-8);
}

// Tests that the leaf system reserved the declared Parameters with default
// values, and that they are modifiable.
TEST_F(LeafSystemTest, NumericParameters) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const BasicVector<double>& vec =
      system_.GetVanillaNumericParameters(*context);
  EXPECT_EQ(13.0, vec[0]);
  EXPECT_EQ(7.0, vec[1]);
  BasicVector<double>& mutable_vec =
      system_.GetVanillaMutableNumericParameters(context.get());
  mutable_vec.SetAtIndex(1, 42.0);
  EXPECT_EQ(42.0, vec[1]);
}

// Tests that the leaf system reserved the declared abstract Parameters with
// default values, and that they are modifiable.
TEST_F(LeafSystemTest, AbstractParameters) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const std::string& param = context->get_abstract_parameter(0 /*index*/)
                                 .GetValueOrThrow<std::string>();
  EXPECT_EQ(param, "parameter value");
  std::string& mutable_param =
      context->get_mutable_abstract_parameter(0 /*index*/)
          .GetMutableValueOrThrow<std::string>();
  mutable_param = "modified parameter value";
  EXPECT_EQ("modified parameter value", param);
}

// Tests that the leaf system reserved the declared misc continuous state.
TEST_F(LeafSystemTest, DeclareVanillaMiscContinuousState) {
  system_.DeclareContinuousState(2);
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
  using MyVector9d = MyVector<4 + 3 + 2, double>;
  system_.DeclareContinuousState(MyVector9d(), 4, 3, 2);
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
}

TEST_F(LeafSystemTest, DeclarePerStepEvents) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();

  system_.AddPerStepEvent<PublishEvent<double>>();
  system_.AddPerStepEvent<DiscreteUpdateEvent<double>>();
  system_.AddPerStepEvent<UnrestrictedUpdateEvent<double>>();

  system_.GetPerStepEvents(*context, event_info_.get());

  {
    const auto& events = leaf_info_->get_publish_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(),
              Event<double>::TriggerType::kPerStep);
  }
  {
    const auto& events = leaf_info_->get_discrete_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(),
              Event<double>::TriggerType::kPerStep);
  }
  {
    const auto& events =
        leaf_info_->get_unrestricted_update_events().get_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger_type(),
              Event<double>::TriggerType::kPerStep);
  }
}

// A system that exercises the model_value-based input and output ports,
// as well as model-declared params.
class DeclaredModelPortsSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeclaredModelPortsSystem);

  DeclaredModelPortsSystem() {
    this->DeclareInputPort(kVectorValued, 1);
    this->DeclareVectorInputPort(MyVector2d());
    this->DeclareAbstractInputPort(Value<int>(22));
    this->DeclareVectorInputPort(MyVector2d(),
                                 RandomDistribution::kUniform);
    this->DeclareVectorInputPort(MyVector2d(),
                                 RandomDistribution::kGaussian);

    // Output port 0 uses a BasicVector base class model.
    this->DeclareVectorOutputPort(BasicVector<double>(3),
                                  &DeclaredModelPortsSystem::CalcBasicVector3);
    // Output port 1 uses a class derived from BasicVector.
    this->DeclareVectorOutputPort(MyVector4d(),
                                  &DeclaredModelPortsSystem::CalcMyVector4d);

    // Output port 2 uses a concrete string model.
    this->DeclareAbstractOutputPort(std::string("45"),
                                    &DeclaredModelPortsSystem::CalcString);

    // Output port 3 uses the "Advanced" methods that take a model
    // and a general calc function rather than a calc method.
    this->DeclareVectorOutputPort(
        BasicVector<double>(2),
        [](const Context<double>&, BasicVector<double>* out) {
          ASSERT_NE(out, nullptr);
          EXPECT_EQ(out->size(), 2);
          out->SetAtIndex(0, 10.);
          out->SetAtIndex(1, 20.);
        });

    this->DeclareNumericParameter(*MyVector2d::Make(1.1, 2.2));
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
    out->GetMutableValueOrThrow<std::string>() = "abstract string";
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
// correct topology.
GTEST_TEST(ModelLeafSystemTest, ModelPortsTopology) {
  DeclaredModelPortsSystem dut;

  ASSERT_EQ(dut.get_num_input_ports(), 5);
  ASSERT_EQ(dut.get_num_output_ports(), 4);

  const InputPortDescriptor<double>& in0 = dut.get_input_port(0);
  const InputPortDescriptor<double>& in1 = dut.get_input_port(1);
  const InputPortDescriptor<double>& in2 = dut.get_input_port(2);
  const InputPortDescriptor<double>& in3 = dut.get_input_port(3);
  const InputPortDescriptor<double>& in4 = dut.get_input_port(4);

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
  EXPECT_NO_THROW(downcast_input2 = input2->GetValueOrThrow<int>());
  EXPECT_EQ(downcast_input2, 22);
}

// Tests that Declare{Vector,Abstract}OutputPort flow through to allocating the
// correct values.
GTEST_TEST(ModelLeafSystemTest, ModelPortsAllocOutput) {
  DeclaredModelPortsSystem dut;
  auto context = dut.CreateDefaultContext();
  auto system_output = dut.AllocateOutput(*context);

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
  EXPECT_NO_THROW(downcast_output2 = output2->GetValueOrThrow<std::string>());
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

  std::vector<std::unique_ptr<AbstractValue>> values;
  for (int i = 0; i < 4; ++i) {
    const OutputPort<double>& out = dut.get_output_port(i);
    values.emplace_back(out.Allocate());
    out.Calc(*context, values.back().get());
  }

  // Downcast to concrete types.
  const BasicVector<double>* vec0{};
  const MyVector4d* vec1{};
  const std::string* str2{};
  const BasicVector<double>* vec3{};
  EXPECT_NO_THROW(vec0 = &values[0]->GetValueOrThrow<BasicVector<double>>());
  EXPECT_NO_THROW(vec1 = dynamic_cast<const MyVector4d*>(
                      &values[1]->GetValueOrThrow<BasicVector<double>>()));
  EXPECT_NO_THROW(str2 = &values[2]->GetValueOrThrow<std::string>());
  EXPECT_NO_THROW(vec3 = &values[3]->GetValueOrThrow<BasicVector<double>>());

  // Check the calculated values.
  EXPECT_EQ(vec0->get_value(), dut.expected_basic().get_value());
  EXPECT_EQ(vec1->get_value(), dut.expected_myvector().get_value());
  EXPECT_EQ(*str2, "concrete string");
  EXPECT_EQ(vec3->get_value(), Eigen::Vector2d(10., 20.));
}

// Tests that the leaf system reserved the declared parameters of interesting
// custom type.
GTEST_TEST(ModelLeafSystemTest, ModelNumericParams) {
  DeclaredModelPortsSystem dut;
  auto context = dut.CreateDefaultContext();
  ASSERT_EQ(context->num_numeric_parameters(), 1);
  const BasicVector<double>& param = context->get_numeric_parameter(0);
  // Check that type was preserved.
  ASSERT_TRUE(is_dynamic_castable<const MyVector2d>(&param));
  EXPECT_EQ(2, param.size());
  EXPECT_EQ(1.1, param.GetAtIndex(0));
  EXPECT_EQ(2.2, param.GetAtIndex(1));
}

// Tests that DeclareAbstractState works expectedly.
GTEST_TEST(ModelLeafSystemTest, ModelAbstractState) {
  class DeclaredModelAbstractStateSystem : public LeafSystem<double> {
   public:
    DeclaredModelAbstractStateSystem() {
      DeclareAbstractState(AbstractValue::Make<int>(1));
      DeclareAbstractState(AbstractValue::Make<std::string>("wow"));
    }
  };

  DeclaredModelAbstractStateSystem dut;

  // Allocate the resources that were created on system construction.
  auto context = dut.AllocateContext();

  // Check that the allocations were made and with the correct type
  EXPECT_NO_THROW(context->get_abstract_state<int>(0));
  EXPECT_NO_THROW(context->get_abstract_state<std::string>(1));

  // Mess with the abstract values on the context.
  drake::systems::AbstractValues& values =
      context->get_mutable_abstract_state();
  drake::systems::AbstractValue& value = values.get_mutable_value(1);
  EXPECT_NO_THROW(value.SetValue<std::string>("whoops"));
  EXPECT_EQ(context->get_abstract_state<std::string>(1), "whoops");

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
    // Output port 0 default-constructs a class derived from BasicVector as
    // its allocator.
    this->DeclareVectorOutputPort(&DeclaredNonModelOutputSystem::CalcDummyVec2);
    // Output port 1 default-constructs a string as its allocator.
    this->DeclareAbstractOutputPort(&DeclaredNonModelOutputSystem::CalcString);

    // Output port 2 uses the "Advanced" method for abstract ports, providing
    // explicit non-member functors for allocator and calculator.
    this->DeclareAbstractOutputPort(
        []() {
          return AbstractValue::Make<int>(-2);
        },
        [](const Context<double>&, AbstractValue* out) {
          ASSERT_NE(out, nullptr);
          int* int_out{};
          EXPECT_NO_THROW(int_out = &out->GetMutableValueOrThrow<int>());
          *int_out = 321;
        });

    // Output port 3 is declared with a commonly-used signature taking
    // methods for both allocator and calculator for an abstract port.
    this->DeclareAbstractOutputPort(&DeclaredNonModelOutputSystem::MakeString,
                                    &DeclaredNonModelOutputSystem::CalcString);

    // Output port 4 uses a default-constructed bare struct which should be
    // value-initialized.
    this->DeclareAbstractOutputPort(&DeclaredNonModelOutputSystem::CalcPOD);
  }

 private:
  void CalcDummyVec2(const Context<double>&, DummyVec2* out) const {
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->size(), 2);
    out->get_mutable_value() = Eigen::Vector2d(-100., -200);
  }

  // Explicit allocator method.
  std::string MakeString() const {
    return std::string("freshly made");
  }

  void CalcString(const Context<double>&, std::string* out) const {
    ASSERT_NE(out, nullptr);
    *out = "calc'ed string";
  }

  void CalcPOD(const Context<double>&, SomePOD* out) const {
    ASSERT_NE(out, nullptr);
    *out = {-10, 3.25};
  }
};

// Tests that non-model based Declare{Vector,Abstract}OutputPort generate
// the expected output port allocators, and that their calc functions work.
GTEST_TEST(NonModelLeafSystemTest, NonModelPortsOutput) {
  DeclaredNonModelOutputSystem dut;
  auto context = dut.CreateDefaultContext();
  auto system_output = dut.AllocateOutput(*context);  // Invokes all allocators.

  // Check topology.
  EXPECT_EQ(dut.get_num_input_ports(), 0);
  EXPECT_EQ(dut.get_num_output_ports(), 5);

  auto& out0 = dut.get_output_port(0);
  auto& out1 = dut.get_output_port(1);
  auto& out2 = dut.get_output_port(2);
  auto& out3 = dut.get_output_port(3);
  auto& out4 = dut.get_output_port(4);
  EXPECT_EQ(out0.get_data_type(), kVectorValued);
  EXPECT_EQ(out1.get_data_type(), kAbstractValued);
  EXPECT_EQ(out2.get_data_type(), kAbstractValued);
  EXPECT_EQ(out3.get_data_type(), kAbstractValued);
  EXPECT_EQ(out4.get_data_type(), kAbstractValued);

  // Check that DummyVec2 came out, default constructed to (100,200).
  auto output0 = system_output->GetMutableVectorData(0);
  ASSERT_NE(output0, nullptr);
  EXPECT_EQ(output0->size(), 2);
  auto out0_dummy = dynamic_cast<DummyVec2*>(output0);
  EXPECT_NE(out0_dummy, nullptr);
  EXPECT_EQ(out0_dummy->get_value(), Eigen::Vector2d(100., 200.));
  out0.Calc(*context, system_output->GetMutableData(0));
  EXPECT_EQ(out0_dummy->get_value(), Eigen::Vector2d(-100., -200.));

  // Check that Value<string>() came out, default initialized to empty.
  auto output1 = system_output->GetMutableData(1);
  ASSERT_NE(output1, nullptr);
  const std::string* downcast_output1{};
  EXPECT_NO_THROW(downcast_output1 = &output1->GetValueOrThrow<std::string>());
  EXPECT_TRUE(downcast_output1->empty());
  out1.Calc(*context, output1);
  EXPECT_EQ(*downcast_output1, "calc'ed string");

  // Check that Value<int> came out, default initialized to -2.
  auto output2 = system_output->GetMutableData(2);
  ASSERT_NE(output2, nullptr);
  const int* downcast_output2{};
  EXPECT_NO_THROW(downcast_output2 = &output2->GetValueOrThrow<int>());
  EXPECT_EQ(*downcast_output2, -2);
  out2.Calc(*context, output2);
  EXPECT_EQ(*downcast_output2, 321);

  // Check that Value<string>() came out, custom initialized.
  auto output3 = system_output->GetMutableData(3);
  ASSERT_NE(output3, nullptr);
  const std::string* downcast_output3{};
  EXPECT_NO_THROW(downcast_output3 = &output3->GetValueOrThrow<std::string>());
  EXPECT_EQ(*downcast_output3, "freshly made");
  out3.Calc(*context, output3);
  EXPECT_EQ(*downcast_output3, "calc'ed string");

  // Check that Value<SomePOD>{} came out, value initialized. Note that this
  // is not a perfect test since the values *could* come out zero by accident
  // even if the value initializer had not been called. Better than nothing!
  auto output4 = system_output->GetMutableData(4);
  ASSERT_NE(output4, nullptr);
  const SomePOD* downcast_output4{};
  EXPECT_NO_THROW(downcast_output4 = &output4->GetValueOrThrow<SomePOD>());
  EXPECT_EQ(downcast_output4->some_int, 0);
  EXPECT_EQ(downcast_output4->some_double, 0.0);
  out4.Calc(*context, output4);
  EXPECT_EQ(downcast_output4->some_int, -10);
  EXPECT_EQ(downcast_output4->some_double, 3.25);
}

// Tests both that an unrestricted update callback is called and that
// modifications to state dimension are caught.
TEST_F(LeafSystemTest, CallbackAndInvalidUpdates) {
  // Create 9, 1, and 3 dimensional continuous, discrete, and abstract state
  // vectors.
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  context->set_continuous_state(std::make_unique<ContinuousState<double>>(
      std::make_unique<BasicVector<double>>(9), 3, 3, 3));
  context->set_discrete_state(std::make_unique<DiscreteValues<double>>(
      std::make_unique<BasicVector<double>>(1)));
  std::vector<std::unique_ptr<AbstractValue>> abstract_data;
  abstract_data.push_back(PackValue(3));
  abstract_data.push_back(PackValue(5));
  abstract_data.push_back(PackValue(7));
  context->set_abstract_state(
      std::make_unique<AbstractValues>(std::move(abstract_data)));

  // Copy the state.
  std::unique_ptr<State<double>> x = context->CloneState();

  // Create an unrestricted update callback that just copies the state.
  LeafCompositeEventCollection<double> leaf_events;
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback = [](
        const Context<double>& c, const Event<double>&, State<double>* s) {
      s->CopyFrom(*c.CloneState());
    };

    UnrestrictedUpdateEvent<double> event(Event<double>::TriggerType::kPeriodic,
                                          callback);

    event.add_to_composite(&leaf_events);
  }

  // Verify no exception is thrown.
  EXPECT_NO_THROW(system_.CalcUnrestrictedUpdate(
      *context, leaf_events.get_unrestricted_update_events(), x.get()));

  // Change the function to change the continuous state dimension.
  // Call the unrestricted update function again, now verifying that an
  // exception is thrown.
  leaf_events.Clear();
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback = [](
        const Context<double>& c, const Event<double>&, State<double>* s) {
      s->CopyFrom(*c.CloneState());
      s->set_continuous_state(std::make_unique<ContinuousState<double>>(
          std::make_unique<BasicVector<double>>(4), 4, 0, 0));
    };

    UnrestrictedUpdateEvent<double> event(Event<double>::TriggerType::kPeriodic,
                                          callback);

    event.add_to_composite(&leaf_events);
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
      s->CopyFrom(*c.CloneState());
      disc_data.push_back(std::make_unique<BasicVector<double>>(1));
      disc_data.push_back(std::make_unique<BasicVector<double>>(1));
      s->set_discrete_state(
          std::make_unique<DiscreteValues<double>>(std::move(disc_data)));
    };

    UnrestrictedUpdateEvent<double> event(Event<double>::TriggerType::kPeriodic,
                                          callback);

    event.add_to_composite(&leaf_events);
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
      s->CopyFrom(*c.CloneState());
      s->set_abstract_state(std::make_unique<AbstractValues>());
    };

    UnrestrictedUpdateEvent<double> event(Event<double>::TriggerType::kPeriodic,
                                          callback);

    event.add_to_composite(&leaf_events);
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
  LeafContext<AutoDiffXd> context;

  context.set_time(21.0);
  system.AddPeriodicUpdate();

  auto event_info = system.AllocateCompositeEventCollection();
  auto time = system.CalcNextUpdateTime(context, event_info.get());

  EXPECT_EQ(25.0, time);
}

// A LeafSystem that uses the default, conservative direct-feedthrough
// implementation, informed by neither symbolic sparsity analysis nor manual
// sparsity declarations.
class DefaultFeedthroughSystem : public LeafSystem<double> {
 public:
  DefaultFeedthroughSystem() {}

  ~DefaultFeedthroughSystem() override {}

  void AddAbstractInputPort() { this->DeclareAbstractInputPort(); }

  void AddAbstractOutputPort() {
    this->DeclareAbstractOutputPort(nullptr, nullptr);  // No alloc or calc.
  }
};

GTEST_TEST(FeedthroughTest, DefaultWithNoInputsOrOutputs) {
  DefaultFeedthroughSystem system;
  EXPECT_FALSE(system.HasAnyDirectFeedthrough());
  // No ports implies no pairs reported for direct feedthrough.
  std::multimap<int, int> expected;
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
  std::multimap<int, int> expected;
  expected.emplace(0, 0);
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);
}

GTEST_TEST(FeedthroughTest, DefaultWithInputsOnly) {
  DefaultFeedthroughSystem system;
  system.AddAbstractInputPort();
  EXPECT_FALSE(system.HasAnyDirectFeedthrough());
  // No output ports implies no pairs reported for direct feedthrough.
  std::multimap<int, int> expected;
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);
}

GTEST_TEST(FeedthroughTest, DefaultWithOutputsOnly) {
  DefaultFeedthroughSystem system;
  system.AddAbstractOutputPort();
  EXPECT_FALSE(system.HasAnyDirectFeedthrough());
  EXPECT_FALSE(system.HasDirectFeedthrough(0));
  // No input ports implies no pairs reported for direct feedthrough.
  std::multimap<int, int> expected;
  EXPECT_EQ(system.GetDirectFeedthroughs(), expected);
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

// A MIMO system with manually-configured direct feedthrough properties: input
// 0 affects only output 1, and input 1 affects only output 0.
class ManualSparsitySystem : public DefaultFeedthroughSystem {
 public:
  ManualSparsitySystem() {
    this->AddAbstractInputPort();
    this->AddAbstractInputPort();
    this->AddAbstractOutputPort();
    this->AddAbstractOutputPort();
  }

 protected:
  optional<bool> DoHasDirectFeedthrough(
      int input_port, int output_port) const override {
    if (input_port == 0 && output_port == 1) {
      return true;
    }
    if (input_port == 1 && output_port == 0) {
      return true;
    }
    return false;
  }
};

GTEST_TEST(FeedthroughTest, ManualSparsity) {
  ManualSparsitySystem system;
  // Both the output ports have direct feedthrough from some input.
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  EXPECT_TRUE(system.HasDirectFeedthrough(0));
  EXPECT_TRUE(system.HasDirectFeedthrough(1));
  // Check the entire matrix.
  EXPECT_FALSE(system.HasDirectFeedthrough(0, 0));
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 1));
  EXPECT_TRUE(system.HasDirectFeedthrough(1, 0));
  EXPECT_FALSE(system.HasDirectFeedthrough(1, 1));
  // Confirm all pairs are returned.
  std::multimap<int, int> expected;
  expected.emplace(1, 0);
  expected.emplace(0, 1);
  auto feedthrough_pairs = system.GetDirectFeedthroughs();
  EXPECT_EQ(feedthrough_pairs, expected);
}

// SymbolicSparsitySystem has the same sparsity matrix as ManualSparsitySystem,
// but the matrix can be inferred from the symbolic form.
template <typename T>
class SymbolicSparsitySystem : public LeafSystem<T> {
 public:
  SymbolicSparsitySystem()
      : LeafSystem<T>(SystemTypeTag<systems::SymbolicSparsitySystem>{}) {
    this->DeclareInputPort(kVectorValued, kSize);
    this->DeclareInputPort(kVectorValued, kSize);

    this->DeclareVectorOutputPort(BasicVector<T>(kSize),
                                  &SymbolicSparsitySystem::CalcY0);
    this->DeclareVectorOutputPort(BasicVector<T>(kSize),
                                  &SymbolicSparsitySystem::CalcY1);
  }

  template <typename U>
  SymbolicSparsitySystem(const SymbolicSparsitySystem<U>&)
      : SymbolicSparsitySystem<T>() {}

 private:
  void CalcY0(const Context<T>& context,
                    BasicVector<T>* y0) const {
    const auto& u1 = *(this->EvalVectorInput(context, 1));
    y0->set_value(u1.get_value());
  }

  void CalcY1(const Context<T>& context,
              BasicVector<T>* y1) const {
    const auto& u0 = *(this->EvalVectorInput(context, 0));
    y1->set_value(u0.get_value());
  }

  const int kSize = 1;
};

GTEST_TEST(FeedthroughTest, SymbolicSparsity) {
  SymbolicSparsitySystem<double> system;
  // Both the output ports have direct feedthrough from some input.
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  EXPECT_TRUE(system.HasDirectFeedthrough(0));
  EXPECT_TRUE(system.HasDirectFeedthrough(1));
  // Check the entire matrix.
  EXPECT_FALSE(system.HasDirectFeedthrough(0, 0));
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 1));
  EXPECT_TRUE(system.HasDirectFeedthrough(1, 0));
  EXPECT_FALSE(system.HasDirectFeedthrough(1, 1));
  // Confirm all pairs are returned.
  std::multimap<int, int> expected;
  expected.emplace(1, 0);
  expected.emplace(0, 1);
  auto feedthrough_pairs = system.GetDirectFeedthroughs();
  EXPECT_EQ(feedthrough_pairs, expected);
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

  // Instance method that reports failures via exception.
  EXPECT_NE(dut.ToAutoDiffXd(), nullptr);

  // Instance method that reports failures via nullptr.
  auto maybe = dut.ToAutoDiffXdMaybe();
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

  // Instance method that reports failures via exception.
  EXPECT_THROW(dut.ToAutoDiffXd(), std::exception);

  // Instance method that reports failures via nullptr.
  EXPECT_EQ(dut.ToAutoDiffXdMaybe(), nullptr);
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

  // Instance method that reports failures via exception.
  EXPECT_NE(dut.ToSymbolic(), nullptr);

  // Instance method that reports failures via nullptr.
  auto maybe = dut.ToSymbolicMaybe();
  ASSERT_NE(maybe, nullptr);
  EXPECT_EQ(maybe->get_name(), "special_name");
}

// Sanity check the default implementation of ToSymbolic, for cases that
// should fail.
GTEST_TEST(LeafSystemScalarConverterTest, SymbolicNo) {
  TestSystem<double> dut;

  // Static method.
  EXPECT_THROW(System<double>::ToSymbolic(dut), std::exception);

  // Instance method that reports failures via exception.
  EXPECT_THROW(dut.ToSymbolic(), std::exception);

  // Instance method that reports failures via nullptr.
  EXPECT_EQ(dut.ToSymbolicMaybe(), nullptr);
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

// This system schedules two simultaneous publish events with
// GetPerStepEvents(). Both events have abstract data, but of different types.
// Both events have different custom handler callbacks. And DoPublish() is also
// overridden.
class TestTriggerSystem : public LeafSystem<double> {
 public:
  TestTriggerSystem() {}

  void DoPublish(
      const Context<double>& context,
      const std::vector<const PublishEvent<double>*>& events) const override {
    for (const PublishEvent<double>* event : events) {
      if (event->get_trigger_type() == Event<double>::TriggerType::kForced)
        continue;

      // Call custom callback handler.
      event->handle(context);
    }

    publish_count_++;
  }

  void DoGetPerStepEvents(
      const Context<double>& context,
      CompositeEventCollection<double>* events) const override {
    {
      PublishEvent<double> event(
          Event<double>::TriggerType::kPerStep,
          std::bind(&TestTriggerSystem::StringCallback, this,
              std::placeholders::_1, std::placeholders::_2,
              std::make_shared<const std::string>("hello")));
      event.add_to_composite(events);
    }

    {
      PublishEvent<double> event(
          Event<double>::TriggerType::kPerStep,
          std::bind(&TestTriggerSystem::IntCallback, this,
              std::placeholders::_1, std::placeholders::_2, 42));
      event.add_to_composite(events);
    }
  }

  const std::vector<std::string>& get_string_data() const {
    return string_data_;
  }

  const std::vector<int>& get_int_data() const { return int_data_; }

  int get_publish_count() const { return publish_count_; }

 private:
  // Pass data by a shared_ptr<const stuff>.
  void StringCallback(const Context<double>&, const PublishEvent<double>&,
      std::shared_ptr<const std::string> data) const {
    string_data_.push_back(*data);
  }

  // Pass data by value.
  void IntCallback(const Context<double>&, const PublishEvent<double>&,
      int data) const {
    int_data_.push_back(data);
  }

  // Stores data copied from the abstract values in handled events.
  mutable std::vector<std::string> string_data_;
  mutable std::vector<int> int_data_;
  mutable int publish_count_{0};
};

class TriggerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = dut_.CreateDefaultContext();
    info_ = dut_.AllocateCompositeEventCollection();
    leaf_info_ =
        dynamic_cast<const LeafCompositeEventCollection<double>*>(info_.get());
    DRAKE_DEMAND(leaf_info_ != nullptr);
  }

  TestTriggerSystem dut_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<CompositeEventCollection<double>> info_;
  const LeafCompositeEventCollection<double>* leaf_info_;
};

// After handling of the events, int_data_ should be {42},
// string_data_ should be {"hello"}.
// Then forces a Publish() call on dut_, which should only increase
// publish_count_ without changing any of the data_ vectors.
TEST_F(TriggerTest, AbstractTrigger) {
  // Schedules two publish events.
  dut_.GetPerStepEvents(*context_, info_.get());
  const auto& events = leaf_info_->get_publish_events().get_events();
  EXPECT_EQ(events.size(), 2);

  // Calls handler.
  dut_.Publish(*context_, info_->get_publish_events());

  // Checks string_data_ in dut.
  const auto& string_data = dut_.get_string_data();
  EXPECT_EQ(string_data.size(), 1);
  EXPECT_EQ(string_data.front(), "hello");

  // Checks int_data_ in dut.
  const auto& int_data = dut_.get_int_data();
  EXPECT_EQ(int_data.size(), 1);
  EXPECT_EQ(int_data.front(), 42);

  // Now force a publish call, this should only increment the counter, without
  // touching any of the x_data_ in dut.
  dut_.Publish(*context_);
  EXPECT_EQ(dut_.get_publish_count(), 2);
  EXPECT_EQ(string_data.size(), 1);
  EXPECT_EQ(int_data.size(), 1);
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

  // Declare a single constraint `this[0] >= bias`.
  void CalcInequalityConstraint(VectorX<T>* value) const override {
    value->resize(1);
    (*value)[0] = (*this)[0] - T{bias};
  }
};

class ConstraintTestSystem : public LeafSystem<double> {
 public:
  ConstraintTestSystem() { DeclareContinuousState(2); }

  // Expose some protected methods for testing.
  using LeafSystem<double>::DeclareContinuousState;
  using LeafSystem<double>::DeclareEqualityConstraint;
  using LeafSystem<double>::DeclareInequalityConstraint;
  using LeafSystem<double>::DeclareNumericParameter;
  using LeafSystem<double>::DeclareVectorInputPort;
  using LeafSystem<double>::DeclareVectorOutputPort;

  void CalcState0Constraint(const Context<double>& context,
                            Eigen::VectorXd* value) const {
    *value = Vector1d(context.get_continuous_state_vector().GetAtIndex(0));
  }
  void CalcStateConstraint(const Context<double>& context,
                           Eigen::VectorXd* value) const {
    *value = context.get_continuous_state_vector().CopyToVector();
  }

  void CalcOutput(
      const Context<double>& context,
      ConstraintBasicVector<double, 44>* output) const {
    output->SetFromVector(Eigen::VectorXd::Constant(output->size(), 4.0));
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
  EXPECT_EQ(dut.get_num_constraints(), 0);

  EXPECT_EQ(dut.DeclareEqualityConstraint(
                &ConstraintTestSystem::CalcState0Constraint, 1, "x0"),
            0);
  EXPECT_EQ(dut.get_num_constraints(), 1);

  EXPECT_EQ(dut.DeclareInequalityConstraint(
                &ConstraintTestSystem::CalcStateConstraint, 2, "x"),
            1);
  EXPECT_EQ(dut.get_num_constraints(), 2);

  auto context = dut.CreateDefaultContext();
  context->get_mutable_continuous_state_vector().SetFromVector(
      Eigen::Vector2d(5.0, 7.0));

  EXPECT_EQ(dut.get_constraint(SystemConstraintIndex(0)).size(), 1);
  EXPECT_EQ(dut.get_constraint(SystemConstraintIndex(1)).size(), 2);

  Eigen::VectorXd value;
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
  EXPECT_EQ(dut.get_num_constraints(), 0);

  SystemConstraint<double>::CalcCallback calc = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    *value = Vector1d(context.get_continuous_state_vector().GetAtIndex(1));
  };
  EXPECT_EQ(dut.DeclareInequalityConstraint(calc, 1, "x1"), 0);
  EXPECT_EQ(dut.get_num_constraints(), 1);

  auto context = dut.CreateDefaultContext();
  context->get_mutable_continuous_state_vector().SetFromVector(
      Eigen::Vector2d(5.0, 7.0));

  Eigen::VectorXd value;
  const SystemConstraint<double>& inequality_constraint =
      dut.get_constraint(SystemConstraintIndex(0));
  inequality_constraint.Calc(*context, &value);
  EXPECT_EQ(value.rows(), 1);
  EXPECT_EQ(value[0], 7.0);
  EXPECT_FALSE(inequality_constraint.is_equality_constraint());
  EXPECT_EQ(inequality_constraint.description(), "x1");

  EXPECT_EQ(dut.DeclareEqualityConstraint(calc, 1, "x1eq"), 1);
  EXPECT_EQ(dut.get_num_constraints(), 2);

  const SystemConstraint<double>& equality_constraint =
      dut.get_constraint(SystemConstraintIndex(1));
  equality_constraint.Calc(*context, &value);
  EXPECT_EQ(value.rows(), 1);
  EXPECT_EQ(value[0], 7.0);
  EXPECT_TRUE(equality_constraint.is_equality_constraint());
  EXPECT_EQ(equality_constraint.description(), "x1eq");
}

// Tests constraints implied by BasicVector subtypes.
GTEST_TEST(SystemConstraintTest, ModelVectorTest) {
  ConstraintTestSystem dut;
  EXPECT_EQ(dut.get_num_constraints(), 0);

  // Declaring a constrained model vector parameter should add constraints.
  // We want `vec[0] >= 11` on the parameter vector.
  using ParameterVector = ConstraintBasicVector<double, 11>;
  dut.DeclareNumericParameter(ParameterVector{});
  ASSERT_EQ(dut.get_num_constraints(), 1);
  using Index = SystemConstraintIndex;
  const SystemConstraint<double>& constraint0 = dut.get_constraint(Index{0});
  EXPECT_FALSE(constraint0.is_equality_constraint());
  EXPECT_THAT(constraint0.description(), ::testing::ContainsRegex(
      "^parameter 0 of type .*ConstraintBasicVector<double,11>$"));

  // Declaring constrained model continuous state should add constraints.
  // We want `vec[0] >= 22` on the state vector.
  using StateVector = ConstraintBasicVector<double, 22>;
  dut.DeclareContinuousState(StateVector{}, 0, 0, StateVector::kSize);
  EXPECT_EQ(dut.get_num_constraints(), 2);
  const SystemConstraint<double>& constraint1 = dut.get_constraint(Index{1});
  EXPECT_FALSE(constraint1.is_equality_constraint());
  EXPECT_THAT(constraint1.description(), ::testing::ContainsRegex(
      "^continuous state of type .*ConstraintBasicVector<double,22>$"));

  // Declaring a constrained model vector input should add constraints.
  // We want `vec[0] >= 33` on the input vector.
  using InputVector = ConstraintBasicVector<double, 33>;
  dut.DeclareVectorInputPort(InputVector{});
  EXPECT_EQ(dut.get_num_constraints(), 3);
  const SystemConstraint<double>& constraint2 = dut.get_constraint(Index{2});
  EXPECT_FALSE(constraint2.is_equality_constraint());
  EXPECT_THAT(constraint2.description(), ::testing::ContainsRegex(
      "^input 0 of type .*ConstraintBasicVector<double,33>$"));

  // Declaring a constrained model vector output should add constraints.
  // We want `vec[0] >= 44` on the output vector.
  dut.DeclareVectorOutputPort(&ConstraintTestSystem::CalcOutput);
  EXPECT_EQ(dut.get_num_constraints(), 4);
  const SystemConstraint<double>& constraint3 = dut.get_constraint(Index{3});
  EXPECT_FALSE(constraint3.is_equality_constraint());
  EXPECT_THAT(constraint3.description(), ::testing::ContainsRegex(
      "^output 0 of type .*ConstraintBasicVector<double,44>$"));

  // We'll work through the Calc results all at the end, so that we don't
  // change the shape of the System and Context while we're Calc'ing.
  auto context = dut.CreateDefaultContext();

  // `param0[0] >= 11.0` with `param0[0] == 1.0` produces `-10.0 >= 0.0`.
  context->get_mutable_numeric_parameter(0).SetAtIndex(0, 1.0);
  Eigen::VectorXd value0;
  constraint0.Calc(*context, &value0);
  EXPECT_TRUE(CompareMatrices(value0, Vector1<double>::Constant(-10.0)));

  // `xc[0] >= 22.0` with `xc[0] == 2.0` produces `-20.0 >= 0.0`.
  context->get_mutable_continuous_state_vector().SetAtIndex(0, 2.0);
  Eigen::VectorXd value1;
  constraint1.Calc(*context, &value1);
  EXPECT_TRUE(CompareMatrices(value1, Vector1<double>::Constant(-20.0)));

  // `u0[0] >= 33.0` with `u0[0] == 3.0` produces `-30.0 >= 0.0`.
  InputVector input;
  input.SetAtIndex(0, 3.0);
  context->FixInputPort(0, input);
  Eigen::VectorXd value2;
  constraint2.Calc(*context, &value2);
  EXPECT_TRUE(CompareMatrices(value2, Vector1<double>::Constant(-30.0)));

  // `y0[0] >= 44.0` with `y0[0] == 4.0` produces `-40.0 >= 0.0`.
  Eigen::VectorXd value3;
  constraint3.Calc(*context, &value3);
  EXPECT_TRUE(CompareMatrices(value3, Vector1<double>::Constant(-40.0)));
}

// Note: this class is duplicated in diagram_test.
class RandomContextTestSystem : public LeafSystem<double> {
 public:
  RandomContextTestSystem() {
    this->DeclareContinuousState(
        BasicVector<double>(Eigen::Vector2d(-1.0, -2.0)));
    this->DeclareNumericParameter(
        BasicVector<double>(Eigen::Vector3d(1.0, 2.0, 3.0)));
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
      params->get_mutable_numeric_parameter(0).SetAtIndex(i,
                                                          uniform(*generator));
    }
  }
};

GTEST_TEST(RandomContextTest, SetRandomTest) {
  RandomContextTestSystem system;

  auto context = system.CreateDefaultContext();

  // Back-up the numeric context values.
  Eigen::Vector2d state = context->get_continuous_state_vector().CopyToVector();
  Eigen::Vector3d params = context->get_numeric_parameter(0).CopyToVector();

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
          Event<double>::TriggerType::kInitialization,
          std::bind(&InitializationTestSystem::InitPublish, this,
                    std::placeholders::_1, std::placeholders::_2));
      DeclareInitializationEvent(pub_event);

      DeclareInitializationEvent(DiscreteUpdateEvent<double>(
          Event<double>::TriggerType::kInitialization));
      DeclareInitializationEvent(UnrestrictedUpdateEvent<double>(
          Event<double>::TriggerType::kInitialization));
    }

    bool get_pub_init() const { return pub_init_; }
    bool get_dis_update_init() const { return dis_update_init_; }
    bool get_unres_update_init() const { return unres_update_init_; }

   private:
    void InitPublish(const Context<double>&,
                     const PublishEvent<double>& event) const {
      EXPECT_EQ(event.get_trigger_type(),
                Event<double>::TriggerType::kInitialization);
      pub_init_ = true;
    }

    void DoCalcDiscreteVariableUpdates(
        const Context<double>&,
        const std::vector<const DiscreteUpdateEvent<double>*>& events,
        DiscreteValues<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      EXPECT_EQ(events.front()->get_trigger_type(),
                Event<double>::TriggerType::kInitialization);
      dis_update_init_ = true;
    }

    void DoCalcUnrestrictedUpdate(
        const Context<double>&,
        const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
        State<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      EXPECT_EQ(events.front()->get_trigger_type(),
                Event<double>::TriggerType::kInitialization);
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


}  // namespace
}  // namespace systems
}  // namespace drake
