#include "drake/systems/framework/leaf_system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
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
template<typename T>
class TestSystem : public LeafSystem<T> {
 public:
  TestSystem() {
    this->set_name("TestSystem");
    this->DeclareOutputPort(kVectorValued, 17);
    this->DeclareAbstractOutputPort();
  }
  ~TestSystem() override {}

  using LeafSystem<T>::DeclareContinuousState;

  void AddPeriodicUpdate() {
    const double period = 10.0;
    const double offset = 5.0;
    this->DeclarePeriodicDiscreteUpdate(period, offset);
  }

  void AddPeriodicUpdate(double period) {
    const double offset = 0.0;
    this->DeclarePeriodicDiscreteUpdate(period, offset);
  }

  void AddPeriodicUnrestrictedUpdate(double period, double offset) {
    this->DeclarePeriodicUnrestrictedUpdate(period, offset);
  }

  void AddPublish(double period) {
    this->DeclarePublishPeriodSec(period);
  }

  template <typename EventType>
  void AddPerStepEvent() {
    EventType event(Trigger::TriggerType::kPerStep);
    this->DeclarePerStepEvent(event);
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {}

  void DoCalcTimeDerivatives(
      const Context<T>& context,
      ContinuousState<T>* derivatives) const override {}

  std::unique_ptr<Parameters<T>> AllocateParameters() const override {
    return std::make_unique<Parameters<T>>(
        std::make_unique<BasicVector<T>>(2));
  }

  std::unique_ptr<BasicVector<T>> AllocateOutputVector(
      const OutputPortDescriptor<T>& descriptor) const override {
    return std::make_unique<BasicVector<T>>(17);
  }

  std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<T>& descriptor) const override {
    return AbstractValue::Make<int>(42);
  }

  void SetDefaultParameters(const LeafContext<T>& context,
                            Parameters<T>* params) const override {
    BasicVector<T>* param = params->get_mutable_numeric_parameter(0);
    Vector2<T> p0;
    p0 << 13.0, 7.0;
    param->SetFromVector(p0);
  }

  const BasicVector<T>& GetVanillaNumericParameters(
      const Context<T>& context) const {
    return this->GetNumericParameter(context, 0 /* index */);
  }

  BasicVector<T>* GetVanillaMutableNumericParameters(
      Context<T>* context) const {
    return this->GetMutableNumericParameter(context, 0 /* index */);
  }
};

class LeafSystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    event_info_ = system_.AllocateEventCollection();
    leaf_info_ =
        dynamic_cast<const LeafEventCollection<double>*>(event_info_.get());
  }

  TestSystem<double> system_;
  LeafContext<double> context_;

  std::unique_ptr<EventCollection> event_info_;
  const LeafEventCollection<double>* leaf_info_;
};

// Tests that if no update events are configured, none are reported.
TEST_F(LeafSystemTest, NoUpdateEvents) {
  context_.set_time(25.0);
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(std::numeric_limits<double>::infinity(), time);
  EXPECT_TRUE(leaf_info_->HasNoEvents());
}

// Tests that if the current time is smaller than the offset, the next
// update time is the offset.
TEST_F(LeafSystemTest, OffsetHasNotArrivedYet) {
  context_.set_time(2.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(5.0, time);
  const auto& events = leaf_info_->get_discrete_update_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger().get_type(),
            Trigger::TriggerType::kPeriodic);
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
    const auto& events = leaf_info_->get_discrete_update_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger().get_type(),
        Trigger::TriggerType::kPeriodic);
  }
  {
    const auto& events = leaf_info_->get_unrestricted_update_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger().get_type(),
        Trigger::TriggerType::kPeriodic);
  }
}

// Tests that if the current time is exactly the offset, the next
// update time is in the future.
TEST_F(LeafSystemTest, ExactlyAtOffset) {
  context_.set_time(5.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(15.0, time);
  const auto& events = leaf_info_->get_discrete_update_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger().get_type(),
            Trigger::TriggerType::kPeriodic);
}

// Tests that if the current time is larger than the offset, the next
// update time is determined by the period.
TEST_F(LeafSystemTest, OffsetIsInThePast) {
  context_.set_time(23.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(25.0, time);
  const auto& events = leaf_info_->get_discrete_update_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger().get_type(),
            Trigger::TriggerType::kPeriodic);
}

// Tests that if the current time is exactly an update time, the next update
// time is in the future.
TEST_F(LeafSystemTest, ExactlyOnUpdateTime) {
  context_.set_time(25.0);
  system_.AddPeriodicUpdate();
  double time = system_.CalcNextUpdateTime(context_, event_info_.get());

  EXPECT_EQ(35.0, time);
  const auto& events = leaf_info_->get_discrete_update_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger().get_type(),
            Trigger::TriggerType::kPeriodic);
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
    const auto& events = leaf_info_->get_publish_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger().get_type(),
        Trigger::TriggerType::kPeriodic);
  }

  // The update event fires at 15sec.
  context_.set_time(14.0);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(15.0, time);
  {
    const auto& events = leaf_info_->get_discrete_update_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger().get_type(),
        Trigger::TriggerType::kPeriodic);
  }

  // Both events fire at 60sec.
  context_.set_time(59.0);
  time = system_.CalcNextUpdateTime(context_, event_info_.get());
  EXPECT_EQ(60.0, time);
  {
    const auto& events = leaf_info_->get_discrete_update_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger().get_type(),
        Trigger::TriggerType::kPeriodic);
  }
  {
    const auto& events = leaf_info_->get_publish_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger().get_type(),
        Trigger::TriggerType::kPeriodic);
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
TEST_F(LeafSystemTest, Parameters) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const BasicVector<double>& vec =
      system_.GetVanillaNumericParameters(*context);
  EXPECT_EQ(13.0, vec[0]);
  EXPECT_EQ(7.0, vec[1]);
  BasicVector<double>* mutable_vec =
      system_.GetVanillaMutableNumericParameters(context.get());
  mutable_vec->SetAtIndex(1, 42.0);
  EXPECT_EQ(42.0, vec[1]);
}

// Tests that the leaf system reserved the declared misc continuous state.
TEST_F(LeafSystemTest, DeclareVanillaMiscContinuousState) {
  system_.DeclareContinuousState(2);
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>* xc = context->get_continuous_state();
  EXPECT_EQ(2, xc->size());
  EXPECT_EQ(0, xc->get_generalized_position().size());
  EXPECT_EQ(0, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());
}

// Tests that the leaf system reserved the declared misc continuous state of
// interesting custom type.
TEST_F(LeafSystemTest, DeclareTypedMiscContinuousState) {
  system_.DeclareContinuousState(MyVector2d());
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>* xc = context->get_continuous_state();
  // Check that type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<MyVector2d*>(
                         context->get_mutable_continuous_state_vector()));
  EXPECT_EQ(2, xc->size());
  EXPECT_EQ(0, xc->get_generalized_position().size());
  EXPECT_EQ(0, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());
}

// Tests that the leaf system reserved the declared continuous state with
// second-order structure.
TEST_F(LeafSystemTest, DeclareVanillaContinuousState) {
  system_.DeclareContinuousState(4, 3, 2);
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>* xc = context->get_continuous_state();
  EXPECT_EQ(4 + 3 + 2, xc->size());
  EXPECT_EQ(4, xc->get_generalized_position().size());
  EXPECT_EQ(3, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());
}

// Tests that the leaf system reserved the declared continuous state with
// second-order structure of interesting custom type.
TEST_F(LeafSystemTest, DeclareTypedContinuousState) {
  using MyVector9d = MyVector<4 + 3 + 2, double>;
  system_.DeclareContinuousState(MyVector9d(), 4, 3, 2);
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>* xc = context->get_continuous_state();
  // Check that type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<MyVector9d*>(
                         context->get_mutable_continuous_state_vector()));
  // Check that dimensions were preserved.
  EXPECT_EQ(4 + 3 + 2, xc->size());
  EXPECT_EQ(4, xc->get_generalized_position().size());
  EXPECT_EQ(3, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());
}

// Tests that the vector-valued output has been allocated with the correct
// dimensions.
TEST_F(LeafSystemTest, DeclareVectorOutput) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  auto output = system_.AllocateOutput(*context);
  EXPECT_EQ(17, output->get_vector_data(0)->size());
}

// Tests that the abstract-valued output has been allocated with the correct
// type.
TEST_F(LeafSystemTest, DeclareAbstractOutput) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  auto output = system_.AllocateOutput(*context);
  EXPECT_EQ(42, UnpackIntValue(output->get_data(1)));
}

TEST_F(LeafSystemTest, DeclarePerStepActions) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();

  system_.AddPerStepEvent<PublishEvent<double>>();
  system_.AddPerStepEvent<DiscreteUpdateEvent<double>>();
  system_.AddPerStepEvent<UnrestrictedUpdateEvent<double>>();

  system_.GetPerStepEvents(*context, event_info_.get());

  {
    const auto& events = leaf_info_->get_publish_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger().get_type(),
        Trigger::TriggerType::kPerStep);
  }
  {
    const auto& events = leaf_info_->get_discrete_update_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger().get_type(),
        Trigger::TriggerType::kPerStep);
  }
  {
    const auto& events = leaf_info_->get_unrestricted_update_events();
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.front()->get_trigger().get_type(),
        Trigger::TriggerType::kPerStep);
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

    this->DeclareOutputPort(kVectorValued, 3);
    this->DeclareVectorOutputPort(MyVector4d());
    this->DeclareAbstractOutputPort(Value<std::string>("44"));

    this->DeclareNumericParameter(*MyVector2d::Make(1.1, 2.2));
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}
};

// Tests that Declare{Vector,Abstract}{Input,Output}Port end up with the
// correct topology.
GTEST_TEST(ModelLeafSystemTest, ModelPortsTopology) {
  DeclaredModelPortsSystem dut;

  ASSERT_EQ(dut.get_num_input_ports(), 3);
  ASSERT_EQ(dut.get_num_output_ports(), 3);

  const InputPortDescriptor<double>& in0 = dut.get_input_port(0);
  const InputPortDescriptor<double>& in1 = dut.get_input_port(1);
  const InputPortDescriptor<double>& in2 = dut.get_input_port(2);

  const OutputPortDescriptor<double>& out0 = dut.get_output_port(0);
  const OutputPortDescriptor<double>& out1 = dut.get_output_port(1);
  const OutputPortDescriptor<double>& out2 = dut.get_output_port(2);

  EXPECT_EQ(in0.get_data_type(), kVectorValued);
  EXPECT_EQ(in1.get_data_type(), kVectorValued);
  EXPECT_EQ(in2.get_data_type(), kAbstractValued);

  EXPECT_EQ(out0.get_data_type(), kVectorValued);
  EXPECT_EQ(out1.get_data_type(), kVectorValued);
  EXPECT_EQ(out2.get_data_type(), kAbstractValued);

  EXPECT_EQ(in0.size(), 1);
  EXPECT_EQ(in1.size(), 2);

  EXPECT_EQ(out0.size(), 3);
  EXPECT_EQ(out1.size(), 4);
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
GTEST_TEST(ModelLeafSystemTest, ModelPortsOutput) {
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

  // Check that Value<string>("44") came out.
  auto output2 = system_output->get_data(2);
  ASSERT_NE(output2, nullptr);
  std::string downcast_output2{};
  EXPECT_NO_THROW(downcast_output2 = output2->GetValueOrThrow<std::string>());
  EXPECT_EQ(downcast_output2, "44");
}

// Tests that the leaf system reserved the declared parameters of interesting
// custom type.
GTEST_TEST(ModelLeafSystemTest, ModelNumericParams) {
  DeclaredModelPortsSystem dut;
  auto context = dut.CreateDefaultContext();
  ASSERT_EQ(context->num_numeric_parameters(), 1);
  const BasicVector<double>* const param = context->get_numeric_parameter(0);
  // Check that type was preserved.
  ASSERT_NE(nullptr, dynamic_cast<const MyVector2d*>(param));
  EXPECT_EQ(2, param->size());
  EXPECT_EQ(1.1, param->GetAtIndex(0));
  EXPECT_EQ(2.2, param->GetAtIndex(1));
}

// Tests that DeclareAbstractState works expectedly.
GTEST_TEST(ModelLeafSystemTest, ModelAbstractState) {
  class DeclaredModelAbstractStateSystem : public LeafSystem<double> {
   public:
    DeclaredModelAbstractStateSystem() {
      DeclareAbstractState(AbstractValue::Make<int>(1));
      DeclareAbstractState(AbstractValue::Make<std::string>("wow"));
    }
    void DoCalcOutput(const Context<double>& context,
                      SystemOutput<double>* output) const override {}
  };

  DeclaredModelAbstractStateSystem dut;
  auto context = dut.CreateDefaultContext();

  EXPECT_EQ(context->get_abstract_state<int>(0), 1);
  EXPECT_EQ(context->get_abstract_state<std::string>(1), "wow");
}

// Tests both that an unrestricted update callback is called and that
// modifications to state dimension are caught.
TEST_F(LeafSystemTest, CallbackAndInvalidUpdates) {
  // Create 9, 1, and 3 dimensional continuous, discrete, and abstract state
  // vectors.
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  context->set_continuous_state(
    std::make_unique<ContinuousState<double>>(
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
  LeafEventCollection<double> leaf_events;
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback =
      [](const Context<double>& c, const Trigger&, State<double>* s) {
        s->CopyFrom(*c.CloneState());
    };

    UnrestrictedUpdateEvent<double> event(
        Trigger::TriggerType::kPeriodic, callback);

    event.add_to(&leaf_events);
  }

  // Verify no exception is thrown.
  EXPECT_NO_THROW(
      system_.CalcUnrestrictedUpdate(*context, &leaf_events, x.get()));

  // Change the function to change the continuous state dimension.
  // Call the unrestricted update function again, now verifying that an
  // exception is thrown.
  leaf_events.Clear();
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback =
      [](const Context<double>& c, const Trigger&, State<double>* s) {
        s->CopyFrom(*c.CloneState());
        s->set_continuous_state(
            std::make_unique<ContinuousState<double>>(
              std::make_unique<BasicVector<double>>(4), 4, 0, 0));
    };

    UnrestrictedUpdateEvent<double> event(
        Trigger::TriggerType::kPeriodic, callback);

    event.add_to(&leaf_events);
  }

  // Call the unrestricted update function, verifying that an exception
  // is thrown
  EXPECT_THROW(system_.CalcUnrestrictedUpdate(*context, &leaf_events, x.get()),
               std::logic_error);

  // Restore the continuous state (size).
  x->set_continuous_state(
    std::make_unique<ContinuousState<double>>(
      std::make_unique<BasicVector<double>>(9), 3, 3, 3));

  // Change the event to indicate to change the discrete state dimension.
  leaf_events.Clear();
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback =
      [](const Context<double>& c, const Trigger&, State<double>* s) {
        std::vector<std::unique_ptr<BasicVector<double>>> disc_data;
        s->CopyFrom(*c.CloneState());
        disc_data.push_back(std::make_unique<BasicVector<double>>(1));
        disc_data.push_back(std::make_unique<BasicVector<double>>(1));
        s->set_discrete_state(
            std::make_unique<DiscreteValues<double>>(std::move(disc_data)));
    };

    UnrestrictedUpdateEvent<double> event(
        Trigger::TriggerType::kPeriodic, callback);

    event.add_to(&leaf_events);
  }

  // Call the unrestricted update function again, again verifying that an
  // exception is thrown.
  EXPECT_THROW(system_.CalcUnrestrictedUpdate(*context, &leaf_events, x.get()),
               std::logic_error);

  // Restore the discrete state (size).
  x->set_discrete_state(std::make_unique<DiscreteValues<double>>(
      std::make_unique<BasicVector<double>>(1)));

  // Change the event to indicate to change the abstract state dimension.
  leaf_events.Clear();
  {
    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback =
      [](const Context<double>& c, const Trigger&, State<double>* s) {
        s->CopyFrom(*c.CloneState());
        s->set_abstract_state(std::make_unique<AbstractValues>());
    };

    UnrestrictedUpdateEvent<double> event(
        Trigger::TriggerType::kPeriodic, callback);

    event.add_to(&leaf_events);
  }

  // Call the unrestricted update function again, again verifying that an
  // exception is thrown.
  EXPECT_THROW(system_.CalcUnrestrictedUpdate(*context, &leaf_events, x.get()),
               std::logic_error);
}

// Tests that the next update time is computed correctly for LeafSystems
// templated on AutoDiffXd. Protects against regression on #4431.
GTEST_TEST(AutodiffLeafSystemTest, NextUpdateTimeAutodiff) {
  TestSystem<AutoDiffXd> system;
  LeafContext<AutoDiffXd> context;

  context.set_time(21.0);
  system.AddPeriodicUpdate();

  auto event_info = system.AllocateEventCollection();
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

  void AddAbstractInputPort() {
    this->DeclareAbstractInputPort();
  }

  void AddAbstractOutputPort() {
    this->DeclareAbstractOutputPort();
  }

 protected:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}
};


GTEST_TEST(FeedthroughTest, DefaultWithNoInputsOrOutputs) {
  DefaultFeedthroughSystem system;
  EXPECT_FALSE(system.HasAnyDirectFeedthrough());
}

GTEST_TEST(FeedthroughTest, DefaultWithBothInputsAndOutputs) {
  DefaultFeedthroughSystem system;
  system.AddAbstractInputPort();
  system.AddAbstractOutputPort();
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  EXPECT_TRUE(system.HasDirectFeedthrough(0));
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 0));
}

GTEST_TEST(FeedthroughTest, DefaultWithInputsOnly) {
  DefaultFeedthroughSystem input_only;
  input_only.AddAbstractInputPort();
  EXPECT_FALSE(input_only.HasAnyDirectFeedthrough());
}

GTEST_TEST(FeedthroughTest, DefaultWithOutputsOnly) {
  DefaultFeedthroughSystem output_only;
  output_only.AddAbstractOutputPort();
  EXPECT_FALSE(output_only.HasAnyDirectFeedthrough());
  EXPECT_FALSE(output_only.HasDirectFeedthrough(0));
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
  bool DoHasDirectFeedthrough(const SparsityMatrix* sparsity,
                              int input_port,
                              int output_port) const override {
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
}

// SymbolicSparsitySystem has the same sparsity matrix as ManualSparsitySystem,
// but the matrix can be inferred from the symbolic form.
template <typename T>
class SymbolicSparsitySystem : public LeafSystem<T> {
 public:
  SymbolicSparsitySystem() {
    this->DeclareInputPort(kVectorValued, kSize);
    this->DeclareInputPort(kVectorValued, kSize);
    this->DeclareOutputPort(kVectorValued, kSize);
    this->DeclareOutputPort(kVectorValued, kSize);
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {
    const auto& u0 = *(this->EvalVectorInput(context, 0));
    const auto& u1 = *(this->EvalVectorInput(context, 1));
    auto& y0 = *(output->GetMutableVectorData(0));
    auto& y1 = *(output->GetMutableVectorData(1));

    y0.set_value(u1.get_value());
    y1.set_value(u0.get_value());
  }

 protected:
  SymbolicSparsitySystem<symbolic::Expression>* DoToSymbolic() const override {
    return new SymbolicSparsitySystem<symbolic::Expression>();
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
}

GTEST_TEST(GraphvizTest, Attributes) {
  DefaultFeedthroughSystem system;
  // Check that the ID is the memory address.
  ASSERT_EQ(reinterpret_cast<int64_t>(&system), system.GetGraphvizId());
  const std::string dot = system.GetGraphvizString();
  // Check that left-to-right ranking is imposed.
  EXPECT_THAT(dot, ::testing::HasSubstr("rankdir=LR"));
  // Check that NiceTypeName is used to compute the label.
  EXPECT_THAT(dot, ::testing::HasSubstr(
      "label=\"drake/systems/(anonymous)/DefaultFeedthroughSystem@"));
}

GTEST_TEST(GraphvizTest, Ports) {
  DefaultFeedthroughSystem system;
  system.AddAbstractInputPort();
  system.AddAbstractInputPort();
  system.AddAbstractOutputPort();
  const std::string dot = system.GetGraphvizString();
  EXPECT_THAT(dot, ::testing::HasSubstr(
      "{{<u0>u0|<u1>u1} | {<y0>y0}}"));
}

// This system schedules two simultaneous publish events with
// GetPerStepEvents(). Both events have abstract data, but of different types.
// Both events have different custom handler callbacks. And DoPublish() is also
// overriding.
class TestTriggerSystem : public LeafSystem<double> {
 public:
  TestTriggerSystem() {}

  void DoPublish(const Context<double>& context,
      const std::vector<const PublishEvent<double>*>& events) const override {
    for (const PublishEvent<double>* event : events) {
      DRAKE_DEMAND(event->Handle != nullptr);
      // Call custom callback handler.
      event->Handle(context, event->get_trigger());
      // Also clone the abstract value and store it in abs_data_.
      abs_data_.push_back(event->get_trigger().get_data()->Clone());
    }

    publish_count_++;
  }

  void DoGetPerStepEvents(const Context<double>& context,
      EventCollection* event_info) const override {
    {
      auto trigger = std::make_unique<Trigger>(Trigger::TriggerType::kPerStep);
      trigger->set_data(AbstractValue::Make<std::string>("hello"));

      PublishEvent<double> event(std::move(trigger),
          std::bind(&TestTriggerSystem::CopyString, this,
                    std::placeholders::_1, std::placeholders::_2));
      event.add_to(event_info);
    }

    {
      auto trigger = std::make_unique<Trigger>(Trigger::TriggerType::kPerStep);
      trigger->set_data(AbstractValue::Make<int>(42));

      PublishEvent<double> event(std::move(trigger),
          std::bind(&TestTriggerSystem::CopyInt, this,
                    std::placeholders::_1, std::placeholders::_2));
      event.add_to(event_info);
    }
  }

  const std::vector<std::unique_ptr<AbstractValue>>& get_abstract_data() const {
    return abs_data_;
  }

  const std::vector<std::string>& get_string_data() const {
    return string_data_;
  }

  const std::vector<int>& get_int_data() const {
    return int_data_;
  }

  int get_publish_count() const {
    return publish_count_;
  }

 private:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  // Casts the data in @p trigger as string and store it in string_data_.
  void CopyString(const Context<double>& context,
                  const Trigger& trigger) const {
    if (trigger.get_data() != nullptr)
      string_data_.push_back(trigger.get_data()->GetValue<std::string>());
  }

  // Casts the data in @p trigger as int and store it in int_data_.
  void CopyInt(const Context<double>& context,
               const Trigger& trigger) const {
    if (trigger.get_data() != nullptr)
      int_data_.push_back(trigger.get_data()->GetValue<int>());
  }

  // Stores data copied from the abstract values in handled events.
  mutable std::vector<std::unique_ptr<AbstractValue>> abs_data_;
  mutable std::vector<std::string> string_data_;
  mutable std::vector<int> int_data_;

  mutable int publish_count_{0};
};

class TriggerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = dut_.CreateDefaultContext();
    info_ = dut_.AllocateEventCollection();
    leaf_info_ = dynamic_cast<const LeafEventCollection<double>*>(info_.get());
    DRAKE_DEMAND(leaf_info_ != nullptr);
  }

  TestTriggerSystem dut_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<EventCollection> info_;
  const LeafEventCollection<double>* leaf_info_;
};

// After handling of the events, the abs_data_ should be {"hello", 42},
// int_data_ should be {42}, string_data_ should be {"hello"}.
// Then forces a Publish() call on dut_, which should only increase
// publish_count_ without changing any of the data_ vectors.
TEST_F(TriggerTest, AbstractTrigger) {
  // Schedules two publish events.
  dut_.GetPerStepEvents(*context_, info_.get());
  const auto& events = leaf_info_->get_publish_events();
  EXPECT_EQ(events.size(), 2);

  // Calls handler.
  dut_.Publish(*context_, info_.get());

  // Checks abs_data_ in dut.
  const auto& data = dut_.get_abstract_data();
  EXPECT_EQ(data.size(), 2);
  EXPECT_EQ(data[0]->GetValue<std::string>(), "hello");
  EXPECT_EQ(data[1]->GetValue<int>(), 42);

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
  EXPECT_EQ(data.size(), 2);
}

}  // namespace
}  // namespace systems
}  // namespace drake
