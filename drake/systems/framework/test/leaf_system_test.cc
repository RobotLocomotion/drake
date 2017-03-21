#include "drake/systems/framework/leaf_system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
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
};

class LeafSystemTest : public ::testing::Test {
 protected:
  TestSystem<double> system_;
  LeafContext<double> context_;
};

// Tests that if no update events are configured, none are reported.
TEST_F(LeafSystemTest, NoUpdateEvents) {
  context_.set_time(25.0);
  UpdateActions<double> actions;
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), actions.time);
  EXPECT_EQ(0u, actions.events.size());
}

// Tests that if the current time is smaller than the offset, the next
// update time is the offset.
TEST_F(LeafSystemTest, OffsetHasNotArrivedYet) {
  context_.set_time(2.0);
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate();
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(5.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if the current time is smaller than the offset, the next
// update time is the offset, DiscreteUpdate and UnrestrictedUpdate happen
// at the same time.
TEST_F(LeafSystemTest, EventsAtTheSameTime) {
  context_.set_time(2.0);
  UpdateActions<double> actions;
  // Both actions happen at t = 5.
  system_.AddPeriodicUpdate();
  system_.AddPeriodicUnrestrictedUpdate(3, 5);
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(5.0, actions.time);
  ASSERT_EQ(2u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
  EXPECT_EQ(DiscreteEvent<double>::kUnrestrictedUpdateAction,
            actions.events[1].action);
}

// Tests that if the current time is exactly the offset, the next
// update time is in the future.
TEST_F(LeafSystemTest, ExactlyAtOffset) {
  context_.set_time(5.0);
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate();
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(15.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if the current time is larger than the offset, the next
// update time is determined by the period.
TEST_F(LeafSystemTest, OffsetIsInThePast) {
  context_.set_time(23.0);
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate();
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(25.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if the current time is exactly an update time, the next update
// time is in the future.
TEST_F(LeafSystemTest, ExactlyOnUpdateTime) {
  context_.set_time(25.0);
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate();
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(35.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if a LeafSystem has both a discrete update and a periodic Publish,
// the update actions are computed appropriately.
TEST_F(LeafSystemTest, UpdateAndPublish) {
  system_.AddPeriodicUpdate(15.0);
  system_.AddPublish(12.0);

  UpdateActions<double> actions;

  // The publish event fires at 12sec.
  context_.set_time(9.0);
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_EQ(12.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kPublishAction, actions.events[0].action);

  // The update event fires at 15sec.
  context_.set_time(14.0);
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_EQ(15.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);

  // Both events fire at 60sec.
  context_.set_time(59.0);
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_EQ(60.0, actions.time);
  ASSERT_EQ(2u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if the integrator has stopped on the k-th sample, and the current
// time for that sample is slightly less than k * period due to floating point
// rounding, the next sample time is (k + 1) * period.
TEST_F(LeafSystemTest, FloatingPointRoundingZeroPointZeroOneFive) {
  context_.set_time(0.015 * 11);  // Slightly less than 0.165.
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate(0.015);
  system_.CalcNextUpdateTime(context_, &actions);
  // 0.015 * 12 = 0.18.
  EXPECT_NEAR(0.18, actions.time, 1e-8);
}

// Tests that if the integrator has stopped on the k-th sample, and the current
// time for that sample is slightly less than k * period due to floating point
// rounding, the next sample time is (k + 1) * period.
TEST_F(LeafSystemTest, FloatingPointRoundingZeroPointZeroZeroTwoFive) {
  context_.set_time(0.0025 * 977);  // Slightly less than 2.4425
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate(0.0025);
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_NEAR(2.445, actions.time, 1e-8);
}

// Tests that the leaf system reserved the declared Parameters with default
// values.
TEST_F(LeafSystemTest, Parameters) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const BasicVector<double>& vec =
      system_.GetVanillaNumericParameters(*context);
  EXPECT_EQ(13.0, vec[0]);
  EXPECT_EQ(7.0, vec[1]);
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

// A system that exercises the model_value-based input and output ports.
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

// Tests both that an unrestricted update callback is called and that
// modifications to state dimension are caught.
TEST_F(LeafSystemTest, CallbackAndInvalidUpdates) {
  // Create 9, 1, and 3 dimensional continuous, discrete, and abstract state
  // vectors.
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  context->set_continuous_state(
    std::make_unique<ContinuousState<double>>(
      std::make_unique<BasicVector<double>>(9), 3, 3, 3));
  context->set_discrete_state(
    std::make_unique<DiscreteState<double>>(
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
  DiscreteEvent<double> event;
  event.action = DiscreteEvent<double>::kUnrestrictedUpdateAction;
  event.do_unrestricted_update = [](const Context<double>& c,
                                  State<double>* s) {
    s->CopyFrom(*c.CloneState());
  };

  // Verify no exception is thrown.
  EXPECT_NO_THROW(system_.CalcUnrestrictedUpdate(*context, event, x.get()));

  // Change the function to change the continuous state dimension.
  // Call the unrestricted update function again, now verifying that an
  // exception is thrown.
  event.do_unrestricted_update = [](const Context<double>& c,
                                  State<double>* s) {
    s->CopyFrom(*c.CloneState());
    s->set_continuous_state(
      std::make_unique<ContinuousState<double>>(
        std::make_unique<BasicVector<double>>(4), 4, 0, 0));
  };

  // Call the unrestricted update function, verifying that an exception
  // is thrown
  EXPECT_THROW(system_.CalcUnrestrictedUpdate(*context, event, x.get()),
               std::logic_error);

  // Restore the continuous state (size).
  x->set_continuous_state(
    std::make_unique<ContinuousState<double>>(
      std::make_unique<BasicVector<double>>(9), 3, 3, 3));

  // Change the event to indicate to change the discrete state dimension.
  event.do_unrestricted_update = [](const Context<double>& c,
                                  State<double>* s) {
    std::vector<std::unique_ptr<BasicVector<double>>> disc_data;
    s->CopyFrom(*c.CloneState());
    disc_data.push_back(std::make_unique<BasicVector<double>>(1));
    disc_data.push_back(std::make_unique<BasicVector<double>>(1));
    s->set_discrete_state(
         std::make_unique<DiscreteState<double>>(std::move(disc_data)));
  };

  // Call the unrestricted update function again, again verifying that an
  // exception is thrown.
  EXPECT_THROW(system_.CalcUnrestrictedUpdate(*context, event, x.get()),
               std::logic_error);

  // Restore the discrete state (size).
  x->set_discrete_state(
    std::make_unique<DiscreteState<double>>(
      std::make_unique<BasicVector<double>>(1)));

  // Change the event to indicate to change the abstract state dimension.
  event.do_unrestricted_update = [](const Context<double>& c,
                                  State<double>* s) {
    s->CopyFrom(*c.CloneState());
    s->set_abstract_state(std::make_unique<AbstractValues>());
  };

  // Call the unrestricted update function again, again verifying that an
  // exception is thrown.
  EXPECT_THROW(system_.CalcUnrestrictedUpdate(*context, event, x.get()),
               std::logic_error);
}

// Tests that the next update time is computed correctly for LeafSystems
// templated on AutoDiffXd. Protects against regression on #4431.
GTEST_TEST(AutodiffLeafSystemTest, NextUpdateTimeAutodiff) {
  TestSystem<AutoDiffXd> system;
  LeafContext<AutoDiffXd> context;

  context.set_time(21.0);
  UpdateActions<AutoDiffXd> actions;
  system.AddPeriodicUpdate();
  system.CalcNextUpdateTime(context, &actions);

  EXPECT_EQ(25.0, actions.time);
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
  EXPECT_NE(std::string::npos, dot.find("rankdir=LR")) << dot;
  // Check that NiceTypeName is used to compute the label.
  EXPECT_NE(std::string::npos, dot.find(
      "label=\"drake::systems::(anonymous)::DefaultFeedthroughSystem|"));
}

GTEST_TEST(GraphvizTest, Ports) {
  DefaultFeedthroughSystem system;
  system.AddAbstractInputPort();
  system.AddAbstractInputPort();
  system.AddAbstractOutputPort();
  const std::string dot = system.GetGraphvizString();
  EXPECT_NE(std::string::npos, dot.find("{{<u0>u0|<u1>u1} | {<y0>y0}}")) << dot;
}

}  // namespace
}  // namespace systems
}  // namespace drake
