#include "drake/systems/framework/diagram_context.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

constexpr int kSize = 1;
constexpr int kNumSystems = 8;
constexpr double kTime = 12.0;

class SystemWithAbstractState : public LeafSystem<double> {
 public:
  SystemWithAbstractState() {
    DeclareAbstractState(AbstractValue::Make<int>(42));
  }
  ~SystemWithAbstractState() override {}
};

class SystemWithNumericParameters : public LeafSystem<double> {
 public:
  SystemWithNumericParameters() {}
  ~SystemWithNumericParameters() override {}

  std::unique_ptr<Parameters<double>> AllocateParameters() const override {
    return std::make_unique<Parameters<double>>(
        std::make_unique<BasicVector<double>>(2));
  }
};

class SystemWithAbstractParameters : public LeafSystem<double> {
 public:
  SystemWithAbstractParameters() {
    this->DeclareAbstractParameter(Value<int>(2048));
  }
  ~SystemWithAbstractParameters() override {}
};

}  // namespace

// This class must be outside the anonymous namespace to permit the
// DiagramContext friend declaration to work.

class DiagramContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    adder0_ = std::make_unique<Adder<double>>(2 /* inputs */, kSize);
    adder0_->set_name("adder0");
    adder1_ = std::make_unique<Adder<double>>(2 /* inputs */, kSize);
    adder1_->set_name("adder1");

    integrator0_.reset(new Integrator<double>(kSize));
    integrator1_.reset(new Integrator<double>(kSize));
    hold_ = std::make_unique<ZeroOrderHold<double>>(13.0 /* sec */, kSize);

    abstract_state_system_ = std::make_unique<SystemWithAbstractState>();
    system_with_numeric_parameters_ =
        std::make_unique<SystemWithNumericParameters>();
    system_with_abstract_parameters_ =
        std::make_unique<SystemWithAbstractParameters>();

    context_ = std::make_unique<DiagramContext<double>>(kNumSystems);

    AddSystem(*adder0_, SubsystemIndex(0));
    AddSystem(*adder1_, SubsystemIndex(1));
    AddSystem(*integrator0_, SubsystemIndex(2));
    AddSystem(*integrator1_, SubsystemIndex(3));
    AddSystem(*hold_, SubsystemIndex(4));
    AddSystem(*abstract_state_system_, SubsystemIndex(5));
    AddSystem(*system_with_numeric_parameters_, SubsystemIndex(6));
    AddSystem(*system_with_abstract_parameters_, SubsystemIndex(7));

    // Fake up some input ports for this diagram.
    context_->AddInputPort(InputPortIndex(0), DependencyTicket(100));
    context_->AddInputPort(InputPortIndex(1), DependencyTicket(101));

    context_->MakeState();
    context_->MakeParameters();

    context_->set_time(kTime);
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc.get_mutable_vector().SetAtIndex(0, 42.0);
    xc.get_mutable_vector().SetAtIndex(1, 43.0);

    DiscreteValues<double>& xd = context_->get_mutable_discrete_state();
    xd.get_mutable_vector(0).SetAtIndex(0, 44.0);

    context_->get_mutable_numeric_parameter(0).SetAtIndex(0, 76.0);
    context_->get_mutable_numeric_parameter(0).SetAtIndex(1, 77.0);
  }

  void AddSystem(const System<double>& sys, SubsystemIndex index) {
    auto subcontext = sys.CreateDefaultContext();
    context_->AddSystem(index, std::move(subcontext));
  }

  void AttachInputPorts() {
    context_->FixInputPort(0, {128.0});
    context_->FixInputPort(1, {256.0});
  }

  // Reads a FixedInputPortValue connected to @p context at @p index.
  // Returns nullptr if the port is not connected.
  const BasicVector<double>* ReadVectorInputPort(const Context<double>& context,
                                                 int index) {
    const FixedInputPortValue* free_value =
        context.MaybeGetFixedInputPortValue(InputPortIndex(index));
    return free_value ? &free_value->get_vector_value<double>() : nullptr;
  }

  std::unique_ptr<DiagramContext<double>> context_;
  std::unique_ptr<Adder<double>> adder0_;
  std::unique_ptr<Adder<double>> adder1_;
  std::unique_ptr<Integrator<double>> integrator0_;
  std::unique_ptr<Integrator<double>> integrator1_;
  std::unique_ptr<ZeroOrderHold<double>> hold_;
  std::unique_ptr<SystemWithAbstractState> abstract_state_system_;
  std::unique_ptr<SystemWithNumericParameters> system_with_numeric_parameters_;
  std::unique_ptr<SystemWithAbstractParameters>
      system_with_abstract_parameters_;
};

namespace {

// Verifies that @p state is a clone of the state constructed in
// DiagramContextTest::SetUp.
void VerifyClonedState(const State<double>& clone) {
  // - Continuous
  const ContinuousState<double>& xc = clone.get_continuous_state();
  EXPECT_EQ(42.0, xc.get_vector().GetAtIndex(0));
  EXPECT_EQ(43.0, xc.get_vector().GetAtIndex(1));
  // - Discrete
  const DiscreteValues<double>& xd = clone.get_discrete_state();
  EXPECT_EQ(44.0, xd.get_vector(0).GetAtIndex(0));
  // - Abstract
  const AbstractValues& xa = clone.get_abstract_state();
  EXPECT_EQ(42, xa.get_value(0).GetValue<int>());
}

// Verifies that the @p params are a clone of the params constructed in
// DiagramContextTest::SetUp.
void VerifyClonedParameters(const Parameters<double>& params) {
  ASSERT_EQ(1, params.num_numeric_parameters());
  EXPECT_EQ(76.0, params.get_numeric_parameter(0).GetAtIndex(0));
  EXPECT_EQ(77.0, params.get_numeric_parameter(0).GetAtIndex(1));
  ASSERT_EQ(1, params.num_abstract_parameters());
  EXPECT_EQ(2048, UnpackIntValue(params.get_abstract_parameter(0)));
}

// Tests that subsystems have contexts in the DiagramContext.
TEST_F(DiagramContextTest, RetrieveConstituents) {
  // All of the subsystems should be leaf Systems.
  for (SubsystemIndex i(0); i < kNumSystems; ++i) {
    auto context = dynamic_cast<const LeafContext<double>*>(
        &context_->GetSubsystemContext(i));
    EXPECT_TRUE(context != nullptr);
  }
}

// Tests that the time writes through to the subsystem contexts.
TEST_F(DiagramContextTest, Time) {
  context_->set_time(42.0);
  EXPECT_EQ(42.0, context_->get_time());
  for (SubsystemIndex i(0); i < kNumSystems; ++i) {
    EXPECT_EQ(42.0, context_->GetSubsystemContext(i).get_time());
  }
}

// Tests that the accuracy writes through to the subsystem contexts.
TEST_F(DiagramContextTest, Accuracy) {
  const double kAccuracy = 1e-12;
  context_->set_accuracy(kAccuracy);
  EXPECT_TRUE(context_->get_accuracy());
  EXPECT_EQ(kAccuracy, context_->get_accuracy().value());
  for (SubsystemIndex i(0); i < kNumSystems; ++i) {
    EXPECT_TRUE(context_->GetSubsystemContext(i).get_accuracy());
    EXPECT_EQ(kAccuracy,
              context_->GetSubsystemContext(i).get_accuracy().value());
  }
}

// Tests that state variables appear in the diagram context, and write
// transparently through to the constituent system contexts.
TEST_F(DiagramContextTest, State) {
  // Each integrator has a single continuous state variable.
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  EXPECT_EQ(2, xc.size());
  EXPECT_EQ(0, xc.get_generalized_position().size());
  EXPECT_EQ(0, xc.get_generalized_velocity().size());
  EXPECT_EQ(2, xc.get_misc_continuous_state().size());

  // The zero-order hold has a discrete state vector of length 1.
  DiscreteValues<double>& xd = context_->get_mutable_discrete_state();
  EXPECT_EQ(1, xd.num_groups());
  EXPECT_EQ(1, xd.get_vector(0).size());

  // Changes to the diagram state write through to constituent system states.
  // - Continuous
  ContinuousState<double>& integrator0_xc =
      context_->GetMutableSubsystemContext(SubsystemIndex(2))
          .get_mutable_continuous_state();
  ContinuousState<double>& integrator1_xc =
      context_->GetMutableSubsystemContext(SubsystemIndex(3))
          .get_mutable_continuous_state();
  EXPECT_EQ(42.0, integrator0_xc.get_vector().GetAtIndex(0));
  EXPECT_EQ(43.0, integrator1_xc.get_vector().GetAtIndex(0));
  // - Discrete
  DiscreteValues<double>& hold_xd =
      context_->GetMutableSubsystemContext(SubsystemIndex(4))
          .get_mutable_discrete_state();
  EXPECT_EQ(44.0, hold_xd.get_vector(0).GetAtIndex(0));

  // Changes to constituent system states appear in the diagram state.
  // - Continuous
  integrator1_xc.get_mutable_vector().SetAtIndex(0, 1000.0);
  EXPECT_EQ(1000.0, xc.get_vector().GetAtIndex(1));
  // - Discrete
  hold_xd.get_mutable_vector(0).SetAtIndex(0, 1001.0);
  EXPECT_EQ(1001.0, xd.get_vector(0).GetAtIndex(0));
}

// Tests that the pointers to substates in the DiagramState are equal to the
// substates in the subsystem contexts.
TEST_F(DiagramContextTest, DiagramState) {
  auto diagram_state = dynamic_cast<DiagramState<double>*>(
      &context_->get_mutable_state());
  ASSERT_NE(nullptr, diagram_state);
  for (SubsystemIndex i(0); i < kNumSystems; ++i) {
    EXPECT_EQ(&context_->GetMutableSubsystemContext(i).get_mutable_state(),
              &diagram_state->get_mutable_substate(i));
  }
}

// Tests that no exception is thrown when connecting a valid source
// and destination port.
TEST_F(DiagramContextTest, ConnectValid) {
  EXPECT_NO_THROW(context_->SubscribeInputPortToOutputPort(
      {SubsystemIndex(0) /* adder0_ */, OutputPortIndex(0)},
      {SubsystemIndex(1) /* adder1_ */, InputPortIndex(1)}));
}

// Tests that input ports can be assigned to the DiagramContext and then
// retrieved.
TEST_F(DiagramContextTest, SetAndGetInputPorts) {
  ASSERT_EQ(2, context_->get_num_input_ports());
  AttachInputPorts();
  EXPECT_EQ(128, ReadVectorInputPort(*context_, 0)->get_value()[0]);
  EXPECT_EQ(256, ReadVectorInputPort(*context_, 1)->get_value()[0]);
}

TEST_F(DiagramContextTest, Clone) {
  context_->SubscribeInputPortToOutputPort(
      {SubsystemIndex(0) /* adder0_ */, OutputPortIndex(0)},
      {SubsystemIndex(1) /* adder1_ */, InputPortIndex(1)});
  AttachInputPorts();

  auto clone = dynamic_pointer_cast<DiagramContext<double>>(context_->Clone());
  ASSERT_TRUE(clone != nullptr);

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the state has the same value.
  VerifyClonedState(clone->get_state());
  // Verify that the parameters have the same value.
  VerifyClonedParameters(clone->get_parameters());

  // Verify that changes to the state do not write through to the original
  // context.
  clone->get_mutable_continuous_state_vector().SetAtIndex(0, 1024.0);
  EXPECT_EQ(1024.0, clone->get_continuous_state()[0]);
  EXPECT_EQ(42.0, context_->get_continuous_state()[0]);

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(2, clone->get_num_input_ports());
  for (int i = 0; i < 2; ++i) {
    const BasicVector<double>* orig_port = ReadVectorInputPort(*context_, i);
    const BasicVector<double>* clone_port = ReadVectorInputPort(*clone, i);
    EXPECT_NE(orig_port, clone_port);
    EXPECT_TRUE(CompareMatrices(orig_port->get_value(), clone_port->get_value(),
                                1e-8, MatrixCompareType::absolute));
  }
}

TEST_F(DiagramContextTest, CloneState) {
  std::unique_ptr<State<double>> state = context_->CloneState();
  // Verify that the state was copied.
  VerifyClonedState(*state);
  // Verify that the underlying type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<DiagramState<double>*>(state.get()));
  // Verify that changes to the state do not write through to the original
  // context.
  state->get_mutable_continuous_state()[1] = 1024.0;
  EXPECT_EQ(1024.0, state->get_continuous_state()[1]);
  EXPECT_EQ(43.0, context_->get_continuous_state()[1]);
}

// Verifies that accuracy is set properly during cloning.
TEST_F(DiagramContextTest, CloneAccuracy) {
  // Verify accuracy is not set by default.
  EXPECT_FALSE(context_->get_accuracy());

  // Verify that setting the accuracy is reflected in cloning.
  const double unity = 1.0;
  context_->set_accuracy(unity);
  std::unique_ptr<Context<double>> clone = context_->Clone();
  EXPECT_EQ(clone->get_accuracy().value(), unity);
}

}  // namespace
}  // namespace systems
}  // namespace drake
