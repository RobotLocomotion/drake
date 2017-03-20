#include "drake/systems/framework/siso_vector_system.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

using Eigen::VectorXd;

// For all of the output / xdot / x[n+1] functions, the result is set to be
// either (1) equal to the input, when there is no declared state or (2) equal
// to the input plus the state, when there is declared state.  Unlike most
// Systems, this System allows test code to configure its ports and state even
// beyond what the constructor does by default.  For all of the Siso overrides,
// a call count is retained for tests to examine.
class TestSisoSystem : public SisoVectorSystem<double> {
 public:
  static constexpr int kSize = 2;

  TestSisoSystem() : SisoVectorSystem<double>(kSize, kSize) {}

  // Let test code abuse these by making them public.
  using SisoVectorSystem<double>::DeclareContinuousState;
  using SisoVectorSystem<double>::DeclareDiscreteState;
  using SisoVectorSystem<double>::DeclareAbstractInputPort;
  using SisoVectorSystem<double>::DeclareAbstractOutputPort;

  // Siso override.
  // N.B. This method signature might be used by many downstream projects.
  // Change it only with good reason and with a deprecation period first.
  void DoCalcVectorOutput(
      const Context<double>& context,
      const Eigen::VectorBlock<const VectorXd>& input,
      const Eigen::VectorBlock<const VectorXd>& state,
      Eigen::VectorBlock<VectorXd>* output) const override {
    ++output_count_;
    last_context_ = &context;
    if (state.size() == 0) {
      *output = input;
    } else {
      *output = input + state;
    }
  }

  // Siso override.
  // N.B. This method signature might be used by many downstream projects.
  // Change it only with good reason and with a deprecation period first.
  void DoCalcVectorTimeDerivatives(
      const Context<double>& context,
      const Eigen::VectorBlock<const VectorXd>& input,
      const Eigen::VectorBlock<const VectorXd>& state,
      Eigen::VectorBlock<VectorXd>* derivatives) const override {
    ++time_derivatives_count_;
    last_context_ = &context;
    if (state.size() == 0) {
      *derivatives = input;
    } else {
      *derivatives = input + state;
    }
  }

  // Siso override.
  // N.B. This method signature might be used by many downstream projects.
  // Change it only with good reason and with a deprecation period first.
  void DoCalcVectorDiscreteVariableUpdates(
      const Context<double>& context,
      const Eigen::VectorBlock<const VectorXd>& input,
      const Eigen::VectorBlock<const VectorXd>& state,
      Eigen::VectorBlock<VectorXd>* discrete_updates) const override {
    ++discrete_variable_updates_count_;
    last_context_ = &context;
    if (state.size() == 0) {
      *discrete_updates = input;
    } else {
      *discrete_updates = input + state;
    }
  }

  // LeafSystem override.
  std::unique_ptr<DiscreteState<double>> AllocateDiscreteState()
      const override {
    return prototype_discrete_state_->Clone();
  }

  // LeafSystem override.
  std::unique_ptr<AbstractValues> AllocateAbstractState() const override {
    return prototype_abstract_state_->Clone();
  }

  // Use the given number of discrete state groups for AllocateDiscreteState;
  // when this has not been called, there will be no discrete state.
  void set_prototype_discrete_state_count(int count) {
    std::vector<std::unique_ptr<BasicVector<double>>> vec;
    for (int i = 0; i < count; ++i) {
      vec.emplace_back(std::make_unique<BasicVector<double>>(kSize));
    }
    prototype_discrete_state_ =
        std::make_unique<DiscreteState<double>>(std::move(vec));
  }

  // Use a single Value<S>(value) for AllocateAbstractState; when this has not
  // been called, there will be no abstract state.
  template <typename S>
  void set_prototype_abstract_state(const S& value) {
    std::vector<std::unique_ptr<AbstractValue>> vec;
    vec.emplace_back(std::move(std::make_unique<Value<S>>(value)));
    prototype_abstract_state_ =
        std::make_unique<AbstractValues>(std::move(vec));
  }

  // Testing accessors.
  const Context<double>* get_last_context() const { return last_context_; }
  int get_output_count() const { return output_count_; }
  int get_time_derivatives_count() const { return time_derivatives_count_; }
  int get_discrete_variable_updates_count() const {
    return discrete_variable_updates_count_;
  }

 private:
  std::unique_ptr<DiscreteState<double>> prototype_discrete_state_{
    std::make_unique<DiscreteState<double>>()};
  std::unique_ptr<AbstractValues> prototype_abstract_state_{
      std::make_unique<AbstractValues>()};
  mutable const Context<double>* last_context_{nullptr};
  mutable int output_count_{0};
  mutable int time_derivatives_count_{0};
  mutable int discrete_variable_updates_count_{0};
};
constexpr int TestSisoSystem::kSize;

class SisoVectorSystemTest : public ::testing::Test {
  // Not yet needed, but placeholder for future common code.
};

// The system has an appropriate topology.
TEST_F(SisoVectorSystemTest, Topology) {
  // The device-under-test ("dut").
  TestSisoSystem dut;

  // One input port.
  ASSERT_EQ(dut.get_num_input_ports(), 1);
  const InputPortDescriptor<double>& descriptor_in = dut.get_input_port();
  EXPECT_EQ(descriptor_in.get_data_type(), kVectorValued);
  EXPECT_EQ(descriptor_in.size(), TestSisoSystem::kSize);

  // One output port.
  ASSERT_EQ(dut.get_num_output_ports(), 1);
  const OutputPortDescriptor<double>& descriptor_out = dut.get_output_port();
  EXPECT_EQ(descriptor_out.get_data_type(), kVectorValued);
  EXPECT_EQ(descriptor_out.size(), TestSisoSystem::kSize);

  // No state by default ...
  auto context = dut.CreateDefaultContext();
  ASSERT_EQ(context->get_num_input_ports(), 1);
  EXPECT_TRUE(context->is_stateless());

  // ... but subclasses may declare it.
  dut.DeclareContinuousState(TestSisoSystem::kSize);
  context = dut.CreateDefaultContext();
  EXPECT_FALSE(context->is_stateless());
}

// Subclasses that violate the SISO premise fail-fast.
TEST_F(SisoVectorSystemTest, TopologyFailFast) {
  { // A second input.
    TestSisoSystem dut;
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.DeclareAbstractInputPort();
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }

  { // A second output.
    TestSisoSystem dut;
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.DeclareAbstractOutputPort();
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }

  { // Abstract state.
    // TODO(jwnimmer-tri) Conceptually, allowing abstract state could be
    // consistent with the SISO contract.  However, if we widened the SISO
    // contract to support that, then we would have add code to fail-fast if
    // the subclass neglected to override the unrestricted updates API.  That
    // doesn't seem worth it yet, but if we ever want abstract state in SISO,
    // then it would be okay to write the code and tests to support it.
    TestSisoSystem dut;
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.set_prototype_abstract_state<double>(1.0);
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }

  { // More than one discrete state group.
    TestSisoSystem dut;
    dut.set_prototype_discrete_state_count(1);
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.set_prototype_discrete_state_count(2);
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }

  { // Both continuous and discrete state.
    TestSisoSystem dut;
    dut.DeclareContinuousState(1);
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.set_prototype_discrete_state_count(1);
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }
}

// Forwarding of CalcOutput with no state.
TEST_F(SisoVectorSystemTest, OutputStateless) {
  TestSisoSystem dut;
  auto context = dut.CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);
  context->FixInputPort(0, BasicVector<double>::Make({1, 2}));
  dut.CalcOutput(*context, output.get());
  EXPECT_EQ(dut.get_output_count(), 1);
  EXPECT_EQ(dut.get_last_context(), context.get());
  EXPECT_EQ(output->get_vector_data(0)->GetAtIndex(0), 1.0);
  EXPECT_EQ(output->get_vector_data(0)->GetAtIndex(1), 2.0);
}

// Forwarding of CalcOutput with continuous state.
TEST_F(SisoVectorSystemTest, OutputContinuous) {
  TestSisoSystem dut;
  dut.DeclareContinuousState(TestSisoSystem::kSize);
  auto context = dut.CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);
  context->FixInputPort(0, BasicVector<double>::Make({1, 2}));
  context->get_mutable_continuous_state_vector()->SetFromVector(
      Eigen::Vector2d::Ones());
  dut.CalcOutput(*context, output.get());
  EXPECT_EQ(dut.get_output_count(), 1);
  EXPECT_EQ(dut.get_last_context(), context.get());
  EXPECT_EQ(output->get_vector_data(0)->GetAtIndex(0), 2.0);
  EXPECT_EQ(output->get_vector_data(0)->GetAtIndex(1), 3.0);
}

// Forwarding of CalcOutput with discrete state.
TEST_F(SisoVectorSystemTest, OutputDiscrete) {
  TestSisoSystem dut;
  dut.set_prototype_discrete_state_count(1);
  auto context = dut.CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);
  context->FixInputPort(0, BasicVector<double>::Make({1, 2}));
  context->get_mutable_discrete_state(0)->SetFromVector(
      Eigen::Vector2d::Ones());
  dut.CalcOutput(*context, output.get());
  EXPECT_EQ(dut.get_output_count(), 1);
  EXPECT_EQ(dut.get_last_context(), context.get());
  EXPECT_EQ(output->get_vector_data(0)->GetAtIndex(0), 2.0);
  EXPECT_EQ(output->get_vector_data(0)->GetAtIndex(1), 3.0);

  // Nothing else weird happened.
  EXPECT_EQ(dut.get_discrete_variable_updates_count(), 0);
  EXPECT_EQ(dut.get_time_derivatives_count(), 0);
}

// Forwarding of CalcTimeDerivatives works.
TEST_F(SisoVectorSystemTest, TimeDerivatives) {
  TestSisoSystem dut;
  auto context = dut.CreateDefaultContext();
  std::unique_ptr<ContinuousState<double>> derivatives =
      dut.AllocateTimeDerivatives();

  // There's no state declared yet, so the Siso base shouldn't call our DUT.
  dut.CalcTimeDerivatives(*context, derivatives.get());
  EXPECT_EQ(dut.get_time_derivatives_count(), 0);

  // Now we have state, so the Siso base should call our DUT.
  dut.DeclareContinuousState(TestSisoSystem::kSize);
  context = dut.CreateDefaultContext();
  context->FixInputPort(0, BasicVector<double>::Make({1, 2}));
  context->get_mutable_continuous_state_vector()->SetFromVector(
      Eigen::Vector2d::Ones());
  derivatives = dut.AllocateTimeDerivatives();
  dut.CalcTimeDerivatives(*context, derivatives.get());
  EXPECT_EQ(dut.get_last_context(), context.get());
  EXPECT_EQ(dut.get_time_derivatives_count(), 1);
  EXPECT_EQ(derivatives->get_vector().GetAtIndex(0), 2.0);
  EXPECT_EQ(derivatives->get_vector().GetAtIndex(1), 3.0);

  // Nothing else weird happened.
  EXPECT_EQ(dut.get_discrete_variable_updates_count(), 0);
  EXPECT_EQ(dut.get_output_count(), 0);
}

// Forwarding of CalcDiscreteVariableUpdates works.
TEST_F(SisoVectorSystemTest, DiscreteVariableUpdates) {
  TestSisoSystem dut;
  auto context = dut.CreateDefaultContext();
  auto discrete_updates = dut.AllocateDiscreteVariables();
  DiscreteEvent<double> event;
  event.action = DiscreteEvent<double>::kDiscreteUpdateAction;

  // There's no state declared yet, so the Siso base shouldn't call our DUT.
  dut.CalcDiscreteVariableUpdates(*context, event, discrete_updates.get());
  EXPECT_EQ(dut.get_discrete_variable_updates_count(), 0);

  // Now we have state, so the Siso base should call our DUT.
  dut.set_prototype_discrete_state_count(1);
  context = dut.CreateDefaultContext();
  context->FixInputPort(0, BasicVector<double>::Make({1, 2}));
  context->get_mutable_discrete_state(0)->SetFromVector(
      Eigen::Vector2d::Ones());
  discrete_updates = dut.AllocateDiscreteVariables();
  dut.CalcDiscreteVariableUpdates(*context, event, discrete_updates.get());
  EXPECT_EQ(dut.get_last_context(), context.get());
  EXPECT_EQ(dut.get_discrete_variable_updates_count(), 1);
  EXPECT_EQ(
      discrete_updates->get_mutable_discrete_state(0)->GetAtIndex(0),
      2.0);
  EXPECT_EQ(
      discrete_updates->get_mutable_discrete_state(0)->GetAtIndex(1),
      3.0);

  // Nothing else weird happened.
  EXPECT_EQ(dut.get_time_derivatives_count(), 0);
  EXPECT_EQ(dut.get_output_count(), 0);
}

}  // namespace
}  // namespace systems
}  // namespace drake
