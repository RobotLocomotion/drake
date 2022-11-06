#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

// This is used for friendship, so can't be anonymous.
class LeafSystemDeprecationTest : public ::testing::Test {
 protected:
  class PortDeprecationSystem : public LeafSystem<double> {
   public:
    using LeafSystem::DeclareVectorInputPort;
    using LeafSystem::DeclareVectorOutputPort;
    using LeafSystem::DeprecateInputPort;
    using LeafSystem::DeprecateOutputPort;
  };

  InputPort<double>& DeclareDeprecatedInput(const std::string& message = "") {
    auto& result = dut_.DeclareVectorInputPort(kUseDefaultName, 1);
    dut_.DeprecateInputPort(result, message);
    return result;
  }

  OutputPort<double>& DeclareDeprecatedOutput(const std::string& message = "") {
    auto& result = dut_.DeclareVectorOutputPort(
        kUseDefaultName, 1, noop_calc_, {SystemBase::all_input_ports_ticket()});
    dut_.DeprecateOutputPort(result, message);
    return result;
  }

  bool has_warned(const PortBase& port) const {
    using internal::PortBaseAttorney;
    PortBase& mutable_port = const_cast<PortBase&>(port);
    return PortBaseAttorney::deprecation_already_warned(&mutable_port)->load();
  }

  PortDeprecationSystem dut_;

  const LeafOutputPort<double>::CalcVectorCallback noop_calc_ =
      [](const Context<double>&, BasicVector<double>*) {};
};

namespace {

// ======== Positive test cases: we want these to generate warnings ========

// Give a deprecation message (or not) and make sure nothing crashes.
// You can manually look at the test's output to see what it looks like.
TEST_F(LeafSystemDeprecationTest, Messages) {
  // Use default messages.
  EXPECT_NO_THROW(DeclareDeprecatedInput());
  EXPECT_NO_THROW(DeclareDeprecatedOutput());

  // Use custom messages.
  EXPECT_NO_THROW(DeclareDeprecatedInput("this string gets logged"));
  EXPECT_NO_THROW(DeclareDeprecatedOutput("this string gets logged, too"));

  // Log the warnings.
  EXPECT_NO_THROW(dut_.get_input_port(0));
  EXPECT_NO_THROW(dut_.get_input_port(1));
  EXPECT_NO_THROW(dut_.get_output_port(0));
  EXPECT_NO_THROW(dut_.get_output_port(1));
}

// Accessing input by index triggers the message.
TEST_F(LeafSystemDeprecationTest, InputByIndex) {
  auto& port = DeclareDeprecatedInput();
  EXPECT_EQ(has_warned(port), false);
  dut_.get_input_port(0);
  EXPECT_EQ(has_warned(port), true);
}

// Accessing base input by index triggers the message.
TEST_F(LeafSystemDeprecationTest, BaseInputByIndex) {
  auto& port = DeclareDeprecatedInput();
  EXPECT_EQ(has_warned(port), false);
  dut_.get_input_port_base(InputPortIndex{0});
  EXPECT_EQ(has_warned(port), true);
}

// Accessing input ticket by index triggers the message.
TEST_F(LeafSystemDeprecationTest, TicketInputByIndex) {
  auto& port = DeclareDeprecatedInput();
  EXPECT_EQ(has_warned(port), false);
  dut_.input_port_ticket(InputPortIndex{0});
  EXPECT_EQ(has_warned(port), true);
}

// Accessing output by index triggers the message.
TEST_F(LeafSystemDeprecationTest, OutputByIndex) {
  auto& port = DeclareDeprecatedOutput();
  EXPECT_EQ(has_warned(port), false);
  dut_.get_output_port(0);
  EXPECT_EQ(has_warned(port), true);
}

// Accessing base output by index triggers the message.
TEST_F(LeafSystemDeprecationTest, BaseOutputByIndex) {
  auto& port = DeclareDeprecatedOutput();
  EXPECT_EQ(has_warned(port), false);
  dut_.get_output_port_base(OutputPortIndex{0});
  EXPECT_EQ(has_warned(port), true);
}

// Accessing input by name triggers the message.
TEST_F(LeafSystemDeprecationTest, InputByName) {
  auto& port = DeclareDeprecatedInput();
  EXPECT_EQ(has_warned(port), false);
  dut_.GetInputPort("u0");
  EXPECT_EQ(has_warned(port), true);
}

// Checking for input by name triggers the message.
TEST_F(LeafSystemDeprecationTest, HasInputByName) {
  auto& port = DeclareDeprecatedInput();
  EXPECT_EQ(has_warned(port), false);
  dut_.HasInputPort("u0");
  EXPECT_EQ(has_warned(port), true);
}

// Accessing output by name triggers the message.
TEST_F(LeafSystemDeprecationTest, OutputByName) {
  auto& port = DeclareDeprecatedOutput();
  EXPECT_EQ(has_warned(port), false);
  dut_.GetOutputPort("y0");
  EXPECT_EQ(has_warned(port), true);
}

// Checking for output by name triggers the message.
TEST_F(LeafSystemDeprecationTest, HasOutputByName) {
  auto& port = DeclareDeprecatedOutput();
  EXPECT_EQ(has_warned(port), false);
  dut_.HasOutputPort("y0");
  EXPECT_EQ(has_warned(port), true);
}

// Evaluating the input (using the dispreferred SystemBase function) triggers
// the message.
TEST_F(LeafSystemDeprecationTest, InputEval) {
  auto& port = DeclareDeprecatedInput();
  auto context = dut_.CreateDefaultContext();
  EXPECT_EQ(has_warned(port), false);
  dut_.EvalAbstractInput(*context, 0);
  EXPECT_EQ(has_warned(port), true);
}

// ======== Negative test cases: we want these to NOT generate warnings ========

// Non-deprecated input ports don't trigger the message.
TEST_F(LeafSystemDeprecationTest, OtherInputNoSpurious) {
  auto& in0 = DeclareDeprecatedInput();
  auto& in1 = dut_.DeclareVectorInputPort("ok", 1);
  EXPECT_EQ(has_warned(in0), false);
  EXPECT_EQ(has_warned(in1), false);
  dut_.HasInputPort("ok");
  dut_.GetInputPort("ok");
  EXPECT_EQ(has_warned(in0), false);
  EXPECT_EQ(has_warned(in1), false);
}

// Non-deprecated output ports don't trigger the message.
TEST_F(LeafSystemDeprecationTest, OtherOutputNoSpurious) {
  auto& out0 = DeclareDeprecatedOutput();
  auto& out1 = dut_.DeclareVectorOutputPort("ok", 1, noop_calc_);
  EXPECT_EQ(has_warned(out0), false);
  EXPECT_EQ(has_warned(out1), false);
  dut_.HasOutputPort("ok");
  dut_.GetOutputPort("ok");
  EXPECT_EQ(has_warned(out0), false);
  EXPECT_EQ(has_warned(out1), false);
}

// Multiple inputs don't trigger the message when checking for duplicate names.
TEST_F(LeafSystemDeprecationTest, HeterogeneousInputNoSpurious) {
  auto& port0 = DeclareDeprecatedInput();
  auto& port1 = DeclareDeprecatedInput();
  EXPECT_EQ(has_warned(port0), false);
  EXPECT_EQ(has_warned(port1), false);
}

// Multiple outputs don't trigger the message when checking for duplicate names.
TEST_F(LeafSystemDeprecationTest, HeterogeneousOutputNoSpurious) {
  auto& port0 = DeclareDeprecatedOutput();
  auto& port1 = DeclareDeprecatedOutput();
  EXPECT_EQ(has_warned(port0), false);
  EXPECT_EQ(has_warned(port1), false);
}

// Feedthrough calculations don't trigger the message.
TEST_F(LeafSystemDeprecationTest, FeedthroughNoSpurious) {
  auto& in0 = DeclareDeprecatedInput();
  auto& out0 = DeclareDeprecatedOutput();
  dut_.GetDirectFeedthroughs();
  EXPECT_EQ(has_warned(in0), false);
  EXPECT_EQ(has_warned(out0), false);
}

// Fixed input port allocation doesn't trigger the message.
TEST_F(LeafSystemDeprecationTest, FixedAllocateNoSpurious) {
  auto& in0 = DeclareDeprecatedInput();
  auto context = dut_.CreateDefaultContext();
  dut_.AllocateFixedInputs(context.get());
  EXPECT_EQ(has_warned(in0), false);
}

// Fixed input port bulk setting doesn't trigger the message.
TEST_F(LeafSystemDeprecationTest, FixedFromNoSpurious) {
  PortDeprecationSystem other;
  auto& other_in = other.DeclareVectorInputPort("ok", 1);
  auto other_context = other.CreateDefaultContext();
  other.get_input_port().FixValue(
      other_context.get(), Eigen::VectorXd::Constant(1, 22.0));

  auto& dut_in = DeclareDeprecatedInput();
  auto dut_context = dut_.CreateDefaultContext();
  dut_.FixInputPortsFrom(other, *other_context, dut_context.get());
  EXPECT_EQ(has_warned(other_in), false);
  EXPECT_EQ(has_warned(dut_in), false);
}

// Bulk output calculation doesn't trigger the message.
TEST_F(LeafSystemDeprecationTest, CalcOutputNoSpurious) {
  auto& port = DeclareDeprecatedOutput();
  auto context = dut_.CreateDefaultContext();
  auto output = dut_.AllocateOutput();
  dut_.CalcOutput(*context, output.get());
  EXPECT_EQ(has_warned(port), false);
}

}  // namespace
}  // namespace systems
}  // namespace drake
