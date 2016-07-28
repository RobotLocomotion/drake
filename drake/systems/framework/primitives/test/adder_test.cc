#include "drake/systems/framework/primitives/adder.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

class AdderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    adder_.reset(new Adder<double>(2 /* inputs */, 3 /* length */));
    context_ = adder_->CreateDefaultContext();
    output_ = adder_->AllocateOutput(*context_);
    input0_.reset(new BasicVector<double>(3 /* length */));
    input1_.reset(new BasicVector<double>(3 /* length */));
  }

  static std::unique_ptr<FreestandingInputPort<double>> MakeInput(
      std::unique_ptr<BasicVector<double>> data) {
    return std::unique_ptr<FreestandingInputPort<double>>(
        new FreestandingInputPort<double>(std::move(data)));
  }

  std::unique_ptr<Adder<double>> adder_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
};

TEST_F(AdderTest, AddTwoVectors) {
  // Hook up two inputs of the expected size.
  ASSERT_EQ(2, context_->get_num_input_ports());
  input0_->get_mutable_value() << 1, 2, 3;
  input1_->get_mutable_value() << 4, 5, 6;
  context_->SetInputPort(0, MakeInput(std::move(input0_)));
  context_->SetInputPort(1, MakeInput(std::move(input1_)));

  adder_->EvalOutput(*context_, output_.get());

  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_port =
      dynamic_cast<const BasicVector<double>*>(
          output_->get_port(0).get_vector_data());
  ASSERT_NE(nullptr, output_port);
  Eigen::Vector3d expected;
  expected << 5, 7, 9;
  EXPECT_EQ(expected, output_port->get_value());
}

// Tests that std::out_of_range is thrown when the wrong number of input ports
// are connected.
TEST_F(AdderTest, WrongNumberOfInputPorts) {
  // Hook up just one input.
  context_->SetInputPort(0, MakeInput(std::move(input0_)));
  EXPECT_THROW(adder_->EvalOutput(*context_, output_.get()), std::out_of_range);
}

// Tests that std::out_of_range is thrown when input ports of the wrong size
// are connected.
TEST_F(AdderTest, WrongSizeOfInputPorts) {
  // Hook up two inputs, but one of them is the wrong size.
  ASSERT_EQ(2, context_->get_num_input_ports());
  std::unique_ptr<BasicVector<double>> short_input(
      new BasicVector<double>(2 /* length */));
  input0_->get_mutable_value() << 1, 2, 3;
  short_input->get_mutable_value() << 4, 5;
  context_->SetInputPort(0, MakeInput(std::move(input0_)));
  context_->SetInputPort(1, MakeInput(std::move(short_input)));

  EXPECT_THROW(adder_->EvalOutput(*context_, output_.get()), std::out_of_range);
}

// Tests that Adder allocates no state variables in the context_.
TEST_F(AdderTest, AdderIsStateless) {
  EXPECT_EQ(nullptr, context_->get_state().continuous_state);
}

}  // namespace
}  // namespace systems
}  // namespace drake
