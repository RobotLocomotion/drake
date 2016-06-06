#include "drake/systems/framework/primitives/adder.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/systems/framework/basic_vector.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

class AdderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    adder_.reset(new Adder<double>(2 /* inputs */, 3 /* length */));
    context_ = adder_->CreateDefaultContext();
    output_ = adder_->AllocateOutput();
  }

  std::unique_ptr<Adder<double>> adder_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

TEST_F(AdderTest, AddTwoVectors) {
  // Hook up two inputs of the expected size.
  ASSERT_EQ(2, context_->get_mutable_input()->ports.size());
  BasicVector<double> input0(3 /* length */);
  BasicVector<double> input1(3 /* length */);
  input0.get_mutable_value() << 1, 2, 3;
  input1.get_mutable_value() << 4, 5, 6;
  context_->get_mutable_input()->ports[0].vector_input = &input0;
  context_->get_mutable_input()->ports[1].vector_input = &input1;

  adder_->Output(*context_, output_.get());

  ASSERT_EQ(1, output_->ports.size());
  BasicVector<double>* output_port = dynamic_cast<BasicVector<double>*>(
      output_->ports[0].vector_output.get());
  ASSERT_NE(nullptr, output_port);
  Eigen::Matrix<double, 3, 1> expected;
  expected << 5, 7, 9;
  EXPECT_EQ(expected, output_port->get_value());
}

// Tests that std::out_of_range is thrown when the wrong number of input ports
// are connected.
TEST_F(AdderTest, WrongNumberOfInputPorts) {
  // Hook up just one input.
  BasicVector<double> input0(3 /* length */);
  context_->get_mutable_input()->ports[0].vector_input = &input0;

  EXPECT_THROW(adder_->Output(*context_, output_.get()), std::out_of_range);
}

// Tests that std::out_of_range is thrown when input ports of the wrong size
// are connected.
TEST_F(AdderTest, WrongSizeOfInputPorts) {
  // Hook up two inputs, but one of them is the wrong size.
  ASSERT_EQ(2, context_->get_mutable_input()->ports.size());
  BasicVector<double> input0(3 /* length */);
  BasicVector<double> input1(2 /* length */);
  context_->get_mutable_input()->ports[0].vector_input = &input0;
  context_->get_mutable_input()->ports[1].vector_input = &input1;

  EXPECT_THROW(adder_->Output(*context_, output_.get()), std::out_of_range);
}

// Tests that Adder allocates no state variables in the context_.
TEST_F(AdderTest, AdderIsStateless) {
  /// TODO(david-german-tri): Once state exists in Context, assert it's empty.
}

}  // namespace
}  // namespace systems
}  // namespace drake
