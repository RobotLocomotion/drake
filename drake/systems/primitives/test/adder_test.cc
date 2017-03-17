#include "drake/systems/primitives/adder.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/system_port_descriptor.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

class AdderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    adder_.reset(new Adder<double>(2 /* inputs */, 3 /* size */));
    context_ = adder_->CreateDefaultContext();
    output_ = adder_->AllocateOutput(*context_);
    input0_.reset(new BasicVector<double>(3 /* size */));
    input1_.reset(new BasicVector<double>(3 /* size */));
  }

  std::unique_ptr<System<double>> adder_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
};

// Tests that the system exports the correct topology.
TEST_F(AdderTest, Topology) {
  ASSERT_EQ(2, adder_->get_num_input_ports());
  for (int i = 0; i < 2; ++i) {
    const InputPortDescriptor<double>& descriptor = adder_->get_input_port(i);
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(3, descriptor.size());
  }

  ASSERT_EQ(1, adder_->get_num_output_ports());
  const OutputPortDescriptor<double>& descriptor = adder_->get_output_port(0);
  EXPECT_EQ(kVectorValued, descriptor.get_data_type());
  EXPECT_EQ(3, descriptor.size());
}

// Tests that the system computes the correct sum.
TEST_F(AdderTest, AddTwoVectors) {
  // Hook up two inputs of the expected size.
  ASSERT_EQ(2, context_->get_num_input_ports());
  input0_->get_mutable_value() << 1, 2, 3;
  input1_->get_mutable_value() << 4, 5, 6;
  context_->FixInputPort(0, std::move(input0_));
  context_->FixInputPort(1, std::move(input1_));

  adder_->CalcOutput(*context_, output_.get());

  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_port = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_port);
  Eigen::Vector3d expected(5, 7, 9);
  EXPECT_EQ(expected, output_port->get_value());
}

// Tests that Adder allocates no state variables in the context_.
TEST_F(AdderTest, AdderIsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state()->size());
}

// Asserts that adders are systems with direct feedthrough inputs.
TEST_F(AdderTest, AdderIsDirectFeedthrough) {
  EXPECT_TRUE(adder_->has_any_direct_feedthrough());
}

}  // namespace
}  // namespace systems
}  // namespace drake
