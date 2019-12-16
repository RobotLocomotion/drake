#include "drake/systems/primitives/adder.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

class AdderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    adder_.reset(new Adder<double>(2 /* inputs */, 3 /* size */));
    context_ = adder_->CreateDefaultContext();
    input0_ = Eigen::VectorXd(3 /* size */);
    input1_ = Eigen::VectorXd(3 /* size */);
  }

  std::unique_ptr<Adder<double>> adder_;
  std::unique_ptr<Context<double>> context_;
  Eigen::VectorXd input0_;
  Eigen::VectorXd input1_;
};

// Tests that the system exports the correct topology.
TEST_F(AdderTest, Topology) {
  ASSERT_EQ(2, adder_->num_input_ports());
  for (int i = 0; i < 2; ++i) {
    const InputPort<double>& input_port = adder_->get_input_port(i);
    EXPECT_EQ(kVectorValued, input_port.get_data_type());
    EXPECT_EQ(3, input_port.size());
  }

  ASSERT_EQ(1, adder_->num_output_ports());
  const OutputPort<double>& output_port =
      static_cast<LeafSystem<double>*>(adder_.get())->get_output_port(0);
  EXPECT_EQ(&output_port, &adder_->get_output_port());
  EXPECT_EQ(kVectorValued, output_port.get_data_type());
  EXPECT_EQ(3, output_port.size());
}

// Tests that the system computes the correct sum.
TEST_F(AdderTest, AddTwoVectors) {
  // Hook up two inputs of the expected size.
  ASSERT_EQ(2, context_->num_input_ports());
  input0_ << 1, 2, 3;
  input1_ << 4, 5, 6;
  adder_->get_input_port(0).FixValue(context_.get(), input0_);
  adder_->get_input_port(1).FixValue(context_.get(), input1_);

  Eigen::Vector3d expected(5, 7, 9);
  EXPECT_EQ(expected, adder_->get_output_port().Eval(*context_));
}

// Tests that Adder allocates no state variables in the context_.
TEST_F(AdderTest, AdderIsStateless) {
  EXPECT_EQ(0, context_->num_continuous_states());
}

// Asserts that adders have direct-feedthrough from all inputs to the output.
TEST_F(AdderTest, AdderIsDirectFeedthrough) {
  EXPECT_TRUE(adder_->HasAnyDirectFeedthrough());
  for (int i = 0; i < adder_->num_input_ports(); ++i) {
    EXPECT_TRUE(adder_->HasDirectFeedthrough(i, 0));
  }
}

TEST_F(AdderTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*adder_));
}

TEST_F(AdderTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*adder_));
}

}  // namespace
}  // namespace systems
}  // namespace drake
