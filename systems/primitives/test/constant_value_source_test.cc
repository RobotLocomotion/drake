#include "drake/systems/primitives/constant_value_source.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/framework/value.h"

using Eigen::Matrix;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

class ConstantValueSourceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<AbstractValue> value(new Value<std::string>("foo"));
    source_ = make_unique<ConstantValueSource<double>>(std::move(value));
    context_ = source_->CreateDefaultContext();
    output_ = source_->get_output_port(0).Allocate(*context_);
    input_ = make_unique<BasicVector<double>>(3 /* size */);
  }

  std::unique_ptr<System<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<AbstractValue> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

TEST_F(ConstantValueSourceTest, Output) {
  ASSERT_EQ(source_->get_num_input_ports(), context_->get_num_input_ports());

  source_->get_output_port(0).Calc(*context_, output_.get());

  EXPECT_EQ("foo", output_->GetValue<std::string>());
}

// Tests that ConstantValueSource allocates no state variables in the context_.
TEST_F(ConstantValueSourceTest, ConstantValueSourceIsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state().size());
}

// Tests conversion to different scalar types.
TEST_F(ConstantValueSourceTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*source_));
}

TEST_F(ConstantValueSourceTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*source_));
}

}  // namespace
}  // namespace systems
}  // namespace drake
