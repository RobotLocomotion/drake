#include "drake/systems/framework/primitives/constant_value_source.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/value.h"

#include "gtest/gtest.h"

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
    output_ = source_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  static std::unique_ptr<FreestandingInputPort> MakeInput(
      std::unique_ptr<BasicVector<double>> data) {
    return make_unique<FreestandingInputPort>(std::move(data));
  }

  std::unique_ptr<System<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

TEST_F(ConstantValueSourceTest, Output) {
  ASSERT_EQ(source_->get_num_input_ports(), context_->get_num_input_ports());
  ASSERT_EQ(source_->get_num_output_ports(), output_->get_num_ports());

  source_->EvalOutput(*context_, output_.get());

  EXPECT_EQ("foo",
            output_->get_port(0).get_abstract_data()->GetValue<std::string>());
}

// Tests that inputs cannot be set for a ConstantValueSource.
TEST_F(ConstantValueSourceTest, ShouldNotBePossibleToConnectInputs) {
  EXPECT_THROW(context_->SetInputPort(0, MakeInput(std::move(input_))),
               std::out_of_range);
}

// Tests that ConstantValueSource allocates no state variables in the context_.
TEST_F(ConstantValueSourceTest, ConstantValueSourceIsStateless) {
  EXPECT_EQ(nullptr, context_->get_state().continuous_state);
}

}  // namespace
}  // namespace systems
}  // namespace drake
