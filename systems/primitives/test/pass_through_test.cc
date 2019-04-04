#include "drake/systems/primitives/pass_through.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

// A simple type containing a vector, to simplify checking expected values for
// a vector-valued pass through (`BasicValue`) and an abstract-valued pass
// through (`Value<SimpleAbstractType>`).
class SimpleAbstractType {
 public:
  explicit SimpleAbstractType(int size)
      : value_(size) {}
  explicit SimpleAbstractType(const Eigen::VectorXd& value)
      : value_(value) {}
  const Eigen::VectorXd& value() const { return value_; }
 private:
  Eigen::VectorXd value_;
};

class PassThroughTest : public ::testing::TestWithParam<bool> {
 protected:
  PassThroughTest()
      : is_abstract_(GetParam()) {}

  void SetUp() override {
    const int size = 3;
    input_value_.resize(size);
    input_value_ << 1.0, 3.14, 2.18;

    if (!is_abstract_) {
      pass_through_ = make_unique<PassThrough<double>>(size);
    } else {
      pass_through_ =
          make_unique<PassThrough<double>>(Value<SimpleAbstractType>(size));
    }
    context_ = pass_through_->CreateDefaultContext();
  }

  const bool is_abstract_;

  Eigen::VectorXd input_value_;
  std::unique_ptr<System<double>> pass_through_;
  std::unique_ptr<Context<double>> context_;
};

// Tests that the output of this system equals its input.
TEST_P(PassThroughTest, VectorThroughPassThroughSystem) {
  /// Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(1, context_->num_input_ports());
  ASSERT_EQ(1, pass_through_->num_input_ports());

  // Hook input of the expected size.
  if (!is_abstract_) {
    context_->FixInputPort(
        0, std::make_unique<BasicVector<double>>(input_value_));
  } else {
    context_->FixInputPort(
        0, AbstractValue::Make(SimpleAbstractType(input_value_)));
  }

  // Checks that the number of output ports in the system and in the
  // output are consistent.
  ASSERT_EQ(1, pass_through_->num_output_ports());

  Eigen::VectorXd output;
  if (!is_abstract_) {
    output = pass_through_->get_output_port(0).Eval(*context_);
  } else {
    output = pass_through_->get_output_port(0).
        Eval<SimpleAbstractType>(*context_).value();
  }
  EXPECT_EQ(input_value_, output);
}

// Tests that PassThrough allocates no state variables in the context_.
TEST_P(PassThroughTest, PassThroughIsStateless) {
  EXPECT_EQ(0, context_->num_continuous_states());
  EXPECT_EQ(0, context_->num_abstract_states());
  EXPECT_EQ(0, context_->num_discrete_state_groups());
}

// Tests that PassThrough is direct feedthrough.
TEST_P(PassThroughTest, DirectFeedthrough) {
  EXPECT_TRUE(pass_through_->HasAnyDirectFeedthrough());
}

TEST_P(PassThroughTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*pass_through_));
}

TEST_P(PassThroughTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*pass_through_));
}

// Instantiate parameterized test cases for is_abstract_ = {false, true}
INSTANTIATE_TEST_CASE_P(test, PassThroughTest,
    ::testing::Values(false, true));

}  // namespace
}  // namespace systems
}  // namespace drake
