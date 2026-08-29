#include "drake/systems/primitives/pass_through.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
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
  explicit SimpleAbstractType(int size) : value_(Eigen::VectorXd::Zero(size)) {}
  explicit SimpleAbstractType(const Eigen::VectorXd& value) : value_(value) {}
  const Eigen::VectorXd& value() const { return value_; }

 private:
  Eigen::VectorXd value_;
};

class PassThroughTest
    : public ::testing::TestWithParam<std::tuple<bool, bool>> {
 protected:
  PassThroughTest()
      : is_abstract_(std::get<0>(GetParam())),
        use_default_value_(std::get<1>(GetParam())) {}

  void SetUp() override {
    const int size = 3;
    default_value_.resize(size);
    default_value_ << -.23, 0.5, 3.14;
    input_value_.resize(size);
    input_value_ << 1.0, 3.14, 2.18;

    if (!is_abstract_) {
      if (use_default_value_) {
        pass_through_ = make_unique<PassThrough<double>>(default_value_);
      } else {
        pass_through_ = make_unique<PassThrough<double>>(size);
      }
    } else {
      if (use_default_value_) {
        pass_through_ = make_unique<PassThrough<double>>(
            Value<SimpleAbstractType>(default_value_));
      } else {
        pass_through_ =
            make_unique<PassThrough<double>>(Value<SimpleAbstractType>(size));
      }
    }
    EXPECT_FALSE(
        dynamic_cast<const PassThrough<double>&>(*pass_through_)
            .input_required());
    context_ = pass_through_->CreateDefaultContext();
  }

  const bool is_abstract_;
  const bool use_default_value_;

  Eigen::VectorXd default_value_;
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

  // Test default values.
  Eigen::VectorXd output;
  if (!is_abstract_) {
    output = pass_through_->get_output_port(0).Eval(*context_);
  } else {
    output = pass_through_->get_output_port(0)
                 .Eval<SimpleAbstractType>(*context_)
                 .value();
  }
  if (use_default_value_) {
    EXPECT_EQ(output, default_value_);
  } else {
    EXPECT_TRUE(output.isZero());
  }

  // Hook input of the expected size.
  if (!is_abstract_) {
    pass_through_->get_input_port(0).FixValue(context_.get(), input_value_);
  } else {
    pass_through_->get_input_port(0).FixValue(context_.get(),
                                              SimpleAbstractType(input_value_));
  }

  // Checks that the number of output ports in the system and in the
  // output are consistent.
  ASSERT_EQ(1, pass_through_->num_output_ports());

  if (!is_abstract_) {
    output = pass_through_->get_output_port(0).Eval(*context_);
  } else {
    output = pass_through_->get_output_port(0)
                 .Eval<SimpleAbstractType>(*context_)
                 .value();
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
  // Note: The system will report feedthrough even when no input is connected.
  EXPECT_TRUE(pass_through_->HasAnyDirectFeedthrough());
}

TEST_P(PassThroughTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*pass_through_));
}

TEST_P(PassThroughTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*pass_through_));
}

// Instantiate parameterized test cases for is_abstract_ = {false, true}
INSTANTIATE_TEST_SUITE_P(test, PassThroughTest,
                         ::testing::Values(std::make_tuple(false, false),
                                           std::make_tuple(false, true),
                                           std::make_tuple(true, false),
                                           std::make_tuple(true, true)));

GTEST_TEST(PassThroughTest, AutoDiffFromDouble) {
  const Eigen::Vector3d value(1., 3., 0.42);
  PassThrough<AutoDiffXd> pass(value);
  auto context = pass.CreateDefaultContext();
  EXPECT_EQ(value,
            math::DiscardZeroGradient(pass.get_output_port().Eval(*context)));
}

GTEST_TEST(PassThroughRequiredInputTest, VectorRequiredConnected) {
  const Eigen::Vector3d value(1.0, 2.0, 3.0);
  const Eigen::Vector3d input(4.0, 5.0, 6.0);
  PassThrough<double> dut(value, true);
  EXPECT_TRUE(dut.input_required());
  auto context = dut.CreateDefaultContext();
  dut.get_input_port().FixValue(context.get(), input);
  EXPECT_EQ(dut.get_output_port().Eval(*context), input);
}

GTEST_TEST(PassThroughRequiredInputTest, VectorRequiredUnconnectedThrows) {
  PassThrough<double> dut(3, true);
  EXPECT_TRUE(dut.input_required());
  auto context = dut.CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_output_port().Eval(*context),
                              ".*required InputPort.*\\(u\\).*is not connected.*");
}

GTEST_TEST(PassThroughRequiredInputTest, AbstractRequiredConnected) {
  const Eigen::Vector3d value(1.0, 2.0, 3.0);
  const Eigen::Vector3d input(4.0, 5.0, 6.0);
  PassThrough<double> dut(Value<SimpleAbstractType>(value), true);
  EXPECT_TRUE(dut.input_required());
  auto context = dut.CreateDefaultContext();
  dut.get_input_port().FixValue(context.get(), SimpleAbstractType(input));
  EXPECT_EQ(dut.get_output_port().Eval<SimpleAbstractType>(*context).value(),
            input);
}

GTEST_TEST(PassThroughRequiredInputTest, AbstractRequiredUnconnectedThrows) {
  PassThrough<double> dut(Value<SimpleAbstractType>(3), true);
  EXPECT_TRUE(dut.input_required());
  auto context = dut.CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_output_port().Eval(*context),
                              ".*required InputPort.*\\(u\\).*is not connected.*");
}

GTEST_TEST(PassThroughRequiredInputTest, ScalarConversionPreservesFlag) {
  PassThrough<double> dut(2, true);
  EXPECT_TRUE(is_autodiffxd_convertible(dut, [](const auto& converted) {
    EXPECT_TRUE(converted.input_required());
  }));
  EXPECT_TRUE(is_symbolic_convertible(dut, [](const auto& converted) {
    EXPECT_TRUE(converted.input_required());
  }));
}

}  // namespace
}  // namespace systems
}  // namespace drake
