/* clang-format off to disable clang-format-includes */
#include "drake/systems/primitives/gain.h"
/* clang-format on */

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"

using Eigen::AutoDiffScalar;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

// Tests the ability to take derivatives of the output with respect to some
// (not all) of the inputs.
// In this unit test derivatives are taken with respect to the first and third
// input variables. The second input is set to be a constant and therefore
// derivatives with respect to this input are zero.
GTEST_TEST(GainScalarTypeTest, AutoDiff) {
  // There are only two independent variables in this problem with respect to
  // which we want to take derivatives. Therefore Vector2d is the
  // template argument of AutoDiffScalar.
  typedef AutoDiffScalar<Vector2d> T;

  // Set a Gain system with input and output of size 3. Notice that this size
  // does not necessarily need to be the same as the size of the derivatives
  // vectors and, for this unit test, they are not equal.
  const double kGain = 2.0;
  auto gain = make_unique<Gain<T>>(kGain /* gain */, 3 /* size */);
  auto context = gain->CreateDefaultContext();

  // Sets the input values.
  VectorX<T> input_vector(3);
  input_vector << 1.0, 3.14, 2.18;

  // Sets the independent variables to be the first and third input entries.
  input_vector(0).derivatives() << 1, 0;  // First independent variable.
  input_vector(1).derivatives() << 0, 0;  // Constant input.
  input_vector(2).derivatives() << 0, 1;  // Second independent variable.

  gain->get_input_port().FixValue(context.get(), input_vector);

  const auto& output_vector =
      gain->get_output_port().Eval(*context);

  // The expected output value is the gain times the input vector.
  VectorX<T> expected = kGain * input_vector;

  // The expected derivatives are:
  expected(0).derivatives() << kGain, 0.0;
  expected(1).derivatives() << 0.0, 0.0;
  expected(2).derivatives() << 0.0, kGain;

  const double tolerance = Eigen::NumTraits<double>::epsilon();
  for (int i = 0; i < 3; ++i) {
    // Checks output value.
    EXPECT_NEAR(expected(i).value(), output_vector(i).value(), tolerance);

    // Checks derivatives.
    EXPECT_TRUE(expected(i).derivatives().isApprox(
        output_vector(i).derivatives(), tolerance));
  }
}

class SymbolicGainTest : public ::testing::Test {
 protected:
  void SetUp() override {
    gain_ = make_unique<Gain<symbolic::Expression>>(kGain_ /* gain */,
                                                    3 /* length */);
    context_ = gain_->CreateDefaultContext();
    input_ = make_unique<BasicVector<symbolic::Expression>>(3 /* length */);
  }

  double kGain_{2.0};
  unique_ptr<System<symbolic::Expression>> gain_;
  unique_ptr<Context<symbolic::Expression>> context_;
  unique_ptr<BasicVector<symbolic::Expression>> input_;
};

TEST_F(SymbolicGainTest, VectorThroughGainSystem) {
  // Checks that the number of input ports in the Gain system and the Context
  // are consistent.
  EXPECT_EQ(1, gain_->num_input_ports());
  EXPECT_EQ(1, context_->num_input_ports());
  Eigen::Matrix<symbolic::Expression, 3, 1> input_vector(
      drake::symbolic::Expression{1.0}, drake::symbolic::Expression{3.14},
      drake::symbolic::Expression{2.18});

  // Hook input of the expected size.
  gain_->get_input_port(0).FixValue(context_.get(), input_vector);

  // Checks that the number of output ports in the Gain system and the
  // SystemOutput are consistent.
  EXPECT_EQ(1, gain_->num_output_ports());
  Eigen::Matrix<symbolic::Expression, 3, 1> expected{kGain_ * input_vector};
  EXPECT_EQ(expected, gain_->get_output_port(0).Eval(*context_));
  EXPECT_EQ(expected(0).Evaluate(), kGain_ * 1.0);
  EXPECT_EQ(expected(1).Evaluate(), kGain_ * 3.14);
  EXPECT_EQ(expected(2).Evaluate(), kGain_ * 2.18);
}
}  // namespace
}  // namespace systems
}  // namespace drake
