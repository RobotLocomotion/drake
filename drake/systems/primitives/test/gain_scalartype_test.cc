#include "drake/systems/primitives/gain-inl.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

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
  // which we want to take derivatives. Therefore Vector2d_unaligned is the
  // template argument of AutoDiffScalar.
  // TODO(amcastro-tri): change to Vector2d once #3145 is fixed.
  typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vector2d_unaligned;
  typedef AutoDiffScalar<Vector2d_unaligned> T;

  // Set a Gain system with input and output of size 3. Notice that this size
  // does not necessarily need to be the same as the size of the derivatives
  // vectors and, for this unit test, they are not equal.
  const double kGain = 2.0;
  auto gain = make_unique<Gain<T>>(kGain /* gain */, 3 /* size */);
  auto context = gain->CreateDefaultContext();
  auto output = gain->AllocateOutput(*context);
  auto input = make_unique<BasicVector<T>>(3 /* size */);

  // Sets the input values.
  VectorX<T> input_vector(3);
  input_vector << 1.0, 3.14, 2.18;

  // Sets the independent variables to be the first and third input entries.
  input_vector(0).derivatives() << 1, 0;  // First independent variable.
  input_vector(1).derivatives() << 0, 0;  // Constant input.
  input_vector(2).derivatives() << 0, 1;  // Second independent variable.

  input->get_mutable_value() << input_vector;

  context->FixInputPort(0, std::move(input));

  gain->CalcOutput(*context, output.get());

  ASSERT_EQ(1, output->get_num_ports());
  const auto& output_vector = output->get_vector_data(0)->get_value();

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
    output_ = gain_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<symbolic::Expression>>(3 /* length */);
  }

  const symbolic::Expression kGain_{2.0};
  unique_ptr<System<symbolic::Expression>> gain_;
  unique_ptr<Context<symbolic::Expression>> context_;
  unique_ptr<SystemOutput<symbolic::Expression>> output_;
  unique_ptr<BasicVector<symbolic::Expression>> input_;
};

TEST_F(SymbolicGainTest, VectorThroughGainSystem) {
  // Checks that the number of input ports in the Gain system and the Context
  // are consistent.
  EXPECT_EQ(1, gain_->get_num_input_ports());
  EXPECT_EQ(1, context_->get_num_input_ports());
  Eigen::Matrix<symbolic::Expression, 3, 1> input_vector(
      drake::symbolic::Expression{1.0}, drake::symbolic::Expression{3.14},
      drake::symbolic::Expression{2.18});
  input_->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context_->FixInputPort(0, move(input_));
  gain_->CalcOutput(*context_, output_.get());

  // Checks that the number of output ports in the Gain system and the
  // SystemOutput are consistent.
  EXPECT_EQ(1, output_->get_num_ports());
  EXPECT_EQ(1, gain_->get_num_output_ports());
  const auto* output_vector(output_->get_vector_data(0));
  EXPECT_NE(nullptr, output_vector);
  Eigen::Matrix<symbolic::Expression, 3, 1> expected{kGain_ * input_vector};
  EXPECT_EQ(expected, output_vector->get_value());
  EXPECT_EQ(expected(0).Evaluate(), kGain_.Evaluate() * 1.0);
  EXPECT_EQ(expected(1).Evaluate(), kGain_.Evaluate() * 3.14);
  EXPECT_EQ(expected(2).Evaluate(), kGain_.Evaluate() * 2.18);
}
}  // namespace
}  // namespace systems
}  // namespace drake
