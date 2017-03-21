#include "drake/systems/primitives/pass_through.h"

#include <memory>

#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

using Eigen::AutoDiffScalar;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

class PassThroughTest : public ::testing::Test {
 protected:
  void SetUp() override {
    pass_through_ = make_unique<PassThrough<double>>(3 /* size */);
    context_ = pass_through_->CreateDefaultContext();
    output_ = pass_through_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<double>>(3 /* size */);
  }

  std::unique_ptr<System<double>> pass_through_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

// Tests that the output of this system equals its input.
TEST_F(PassThroughTest, VectorThroughPassThroughSystem) {
  /// Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(1, context_->get_num_input_ports());
  ASSERT_EQ(1, pass_through_->get_num_input_ports());
  Eigen::Vector3d input_vector(1.0, 3.14, 2.18);
  input_->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context_->FixInputPort(0, std::move(input_));

  pass_through_->CalcOutput(*context_, output_.get());

  // Checks that the number of output ports in the system and in the
  // output are consistent.
  ASSERT_EQ(1, output_->get_num_ports());
  ASSERT_EQ(1, pass_through_->get_num_output_ports());
  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);
  EXPECT_EQ(input_vector, output_vector->get_value());
}

// Tests that PassThrough allocates no state variables in the context_.
TEST_F(PassThroughTest, PassThroughIsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state()->size());
}

}  // namespace
}  // namespace systems
}  // namespace drake
