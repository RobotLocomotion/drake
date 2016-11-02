#include "drake/systems/framework/primitives/pass_through.h"

#include <memory>

#include <unsupported/Eigen/AutoDiff>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using Eigen::AutoDiffScalar;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

// TODO(amcastro-tri): Create a diagram with a ConstantVectorSource feeding
// the input of the PassThrough system.
template<class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

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
  context_->SetInputPort(0, MakeInput(std::move(input_)));

  pass_through_->EvalOutput(*context_, output_.get());

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
