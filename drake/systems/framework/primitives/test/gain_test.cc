#include "drake/systems/framework/primitives/gain.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using Eigen::Vector3d;
using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

// TODO(amcastro-tri): Create a diagram with a ConstantVectorSource feeding
// the input of the Gain system.
template <class T>
unique_ptr<FreestandingInputPort> MakeInput(
    unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

class GainTest : public ::testing::Test {
 protected:
  void SetUp() override {
    gain_ = make_unique<Gain<double>>(kGain_ /* gain */, 3 /* size */);
    context_ = gain_->CreateDefaultContext();
    output_ = gain_->AllocateOutput(*context_);
    input0_ = make_unique<BasicVector<double>>(3 /* size */);
    input1_ = make_unique<BasicVector<double>>(3 /* size */);
  }

  const double kGain_{2.0};
  unique_ptr<System<double>> gain_;
  unique_ptr<Context<double>> context_;
  unique_ptr<SystemOutput<double>> output_;
  unique_ptr<BasicVector<double>> input0_;
  unique_ptr<BasicVector<double>> input1_;
};

TEST_F(GainTest, VectorThroughGainSystem) {
  // Checks that the number of input ports in the Gain system and the Context
  // are consistent.
  ASSERT_EQ(1, gain_->get_num_input_ports());
  ASSERT_EQ(1, context_->get_num_input_ports());
  Eigen::Vector3d input_vector(1.0, 3.14, 2.18);
  input0_->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context_->SetInputPort(0, MakeInput(std::move(input0_)));

  gain_->EvalOutput(*context_, output_.get());

  // Checks that the number of output ports in the Gain system and the
  // SystemOutput are consistent.
  ASSERT_EQ(1, output_->get_num_ports());
  ASSERT_EQ(1, gain_->get_num_output_ports());
  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);
  Eigen::Vector3d expected = kGain_ * input_vector;
  EXPECT_EQ(expected, output_vector->get_value());
}

// Tests that Gain allocates no state variables in the context_.
TEST_F(GainTest, GainIsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state()->size());
}

}  // namespace
}  // namespace systems
}  // namespace drake
