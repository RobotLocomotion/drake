#include "drake/examples/BouncingBall/ball.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace bouncingball {
namespace {

class BallTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new Ball<double>);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  systems::BasicVector<double>* continuous_state() {
    auto result = dynamic_cast<systems::BasicVector<double>*>(
        context_->get_mutable_state()->continuous_state->get_mutable_state());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(BallTest, Topology) {
  ASSERT_EQ(0, dut_->get_num_input_ports());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_ports().at(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_descriptor.get_face());
  EXPECT_EQ(systems::kContinuousSampling, output_descriptor.get_sampling());
}

TEST_F(BallTest, Output) {
  // Grab a pointer to where the EvalOutput results end up.
  const systems::BasicVector<double>* const result =
      dynamic_cast<
    const systems::BasicVector<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Initial state and output.
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(10.0, result->GetAtIndex(0));
  EXPECT_EQ(0.0, result->GetAtIndex(1));

  // New state just propagates through.
  continuous_state()->SetAtIndex(0, 1.0);
  continuous_state()->SetAtIndex(1, 2.0);
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->GetAtIndex(0));
  EXPECT_EQ(2.0, result->GetAtIndex(1));
}

TEST_F(BallTest, Derivatives) {
  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const systems::BasicVector<double>* const result =
    dynamic_cast<const systems::BasicVector<double>*>(
          derivatives_->get_mutable_state());
  ASSERT_NE(nullptr, result);

  // Evaluate time derivatives.
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_EQ(-9.81, result->GetAtIndex(1));
}

}  // namespace
}  // namespace bouncingball
}  // namespace drake
