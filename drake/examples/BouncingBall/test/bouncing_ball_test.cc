#include "drake/examples/BouncingBall/bouncing_ball.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace bouncingball {
namespace {

class BouncingBallTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new BouncingBall<double>);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  systems::BasicVector<double>* continuous_state() {
    auto result = dynamic_cast<systems::BasicVector<double>*>
      (context_->get_mutable_state()->continuous_state->get_mutable_state());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(BouncingBallTest, Topology) {
  ASSERT_EQ(0, dut_->get_num_input_ports());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_ports().at(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_descriptor.get_face());
  EXPECT_EQ(systems::kContinuousSampling, output_descriptor.get_sampling());
}

TEST_F(BouncingBallTest, Output) {
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
  continuous_state()->SetAtIndex(0,1.0);
  continuous_state()->SetAtIndex(1,2.0);
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->GetAtIndex(0));
  EXPECT_EQ(2.0, result->GetAtIndex(1));
}

TEST_F(BouncingBallTest, Derivatives) {
  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const systems::BasicVector<double>* const result =
    dynamic_cast<const systems::BasicVector<double>*>
    (derivatives_->get_mutable_state());
  ASSERT_NE(nullptr, result);

  // Evaluate initial derivatives.
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_EQ(-9.81, result->GetAtIndex(1));
}

TEST_F(BouncingBallTest, Guard) {
  // Evaluate the guard at the initial state.
  EXPECT_EQ(10.0, dut_->EvalGuard(*context_));

  // Evaluate at another state, where the guard should be positive.
  continuous_state()->SetAtIndex(0,1.7);
  continuous_state()->SetAtIndex(1,2.3);
  EXPECT_EQ(2.3, dut_->EvalGuard(*context_));

  // Evaluate at yet another state, where the guard should be positive.
  continuous_state()->SetAtIndex(0,1.7);
  continuous_state()->SetAtIndex(1,-2.0);
  EXPECT_EQ(1.7, dut_->EvalGuard(*context_));

  // Evaluate at the moment of impact, where the guard should be non-positive.
  continuous_state()->SetAtIndex(0,0.0);
  continuous_state()->SetAtIndex(1,-3.7);
  EXPECT_EQ(0.0, dut_->EvalGuard(*context_));
}

TEST_F(BouncingBallTest, Reset) {
  // Grab a pointer to where the context results end up.
  const systems::BasicVector<double>* const result =
      dynamic_cast<
    const systems::BasicVector<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Trigger a reset at the initial state.
  dut_->PerformReset(context_.get());
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(10.0, result->GetAtIndex(0));
  EXPECT_EQ(0.0, result->GetAtIndex(1));

  // Trigger a reset at the moment of impact.
  continuous_state()->SetAtIndex(0,0.0);
  continuous_state()->SetAtIndex(1,-5.7);
  dut_->PerformReset(context_.get());
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_NEAR(4.56, result->GetAtIndex(1), 1e-14);
}

}  // namespace
}  // namespace bouncingball
}  // namespace drake
