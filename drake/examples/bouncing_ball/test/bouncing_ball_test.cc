#include "drake/examples/bouncing_ball/bouncing_ball.h"

#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace bouncingball {
namespace {

class BouncingBallTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<BouncingBall<double>>();
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  systems::VectorBase<double>* continuous_state() {
    return context_->get_mutable_state()->continuous_state->get_mutable_state();
  }

  std::unique_ptr<BouncingBall<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(BouncingBallTest, Guard) {
  // Evaluate the guard at the initial state.
  EXPECT_EQ(10.0, dut_->EvalGuard(*context_));

  // Evaluate at another state, where the guard should be positive.
  continuous_state()->SetAtIndex(0, 1.7);
  continuous_state()->SetAtIndex(1, 2.3);
  EXPECT_EQ(2.3, dut_->EvalGuard(*context_));

  // Evaluate at yet another state, where the guard should be positive.
  continuous_state()->SetAtIndex(0, 1.7);
  continuous_state()->SetAtIndex(1, -2.0);
  EXPECT_EQ(1.7, dut_->EvalGuard(*context_));

  // Evaluate at the moment of impact, where the guard should be non-positive.
  continuous_state()->SetAtIndex(0, 0.0);
  continuous_state()->SetAtIndex(1, -3.7);
  EXPECT_EQ(0.0, dut_->EvalGuard(*context_));
}

TEST_F(BouncingBallTest, Reset) {
  // Grab a pointer to where the context results end up.
  const auto result = output_->get_vector_data(0);

  // Trigger a reset at the initial state.
  dut_->PerformReset(context_.get());
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(10.0, result->GetAtIndex(0));
  EXPECT_EQ(0.0, result->GetAtIndex(1));

  // Trigger a reset at the moment of impact.
  continuous_state()->SetAtIndex(0, 0.0);
  continuous_state()->SetAtIndex(1, -5.7);
  dut_->PerformReset(context_.get());
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_NEAR(-5.7 * -1 * dut_->GetRestitutionCoef(),
              result->GetAtIndex(1), 1e-14);
}

}  // namespace
}  // namespace bouncingball
}  // namespace drake
