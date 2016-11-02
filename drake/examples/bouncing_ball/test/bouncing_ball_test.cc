#include "drake/examples/bouncing_ball/bouncing_ball.h"

#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace bouncing_ball {
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
    return context_->get_mutable_continuous_state_vector();
  }

  std::unique_ptr<BouncingBall<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(BouncingBallTest, Guard) {
  // Evaluate the guard at the initial state.
  EXPECT_EQ(10.0, dut_->EvalGuard(*context_));

  // Evaluate at a state where the ball is rising.
  // The guard should be positive, meaning a mode transition cannot be made.
  continuous_state()->SetAtIndex(0, 1.7);
  continuous_state()->SetAtIndex(1, 2.3);
  EXPECT_EQ(2.3, dut_->EvalGuard(*context_));

  // Evaluate at a state where the ball is falling.
  // The guard should be positive, meaning a mode transition cannot be made.
  continuous_state()->SetAtIndex(0, 1.7);
  continuous_state()->SetAtIndex(1, -2.0);
  EXPECT_EQ(1.7, dut_->EvalGuard(*context_));

  // Evaluate at the moment of impact, where the ball is falling.
  // The guard is non-positive, so a mode transition is possible.
  continuous_state()->SetAtIndex(0, 0.0);
  continuous_state()->SetAtIndex(1, -3.7);
  EXPECT_EQ(0.0, dut_->EvalGuard(*context_));
}

TEST_F(BouncingBallTest, Reset) {
  // Grab a pointer to where the context results will be saved.
  const auto result = output_->get_vector_data(0);

  // Perform a reset at the initial state.
  dut_->PerformReset(context_.get());
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(10.0, result->GetAtIndex(0));
  EXPECT_EQ(0.0, result->GetAtIndex(1));

  // Perform a reset at the moment of impact.
  continuous_state()->SetAtIndex(0, 0.0);
  continuous_state()->SetAtIndex(1, -5.7);
  dut_->PerformReset(context_.get());
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_NEAR(-5.7 * -1 * dut_->GetRestitutionCoef(),
              result->GetAtIndex(1), 1e-14);
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace drake
