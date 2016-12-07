#include "drake/examples/bouncing_ball/bouncing_ball.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"

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
  EXPECT_NEAR(-5.7 * -1 * dut_->get_restitution_coef(),
              result->GetAtIndex(1), 1e-14);
}

TEST_F(BouncingBallTest, Simulate) {
  const double t_final = 10.0;
  const double x0 = 1.0;
  const double v0 = 0.0;

  // Prepare to integrate.
  drake::systems::Simulator<double> simulator(*dut_, std::move(context_));
  simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(*dut_,
                                              simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);
  simulator.get_mutable_integrator()->set_minimum_step_size(1e-3);
  simulator.get_mutable_integrator()->set_maximum_step_size(1e-3);
  simulator.Initialize();

  // Set the initial state for the bouncing ball.
  systems::VectorBase<double>* xc = simulator.get_mutable_context()->
      get_mutable_continuous_state_vector();
  xc->SetAtIndex(0, x0);
  xc->SetAtIndex(1, v0);

  // Integrate.
  simulator.StepTo(t_final);
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), t_final);

  // Check against closed form solution for the bouncing ball. We anticipate
  // some small integration error.
  const double tol = 1e-12;
  EXPECT_NEAR(xc->GetAtIndex(0), dut_->CalcClosedFormHeight(x0, t_final), tol);

  // Try again in variable step mode.
  simulator.get_mutable_context()->set_time(0.0);
  xc->SetAtIndex(0, x0);
  xc->SetAtIndex(1, v0);
  simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(*dut_,
                                              simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_fixed_step_mode(false);
  simulator.get_mutable_integrator()->set_minimum_step_size(1e-8);
  simulator.get_mutable_integrator()->set_maximum_step_size(1e-2);
  simulator.get_mutable_integrator()->set_target_accuracy(1e-3);
  simulator.Initialize();

  // Integrate.
  simulator.StepTo(t_final);
  EXPECT_EQ(simulator.get_mutable_context()->get_time(), t_final);
  EXPECT_NEAR(xc->GetAtIndex(0), dut_->CalcClosedFormHeight(x0, t_final), tol);
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace drake
