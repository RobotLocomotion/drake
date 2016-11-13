#include "drake/automotive/idm_planner.h"

#include <cmath>
#include <memory>
#include <tuple>

#include "drake/automotive/gen/idm_planner_parameters.h"

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

class IdmPlannerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new IdmPlanner<double>(v_0_));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
  }

  double get_v_0() const { return v_0_; }

  void SetInputValue(const std::vector<double>& state) {
    unsigned int state_size = dut_->get_ego_port().get_size()
      + dut_->get_ego_port().get_size();
    DRAKE_DEMAND(state_size == state.size());
    // Get the state values.
    const double x_ego = state[0];
    const double v_ego = state[1];
    const double x_agent = state[2];
    const double v_agent = state[3];

    auto input_ego = std::make_unique<systems::BasicVector<double>>(2);
    auto input_agent = std::make_unique<systems::BasicVector<double>>(2);
    input_ego->SetAtIndex(0, x_ego);
    input_ego->SetAtIndex(1, v_ego);
    input_agent->SetAtIndex(0, x_agent);
    input_agent->SetAtIndex(1, v_agent);

    // TODO(jadecastro): Take advantage of
    // `SetInputPortToConstantValue` in #4041.
    context_->FixInputPort(dut_->get_ego_port().get_index(),
                           std::move(input_ego));
    context_->FixInputPort(dut_->get_agent_port().get_index(),
                           std::move(input_agent));
  }

  std::unique_ptr<IdmPlanner<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;

 private:
  const double v_0_ = 10.0;
};

TEST_F(IdmPlannerTest, Topology) {
  ASSERT_EQ(2, dut_->get_num_input_ports());
  const auto& input_ego = dut_->get_ego_port();
  const auto& input_agent = dut_->get_agent_port();
  EXPECT_EQ(systems::kVectorValued, input_ego.get_data_type());
  EXPECT_EQ(systems::kVectorValued, input_agent.get_data_type());
  EXPECT_EQ(systems::kInputPort, input_ego.get_face());
  EXPECT_EQ(systems::kInputPort, input_agent.get_face());
  EXPECT_EQ(systems::kContinuousSampling, input_ego.get_sampling());
  EXPECT_EQ(systems::kContinuousSampling, input_agent.get_sampling());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_ports().at(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_descriptor.get_face());
  EXPECT_EQ(1, output_descriptor.get_size());
  EXPECT_EQ(systems::kContinuousSampling, output_descriptor.get_sampling());
}

TEST_F(IdmPlannerTest, Input) {
  // Define a pointer to where the EvalOutput results are stored.
  const auto result = output_->get_vector_data(0);
  ASSERT_NE(nullptr, result);

  // TODO(jadecastro): Add unit testing for parametric variations.
  std::vector<double> param = {1.0, 3.0, 1.0, 0.1, 4.0, 4.5};
  std::vector<double> state;

  // Test case #1: Set the initial states such that the agent and ego
  // start at the headway distance, both at the desired speed.
  state = {0.0, IdmPlannerTest::get_v_0(),
           IdmPlannerTest::get_v_0() * param[4],
           IdmPlannerTest::get_v_0()};
  // Set the inputs to IdmPlanner.
  SetInputValue(state);
  dut_->EvalOutput(*context_, output_.get());
  // Expect there to be no acceleration or deceleration.
  EXPECT_NEAR(result->GetAtIndex(0), 0.0, 1e-2);

  // Test case #2: Set the initial states such that the agent and ego
  // start within the headway distance, both at the desired speed.
  state = {0.0, IdmPlannerTest::get_v_0(), 6.0, IdmPlannerTest::get_v_0()};
  // Set the inputs to IdmPlanner.
  SetInputValue(state);
  dut_->EvalOutput(*context_, output_.get());
  // Expect the car to decelerate.
  EXPECT_LE(result->GetAtIndex(0), -1e-2);

  // Test case #3: Set the initial states such that the agent and ego
  // starting close together at different speeds.
  state = {0.0, 7.0, 6.0, 4.0};
  // Set the inputs to IdmPlanner.
  SetInputValue(state);
  dut_->EvalOutput(*context_, output_.get());
  // Expect the car to decelerate.
  EXPECT_LE(result->GetAtIndex(0), -1e-2);

  // Test case #4: Set the agent and ego sufficiently far apart from
  // one another, with the ego car initially at the desired speed.
  // set-point.
  state = {0.0, IdmPlannerTest::get_v_0(), 1e6, 0.0};
  // Set the inputs to IdmPlanner.
  SetInputValue(state);
  dut_->EvalOutput(*context_, output_.get());
  // Expect there to be no acceleration or deceleration.
  EXPECT_NEAR(result->GetAtIndex(0), 0.0, 1e-2);

  // Test case #5: Set the agent and ego sufficiently far apart from
  // one another, with the ego car speed initially zero.
  // set-point.
  state = {0.0, 0.0, 1e6, 0.0};
  // Set the inputs to IdmPlanner.
  SetInputValue(state);
  dut_->EvalOutput(*context_, output_.get());
  // Expect the car to accelerate.
  EXPECT_GE(result->GetAtIndex(0), 1e-2);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
