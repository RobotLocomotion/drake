#include "drake/automotive/idm_with_trajectory_agent.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

class IdmWithTrajectoryAgentTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new IdmWithTrajectoryAgent<double>);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  IdmWithTrajectoryAgentState<double>* continuous_state() {
    auto result = dynamic_cast<IdmWithTrajectoryAgentState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(IdmWithTrajectoryAgentTest, Topology) {
  ASSERT_EQ(0, dut_->get_num_input_ports());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_ports().at(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_descriptor.get_face());
  EXPECT_EQ(IdmWithTrajectoryAgentStateIndices::kNumCoordinates,
            output_descriptor.get_size());
  EXPECT_EQ(systems::kContinuousSampling, output_descriptor.get_sampling());
}

TEST_F(IdmWithTrajectoryAgentTest, Output) {
  // Grab a pointer to where the EvalOutput results end up.
  const IdmWithTrajectoryAgentState<double>* const result =
      dynamic_cast<
    const IdmWithTrajectoryAgentState<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Starting state and output is all zeros.
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->x_e());
  EXPECT_EQ(0.0, result->v_e());
  EXPECT_EQ(0.0, result->x_a());
  EXPECT_EQ(0.0, result->v_a());
  EXPECT_EQ(0.0, result->a_a());

  // New state just propagates through.
  continuous_state()->set_x_e(1.0);
  continuous_state()->set_v_e(2.0);
  continuous_state()->set_x_a(3.0);
  continuous_state()->set_v_a(4.0);
  continuous_state()->set_a_a(5.0);
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->x_e());
  EXPECT_EQ(2.0, result->v_e());
  EXPECT_EQ(3.0, result->x_a());
  EXPECT_EQ(4.0, result->v_a());
  EXPECT_EQ(5.0, result->a_a());
}

TEST_F(IdmWithTrajectoryAgentTest, Derivatives) {
  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const IdmWithTrajectoryAgentState<double>* const result =
      dynamic_cast<const IdmWithTrajectoryAgentState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Starting derivatives are almost all zeros, except for ego car velocity.
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x_e());
  EXPECT_NEAR(0.950617, result->v_e(), 1e-6);
  EXPECT_EQ(0.0, result->x_a());
  EXPECT_EQ(0.0, result->v_a());
  EXPECT_EQ(0.0, result->a_a());

  // TODO(rick.poyner@tri.global): exercise the dynamics at all, ever.
}

}  // namespace
}  // namespace automotive
}  // namespace drake
