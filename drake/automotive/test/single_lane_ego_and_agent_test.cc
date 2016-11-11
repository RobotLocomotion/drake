#include "drake/automotive/single_lane_ego_and_agent.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using std::make_unique;

namespace drake {
namespace automotive {
namespace {

class SingleLaneEgoAndAgentTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = dut_.CreateDefaultContext();
    output_ = dut_.AllocateOutput(*context_);
    derivatives_ = dut_.AllocateTimeDerivatives();
  }

  systems::VectorBase<double>* continuous_state(
      const systems::System<double>* system) {
    systems::Context<double>* subcontext =
        dut_.GetMutableSubsystemContext(context_.get(), system);
    return subcontext->get_mutable_continuous_state_vector();
  }

  const systems::VectorBase<double>* state_derivatives(
      const systems::System<double>* system) {
    const systems::ContinuousState<double>* subderivatives =
        dut_.GetSubsystemDerivatives(*derivatives_, system);
    return &subderivatives->get_vector();
  }

  // Intialize the diagram with the agent 5 meters ahead of the ego
  // at zero velocity with the settings v_ref_ego = 50, vdot_agent = 2.7.
  SingleLaneEgoAndAgent<double> dut_{0.0, 0.0, 5.0, 0.0, 50.0, 2.7};
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;

  // Define the output ports of the diagram. This should match the
  // ordering in which ExportOutput is called.
  const int diagram_outport_ego_ = 0;
  const int diagram_outport_agent_ = 1;
};

TEST_F(SingleLaneEgoAndAgentTest, Topology) {
  ASSERT_EQ(0 /* no inputs */, dut_.get_num_input_ports());
  ASSERT_EQ(2 /* one port each for the ego and agent */,
            dut_.get_num_output_ports());

  const auto& output_ego_car_pos =
      dut_.get_output_ports().at(diagram_outport_ego_);
  const auto& output_agent_car_pos =
      dut_.get_output_ports().at(diagram_outport_agent_);

  EXPECT_EQ(systems::kVectorValued, output_ego_car_pos.get_data_type());
  EXPECT_EQ(systems::kVectorValued, output_agent_car_pos.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_ego_car_pos.get_face());
  EXPECT_EQ(systems::kOutputPort, output_agent_car_pos.get_face());
  EXPECT_EQ(2 /* two outputs: x, v */, output_ego_car_pos.get_size());
  EXPECT_EQ(2 /* two outputs: x, v */, output_agent_car_pos.get_size());
  EXPECT_EQ(systems::kContinuousSampling, output_ego_car_pos.get_sampling());
  EXPECT_EQ(systems::kContinuousSampling, output_agent_car_pos.get_sampling());
}

TEST_F(SingleLaneEgoAndAgentTest, EvalOutput) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, output_);

  // Obtain pointers to the (continuous) state vector and output
  // vector for each car.
  auto state_vec_ego = continuous_state(dut_.get_ego_car_system());
  auto state_vec_agent = continuous_state(dut_.get_agent_car_system());

  const systems::BasicVector<double>* output_ego =
      output_->get_vector_data(diagram_outport_ego_);
  const systems::BasicVector<double>* output_agent =
      output_->get_vector_data(diagram_outport_agent_);

  dut_.SetDefaultState(context_.get());
  dut_.EvalOutput(*context_, output_.get());

  // Default state vector is all zeros; outputs are one-to-one wrt. states.
  EXPECT_EQ(0.0, state_vec_ego->GetAtIndex(0));
  EXPECT_EQ(0.0, state_vec_ego->GetAtIndex(1));
  EXPECT_EQ(5.0, state_vec_agent->GetAtIndex(0));
  EXPECT_EQ(0.0, state_vec_agent->GetAtIndex(1));

  EXPECT_EQ(0.0, output_ego->GetAtIndex(0));
  EXPECT_EQ(0.0, output_ego->GetAtIndex(1));
  EXPECT_EQ(5.0, output_agent->GetAtIndex(0));
  EXPECT_EQ(0.0, output_agent->GetAtIndex(1));

  // Define a new set of assignments to the states.
  state_vec_ego->SetAtIndex(0, 1.0);
  state_vec_ego->SetAtIndex(1, 2.0);
  state_vec_agent->SetAtIndex(0, 6.0);
  state_vec_agent->SetAtIndex(1, 4.0);

  dut_.EvalOutput(*context_, output_.get());

  // Default state vector is all zeros; outputs are one-to-one wrt. states.
  EXPECT_EQ(1.0, state_vec_ego->GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec_ego->GetAtIndex(1));
  EXPECT_EQ(6.0, state_vec_agent->GetAtIndex(0));
  EXPECT_EQ(4.0, state_vec_agent->GetAtIndex(1));

  EXPECT_EQ(1.0, output_ego->GetAtIndex(0));
  EXPECT_EQ(2.0, output_ego->GetAtIndex(1));
  EXPECT_EQ(6.0, output_agent->GetAtIndex(0));
  EXPECT_EQ(4.0, output_agent->GetAtIndex(1));
}

TEST_F(SingleLaneEgoAndAgentTest, EvalTimeDerivatives) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, derivatives_);

  // Obtain pointers to the (continuous) state vector and state
  // derivative vector for each car.
  auto state_vec_ego = continuous_state(dut_.get_ego_car_system());
  auto state_vec_agent = continuous_state(dut_.get_agent_car_system());

  auto derivatives_ego = state_derivatives(dut_.get_ego_car_system());
  auto derivatives_agent = state_derivatives(dut_.get_agent_car_system());

  // Verify behavior at a new set of state assignments.
  state_vec_ego->SetAtIndex(0, 0.0);
  state_vec_ego->SetAtIndex(1, 0.0);
  state_vec_agent->SetAtIndex(0, 6.0);
  state_vec_agent->SetAtIndex(1, 0.0);

  dut_.EvalTimeDerivatives(*context_, derivatives_.get());
  dut_.EvalOutput(*context_, output_.get());

  // Expected state derivatives wrt. the given states.
  EXPECT_EQ(0.0, derivatives_ego->GetAtIndex(0));
  EXPECT_NEAR(0.555556, derivatives_ego->GetAtIndex(1), 1e-6);
  EXPECT_EQ(0.0, derivatives_agent->GetAtIndex(0));
  EXPECT_EQ(2.7, derivatives_agent->GetAtIndex(1));

  // Verify behavior at yet another set of state assignments.
  state_vec_ego->SetAtIndex(0, 0.0);
  state_vec_ego->SetAtIndex(1, 25.0);
  state_vec_agent->SetAtIndex(0, 10.0);
  state_vec_agent->SetAtIndex(1, 10.0);

  dut_.EvalTimeDerivatives(*context_, derivatives_.get());
  dut_.EvalOutput(*context_, output_.get());

  // Expected state derivatives wrt. the given states.
  EXPECT_EQ(25.0, derivatives_ego->GetAtIndex(0));
  EXPECT_NEAR(-411.914474, derivatives_ego->GetAtIndex(1), 1e-6);
  EXPECT_EQ(10.0, derivatives_agent->GetAtIndex(0));
  EXPECT_EQ(2.7, derivatives_agent->GetAtIndex(1));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
