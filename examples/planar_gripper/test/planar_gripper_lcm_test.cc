#include "drake/examples/planar_gripper/planar_gripper_lcm.h"

#include <gtest/gtest.h>

#include "drake/lcmt_planar_gripper_command.hpp"
#include "drake/lcmt_planar_gripper_status.hpp"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace planar_gripper {
namespace {

constexpr int kNumFingers = 1;

// Test that encoding and decoding perform inverse operations by setting some
// values, encoding them, decoding them, and then verifying we get back what we
// put in.
GTEST_TEST(GripperLcmTest, GripperCommandPassthroughTest) {
  systems::DiagramBuilder<double> builder;
  auto command_encoder = builder.AddSystem<GripperCommandEncoder>(kNumFingers);
  auto command_decoder = builder.AddSystem<GripperCommandDecoder>(kNumFingers);
  builder.Connect(command_decoder->get_state_output_port(),
                  command_encoder->get_state_input_port());
  builder.Connect(command_decoder->get_torques_output_port(),
                  command_encoder->get_torques_input_port());
  builder.ExportInput(command_decoder->get_input_port(0));
  const systems::OutputPortIndex command_enc_output =
      builder.ExportOutput(command_encoder->get_output_port(0));
  auto diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();

  lcmt_planar_gripper_command command{};
  command.num_fingers = kNumFingers;
  command.finger_command.resize(kNumFingers);
  auto &fcommand_in = command.finger_command[0];
  fcommand_in.joint_position[0] = 0.1;
  fcommand_in.joint_position[1] = 0.2;
  fcommand_in.joint_velocity[0] = 0.3;
  fcommand_in.joint_velocity[1] = 0.4;
  fcommand_in.joint_torque[0] = 0.5;
  fcommand_in.joint_torque[1] = 0.6;

  diagram->get_input_port(0).FixValue(context.get(), command);

  std::unique_ptr<systems::DiscreteValues<double>> update =
      diagram->AllocateDiscreteVariables();
  update->SetFrom(context->get_mutable_discrete_state());
  diagram->CalcDiscreteVariableUpdates(*context, update.get());
  context->get_mutable_discrete_state().SetFrom(*update);

  lcmt_planar_gripper_command command_out =
      diagram->get_output_port(command_enc_output)
          .Eval<lcmt_planar_gripper_command>(*context);
  auto& fcommand_out = command_out.finger_command[0];

  ASSERT_EQ(command.num_fingers, command_out.num_fingers);
  ASSERT_EQ(fcommand_in.joint_position[0], fcommand_out.joint_position[0]);
  ASSERT_EQ(fcommand_in.joint_position[1], fcommand_out.joint_position[1]);
  ASSERT_EQ(fcommand_in.joint_velocity[0], fcommand_out.joint_velocity[0]);
  ASSERT_EQ(fcommand_in.joint_velocity[1], fcommand_out.joint_velocity[1]);
  ASSERT_EQ(fcommand_in.joint_torque[0], fcommand_out.joint_torque[0]);
  ASSERT_EQ(fcommand_in.joint_torque[1], fcommand_out.joint_torque[1]);
}

// Test that encoding and decoding perform inverse operations by setting some
// values, encoding them, decoding them, and then verifying we get back what we
// put in.
GTEST_TEST(GripperLcmTest, GripperStatusPassthroughTest) {
  systems::DiagramBuilder<double> builder;
  auto status_encoder = builder.AddSystem<GripperStatusEncoder>(kNumFingers);
  auto status_decoder = builder.AddSystem<GripperStatusDecoder>(kNumFingers);
  builder.Connect(status_decoder->get_state_output_port(),
                  status_encoder->get_state_input_port());
  builder.Connect(status_decoder->get_force_output_port(),
                  status_encoder->get_force_input_port());
  builder.ExportInput(status_decoder->get_input_port(0));
  const systems::OutputPortIndex status_enc_output =
      builder.ExportOutput(status_encoder->get_output_port(0));
  auto diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();

  lcmt_planar_gripper_status status{};
  status.num_fingers = kNumFingers;
  status.finger_status.resize(kNumFingers);
  auto &fstatus_in = status.finger_status[0];
  fstatus_in.joint_position[0] = 0.1;
  fstatus_in.joint_position[1] = 0.2;
  fstatus_in.joint_velocity[0] = 0.3;
  fstatus_in.joint_velocity[1] = 0.4;
  fstatus_in.fingertip_force.fx = 0;
  fstatus_in.fingertip_force.fy = 0.5;
  fstatus_in.fingertip_force.fz = 0.6;

  diagram->get_input_port(0).FixValue(context.get(), status);

  std::unique_ptr<systems::DiscreteValues<double>> update =
      diagram->AllocateDiscreteVariables();
  update->SetFrom(context->get_mutable_discrete_state());
  diagram->CalcDiscreteVariableUpdates(*context, update.get());
  context->get_mutable_discrete_state().SetFrom(*update);

  lcmt_planar_gripper_status status_out =
      diagram->get_output_port(status_enc_output)
          .Eval<lcmt_planar_gripper_status>(*context);
  auto& fstatus_out = status_out.finger_status[0];

  ASSERT_EQ(fstatus_in.joint_position[0], fstatus_out.joint_position[0]);
  ASSERT_EQ(fstatus_in.joint_position[1], fstatus_out.joint_position[1]);
  ASSERT_EQ(fstatus_in.joint_velocity[0], fstatus_out.joint_velocity[0]);
  ASSERT_EQ(fstatus_in.joint_velocity[1], fstatus_out.joint_velocity[1]);
  ASSERT_EQ(fstatus_in.fingertip_force.fx, fstatus_out.fingertip_force.fx);
  ASSERT_EQ(fstatus_in.fingertip_force.fy, fstatus_out.fingertip_force.fy);
  ASSERT_EQ(fstatus_in.fingertip_force.fz, fstatus_out.fingertip_force.fz);
}

}  // namespace
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
