#include "drake/examples/kinova_jaco_arm/jaco_lcm.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace kinova_jaco_arm {

// Test that any modification of message values (e.g. finger
// positions) is symmetric on both sides.
GTEST_TEST(JacoLcmTest, JacoCommandPassthroughTest) {
  systems::DiagramBuilder<double> builder;
  auto command_sender = builder.AddSystem<JacoCommandSender>();
  auto command_receiver = builder.AddSystem<JacoCommandReceiver>();
  builder.Connect(command_receiver->get_output_port(0),
                  command_sender->get_input_port(0));
  builder.ExportInput(command_receiver->get_input_port(0));
  builder.ExportOutput(command_sender->get_output_port(0));
  auto diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput(*context);

  lcmt_jaco_command command{};
  command.num_joints = kJacoDefaultArmNumJoints;
  command.joint_position = std::vector<double>{1, 2, 3, 4, 5, 6, 7};
  command.joint_velocity = std::vector<double>{-1, -2, -3, -4, -5, -6, -7};
  command.num_fingers = kJacoDefaultArmNumFingers;
  command.finger_position = std::vector<double>{10, 20, 30, 40, 50, 60, 70};
  command.finger_velocity = std::vector<double>{
    -10, -20, -30, -40, -50, -60, -70};

  context->FixInputPort(
      0, std::make_unique<systems::Value<lcmt_jaco_command>>(command));

  std::unique_ptr<systems::DiscreteValues<double>> update =
      diagram->AllocateDiscreteVariables();
  update->SetFrom(context->get_mutable_discrete_state());
  diagram->CalcDiscreteVariableUpdates(*context, update.get());
  context->get_mutable_discrete_state().CopyFrom(*update);
  diagram->CalcOutput(*context, output.get());

  lcmt_jaco_command command_out =
      output->get_data(0)->GetValue<lcmt_jaco_command>();

  ASSERT_EQ(command.num_joints, command_out.num_joints);
  for (int i = 0; i < command.num_joints; i++) {
    EXPECT_DOUBLE_EQ(command.joint_position[i], command_out.joint_position[i]);
    EXPECT_DOUBLE_EQ(command.joint_velocity[i], command_out.joint_velocity[i]);
  }

  ASSERT_EQ(command.num_fingers, command_out.num_fingers);
  for (int i = 0; i < command.num_fingers; i++) {
    EXPECT_DOUBLE_EQ(command.finger_position[i],
                     command_out.finger_position[i]);
    EXPECT_DOUBLE_EQ(command.finger_velocity[i],
                     command_out.finger_velocity[i]);
  }
}

// Test that any modification of message values (e.g. finger
// positions) is symmetric on both sides.
GTEST_TEST(JacoLcmTest, JacoStatusPassthroughTest) {
  systems::DiagramBuilder<double> builder;
  auto status_sender = builder.AddSystem<JacoStatusSender>();
  auto status_receiver = builder.AddSystem<JacoStatusReceiver>();
  builder.Connect(status_receiver->get_output_port(0),
                  status_sender->get_input_port(0));
  builder.ExportInput(status_receiver->get_input_port(0));
  builder.ExportOutput(status_sender->get_output_port(0));
  auto diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput(*context);

  lcmt_jaco_status status{};
  status.num_joints = kJacoDefaultArmNumJoints;
  status.joint_position.resize(status.num_joints);
  status.joint_velocity.resize(status.num_joints);
  status.joint_torque.resize(status.num_joints);
  status.joint_current.resize(status.num_joints);
  for (int i = 0; i < status.num_joints; i++) {
    status.joint_position[i] = i;
    status.joint_velocity[i] = i * 10;
    status.joint_torque[i] = i * 100;
    status.joint_current[i] = i * 1000;
  }

  status.num_fingers = kJacoDefaultArmNumFingers;
  status.finger_position.resize(status.num_fingers);
  status.finger_velocity.resize(status.num_fingers);
  status.finger_torque.resize(status.num_fingers);
  status.finger_current.resize(status.num_fingers);
  for (int i = 0; i < status.num_fingers; i++) {
    status.finger_position[i] = -i;
    status.finger_velocity[i] = -i * 10;
    status.finger_torque[i] = -i * 100;
    status.finger_current[i] = -i * 1000;
  }

  context->FixInputPort(
      0, std::make_unique<systems::Value<lcmt_jaco_status>>(status));

  std::unique_ptr<systems::DiscreteValues<double>> update =
      diagram->AllocateDiscreteVariables();
  update->SetFrom(context->get_mutable_discrete_state());
  diagram->CalcDiscreteVariableUpdates(*context, update.get());
  context->get_mutable_discrete_state().CopyFrom(*update);
  diagram->CalcOutput(*context, output.get());

  lcmt_jaco_status status_out =
      output->get_data(0)->GetValue<lcmt_jaco_status>();

  // Force and current won't actually pass through since they're not
  // part of the state in drake.
  ASSERT_EQ(status.num_joints, status_out.num_joints);
  for (int i = 0; i < status.num_joints; i++) {
    EXPECT_DOUBLE_EQ(status.joint_position[i], status_out.joint_position[i]);
    EXPECT_DOUBLE_EQ(status.joint_velocity[i], status_out.joint_velocity[i]);
  }

  ASSERT_EQ(status.num_fingers, status_out.num_fingers);
  for (int i = 0; i < status.num_fingers; i++) {
    EXPECT_DOUBLE_EQ(status.finger_position[i], status_out.finger_position[i]);
    EXPECT_DOUBLE_EQ(status.finger_velocity[i], status_out.finger_velocity[i]);
  }
}

}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake
