#include "drake/examples/Valkyrie/robot_command_to_desired_effort_converter.h"

#include <gtest/gtest.h>
#include "lcmtypes/bot_core/atlas_command_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::move;

using multibody::joints::FloatingBaseType;

void TestCommandToDesiredEffortConverter(
    const RigidBodyTree<double>& tree, const bot_core::atlas_command_t& message,
    const std::map<const RigidBodyActuator*, double>& expected_efforts) {
  DiagramBuilder<double> builder;

  std::vector<const RigidBodyActuator*> actuators;
  for (const auto& actuator : tree.actuators) {
    actuators.push_back(&actuator);
  }

  // Constant LCM message source.
  auto robot_command = make_unique<Value<bot_core::atlas_command_t>>(message);
  auto& robot_command_source =
      *builder.AddSystem<ConstantValueSource<double>>(move(robot_command));

  // Device under test.
  auto& converter =
      *builder.AddSystem<RobotCommandToDesiredEffortConverter>(actuators);

  // Connect LCM message source to converter.
  builder.Connect(robot_command_source, converter);

  // Define outputs.
  for (const auto& actuator : tree.actuators) {
    builder.ExportOutput(converter.desired_effort_output_port(actuator));
  }

  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);
  diagram->CalcOutput(*context, output.get());

  // TODO(tkoolen): assumption about ordering of exported output ports.
  int output_port_id = 0;
  for (const auto& actuator : tree.actuators) {
    EXPECT_NEAR(output->get_vector_data(output_port_id++)->GetAtIndex(0),
                expected_efforts.at(&actuator), 1e-10);
  }
}

GTEST_TEST(EffortToInputConverterTest, TestEmptyMessage) {
  bot_core::atlas_command_t message({});

  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      FloatingBaseType::kRollPitchYaw, nullptr /* weld to frame */, &tree);

  std::map<const RigidBodyActuator*, double> expected_efforts;
  for (const auto& actuator : tree.actuators) {
    expected_efforts[&actuator] = 0.0;
  }

  TestCommandToDesiredEffortConverter(tree, message, expected_efforts);
}

GTEST_TEST(EffortToInputConverterTest, TestNonEmptyMessage) {
  bot_core::atlas_command_t message;

  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      FloatingBaseType::kRollPitchYaw, nullptr /* weld to frame */, &tree);

  std::map<const RigidBodyActuator*, double> expected_efforts;
  double effort = 0.;

  // Reverse iterate to test ordering.

  for (auto it = tree.actuators.rbegin(); it != tree.actuators.rend(); ++it) {
    const auto& actuator = *it;
    message.joint_names.push_back(actuator.name_);
    message.effort.push_back(effort);
    expected_efforts[&actuator] = effort;
    effort += 1.;
  }
  message.num_joints = static_cast<int32_t>(message.joint_names.size());

  TestCommandToDesiredEffortConverter(tree, message, expected_efforts);
}

}  // namespace
}  // namespace systems
}  // namespace drake
