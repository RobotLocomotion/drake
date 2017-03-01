#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/atlas_joint_level_controller_system.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"

#include <memory>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "bot_core/atlas_command_t.hpp"
#include "drake/systems/framework/value.h"
#include "gtest/gtest.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// Checks TrivialJointLevelControllerSystem's torque output port equals:
// RBT.B^T * qp_output.dof_torques()
GTEST_TEST(TestJointLevelController, TestTrivialJointLevelController) {
  auto robot = std::make_unique<RigidBodyTree<double>>();
  // Use this model because the dof order and actuator order are not the same.
  parsers::urdf::AddModelInstanceFromUrdfFile(
      GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      drake::multibody::joints::kQuaternion, nullptr /* weld to frame */,
      robot.get());

  auto dut = std::make_unique<TrivialJointLevelControllerSystem>(*robot);
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput(*context);

  QpOutput qp_output(GetDofNames(*robot));
  for (int i = 0; i < qp_output.dof_torques().size(); i++) {
    qp_output.mutable_dof_torques()[i] = i * 0.3;
  }

  auto input = systems::AbstractValue::Make<QpOutput>(qp_output);
  context->FixInputPort(dut->get_input_port_qp_output().get_index(),
                        std::move(input));

  dut->CalcOutput(*context, output.get());

  VectorX<double> expected_torque =
      robot->B.transpose() * qp_output.dof_torques();
  VectorX<double> dut_output =
      output->get_vector_data(dut->get_output_port_torque().get_index())
          ->get_value();

  EXPECT_TRUE(drake::CompareMatrices(expected_torque, dut_output, 1e-12,
                                     drake::MatrixCompareType::absolute));
}

// Checks AtlasJointLevelControllerSystem's torque output port equals:
// RBT.B^T * qp_output.dof_torques()
// Checks AtlasJointLevelControllerSystem's bot_core::atlas_command_t output
// matches expected.
GTEST_TEST(TestJointLevelController, TestAtlasJointLevelController) {
  auto robot = std::make_unique<RigidBodyTree<double>>();
  // Use this model because the dof order and actuator order are not the same.
  parsers::urdf::AddModelInstanceFromUrdfFile(
      GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      drake::multibody::joints::kRollPitchYaw, nullptr /* weld to frame */,
      robot.get());

  auto dut = std::make_unique<AtlasJointLevelControllerSystem>(*robot);
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput(*context);

  QpOutput qp_output(GetDofNames(*robot));
  for (int i = 0; i < qp_output.dof_torques().size(); i++) {
    qp_output.mutable_dof_torques()[i] = i * 0.3;
  }

  auto input = systems::AbstractValue::Make<QpOutput>(qp_output);
  context->FixInputPort(dut->get_input_port_qp_output().get_index(),
                        std::move(input));
  context->set_time(3.);

  dut->CalcOutput(*context, output.get());

  VectorX<double> expected_torque =
      robot->B.transpose() * qp_output.dof_torques();
  VectorX<double> dut_output =
      output->get_vector_data(dut->get_output_port_torque().get_index())
          ->get_value();

  // Checks raw vector output.
  EXPECT_TRUE(drake::CompareMatrices(expected_torque, dut_output, 1e-12,
                                     drake::MatrixCompareType::absolute));

  // Checks bot_core::atlas_command_t output.
  const bot_core::atlas_command_t& dut_output_msg =
      output->get_data(dut->get_output_port_atlas_command().get_index())
          ->GetValue<bot_core::atlas_command_t>();

  EXPECT_EQ(dut_output_msg.utime,
            static_cast<uint64_t>(context->get_time() * 1e6));
  EXPECT_EQ(dut_output_msg.num_joints, robot->get_num_actuators());
  for (int i = 0; i < dut_output_msg.num_joints; i++) {
    EXPECT_EQ(dut_output_msg.joint_names[i], robot->actuators[i].name_);
    EXPECT_EQ(dut_output_msg.position[i], 0);
    EXPECT_EQ(dut_output_msg.velocity[i], 0);
    EXPECT_EQ(dut_output_msg.effort[i], expected_torque[i]);

    EXPECT_EQ(dut_output_msg.k_q_p[i], 0);
    EXPECT_EQ(dut_output_msg.k_q_i[i], 0);
    EXPECT_EQ(dut_output_msg.k_qd_p[i], 0);
    EXPECT_EQ(dut_output_msg.k_f_p[i], 0);
    EXPECT_EQ(dut_output_msg.ff_qd[i], 0);
    EXPECT_EQ(dut_output_msg.ff_qd_d[i], 0);
    EXPECT_EQ(dut_output_msg.ff_f_d[i], 1);
    EXPECT_EQ(dut_output_msg.ff_const[i], 0);
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
