#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"

#include <memory>

#include <gtest/gtest.h>
#include "bot_core/atlas_command_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/atlas_joint_level_controller_system.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class JointLevelControllerTest : public ::testing::Test {
 protected:
  template <typename ControllerType>
  void Init() {
    robot_ = std::make_unique<RigidBodyTree<double>>();
    // Use this model because the dof order and actuator order are different.
    parsers::urdf::AddModelInstanceFromUrdfFile(
        GetDrakePath() +
            "/examples/Valkyrie/urdf/urdf/"
            "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
        drake::multibody::joints::kQuaternion, nullptr /* weld to frame */,
        robot_.get());

    dut_ = std::make_unique<ControllerType>(*robot_);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);

    context_->set_time(3.);

    qp_output_ = std::make_unique<QpOutput>(GetDofNames(*robot_));
    for (int i = 0; i < qp_output_->dof_torques().size(); i++) {
      qp_output_->mutable_dof_torques()[i] = i * 0.3;
    }

    auto input = systems::AbstractValue::Make<QpOutput>(*qp_output_);
    context_->FixInputPort(dut_->get_input_port_qp_output().get_index(),
                           std::move(input));

    dut_->CalcOutput(*context_, output_.get());
  }

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<JointLevelControllerBaseSystem> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<QpOutput> qp_output_;
};

// Checks that TrivialJointLevelControllerSystem's torque output port's value
// equals RBT.B^T * qp_output.dof_torques().
TEST_F(JointLevelControllerTest, TrivialJointLevelControllerSystemTest) {
  Init<TrivialJointLevelControllerSystem>();

  VectorX<double> expected_torque =
      robot_->B.transpose() * qp_output_->dof_torques();
  VectorX<double> dut_output =
      output_->get_vector_data(dut_->get_output_port_torque().get_index())
          ->get_value();

  EXPECT_TRUE(drake::CompareMatrices(expected_torque, dut_output, 1e-12,
                                     drake::MatrixCompareType::absolute));
}

// Checks that AtlasJointLevelControllerSystem's torque output port's value
// equals RBT.B^T * qp_output.dof_torques().
// Also checks that AtlasJointLevelControllerSystem's bot_core::atlas_command_t
// output matches expected.
TEST_F(JointLevelControllerTest, AtlasJointLevelControllerTest) {
  Init<AtlasJointLevelControllerSystem>();

  VectorX<double> expected_torque =
      robot_->B.transpose() * qp_output_->dof_torques();
  VectorX<double> dut_output =
      output_->get_vector_data(dut_->get_output_port_torque().get_index())
          ->get_value();

  // Checks raw vector output.
  EXPECT_TRUE(drake::CompareMatrices(expected_torque, dut_output, 1e-12,
                                     drake::MatrixCompareType::absolute));

  // Checks bot_core::atlas_command_t output.
  const bot_core::atlas_command_t& dut_output_msg =
      output_
          ->get_data(dynamic_cast<AtlasJointLevelControllerSystem*>(dut_.get())
                         ->get_output_port_atlas_command()
                         .get_index())
          ->GetValue<bot_core::atlas_command_t>();

  EXPECT_EQ(dut_output_msg.utime,
            static_cast<uint64_t>(context_->get_time() * 1e6));
  EXPECT_EQ(dut_output_msg.num_joints, robot_->get_num_actuators());
  for (int i = 0; i < dut_output_msg.num_joints; i++) {
    EXPECT_EQ(dut_output_msg.joint_names[i], robot_->actuators[i].name_);
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
