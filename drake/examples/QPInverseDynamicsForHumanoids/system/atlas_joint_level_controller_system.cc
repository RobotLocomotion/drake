#include "drake/examples/QPInverseDynamicsForHumanoids/system/atlas_joint_level_controller_system.h"

#include <memory>

#include "bot_core/atlas_command_t.hpp"

#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

AtlasJointLevelControllerSystem::AtlasJointLevelControllerSystem(
    const RigidBodyTree<double>& robot)
    : JointLevelControllerBaseSystem(robot) {
  output_port_index_atlas_cmd_ = DeclareAbstractOutputPort().get_index();

  // TODO(siyuan.feng): Load gains from some config.
  const int act_size = get_robot().get_num_actuators();
  k_q_p_ = VectorX<double>::Zero(act_size);
  k_q_i_ = VectorX<double>::Zero(act_size);
  k_qd_p_ = VectorX<double>::Zero(act_size);
  k_f_p_ = VectorX<double>::Zero(act_size);
  ff_qd_ = VectorX<double>::Zero(act_size);
  ff_qd_d_ = VectorX<double>::Zero(act_size);
  // Directly feed torque through without any other feedbacks.
  ff_f_d_ = VectorX<double>::Constant(act_size, 1.);
  ff_const_ = VectorX<double>::Zero(act_size);
}

void AtlasJointLevelControllerSystem::DoCalcExtendedOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Gets a mutable reference to bot_core::atlas_command_t from output.
  bot_core::atlas_command_t& msg =
      output->GetMutableData(output_port_index_atlas_cmd_)
          ->GetMutableValue<bot_core::atlas_command_t>();

  // Makes bot_core::atlas_command_t message.
  msg.utime = static_cast<uint64_t>(context.get_time() * 1e6);

  msg.num_joints = get_robot().get_num_actuators();
  msg.joint_names.resize(msg.num_joints);
  msg.position.resize(msg.num_joints);
  msg.velocity.resize(msg.num_joints);
  msg.effort.resize(msg.num_joints);

  // The torques have already been computed and set in
  // JointLevelControllerBaseSystem::DoCalcOutput()
  VectorX<double> act_torques =
      output->get_vector_data(get_output_port_torque().get_index())
          ->get_value();

  // Set desired position, velocity and torque for all actuators.
  for (int i = 0; i < msg.num_joints; ++i) {
    msg.joint_names[i] = get_robot().actuators[i].name_;
    msg.position[i] = 0;
    msg.velocity[i] = 0;
    msg.effort[i] = act_torques[i];
  }

  eigenVectorToStdVector(k_q_p_, msg.k_q_p);
  eigenVectorToStdVector(k_q_i_, msg.k_q_i);
  eigenVectorToStdVector(k_qd_p_, msg.k_qd_p);
  eigenVectorToStdVector(k_f_p_, msg.k_f_p);
  eigenVectorToStdVector(ff_qd_, msg.ff_qd);
  eigenVectorToStdVector(ff_qd_d_, msg.ff_qd_d);
  eigenVectorToStdVector(ff_f_d_, msg.ff_f_d);
  eigenVectorToStdVector(ff_const_, msg.ff_const);

  // This is only used for the Virtual Robotics Challenge's gazebo simulator.
  // Should be deprecated by now.
  msg.k_effort.resize(msg.num_joints, 0);
  // TODO(siyuan.feng): I am not sure what this does exactly, most likely for
  // deprecated simulation as well.
  // Consider removing this from atlas_command_t.
  msg.desired_controller_period_ms = 0;
}

std::unique_ptr<systems::AbstractValue>
AtlasJointLevelControllerSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(output_port_index_atlas_cmd_ == descriptor.get_index());

  return systems::AbstractValue::Make<bot_core::atlas_command_t>(
      bot_core::atlas_command_t());
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
