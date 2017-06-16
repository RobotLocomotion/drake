#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status_translator.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

void StateVectorAndHumanoidStatusTranslator::Encode(
    const systems::BasicVector<double>& data, HumanoidStatus* msg) const {
  const RigidBodyTree<double>& robot = default_status_.robot();
  DRAKE_DEMAND(data.size() ==
               robot.get_num_positions() + robot.get_num_velocities());
  auto block = data.get_value();
  msg->UpdateKinematics(0, block.head(robot.get_num_positions()),
                        block.tail(robot.get_num_velocities()));
}

void StateVectorAndHumanoidStatusTranslator::Decode(
    const HumanoidStatus& msg, systems::BasicVector<double>* data) const {
  const RigidBodyTree<double>& robot = default_status_.robot();
  DRAKE_DEMAND(data->size() ==
               robot.get_num_positions() + robot.get_num_velocities());
  auto block = data->get_mutable_value();
  block.head(robot.get_num_positions()) = msg.position();
  block.head(robot.get_num_velocities()) = msg.velocity();
}

void HumanoidStatusAndRobotStateMsgTranslator::Encode(
    const HumanoidStatus& data, bot_core::robot_state_t* msg) const {
  robot_state_translator_.EncodeMessageKinematics(data.position(),
                                                  data.velocity(), msg);
  robot_state_translator_.EncodeMessageTorque(data.joint_torque(), msg);

  // TODO(siyuan): move this to translator.
  const Vector6<double>& l_foot = data.foot_wrench_raw(Side::LEFT);
  const Vector6<double>& r_foot = data.foot_wrench_raw(Side::RIGHT);
  msg->force_torque.l_foot_torque_x = l_foot[0];
  msg->force_torque.l_foot_torque_y = l_foot[1];
  msg->force_torque.l_foot_force_z = l_foot[5];
  msg->force_torque.r_foot_torque_x = r_foot[0];
  msg->force_torque.r_foot_torque_y = r_foot[1];
  msg->force_torque.r_foot_force_z = r_foot[5];
}

void HumanoidStatusAndRobotStateMsgTranslator::Decode(
    const bot_core::robot_state_t& msg, HumanoidStatus* data) const {
  robot_state_translator_.DecodeMessageKinematics(msg, tmp_position_,
                                                  tmp_velocity_);
  robot_state_translator_.DecodeMessageTorque(msg, tmp_joint_torque_);

  // TODO(siyuan): move this to translator.
  Vector6<double> l_foot_wrench, r_foot_wrench;
  l_foot_wrench.setZero();
  r_foot_wrench.setZero();
  l_foot_wrench[0] = msg.force_torque.l_foot_torque_x;
  l_foot_wrench[1] = msg.force_torque.l_foot_torque_y;
  l_foot_wrench[5] = msg.force_torque.l_foot_force_z;
  r_foot_wrench[0] = msg.force_torque.r_foot_torque_x;
  r_foot_wrench[1] = msg.force_torque.r_foot_torque_y;
  r_foot_wrench[5] = msg.force_torque.r_foot_force_z;

  double time;
  this->DecodeTime(msg, &time);
  data->Update(time, tmp_position_, tmp_velocity_, tmp_joint_torque_,
               l_foot_wrench, r_foot_wrench);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
