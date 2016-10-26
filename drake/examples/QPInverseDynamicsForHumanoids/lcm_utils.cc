#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include <iostream>

#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

void EncodeRobotStateLcmMsg(const std::vector<std::string>& act_joint_names,
                            double time, const Eigen::VectorXd& q,
                            const Eigen::VectorXd& qd,
                            const Eigen::VectorXd& joint_torque,
                            const Eigen::Matrix<double, 6, 1>& l_foot_wrench,
                            const Eigen::Matrix<double, 6, 1>& r_foot_wrench,
                            bot_core::robot_state_t* msg) {
  if (q.size() != qd.size() || q.size() != joint_torque.size() + 6 ||
      act_joint_names.size() != static_cast<size_t>(joint_torque.size())) {
    throw std::runtime_error("invalid dimension");
  }

  msg->utime = static_cast<long>(time * 1e6);
  msg->joint_name = act_joint_names;
  msg->num_joints = static_cast<char>(msg->joint_name.size());
  msg->joint_position.resize(msg->num_joints);
  msg->joint_velocity.resize(msg->num_joints);
  msg->joint_effort.resize(msg->num_joints);

  // Skip the first 6 floating base
  for (int i = 6; i < q.size(); i++) {
    msg->joint_position[i - 6] = static_cast<float>(q[i]);
    msg->joint_velocity[i - 6] = static_cast<float>(qd[i]);
    msg->joint_effort[i - 6] = static_cast<float>(joint_torque[i - 6]);
  }

  // Set force torque readings.
  msg->force_torque.l_foot_force_z = static_cast<float>(l_foot_wrench[5]);
  msg->force_torque.l_foot_torque_x = static_cast<float>(l_foot_wrench[0]);
  msg->force_torque.l_foot_torque_y = static_cast<float>(l_foot_wrench[1]);
  msg->force_torque.r_foot_force_z = static_cast<float>(r_foot_wrench[5]);
  msg->force_torque.r_foot_torque_x = static_cast<float>(r_foot_wrench[0]);
  msg->force_torque.r_foot_torque_y = static_cast<float>(r_foot_wrench[1]);
  for (int i = 0; i < 3; i++) {
    msg->force_torque.l_hand_force[i] = 0;
    msg->force_torque.l_hand_torque[i] = 0;
    msg->force_torque.r_hand_force[i] = 0;
    msg->force_torque.r_hand_torque[i] = 0;
  }

  // Set base
  msg->pose.translation.x = q[0];
  msg->pose.translation.y = q[1];
  msg->pose.translation.z = q[2];
  msg->twist.linear_velocity.x = qd[0];
  msg->twist.linear_velocity.y = qd[1];
  msg->twist.linear_velocity.z = qd[2];

  Eigen::Vector3d rpy = q.segment<3>(3);
  Eigen::Vector3d rpydot = qd.segment<3>(3);
  Eigen::Vector4d quat = math::rpy2quat(rpy);
  Eigen::Matrix3d phi = Eigen::Matrix3d::Zero();
  angularvel2rpydotMatrix(rpy, phi, (Eigen::MatrixXd*)nullptr,
                          (Eigen::MatrixXd*)nullptr);
  Eigen::Vector3d omega = phi.inverse() * rpydot;

  msg->pose.rotation.w = quat[0];
  msg->pose.rotation.x = quat[1];
  msg->pose.rotation.y = quat[2];
  msg->pose.rotation.z = quat[3];
  msg->twist.angular_velocity.x = omega[0];
  msg->twist.angular_velocity.y = omega[1];
  msg->twist.angular_velocity.z = omega[2];
}

void DecodeRobotStateLcmMsg(
    const bot_core::robot_state_t& msg,
    const std::unordered_map<std::string, int>& q_name_to_index, double* time,
    Eigen::VectorXd* q, Eigen::VectorXd* qd, Eigen::VectorXd* joint_torque,
    Eigen::Matrix<double, 6, 1>* l_foot_wrench,
    Eigen::Matrix<double, 6, 1>* r_foot_wrench) {
  if (q->size() != qd->size() || q->size() != joint_torque->size() + 6) {
    throw std::runtime_error("invalid output state dimension");
  }
  if (msg.joint_position.size() != msg.joint_velocity.size() ||
      msg.joint_position.size() != msg.joint_effort.size()) {
    throw std::runtime_error("invalid input state dimension");
  }

  *time = static_cast<double>(msg.utime) / 1e6;

  std::unordered_map<std::string, int>::const_iterator it;

  // Set joint state.
  for (int i = 0; i < msg.num_joints; i++) {
    it = q_name_to_index.find(msg.joint_name.at(i));
    // It's possible that the lcm message have more joints than what we care
    // about,
    // so we will just ignore the extra joints.
    if (it != q_name_to_index.end()) {
      if (it->second > q->size()) {
        throw std::runtime_error("state index output bound");
      }
      (*q)[it->second] = static_cast<double>(msg.joint_position.at(i));
      (*qd)[it->second] = static_cast<double>(msg.joint_velocity.at(i));
      (*joint_torque)[it->second - 6] =
          static_cast<double>(msg.joint_effort.at(i));
    }
  }

  // Set floating base joint state.
  (*q)[q_name_to_index.at("base_x")] =
      static_cast<double>(msg.pose.translation.x);
  (*q)[q_name_to_index.at("base_y")] =
      static_cast<double>(msg.pose.translation.y);
  (*q)[q_name_to_index.at("base_z")] =
      static_cast<double>(msg.pose.translation.z);
  (*qd)[q_name_to_index.at("base_x")] =
      static_cast<double>(msg.twist.linear_velocity.x);
  (*qd)[q_name_to_index.at("base_y")] =
      static_cast<double>(msg.twist.linear_velocity.y);
  (*qd)[q_name_to_index.at("base_z")] =
      static_cast<double>(msg.twist.linear_velocity.z);

  Eigen::Vector4d quat(msg.pose.rotation.w, msg.pose.rotation.x,
                       msg.pose.rotation.y, msg.pose.rotation.z);
  Eigen::Vector3d rpy = math::quat2rpy(quat);
  Eigen::Vector3d omega(msg.twist.angular_velocity.x,
                        msg.twist.angular_velocity.y,
                        msg.twist.angular_velocity.z);
  Eigen::Matrix3d phi = Eigen::Matrix3d::Zero();
  angularvel2rpydotMatrix(rpy, phi, (Eigen::MatrixXd*)nullptr,
                          (Eigen::MatrixXd*)nullptr);
  Eigen::Vector3d rpydot = phi * omega;

  (*q)[q_name_to_index.at("base_roll")] = rpy[0];
  (*q)[q_name_to_index.at("base_pitch")] = rpy[1];
  (*q)[q_name_to_index.at("base_yaw")] = rpy[2];
  (*qd)[q_name_to_index.at("base_roll")] = rpydot[0];
  (*qd)[q_name_to_index.at("base_pitch")] = rpydot[1];
  (*qd)[q_name_to_index.at("base_yaw")] = rpydot[2];

  // Set foot force torque
  l_foot_wrench->setZero();
  r_foot_wrench->setZero();
  (*l_foot_wrench)[0] = msg.force_torque.l_foot_torque_x;
  (*l_foot_wrench)[1] = msg.force_torque.l_foot_torque_y;
  (*l_foot_wrench)[5] = msg.force_torque.l_foot_force_z;
  (*r_foot_wrench)[0] = msg.force_torque.r_foot_torque_x;
  (*r_foot_wrench)[1] = msg.force_torque.r_foot_torque_y;
  (*r_foot_wrench)[5] = msg.force_torque.r_foot_force_z;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
