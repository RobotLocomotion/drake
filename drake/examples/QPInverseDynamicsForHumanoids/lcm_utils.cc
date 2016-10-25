#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include <iostream>

#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

lcmt_contact_information EncodeContactInformation(const ContactInformation& contact_info) {
  lcmt_contact_information msg;
  msg.body_name = contact_info.body()->get_name();
  msg.mu = contact_info.mu();
  msg.num_contact_points = contact_info.num_contact_points();
  msg.num_basis_per_contact_point = contact_info.num_basis_per_contact_point();
  for (int d = 0; d < 3; d++)
    msg.contact_points[d].resize(msg.num_contact_points);
  for (int i = 0; i < msg.num_contact_points; i++) {
    for (int d = 0; d < 3; d++) {
      msg.contact_points[d][i] = contact_info.contact_points()[i][d];
    }
  }
  for (int d = 0; d < 3; d++)
    msg.normal[d] = contact_info.normal()[d];

  return msg;
}

ContactInformation DecodeContactInformation(const lcmt_contact_information& msg, const RigidBodyTree& robot) {
  ContactInformation contact_info(robot.FindBody(msg.body_name), msg.num_basis_per_contact_point);
  contact_info.mutable_contact_points().resize(msg.num_contact_points);
  for (int i = 0; i < msg.num_contact_points; i++) {
    contact_info.mutable_contact_points()[i] = Eigen::Vector3d(msg.contact_points[0][i], msg.contact_points[1][i], msg.contact_points[2][i]);
  }
  contact_info.mutable_normal() = Eigen::Vector3d(msg.normal[0], msg.normal[1], msg.normal[2]);
  contact_info.mutable_mu() = msg.mu;
  return contact_info;
}

lcmt_body_motion EncodeDesiredBodyMotion(const DesiredBodyMotion& body_motion) {
  lcmt_body_motion msg;
  /*
  const CartesianSetpoint& setpoint = body_motion.setpoint();
  msg.body_name = body_motion.body().get_name();
  for (int i = 0; i < 3; i++)
    msg.position[i] = setpoint.desired_pose().translation()[i];
  Eigen::Quaterniond quat(setpoint.desired_pose().linear());
  msg.rotation[0] = quat.w();
  msg.rotation[1] = quat.x();
  msg.rotation[2] = quat.y();
  msg.rotation[3] = quat.z();
  for (int i = 0; i < 6; i++) {
    msg.velocity[i] = setpoint.desired_velocity()[i];
    msg.acceleration[i] = setpoint.desired_acceleration()[i];
    msg.Kp[i] = setpoint.Kp()[i];
    msg.Kd[i] = setpoint.Kd()[i];
    msg.weight[i] = body_motion.weights()[i];
  }
  msg.control_during_contact = body_motion.control_during_contact();
  */
  return msg;
}

DesiredBodyMotion DecodeDesiredBodyMotion(const lcmt_body_motion& msg, const RigidBodyTree& robot) {
  DesiredBodyMotion result(robot.FindBody(msg.body_name));

  /*
  Eigen::Isometry3d pose;
  Eigen::Quaterniond quat(msg.rotation[0], msg.rotation[1], msg.rotation[2], msg.rotation[3]);
  pose.translation() = Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]);
  pose.linear() = Eigen::Matrix3d(quat.normalized());
  Eigen::Vector6d vel, acc, Kp, Kd;
  for (int i = 0; i < 6; i++) {
    vel[i] = msg.velocity[i];
    acc[i] = msg.acceleration[i];
    Kp[i] = msg.Kp[i];
    Kd[i] = msg.Kd[i];
  }

  result.mutable_setpoint() = CartesianSetpoint(pose, vel, acc, Kp, Kd);
  for (int i = 0; i < 6; i++)
    result.mutable_weights()[i] = msg.weight[i];
  result.mutable_control_during_contact() = msg.control_during_contact;
  */
  return result;
}

void EncodeRobotStateLcmMsg(const std::vector<std::string>& joint_names, double time, const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& joint_torque, const Eigen::Matrix<double, 6, 1>& l_foot_wrench, const Eigen::Matrix<double, 6, 1>& r_foot_wrench, bot_core::robot_state_t* msg) {
  if (q.size() != qd.size() || q.size() != joint_torque.size() + 6 ||
      joint_names.size() != static_cast<size_t>(joint_torque.size())) {
    throw std::runtime_error("invalid dimension");
  }

  msg->utime = static_cast<long>(time * 1e6);
  msg->joint_name = joint_names;
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
  angularvel2rpydotMatrix(rpy, phi, (Eigen::MatrixXd*) nullptr, (Eigen::MatrixXd*) nullptr);
  Eigen::Vector3d omega = phi.inverse() * rpydot;

  msg->pose.rotation.w = quat[0];
  msg->pose.rotation.x = quat[1];
  msg->pose.rotation.y = quat[2];
  msg->pose.rotation.z = quat[3];
  msg->twist.angular_velocity.x = omega[0];
  msg->twist.angular_velocity.y = omega[1];
  msg->twist.angular_velocity.z = omega[2];
}

void DecodeRobotStateLcmMsg(const bot_core::robot_state_t& msg, const std::unordered_map<std::string, int>& q_name_to_index, double* time, Eigen::VectorXd* q, Eigen::VectorXd* qd, Eigen::VectorXd* joint_torque, Eigen::Matrix<double, 6, 1>* l_foot_wrench, Eigen::Matrix<double, 6, 1>* r_foot_wrench) {
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
    if (it != q_name_to_index.end()) {
      if (it->second > q->size()) {
        throw std::runtime_error("state index output bound");
      }
      (*q)[it->second] = static_cast<double>(msg.joint_position.at(i));
      (*qd)[it->second] = static_cast<double>(msg.joint_velocity.at(i));
      (*joint_torque)[it->second - 6] = static_cast<double>(msg.joint_effort.at(i));
    }
  }

  // Set floating base joint state.
  (*q)[0] = static_cast<double>(msg.pose.translation.x);
  (*q)[1] = static_cast<double>(msg.pose.translation.y);
  (*q)[2] = static_cast<double>(msg.pose.translation.z);
  (*qd)[0] = static_cast<double>(msg.twist.linear_velocity.x);
  (*qd)[1] = static_cast<double>(msg.twist.linear_velocity.y);
  (*qd)[2] = static_cast<double>(msg.twist.linear_velocity.z);

  Eigen::Vector4d quat(msg.pose.rotation.w, msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z);
  Eigen::Vector3d rpy = math::quat2rpy(quat);
  Eigen::Vector3d omega(msg.twist.angular_velocity.x, msg.twist.angular_velocity.y, msg.twist.angular_velocity.z);
  Eigen::Matrix3d phi = Eigen::Matrix3d::Zero();
  angularvel2rpydotMatrix(rpy, phi, (Eigen::MatrixXd*) nullptr, (Eigen::MatrixXd*) nullptr);
  Eigen::Vector3d rpydot = phi * omega;

  (*q)[3] = rpy[0];
  (*q)[4] = rpy[1];
  (*q)[5] = rpy[2];
  (*qd)[3] = rpydot[0];
  (*qd)[4] = rpydot[1];
  (*qd)[5] = rpydot[2];

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
