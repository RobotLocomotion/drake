#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include <iostream>

#include "drake/common/constants.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

void EncodeRobotStateLcmMsg(const std::vector<std::string>& act_joint_names,
                            double time, const Eigen::VectorXd& q,
                            const Eigen::VectorXd& qd,
                            const Eigen::VectorXd& joint_torque,
                            const Vector6<double>& l_foot_wrench,
                            const Vector6<double>& r_foot_wrench,
                            bot_core::robot_state_t* msg) {
  // Assuming q and qd belongs to a RPY floating based robot, and all the other
  // joints are fully actuated.
  const int floating_base_dim_q_dim = kSpaceDimension + kRpySize;
  if (q.size() != qd.size() ||
      q.size() != joint_torque.size() + floating_base_dim_q_dim ||
      act_joint_names.size() != static_cast<size_t>(joint_torque.size())) {
    throw std::runtime_error("invalid dimension");
  }

  msg->utime = static_cast<int64_t>(time * 1e6);
  msg->joint_name = act_joint_names;
  msg->num_joints = static_cast<char>(msg->joint_name.size());
  msg->joint_position.resize(msg->num_joints);
  msg->joint_velocity.resize(msg->num_joints);
  msg->joint_effort.resize(msg->num_joints);

  // Skip the first 6 floating base
  for (int i = floating_base_dim_q_dim; i < q.size(); ++i) {
    msg->joint_position[i - floating_base_dim_q_dim] = static_cast<float>(q[i]);
    msg->joint_velocity[i - floating_base_dim_q_dim] =
        static_cast<float>(qd[i]);
    msg->joint_effort[i - floating_base_dim_q_dim] =
        static_cast<float>(joint_torque[i - floating_base_dim_q_dim]);
  }

  // Set force torque readings.
  msg->force_torque.l_foot_force_z = static_cast<float>(l_foot_wrench[5]);
  msg->force_torque.l_foot_torque_x = static_cast<float>(l_foot_wrench[0]);
  msg->force_torque.l_foot_torque_y = static_cast<float>(l_foot_wrench[1]);
  msg->force_torque.r_foot_force_z = static_cast<float>(r_foot_wrench[5]);
  msg->force_torque.r_foot_torque_x = static_cast<float>(r_foot_wrench[0]);
  msg->force_torque.r_foot_torque_y = static_cast<float>(r_foot_wrench[1]);

  for (int i = 0; i < 3; ++i) {
    msg->force_torque.l_hand_force[i] = 0;
    msg->force_torque.l_hand_torque[i] = 0;
    msg->force_torque.r_hand_force[i] = 0;
    msg->force_torque.r_hand_torque[i] = 0;
  }

  // Set base
  Eigen::Isometry3d pose;
  pose.translation() = q.head<3>();
  pose.linear() = math::rpy2rotmat(q.segment<3>(3));
  EncodePose(pose, msg->pose);

  Eigen::Vector3d rpy = q.segment<3>(3);
  Eigen::Vector3d rpydot = qd.segment<3>(3);
  Eigen::Matrix3d phi = Eigen::Matrix3d::Zero();
  angularvel2rpydotMatrix(rpy, phi, (Eigen::MatrixXd*)nullptr,
                          (Eigen::MatrixXd*)nullptr);

  Vector6<double> vel;
  vel.head<3>() = phi.inverse() * rpydot;
  vel.tail<3>() = qd.head<3>();
  EncodeTwist(vel, msg->twist);
}

void DecodeRobotStateLcmMsg(
    const bot_core::robot_state_t& msg,
    const std::unordered_map<std::string, int>& q_name_to_index, double* time,
    Eigen::VectorXd* q, Eigen::VectorXd* qd, Eigen::VectorXd* joint_torque,
    Vector6<double>* l_foot_wrench, Vector6<double>* r_foot_wrench) {
  const int floating_base_dim_q_dim = kSpaceDimension + kRpySize;
  if (q->size() != qd->size() ||
      q->size() != joint_torque->size() + floating_base_dim_q_dim) {
    throw std::runtime_error("invalid output state dimension");
  }
  if (msg.joint_position.size() != msg.joint_velocity.size() ||
      msg.joint_position.size() != msg.joint_effort.size()) {
    throw std::runtime_error("invalid input state dimension");
  }

  *time = static_cast<double>(msg.utime) / 1e6;

  std::unordered_map<std::string, int>::const_iterator it;

  // Set joint state.
  for (int i = 0; i < msg.num_joints; ++i) {
    it = q_name_to_index.find(msg.joint_name.at(i));
    // It's possible that the lcm message have more joints than what we care
    // about, so we will just ignore the extra joints.
    if (it != q_name_to_index.end()) {
      if (it->second > q->size()) {
        throw std::runtime_error("state index output bound");
      }
      (*q)[it->second] = static_cast<double>(msg.joint_position.at(i));
      (*qd)[it->second] = static_cast<double>(msg.joint_velocity.at(i));
      (*joint_torque)[it->second - floating_base_dim_q_dim] =
          static_cast<double>(msg.joint_effort.at(i));
    }
  }

  // Set floating base joint state.
  Eigen::Isometry3d pose = DecodePose(msg.pose);
  Vector6<double> vel = DecodeTwist(msg.twist);
  Eigen::Vector3d rpy = math::rotmat2rpy(pose.linear());
  Eigen::Matrix3d phi = Eigen::Matrix3d::Zero();
  angularvel2rpydotMatrix(rpy, phi, (Eigen::MatrixXd*)nullptr,
                          (Eigen::MatrixXd*)nullptr);
  Eigen::Vector3d rpydot = phi * vel.head<3>();

  (*q)[q_name_to_index.at("base_x")] = pose.translation()[0];
  (*q)[q_name_to_index.at("base_y")] = pose.translation()[1];
  (*q)[q_name_to_index.at("base_z")] = pose.translation()[2];
  (*qd)[q_name_to_index.at("base_x")] = vel[3];
  (*qd)[q_name_to_index.at("base_y")] = vel[4];
  (*qd)[q_name_to_index.at("base_z")] = vel[5];

  (*q)[q_name_to_index.at("base_roll")] = rpy[0];
  (*q)[q_name_to_index.at("base_pitch")] = rpy[1];
  (*q)[q_name_to_index.at("base_yaw")] = rpy[2];
  (*qd)[q_name_to_index.at("base_roll")] = rpydot[0];
  (*qd)[q_name_to_index.at("base_pitch")] = rpydot[1];
  (*qd)[q_name_to_index.at("base_yaw")] = rpydot[2];

  // Set foot force torque.
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
