#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "bot_core/robot_state_t.hpp"

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

void EncodeRobotStateLcmMsg(const std::vector<std::string>& joint_names, double time, const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& joint_torque, const Eigen::Matrix<double, 6, 1>& l_foot_wrench, const Eigen::Matrix<double, 6, 1>& r_foot_wrench, bot_core::robot_state_t* msg);
void DecodeRobotStateLcmMsg(const bot_core::robot_state_t& msg, const std::unordered_map<std::string, int>& q_name_to_index, double* time, Eigen::VectorXd* q, Eigen::VectorXd* qd, Eigen::VectorXd* joint_torque, Eigen::Matrix<double, 6, 1>* l_foot_wrench, Eigen::Matrix<double, 6, 1>* r_foot_wrench);

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
