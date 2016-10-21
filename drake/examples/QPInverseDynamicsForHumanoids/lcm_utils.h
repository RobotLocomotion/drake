#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "bot_core/robot_state_t.hpp"

#include "drake/lcmt_contact_information.hpp"
#include "drake/lcmt_body_motion.hpp"
#include "drake/lcmt_new_qp_controller_input.hpp"

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

void EncodeRobotStateLcmMsg(const std::vector<std::string>& joint_names, double time, const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& joint_torque, const Eigen::Matrix<double, 6, 1>& l_foot_wrench, const Eigen::Matrix<double, 6, 1>& r_foot_wrench, bot_core::robot_state_t* msg);
void DecodeRobotStateLcmMsg(const bot_core::robot_state_t& msg, const std::unordered_map<std::string, int>& q_name_to_index, double* time, Eigen::VectorXd* q, Eigen::VectorXd* qd, Eigen::VectorXd* joint_torque, Eigen::Matrix<double, 6, 1>* l_foot_wrench, Eigen::Matrix<double, 6, 1>* r_foot_wrench);

lcmt_body_motion EncodeDesiredBodyMotion(const DesiredBodyMotion& bodydd);
DesiredBodyMotion DecodeDesiredBodyMotion(const lcmt_body_motion& msg, const RigidBodyTree& robot);

lcmt_contact_information EncodeContactInformation(const ContactInformation& contact_info);
ContactInformation DecodeContactInformation(const lcmt_contact_information& msg, const RigidBodyTree& robot);

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
