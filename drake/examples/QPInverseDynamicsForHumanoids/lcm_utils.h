#pragma once

#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "bot_core/robot_state_t.hpp"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Make a bot_core::robot_state_t lcm message based on given information.
 * The current implementation assumes:
 * 1. \param q and \param qd have the same dimension.
 * 2. Has a RPY floating base joint.
 * 3. All DOF are actuated except the floating base.
 * TODO(siyuan.feng@tri.global) Lift these assumptions.
 *
 * @param act_joint_names List of ACTUATED joint names (excluding floating base)
 * @param time Time
 * @param q generalized position (including floating base)
 * @param qd generalized velocity (including floating base)
 * @param joint_torque Actuated joint torques
 * @param l_foot_wrench Left foot wrench in the sensor frame
 * @param r_foot_wrench Right foot wrench in the sensor frame
 * @param msg Output lcm message
 */
void EncodeRobotStateLcmMsg(const std::vector<std::string>& act_joint_names,
                            double time, const Eigen::VectorXd& q,
                            const Eigen::VectorXd& qd,
                            const Eigen::VectorXd& joint_torque,
                            const Eigen::Matrix<double, 6, 1>& l_foot_wrench,
                            const Eigen::Matrix<double, 6, 1>& r_foot_wrench,
                            bot_core::robot_state_t* msg);

/**
 * Decode state related information from a bot_core::robot_state_t message.
 * \param msg can contain extra joint information, but we will only decode
 * joints that are listed in \param q_name_to_index.
 * The current implementation assumes:
 * 1. \param q and \param qd have the same dimension.
 * 2. Has a RPY floating base joint.
 * 3. All DOF are actuated except the floating base.
 * TODO(siyuan.feng@tri.global) Lift these assumptions.
 *
 * @param msg Lcm message
 * @param q_name_to_index A map from coordinate names to index, the joint_name
 *in \param msg is used for lookup.
 * @param time Pointer to decoded time
 * @param q Pointer to decoded generalized position
 * @param qd Pointer to decoded generalized velocity
 * @param joint_torque Pointer to decoded generalized joint torque
 * @param l_foot_wrench Pointer to decoded left foot wrench in the sensor frame
 * @param r_foot_wrench Pointer to decoded right foot wrench in the sensor frame
 */
void DecodeRobotStateLcmMsg(
    const bot_core::robot_state_t& msg,
    const std::unordered_map<std::string, int>& q_name_to_index, double* time,
    Eigen::VectorXd* q, Eigen::VectorXd* qd, Eigen::VectorXd* joint_torque,
    Eigen::Matrix<double, 6, 1>* l_foot_wrench,
    Eigen::Matrix<double, 6, 1>* r_foot_wrench);

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
