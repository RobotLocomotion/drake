#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "bot_core/atlas_command_t.hpp"
#include "bot_core/robot_state_t.hpp"

#include "drake/common/eigen_types.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"
#include "drake/lcmt_body_acceleration.hpp"
#include "drake/lcmt_constrained_values.hpp"
#include "drake/lcmt_contact_information.hpp"
#include "drake/lcmt_desired_body_motion.hpp"
#include "drake/lcmt_desired_centroidal_momentum_dot.hpp"
#include "drake/lcmt_desired_dof_motions.hpp"
#include "drake/lcmt_qp_input.hpp"
#include "drake/lcmt_resolved_contact.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * @param type A constant defined in drake/lcmt_constrained_values.hpp.
 * Should be one of these: lcmt_constrained_values::HARD, SKIP, SOFT.
 */
ConstraintType DecodeConstraintType(int8_t type);

/**
 * @return A constant defined in drake/lcmt_constrained_values.hpp
 * Should be one of these: lcmt_constrained_values::HARD, SKIP, SOFT.
 */
int8_t EncodeConstraintType(ConstraintType type);

void DecodeQpInput(const RigidBodyTree<double>& robot, const lcmt_qp_input& msg,
                   QpInput* qp_input);
void EncodeQpInput(const QpInput& qp_input, lcmt_qp_input* msg);

void DecodeConstrainedValues(const lcmt_constrained_values& msg,
                             ConstrainedValues* val);
void EncodeConstrainedValues(const ConstrainedValues& val,
                             lcmt_constrained_values* msg);

void DecodeContactInformation(const RigidBodyTree<double>& robot,
                              const lcmt_contact_information& msg,
                              ContactInformation* info);
void EncodeContactInformation(const ContactInformation& info,
                              lcmt_contact_information* msg);

void DecodeDesiredCentroidalMomentumDot(
    const lcmt_desired_centroidal_momentum_dot& msg,
    DesiredCentroidalMomentumDot* momdot);
void EncodeDesiredCentroidalMomentumDot(
    const DesiredCentroidalMomentumDot& momdot,
    lcmt_desired_centroidal_momentum_dot* msg);

void DecodeDesiredDofMotions(const lcmt_desired_dof_motions& msg,
                             DesiredDofMotions* dof_motions);
void EncodeDesiredDofMotions(const DesiredDofMotions& dof_motions,
                             lcmt_desired_dof_motions* msg);

void DecodeDesiredBodyMotion(const RigidBodyTree<double>& robot,
                             const lcmt_desired_body_motion& msg,
                             DesiredBodyMotion* body_motion);
void EncodeDesiredBodyMotion(const DesiredBodyMotion& body_motion,
                             lcmt_desired_body_motion* msg);

void DecodeBodyAcceleration(const RigidBodyTree<double>& robot,
                            const lcmt_body_acceleration& msg,
                            BodyAcceleration* acc);
void EncodeBodyAcceleration(const BodyAcceleration& acc,
                            lcmt_body_acceleration* msg);

void DecodeResolvedContact(const RigidBodyTree<double>& robot,
                           const lcmt_resolved_contact& msg,
                           ResolvedContact* contact);
void EncodeResolvedContact(const ResolvedContact& contact,
                           lcmt_resolved_contact* msg);

// TODO(siyuan.feng) Replace these with Twan's similar Encode / Decode methods.
/**
 * Make a bot_core::robot_state_t lcm message based on given information.
 * The current implementation assumes:
 * 1. \p q and \p qd have the same dimension.
 * 2. Has a RPY floating base joint.
 * 3. All DOF are actuated except the floating base.
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
                            double time, const VectorX<double>& q,
                            const VectorX<double>& qd,
                            const VectorX<double>& joint_torque,
                            const Vector6<double>& l_foot_wrench,
                            const Vector6<double>& r_foot_wrench,
                            bot_core::robot_state_t* msg);

/**
 * Decode state related information from a bot_core::robot_state_t message.
 * \p msg can contain extra joint information, but we will only decode
 * joints that are listed in \p q_name_to_index.
 * The current implementation assumes:
 * 1. \p q and \p qd have the same dimension.
 * 2. Has a RPY floating base joint.
 * 3. All DOF are actuated except the floating base.
 *
 * @param msg Lcm message
 * @param q_name_to_index A map from coordinate names to index, the joint_name
 * in \p msg is used for lookup.
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
    VectorX<double>* q, VectorX<double>* qd, VectorX<double>* joint_torque,
    Vector6<double>* l_foot_wrench, Vector6<double>* r_foot_wrench);

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
