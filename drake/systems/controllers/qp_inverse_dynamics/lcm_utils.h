#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "bot_core/atlas_command_t.hpp"

#include "drake/common/eigen_types.h"
#include "drake/lcmt_body_acceleration.hpp"
#include "drake/lcmt_constrained_values.hpp"
#include "drake/lcmt_contact_information.hpp"
#include "drake/lcmt_desired_body_motion.hpp"
#include "drake/lcmt_desired_centroidal_momentum_dot.hpp"
#include "drake/lcmt_desired_dof_motions.hpp"
#include "drake/lcmt_qp_input.hpp"
#include "drake/lcmt_resolved_contact.hpp"
#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_common.h"

namespace drake {
namespace systems {
namespace controllers {
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

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
