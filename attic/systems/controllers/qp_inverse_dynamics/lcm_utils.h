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
#include "drake/systems/controllers/qp_inverse_dynamics/deprecated.h"
#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_common.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

/**
 * @param type A constant defined in drake/lcmt_constrained_values.hpp.
 * Should be one of these: lcmt_constrained_values::HARD, SKIP, SOFT.
 */
DRAKE_DEPRECATED_QPID
ConstraintType DecodeConstraintType(int8_t type);

/**
 * @return A constant defined in drake/lcmt_constrained_values.hpp
 * Should be one of these: lcmt_constrained_values::HARD, SKIP, SOFT.
 */
DRAKE_DEPRECATED_QPID
int8_t EncodeConstraintType(ConstraintType type);

DRAKE_DEPRECATED_QPID
void DecodeQpInput(const RigidBodyTree<double>& robot, const lcmt_qp_input& msg,
                   QpInput* qp_input);
DRAKE_DEPRECATED_QPID
void EncodeQpInput(const QpInput& qp_input, lcmt_qp_input* msg);

DRAKE_DEPRECATED_QPID
void DecodeConstrainedValues(const lcmt_constrained_values& msg,
                             ConstrainedValues* val);
DRAKE_DEPRECATED_QPID
void EncodeConstrainedValues(const ConstrainedValues& val,
                             lcmt_constrained_values* msg);

DRAKE_DEPRECATED_QPID
void DecodeContactInformation(const RigidBodyTree<double>& robot,
                              const lcmt_contact_information& msg,
                              ContactInformation* info);
DRAKE_DEPRECATED_QPID
void EncodeContactInformation(const ContactInformation& info,
                              lcmt_contact_information* msg);

DRAKE_DEPRECATED_QPID
void DecodeDesiredCentroidalMomentumDot(
    const lcmt_desired_centroidal_momentum_dot& msg,
    DesiredCentroidalMomentumDot* momdot);
DRAKE_DEPRECATED_QPID
void EncodeDesiredCentroidalMomentumDot(
    const DesiredCentroidalMomentumDot& momdot,
    lcmt_desired_centroidal_momentum_dot* msg);

DRAKE_DEPRECATED_QPID
void DecodeDesiredDofMotions(const lcmt_desired_dof_motions& msg,
                             DesiredDofMotions* dof_motions);
DRAKE_DEPRECATED_QPID
void EncodeDesiredDofMotions(const DesiredDofMotions& dof_motions,
                             lcmt_desired_dof_motions* msg);

DRAKE_DEPRECATED_QPID
void DecodeDesiredBodyMotion(const RigidBodyTree<double>& robot,
                             const lcmt_desired_body_motion& msg,
                             DesiredBodyMotion* body_motion);
DRAKE_DEPRECATED_QPID
void EncodeDesiredBodyMotion(const DesiredBodyMotion& body_motion,
                             lcmt_desired_body_motion* msg);

DRAKE_DEPRECATED_QPID
void DecodeBodyAcceleration(const RigidBodyTree<double>& robot,
                            const lcmt_body_acceleration& msg,
                            BodyAcceleration* acc);
DRAKE_DEPRECATED_QPID
void EncodeBodyAcceleration(const BodyAcceleration& acc,
                            lcmt_body_acceleration* msg);

DRAKE_DEPRECATED_QPID
void DecodeResolvedContact(const RigidBodyTree<double>& robot,
                           const lcmt_resolved_contact& msg,
                           ResolvedContact* contact);
DRAKE_DEPRECATED_QPID
void EncodeResolvedContact(const ResolvedContact& contact,
                           lcmt_resolved_contact* msg);

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
