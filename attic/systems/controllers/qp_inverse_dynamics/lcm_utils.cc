#include "drake/systems/controllers/qp_inverse_dynamics/lcm_utils.h"

#include "drake/util/drakeUtil.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

void DecodeBodyAcceleration(const RigidBodyTree<double>& robot,
                            const lcmt_body_acceleration& msg,
                            BodyAcceleration* acc) {
  if (!acc) return;
  acc->set_body(*robot.FindBody(msg.body_name));
  cArrayToEigenVector(msg.accelerations, acc->mutable_accelerations());

  if (!acc->is_valid()) {
    throw std::runtime_error("Decoded BodyAcceleration is invalid.");
  }
}

void EncodeBodyAcceleration(const BodyAcceleration& acc,
                            lcmt_body_acceleration* msg) {
  if (!msg) return;
  if (!acc.is_valid()) {
    throw std::runtime_error("Can't encode invalid BodyAcceleration.");
  }
  msg->body_name = acc.body_name();
  eigenVectorToCArray(acc.accelerations(), msg->accelerations);
}

void DecodeResolvedContact(const RigidBodyTree<double>& robot,
                           const lcmt_resolved_contact& msg,
                           ResolvedContact* contact) {
  if (!contact) return;
  contact->set_body(*robot.FindBody(msg.body_name));
  contact->mutable_basis().resize(msg.num_all_basis);
  contact->mutable_num_basis_per_contact_point() =
      msg.num_basis_per_contact_point;
  stdVectorToEigenVector(msg.basis, contact->mutable_basis());

  contact->mutable_point_forces().resize(3, msg.num_contact_points);
  stdVectorOfStdVectorsToEigen(msg.point_forces,
                               contact->mutable_point_forces());
  contact->mutable_contact_points().resize(3, msg.num_contact_points);
  stdVectorOfStdVectorsToEigen(msg.contact_points,
                               contact->mutable_contact_points());

  cArrayToEigenVector(msg.equivalent_wrench,
                      contact->mutable_equivalent_wrench());
  cArrayToEigenVector(msg.reference_point, contact->mutable_reference_point());

  if (!contact->is_valid()) {
    throw std::runtime_error("Decoded ResolvedContact is invalid.");
  }
}

void EncodeResolvedContact(const ResolvedContact& contact,
                           lcmt_resolved_contact* msg) {
  if (!msg) return;
  if (!contact.is_valid()) {
    throw std::runtime_error("Can't encode invalid ResolvedContact.");
  }
  msg->body_name = contact.body_name();
  msg->num_all_basis = contact.basis().size();
  msg->num_basis_per_contact_point = contact.num_basis_per_contact_point();
  eigenVectorToStdVector(contact.basis(), msg->basis);

  msg->num_contact_points = contact.num_contact_points();
  eigenToStdVectorOfStdVectors(contact.point_forces(), msg->point_forces);
  eigenToStdVectorOfStdVectors(contact.contact_points(), msg->contact_points);

  eigenVectorToCArray(contact.equivalent_wrench(), msg->equivalent_wrench);
  eigenVectorToCArray(contact.reference_point(), msg->reference_point);
}

void DecodeQpInput(const RigidBodyTree<double>& robot, const lcmt_qp_input& msg,
                   QpInput* qp_input) {
  if (!qp_input) return;

  ContactInformation info(*robot.FindBody("world"));
  qp_input->mutable_contact_information().clear();
  for (const auto& contact_msg : msg.contact_information) {
    DecodeContactInformation(robot, contact_msg, &info);
    qp_input->mutable_contact_information().emplace(info.body_name(), info);
  }

  DesiredBodyMotion mot(*robot.FindBody("world"));
  qp_input->mutable_desired_body_motions().clear();
  for (const auto& mot_msg : msg.desired_body_motions) {
    DecodeDesiredBodyMotion(robot, mot_msg, &mot);
    qp_input->mutable_desired_body_motions().emplace(mot.body_name(), mot);
  }

  DecodeDesiredDofMotions(msg.desired_dof_motions,
                          &(qp_input->mutable_desired_dof_motions()));
  DecodeDesiredCentroidalMomentumDot(
      msg.desired_centroidal_momentum_dot,
      &(qp_input->mutable_desired_centroidal_momentum_dot()));
  qp_input->mutable_w_basis_reg() = msg.w_basis_reg;

  if (!qp_input->is_valid(robot.get_num_velocities())) {
    throw std::runtime_error("Decoded QpInput is invalid.");
  }
}

void EncodeQpInput(const QpInput& qp_input, lcmt_qp_input* msg) {
  if (!msg) return;

  if (!qp_input.is_valid()) {
    throw std::runtime_error("Can't encode invalid QpInput.");
  }

  msg->num_contacts = static_cast<int>(qp_input.contact_information().size());
  msg->contact_information.resize(msg->num_contacts);
  int contact_ctr = 0;
  for (const auto& contact_pair : qp_input.contact_information()) {
    EncodeContactInformation(contact_pair.second,
                             &(msg->contact_information[contact_ctr]));
    contact_ctr++;
  }

  msg->num_desired_body_motions =
      static_cast<int>(qp_input.desired_body_motions().size());
  msg->desired_body_motions.resize(msg->num_desired_body_motions);
  int desired_body_motion_ctr = 0;
  for (const auto& mot_pair : qp_input.desired_body_motions()) {
    EncodeDesiredBodyMotion(
        mot_pair.second, &(msg->desired_body_motions[desired_body_motion_ctr]));
    desired_body_motion_ctr++;
  }

  EncodeDesiredDofMotions(qp_input.desired_dof_motions(),
                          &(msg->desired_dof_motions));

  EncodeDesiredCentroidalMomentumDot(qp_input.desired_centroidal_momentum_dot(),
                                     &(msg->desired_centroidal_momentum_dot));

  msg->w_basis_reg = qp_input.w_basis_reg();
}

int8_t EncodeConstraintType(ConstraintType type) {
  switch (type) {
    case ConstraintType::Hard:
      return lcmt_constrained_values::HARD;
    case ConstraintType::Skip:
      return lcmt_constrained_values::SKIP;
    case ConstraintType::Soft:
      return lcmt_constrained_values::SOFT;
    default:
      throw std::runtime_error("Can't encode unknown ConstraintType.");
  }
}

ConstraintType DecodeConstraintType(int8_t type) {
  switch (type) {
    case lcmt_constrained_values::HARD:
      return ConstraintType::Hard;
    case lcmt_constrained_values::SKIP:
      return ConstraintType::Skip;
    case lcmt_constrained_values::SOFT:
      return ConstraintType::Soft;
    default:
      throw std::runtime_error("Can't decode unknown ConstraintType.");
  }
}

void DecodeDesiredBodyMotion(const RigidBodyTree<double>& robot,
                             const lcmt_desired_body_motion& msg,
                             DesiredBodyMotion* body_motion) {
  if (!body_motion) return;

  body_motion->set_body(*robot.FindBody(msg.body_name));
  body_motion->mutable_control_during_contact() = msg.control_during_contact;
  DecodeConstrainedValues(msg.constrained_accelerations, body_motion);

  if (!body_motion->is_valid()) {
    throw std::runtime_error("Decoded DesiredBodyMotion is invalid.");
  }
}

void EncodeDesiredBodyMotion(const DesiredBodyMotion& body_motion,
                             lcmt_desired_body_motion* msg) {
  if (!msg) return;
  if (!body_motion.is_valid()) {
    throw std::runtime_error("Can't encode invalid DesiredBodyMotion.");
  }

  msg->body_name = body_motion.body().get_name();
  msg->control_during_contact = body_motion.control_during_contact();
  EncodeConstrainedValues(body_motion, &(msg->constrained_accelerations));
}

void DecodeDesiredDofMotions(const lcmt_desired_dof_motions& msg,
                             DesiredDofMotions* dof_motions) {
  if (!dof_motions) return;

  *dof_motions = DesiredDofMotions(msg.dof_names);
  DecodeConstrainedValues(msg.constrained_accelerations, dof_motions);

  if (!dof_motions->is_valid()) {
    throw std::runtime_error("Decoded DesiredDofMotions is invalid.");
  }
}

void EncodeDesiredDofMotions(const DesiredDofMotions& dof_motions,
                             lcmt_desired_dof_motions* msg) {
  if (!msg) return;
  if (!dof_motions.is_valid()) {
    throw std::runtime_error("Can't encode invalid DesiredDofMotions.");
  }

  msg->num_dof = dof_motions.size();
  msg->dof_names = dof_motions.dof_names();
  EncodeConstrainedValues(dof_motions, &(msg->constrained_accelerations));
}

void DecodeDesiredCentroidalMomentumDot(
    const lcmt_desired_centroidal_momentum_dot& msg,
    DesiredCentroidalMomentumDot* momdot) {
  if (!momdot) return;
  DecodeConstrainedValues(msg.centroidal_momentum_dot, momdot);
  if (!momdot->is_valid()) {
    throw std::runtime_error(
        "Decoded DesiredCentroidalMomentumDot is invalid.");
  }
}

void EncodeDesiredCentroidalMomentumDot(
    const DesiredCentroidalMomentumDot& momdot,
    lcmt_desired_centroidal_momentum_dot* msg) {
  if (!msg) return;
  if (!momdot.is_valid()) {
    throw std::runtime_error(
        "Can't encode invalid DesiredCentroidalMomentumDot.");
  }

  EncodeConstrainedValues(momdot, &(msg->centroidal_momentum_dot));
}

void DecodeConstrainedValues(const lcmt_constrained_values& msg,
                             ConstrainedValues* val) {
  if (!val) return;
  if (msg.size != static_cast<int>(msg.types.size()) ||
      msg.types.size() != msg.weights.size() ||
      msg.types.size() != msg.values.size()) {
    throw std::runtime_error(
        "lcmt_constrained_values has inconsistent dimensions.");
  }

  val->resize(msg.size);
  for (int i = 0; i < msg.size; ++i) {
    val->mutable_constraint_type(i) = DecodeConstraintType(msg.types[i]);
  }
  stdVectorToEigenVector(msg.values, val->mutable_values());
  stdVectorToEigenVector(msg.weights, val->mutable_weights());

  if (!val->is_valid()) {
    throw std::runtime_error("Decoded ConstrainedValues is invalid.");
  }
}

void EncodeConstrainedValues(const ConstrainedValues& val,
                             lcmt_constrained_values* msg) {
  if (!msg) return;
  if (!val.is_valid()) {
    throw std::runtime_error("Can't encode invalid ConstrainedValues.");
  }

  msg->size = val.size();
  msg->types.resize(msg->size);
  msg->weights.resize(msg->size);
  msg->values.resize(msg->size);

  for (int i = 0; i < static_cast<int>(msg->size); ++i) {
    msg->types[i] = EncodeConstraintType(val.constraint_type(i));
  }
  eigenVectorToStdVector(val.values(), msg->values);
  eigenVectorToStdVector(val.weights(), msg->weights);
}

void DecodeContactInformation(const RigidBodyTree<double>& robot,
                              const lcmt_contact_information& msg,
                              ContactInformation* info) {
  if (!info) return;
  info->set_body(*robot.FindBody(msg.body_name));
  info->mutable_num_basis_per_contact_point() = msg.num_basis_per_contact_point;
  info->mutable_contact_points().resize(3, msg.num_contact_points);
  stdVectorOfStdVectorsToEigen(msg.contact_points,
                               info->mutable_contact_points());
  cArrayToEigenVector(msg.normal, info->mutable_normal());
  info->mutable_mu() = msg.mu;
  info->mutable_Kd() = msg.Kd;

  info->mutable_acceleration_constraint_type() =
      DecodeConstraintType(msg.acceleration_constraint_type);
  info->mutable_weight() = msg.weight;

  if (!info->is_valid()) {
    throw std::runtime_error("Decoded ContactInformation is invalid.");
  }
}

void EncodeContactInformation(const ContactInformation& info,
                              lcmt_contact_information* msg) {
  if (!msg) return;
  if (!info.is_valid()) {
    throw std::runtime_error("Can't encode invalid ContactInformation.");
  }

  msg->body_name = info.body().get_name();
  msg->num_contact_points = info.num_contact_points();
  msg->num_basis_per_contact_point = info.num_basis_per_contact_point();
  eigenToStdVectorOfStdVectors(info.contact_points(), msg->contact_points);
  eigenVectorToCArray(info.normal(), msg->normal);

  msg->mu = info.mu();
  msg->Kd = info.Kd();

  msg->acceleration_constraint_type =
      EncodeConstraintType(info.acceleration_constraint_type());
  msg->weight = info.weight();
}

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
