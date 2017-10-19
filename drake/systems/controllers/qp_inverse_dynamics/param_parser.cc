#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"

#include "drake/common/drake_assert.h"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

namespace {
// Attempts to find a ParamType with @p name. If no such parameter exists, the
// default parameter is returned. An exception is thrown if the default
// parameter doesn't exist either.
template <typename ParamType>
const ParamType& FindParam(
    const std::string& name,
    const std::unordered_map<std::string, ParamType>& params) {
  const ParamType* param = nullptr;
  auto find_res = params.find(name);
  if (find_res != params.end()) {
    param = &(find_res->second);
  } else {
    find_res = params.find("default");
    if (find_res != params.end()) {
      param = &(find_res->second);
    } else {
      throw std::runtime_error("Parameter for " + name + " cannot be found. "
          "Default parameter doesn't exist either.");
    }
  }
  return *param;
}

}  // namespace

std::ostream& operator<<(std::ostream& out, const DesiredMotionParam& param) {
  out << param.name << ":\n";
  out << "  Kp: " << param.kp.transpose() << "\n";
  out << "  Kd: " << param.kd.transpose() << "\n";
  out << "  weight: " << param.weight.transpose() << "\n";
  return out;
}

std::ostream& operator<<(std::ostream& out, const ContactParam& param) {
  out << param.name << ":\n";
  out << "  contact_points:\n";
  for (int i = 0; i < param.contact_points.cols(); ++i)
    out << "    " << param.contact_points.col(i).transpose() << "\n";
  out << "  normal: " << param.normal.transpose() << "\n";
  out << "  num_basis_per_contact_point: " << param.num_basis_per_contact_point
      << "\n";
  out << "  mu: " << param.mu << "\n";
  out << "  Kd: " << param.kd << "\n";
  out << "  weight: " << param.weight << "\n";
  return out;
}

ContactInformation ParamSet::MakeContactInformationFromParam(
    const RigidBody<double>& body, const ContactParam& param) const {
  ContactInformation contact(body, param.num_basis_per_contact_point);
  contact.mutable_contact_points() = param.contact_points;
  contact.mutable_weight() = param.weight;
  if (contact.weight() > 0) {
    contact.mutable_acceleration_constraint_type() = ConstraintType::Soft;
  } else if (contact.weight() < 0) {
    contact.mutable_acceleration_constraint_type() = ConstraintType::Hard;
  } else {
    contact.mutable_acceleration_constraint_type() = ConstraintType::Skip;
  }
  contact.mutable_normal() = param.normal.normalized();
  contact.mutable_mu() = param.mu;
  contact.mutable_Kd() = param.kd;

  return contact;
}

std::unordered_map<std::string, ContactInformation>
ParamSet::MakeContactInformation(
    const std::string& group_name,
    const RigidBodyTreeAliasGroups<double>& alias_groups) const {
  std::unordered_map<std::string, ContactInformation> contacts;
  if (alias_groups.has_body_group(group_name)) {
    const std::vector<const RigidBody<double>*>& bodies =
        alias_groups.get_body_group(group_name);

    for (const RigidBody<double>* body : bodies) {
      const ContactParam& param = FindParam(body->get_name(), contact_params_);
      contacts.emplace(body->get_name(),
                       MakeContactInformationFromParam(*body, param));
    }
  }

  return contacts;
}

std::unordered_map<std::string, DesiredBodyMotion>
ParamSet::MakeDesiredBodyMotion(
    const std::string& group_name,
    const RigidBodyTreeAliasGroups<double>& alias_groups) const {
  std::unordered_map<std::string, DesiredBodyMotion> motions;
  if (alias_groups.has_body_group(group_name)) {
    const std::vector<const RigidBody<double>*>& bodies =
        alias_groups.get_body_group(group_name);
    for (const RigidBody<double>* body : bodies) {
      const DesiredMotionParam& param =
          FindParam(body->get_name(), body_motion_params_);
      motions.emplace(body->get_name(),
                      MakeDesiredBodyMotionFromParam(*body, param));
    }
  }

  return motions;
}

ContactInformation ParamSet::MakeContactInformation(
    const RigidBody<double>& body) const {
  const ContactParam& param = FindParam(body.get_name(), contact_params_);
  return MakeContactInformationFromParam(body, param);
}

DesiredBodyMotion ParamSet::MakeDesiredBodyMotion(
    const RigidBody<double>& body) const {
  const DesiredMotionParam& param =
      FindParam(body.get_name(), body_motion_params_);
  return MakeDesiredBodyMotionFromParam(body, param);
}

DesiredBodyMotion ParamSet::MakeDesiredBodyMotionFromParam(
    const RigidBody<double>& body, const DesiredMotionParam& param) const {
  DesiredBodyMotion body_motion(body);
  DRAKE_DEMAND(param.weight.size() == 6);

  body_motion.mutable_weights() = param.weight;
  body_motion.SetAllConstraintTypesBasedOnWeights();

  return body_motion;
}

DesiredCentroidalMomentumDot ParamSet::MakeDesiredCentroidalMomentumDot()
    const {
  DesiredCentroidalMomentumDot cdot;

  cdot.mutable_weights() = centroidal_momentum_dot_params_.weight;
  cdot.SetAllConstraintTypesBasedOnWeights();

  return cdot;
}

DesiredDofMotions ParamSet::MakeDesiredDofMotions() const {
  int dim = static_cast<int>(dof_motion_params_.size());
  std::vector<std::string> dof_names(dim);
  for (int i = 0; i < dim; ++i) {
    dof_names[i] = dof_motion_params_.at(i).name;
  }
  DesiredDofMotions dof_motion(dof_names);

  for (int i = 0; i < dim; ++i) {
    dof_motion.mutable_weight(i) = dof_motion_params_.at(i).weight(0);
  }

  dof_motion.SetAllConstraintTypesBasedOnWeights();
  return dof_motion;
}

void ParamSet::LookupDesiredBodyMotionGains(
    const std::string& group_name,
    const RigidBodyTreeAliasGroups<double>& alias_groups,
    std::vector<Vector6<double>>* kp, std::vector<Vector6<double>>* kd) const {
  if (alias_groups.has_body_group(group_name)) {
    const std::vector<const RigidBody<double>*> bodies =
        alias_groups.get_body_group(group_name);
    int ctr = 0;
    kp->resize(bodies.size());
    kd->resize(bodies.size());
    for (const RigidBody<double>* body : bodies) {
      const DesiredMotionParam& param =
          FindParam(body->get_name(), body_motion_params_);
      DRAKE_DEMAND(param.kp.size() == 6);
      DRAKE_DEMAND(param.kd.size() == 6);
      (*kp)[ctr] = param.kp;
      (*kd)[ctr] = param.kd;
      ctr++;
    }
  } else {
    kp->clear();
    kd->clear();
  }
}

void ParamSet::LookupDesiredBodyMotionGains(const RigidBody<double>& body,
                                            Vector6<double>* kp,
                                            Vector6<double>* kd) const {
  const DesiredMotionParam& param =
      FindParam(body.get_name(), body_motion_params_);
  DRAKE_DEMAND(param.kp.size() == 6);
  DRAKE_DEMAND(param.kd.size() == 6);
  *kp = param.kp;
  *kd = param.kd;
}

void ParamSet::LookupDesiredCentroidalMomentumDotGains(
    Vector6<double>* kp, Vector6<double>* kd) const {
  DRAKE_DEMAND(centroidal_momentum_dot_params_.kp.size() == 6);
  DRAKE_DEMAND(centroidal_momentum_dot_params_.kd.size() == 6);
  *kp = centroidal_momentum_dot_params_.kp;
  *kd = centroidal_momentum_dot_params_.kd;
}

void ParamSet::LookupDesiredDofMotionGains(VectorX<double>* kp,
                                           VectorX<double>* kd) const {
  int dim = static_cast<int>(dof_motion_params_.size());

  kp->resize(dim);
  kd->resize(dim);
  for (int i = 0; i < dim; ++i) {
    (*kp)(i) = dof_motion_params_.at(i).kp(0);
    (*kd)(i) = dof_motion_params_.at(i).kd(0);
  }
}

QpInput ParamSet::MakeQpInput(
    const std::vector<std::string>& contact_body_groups,
    const std::vector<std::string>& tracked_body_groups,
    const RigidBodyTreeAliasGroups<double>& alias_groups) const {
  QpInput qp_input(GetDofNames(alias_groups.get_tree()));

  // Inserts all contacts.
  for (const auto& contact_group : contact_body_groups) {
    std::unordered_map<std::string, ContactInformation> contacts =
        MakeContactInformation(contact_group, alias_groups);
    qp_input.mutable_contact_information().insert(contacts.begin(),
                                                  contacts.end());
  }

  // Inserts all tracked bodies.
  for (const auto& tracked_body_group : tracked_body_groups) {
    std::unordered_map<std::string, DesiredBodyMotion> motions =
        MakeDesiredBodyMotion(tracked_body_group, alias_groups);
    qp_input.mutable_desired_body_motions().insert(motions.begin(),
                                                   motions.end());
  }

  // Makes desired DoF motions.
  qp_input.mutable_desired_dof_motions() = MakeDesiredDofMotions();

  // Copies basis regularization weight.
  qp_input.mutable_w_basis_reg() = get_basis_regularization_weight();

  // Makes DesiredCentroidalMomentumDot
  qp_input.mutable_desired_centroidal_momentum_dot() =
      MakeDesiredCentroidalMomentumDot();

  return qp_input;
}

QpInput ParamSet::MakeQpInput(
    const std::vector<const RigidBody<double>*>& contact_bodies,
    const std::vector<const RigidBody<double>*>& tracked_bodies,
    const RigidBodyTreeAliasGroups<double>& alias_groups) const {
  QpInput qp_input(GetDofNames(alias_groups.get_tree()));

  // Inserts all contacts.
  for (const auto& body : contact_bodies) {
    const ContactParam& param = FindParam(body->get_name(), contact_params_);
    qp_input.mutable_contact_information().emplace(
        body->get_name(), MakeContactInformationFromParam(*body, param));
  }

  // Inserts all tracked bodies.
  for (const auto& body : tracked_bodies) {
    const DesiredMotionParam& param =
        FindParam(body->get_name(), body_motion_params_);
    qp_input.mutable_desired_body_motions().emplace(
        body->get_name(), MakeDesiredBodyMotionFromParam(*body, param));
  }

  // Makes desired DoF motions.
  qp_input.mutable_desired_dof_motions() = MakeDesiredDofMotions();

  // Copies basis regularization weight.
  qp_input.mutable_w_basis_reg() = get_basis_regularization_weight();

  // Makes DesiredCentroidalMomentumDot
  qp_input.mutable_desired_centroidal_momentum_dot() =
      MakeDesiredCentroidalMomentumDot();

  return qp_input;
}

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
