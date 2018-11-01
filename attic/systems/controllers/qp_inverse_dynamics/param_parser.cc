#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"

#include "google/protobuf/text_format.h"

#include "drake/common/drake_assert.h"
#include "drake/common/proto/protobuf.h"
#include "drake/systems/controllers/qp_inverse_dynamics/id_controller_config.pb.h"
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

ContactParam ParseContactParam(const ContactConfig& config) {
  ContactParam param;

  param.name = config.name();
  param.contact_points.resize(3, config.contact_point_size());
  for (int i = 0; i < config.contact_point_size(); ++i) {
    param.contact_points.col(i) = Vector3<double>(config.contact_point(i).x(),
                                                  config.contact_point(i).y(),
                                                  config.contact_point(i).z());
  }
  param.normal =
      Vector3<double>(config.contact_normal().x(), config.contact_normal().y(),
                      config.contact_normal().z());
  param.num_basis_per_contact_point = config.num_basis_per_contact_point();
  param.mu = config.mu();
  param.kd = config.kd();
  param.weight = config.weight();

  return param;
}

DesiredMotionParam ParseDesiredMotionParam(
    const AccelerationConfig& config, int size) {
  DesiredMotionParam param(size);
  param.name = config.name();

  // Parses kp.
  if (config.kp_size() == size) {
    for (int i = 0; i < size; ++i) param.kp(i) = config.kp(i);
  } else if (config.kp_size() == 1) {
    param.kp = VectorX<double>::Constant(size, config.kp(0));
  } else {
    throw std::runtime_error(
        config.name() + " kp dimension mismatch, expecting " +
        std::to_string(size) + ", got " + std::to_string(config.kp_size()));
  }

  // Parses kd.
  if (config.kd_size() == size) {
    for (int i = 0; i < size; ++i) param.kd(i) = config.kd(i);
  } else if (config.kd_size() == 1) {
    param.kd = VectorX<double>::Constant(size, config.kd(0));
  } else {
    throw std::runtime_error(
        config.name() + " kd dimension mismatch, expecting " +
        std::to_string(size) + ", got " + std::to_string(config.kd_size()));
  }

  // Parses weight.
  if (config.weight_size() == size) {
    for (int i = 0; i < size; ++i) param.weight(i) = config.weight(i);
  } else if (config.weight_size() == 1) {
    param.weight = VectorX<double>::Constant(size, config.weight(0));
  } else {
    throw std::runtime_error(
        config.name() + " weight dimension mismatch, expecting " +
        std::to_string(size) + ", got " + std::to_string(config.weight_size()));
  }

  return param;
}

DesiredMotionParam ParseDesiredMotionParam6(
    const AccelerationConfig& config) {
  return ParseDesiredMotionParam(config, 6);
}

// Generates a vector of names for the generalized coordinate. Since q and v
// can have different dimensions, and we are only interested in the
// accelerations (vdot), so it is more consistent to use the velocities' names.
// However these names have the suffix `dot`, which are stripped away here.
template <typename T>
std::string get_dof_name(const RigidBodyTree<T>& robot, int dof_idx) {
  DRAKE_DEMAND(dof_idx >= 0 && dof_idx < robot.get_num_velocities());
  std::string dof_name = robot.get_velocity_name(dof_idx);
  dof_name = dof_name.substr(0, dof_name.size() - 3);
  return dof_name;
}

template <typename ConfigType, typename ParamType>
void BuildParamMapForBodyGroups(
    const ::google::protobuf::RepeatedPtrField<ConfigType>& configs,
    const RigidBodyTreeAliasGroups<double>& alias_group,
    std::function<ParamType(const ConfigType&)> parse_param,
    std::unordered_map<std::string, ParamType>* param_map) {
  for (const auto& config : configs) {
    ParamType param = parse_param(config);
    const std::string& group_name = config.name();
    // Parses the contact parameter for this body group.
    if (alias_group.has_body_group(group_name)) {
      const std::vector<const RigidBody<double>*> bodies =
        alias_group.get_body_group(group_name);
      // Makes a contact param for each body in this body group.
      for (const RigidBody<double>* body : bodies) {
        param.name = body->get_name();

        auto find_res = param_map->find(param.name);
        // If this body already has a contact param defined, checks that they
        // are the same. Throws if they are not.
        if (find_res != param_map->end()) {
          if (param != find_res->second) {
            throw std::runtime_error("Param for " +
                param.name +
                " cannot be set to different values.");
          }
        } else {
          param_map->emplace(param.name, param);
        }
      }
    } else {
      throw std::runtime_error("Param for unknown body group: " +
          group_name);
    }
  }
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

void ParamSet::LoadFromFile(
    const std::string& config_path,
    const RigidBodyTreeAliasGroups<double>& alias_group) {
  InverseDynamicsControllerConfig id_configs;
  auto istream = MakeFileInputStreamOrThrow(config_path);
  if (!google::protobuf::TextFormat::Parse(istream.get(), &id_configs)) {
    throw std::runtime_error("Error parsing " + config_path);
  }

  name_ = id_configs.name();

  // Contact force basis regularization weight.
  basis_regularization_weight_ = id_configs.contact_force_basis_weight();

  // Centroidal momentum.
  centroidal_momentum_dot_params_ =
      ParseDesiredMotionParam(id_configs.centroidal_momentum(), 6);

  // Contacts.
  ContactParam default_contact_param =
      ParseContactParam(id_configs.default_contact());
  contact_params_.emplace(default_contact_param.name, default_contact_param);
  BuildParamMapForBodyGroups<ContactConfig, ContactParam>(
      id_configs.contact(), alias_group,
      ParseContactParam, &contact_params_);

  // Body motion.
  DesiredMotionParam default_motion_param =
      ParseDesiredMotionParam6(id_configs.default_body_motion());
  body_motion_params_.emplace(default_motion_param.name, default_motion_param);
  BuildParamMapForBodyGroups<AccelerationConfig, DesiredMotionParam>(
      id_configs.body_motion(), alias_group,
      ParseDesiredMotionParam6, &body_motion_params_);

  // Dof motion.
  const RigidBodyTree<double>& robot = alias_group.get_tree();
  default_motion_param =
      ParseDesiredMotionParam(id_configs.default_dof_motion(), 1);

  // Initializes everything to the default param.
  dof_motion_params_.resize(robot.get_num_velocities(), default_motion_param);
  for (int i = 0; i < robot.get_num_velocities(); ++i) {
    dof_motion_params_[i].name = get_dof_name(robot, i);
  }

  DesiredMotionParam single_dof_motion_param(1), group_dof_motion_param;
  for (const auto& dof_config : id_configs.dof_motion()) {
    const std::string& group_name = dof_config.name();

    if (alias_group.has_velocity_group(group_name)) {
      const std::vector<int>& v_indices =
          alias_group.get_velocity_group(group_name);

      // This is the param for the entire group.
      group_dof_motion_param = ParseDesiredMotionParam(
          dof_config, static_cast<int>(v_indices.size()));

      for (int i = 0; i < static_cast<int>(v_indices.size()); i++) {
        int dof_idx = v_indices[i];
        // Creates a 1 dim param just for this dof.
        single_dof_motion_param.name = get_dof_name(robot, dof_idx);
        single_dof_motion_param.kp(0) = group_dof_motion_param.kp(i);
        single_dof_motion_param.kd(0) = group_dof_motion_param.kd(i);
        single_dof_motion_param.weight(0) = group_dof_motion_param.weight(i);

        // Throws if we are trying to overwrite something different than the
        // current value and is not the default value.
        if (dof_motion_params_.at(dof_idx) != default_motion_param &&
            dof_motion_params_.at(dof_idx) != single_dof_motion_param) {
          throw std::runtime_error("Dof motion param for " +
                                   single_dof_motion_param.name +
                                   " cannot be set to different values.");
        } else {
          dof_motion_params_.at(dof_idx) = single_dof_motion_param;
        }
      }
    } else {
      throw std::runtime_error("Dof motion param for unknown joint group: " +
          group_name);
    }
  }
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
    const RigidBodyTreeAliasGroups<double>& alias_group,
    std::vector<Vector6<double>>* kp, std::vector<Vector6<double>>* kd) const {
  if (alias_group.has_body_group(group_name)) {
    const std::vector<const RigidBody<double>*> bodies =
        alias_group.get_body_group(group_name);
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
    const RigidBodyTreeAliasGroups<double>& alias_group) const {
  QpInput qp_input(GetDofNames(alias_group.get_tree()));

  // Inserts all contacts.
  for (const auto& contact_group : contact_body_groups) {
    std::unordered_map<std::string, ContactInformation> contacts =
        MakeContactInformation(contact_group, alias_group);
    qp_input.mutable_contact_information().insert(contacts.begin(),
                                                  contacts.end());
  }

  // Inserts all tracked bodies.
  for (const auto& tracked_body_group : tracked_body_groups) {
    std::unordered_map<std::string, DesiredBodyMotion> motions =
        MakeDesiredBodyMotion(tracked_body_group, alias_group);
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
    const RigidBodyTreeAliasGroups<double>& alias_group) const {
  QpInput qp_input(GetDofNames(alias_group.get_tree()));

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
