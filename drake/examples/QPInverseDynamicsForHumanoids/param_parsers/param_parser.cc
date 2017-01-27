#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"

#include "drake/common/drake_assert.h"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
namespace param_parsers {

namespace {

template <typename Type>
std::vector<Type> ParseYAMLNodeAsVector(const YAML::Node& node) {
  std::vector<Type> values;

  if (node.IsNull()) return values;

  // Tries to cast the YAML node as a vector of strings.
  try {
    values = node.as<std::vector<Type>>();
  } catch (std::runtime_error e) {
    // If casting to a vector of strings fails, tries to cast it as a single
    // string.
    try {
      values.push_back(node.as<Type>());
    } catch (std::runtime_error e1) {
      // Throws if both attempts fail.
      throw e1;
    }
  }

  return values;
}

ContactParam ParseContactParam(const YAML::Node& config,
                               const ContactParam* def_param) {
  ContactParam param;

  // Parses contact points.
  YAML::Node contact_pts = config["contact_points"];
  if (contact_pts.IsDefined() && !contact_pts.IsNull()) {
    std::vector<std::vector<double>> points =
        ParseYAMLNodeAsVector<std::vector<double>>(contact_pts);
    param.contact_points.resize(3, points.size());

    int i = 0;
    for (const auto& point : points) {
      DRAKE_DEMAND(point.size() == 3);
      param.contact_points.col(i++) =
          Vector3<double>(point[0], point[1], point[2]);
    }
  } else if (def_param) {
    // Not specified, uses the default value.
    param.contact_points = def_param->contact_points;
  } else {
    // Not specified, and no default value is provided.
    throw std::runtime_error(
        "\"contact_points\" is not specified, and it has no default values "
        "either.");
  }

  // Parses num_basis_per_contact_point.
  YAML::Node num_basis = config["num_basis_per_contact_point"];
  if (num_basis.IsDefined() && !num_basis.IsNull()) {
    DRAKE_DEMAND(num_basis.IsScalar());
    param.num_basis_per_contact_point = num_basis.as<int>();
  } else if (def_param) {
    param.num_basis_per_contact_point = def_param->num_basis_per_contact_point;
  } else {
    throw std::runtime_error(
        "\"num_basis_per_contact_point\" is not specified, and it has no "
        "default values either.");
  }

  // Parses normal.
  YAML::Node normal = config["normal"];
  if (normal.IsDefined() && !normal.IsNull()) {
    std::vector<double> normal_vec = normal.as<std::vector<double>>();
    DRAKE_DEMAND(normal_vec.size() == 3);
    param.normal = Vector3<double>(normal_vec[0], normal_vec[1], normal_vec[2]);
  } else if (def_param) {
    param.normal = def_param->normal;
  } else {
    throw std::runtime_error(
        "\"normal\" is not specified, and it has no default values either.");
  }

  // Parses weight.
  YAML::Node weight = config["weight"];
  if (weight.IsDefined() && !weight.IsNull()) {
    DRAKE_DEMAND(weight.IsScalar());
    param.weight = weight.as<double>();
  } else if (def_param) {
    param.weight = def_param->weight;
  } else {
    throw std::runtime_error(
        "\"weight\" is not specified, and it has no default values either.");
  }

  // Parses damping.
  YAML::Node Kd = config["Kd"];
  if (Kd.IsDefined() && !Kd.IsNull()) {
    DRAKE_DEMAND(Kd.IsScalar());
    param.Kd = Kd.as<double>();
  } else if (def_param) {
    param.Kd = def_param->Kd;
  } else {
    throw std::runtime_error(
        "\"Kd\" is not specified, and it has no default values either.");
  }

  // Parses friction coeff.
  YAML::Node mu = config["mu"];
  if (mu.IsDefined() && !mu.IsNull()) {
    DRAKE_DEMAND(mu.IsScalar());
    param.mu = mu.as<double>();
  } else if (def_param) {
    param.mu = def_param->mu;
  } else {
    throw std::runtime_error(
        "\"mu\" is not specified, and it has no default values either.");
  }

  return param;
}

VectorX<double> ParseVectorOfDouble(const YAML::Node& config, int size,
                                    const VectorX<double>* default_value) {
  VectorX<double> value;
  VectorX<double> tmp_eig;
  std::vector<double> tmp_vec;
  if (config.IsDefined() && !config.IsNull()) {
    // Parses the specified vector.
    tmp_vec = ParseYAMLNodeAsVector<double>(config);
    tmp_eig.resize(tmp_vec.size());
    stdVectorToEigenVector(tmp_vec, tmp_eig);
  } else if (default_value) {
    // Not specified, uses the value.
    tmp_eig = *default_value;
  } else {
    // Not specified, and no default value is provided.
    throw std::runtime_error(
        "Vector parameter is not specified, and it has no default values "
        "either.");
  }

  if (tmp_eig.size() == 1) {
    // If singleton, expands to a full vector.
    value = VectorX<double>::Constant(size, tmp_eig(0));
  } else if (tmp_eig.size() == size) {
    // Already fully specified, just does copy.
    value = tmp_eig;
  } else {
    // Dimension mismatches.
    std::string error_msg = "Vector parameter dimension mismatch: " +
                            std::to_string(tmp_eig.size()) +
                            std::to_string(size);
    throw std::runtime_error(error_msg);
  }

  return value;
}

// Makes a DesiredMotionParam of dimension @p size.
DesiredMotionParam ParseDesiredMotionParam(
    const YAML::Node& config, int size, const DesiredMotionParam* def_param) {
  DesiredMotionParam param(size);
  if (def_param) {
    // Does parsing with default param.
    param.Kp = ParseVectorOfDouble(config["Kp"], size, &(def_param->Kp));
    param.Kd = ParseVectorOfDouble(config["Kd"], size, &(def_param->Kd));
    param.weight =
        ParseVectorOfDouble(config["weight"], size, &(def_param->weight));
  } else {
    // Does parsing without default param.
    param.Kp = ParseVectorOfDouble(config["Kp"], size, nullptr);
    param.Kd = ParseVectorOfDouble(config["Kd"], size, nullptr);
    param.weight = ParseVectorOfDouble(config["weight"], size, nullptr);
  }

  // All dimensions should match size.
  DRAKE_DEMAND(param.Kp.size() == param.Kd.size());
  DRAKE_DEMAND(param.Kp.size() == param.weight.size());
  DRAKE_DEMAND(param.Kp.size() == size);
  return param;
}

// Makes a DesiredMotionParam of dimension 6.
inline DesiredMotionParam ParseDesiredMotionParam6(
    const YAML::Node& config, const DesiredMotionParam* def_param) {
  return ParseDesiredMotionParam(config, 6, def_param);
}

// Attempts to find a Param with name. If name doesn't exist, returns the
// default param. If default doesn't exist, throws an exception.
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
      std::string err_msg = "Parameter for " + name + " cannot be found." +
                            "Default parameter doesn't exist either.";
      throw std::runtime_error(err_msg);
    }
  }
  return *param;
}

// Returns a parameter map (`M`) from body names to parameters of type
// ParamType. The YAML node @p config is assumed to be formatted as a collection
// of entries, where each entry is a `<key, content>` pair. This function first
// looks for an entry whose `key` is `default`, and uses @p ParseParam to
// generate a `default_param`. All information must be provided for this
// `default` entry in the config file. The mapping `default -> default_param`
// is then stored in `M`. This method then iterates through all the other
// entries in @p config, and for each entry `pair<key, content>`, a `param` is
// generated from `content`. `default_param` is used to fill in the blanks if
// `content` is incomplete. `key` is assumed to be a name for a body group,
// which can be expanded to a collection of bodies using @p alias_group. For
// each body in this group, a mapping of `body_name -> param` is generated and
// stored in `M`.
//
// @throws std::runtime_error if @p config does not contain a "default" entry,
// or it fails to parse individual entries, or some group name does not exist in
// @p alias_group, or attempts to set the same body's parameters multiple times
// with different values.
template <typename ParamType>
std::unordered_map<std::string, ParamType> BuildBodyGroupParamMap(
    const YAML::Node& config,
    const RigidBodyTreeAliasGroups<double>& alias_group,
    ParamType (*ParseParam)(const YAML::Node&, const ParamType*)) {
  std::unordered_map<std::string, ParamType> params;
  const ParamType* default_param = nullptr;

  // Parse the default parameters first.
  YAML::Node default_config = config["default"];
  if (default_config.IsDefined() && !default_config.IsNull()) {
    ParamType param = ParseParam(default_config, nullptr);
    param.name = "default";
    params.emplace(param.name, param);
    default_param = &(params.at("default"));
  } else {
    throw std::runtime_error("No \"default\" entry found.");
  }

  for (auto it = config.begin(); it != config.end(); ++it) {
    std::string group_name = it->first.as<std::string>();

    // Skips default since we have parsed it already.
    if (group_name.compare("default") == 0) continue;

    // Parses this entry.
    ParamType param = ParseParam(it->second, default_param);
    param.name = group_name;

    // group_name exists.
    if (alias_group.has_body_group(group_name)) {
      // Makes a param entry for each body in this body group.
      const std::vector<const RigidBody<double>*> bodies =
          alias_group.get_body_group(group_name);
      for (const RigidBody<double>* body : bodies) {
        param.name = body->get_name();

        auto find_res = params.find(param.name);
        // If this body already has a contact param defined, checks that they
        // are the same. If not, throws.
        if (find_res != params.end()) {
          if (param != find_res->second) {
            std::string error_msg =
                "Param for " + param.name +
                " is specified twice, and they are different.";
            throw std::runtime_error(error_msg);
          }
        } else {
          params.emplace(param.name, param);
        }
      }
    } else {
      std::string error_msg = "Param for unknown body group: " + group_name;
      throw std::runtime_error(error_msg);
    }
  }

  return params;
}

}  // namespace

std::ostream& operator<<(std::ostream& out, const DesiredMotionParam& param) {
  out << param.name << ":\n";
  out << "  Kp: " << param.Kp.transpose() << "\n";
  out << "  Kd: " << param.Kd.transpose() << "\n";
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
  out << "  Kd: " << param.Kd << "\n";
  out << "  weight: " << param.weight << "\n";
  return out;
}

void ParamSet::LoadFromYAMLConfigFile(
    const YAML::Node& config,
    const RigidBodyTreeAliasGroups<double>& alias_group) {
  // Parses section: Contacts.
  YAML::Node contact = config["Contacts"];
  contact_params_ = BuildBodyGroupParamMap<ContactParam>(contact, alias_group,
                                                         ParseContactParam);

  // Parses section: BodyMotions.
  YAML::Node body_motions = config["BodyMotions"];
  body_motion_params_ = BuildBodyGroupParamMap<DesiredMotionParam>(
      body_motions, alias_group, ParseDesiredMotionParam6);

  // Parses section: DoFMotions.
  YAML::Node dof_motions = config["DoFMotions"];
  const RigidBodyTree<double>& robot = alias_group.get_tree();
  // Parses the default DoF motion parameters.
  default_dof_motion_param_ =
      ParseDesiredMotionParam(dof_motions["default"], 1, nullptr);

  // Initializes all DoF to the default parameter.
  dof_motion_params_.resize(robot.get_num_velocities(),
                            default_dof_motion_param_);

  std::string dof_name;
  // For each DoF groups that are specified in the config file,
  for (auto it = dof_motions.begin(); it != dof_motions.end(); ++it) {
    std::string group_name = it->first.as<std::string>();
    // Skips default since we have parsed it already.
    if (group_name.compare("default") == 0) continue;

    const std::vector<int>& v_indices =
        alias_group.get_velocity_group(group_name);

    // Parses params for all DoF in this group.
    DesiredMotionParam param =
        ParseDesiredMotionParam(it->second, static_cast<int>(v_indices.size()),
                                &default_dof_motion_param_);

    // Maps params for this group to the full DoF.
    int param_idx = 0;
    for (int dof_idx : v_indices) {
      // Sets name to velocity's name minus the "dot" part.
      dof_name = robot.get_velocity_name(dof_idx);
      dof_name = dof_name.substr(0, dof_name.size() - 3);

      DesiredMotionParam tmp_1dof_param(1);
      tmp_1dof_param.name = dof_name;
      tmp_1dof_param.Kp(0) = param.Kp(param_idx);
      tmp_1dof_param.Kd(0) = param.Kd(param_idx);
      tmp_1dof_param.weight(0) = param.weight(param_idx);

      // Throws exceptions if some DoF's param has been set, and we are trying
      // to set it to something different again.
      if (dof_motion_params_.at(dof_idx) != default_dof_motion_param_ &&
          dof_motion_params_.at(dof_idx) != tmp_1dof_param) {
        std::string error_msg = "Param for " + dof_name +
                                " is specified twice, and they are different.";
        throw std::runtime_error(error_msg);
      } else {
        dof_motion_params_.at(dof_idx) = tmp_1dof_param;
      }
      param_idx++;
    }
  }

  // Parses section: ContactForceBasis.
  YAML::Node ct_force_basis = config["ContactForceBasis"];
  basis_regularization_weight_ = ct_force_basis["weight"].as<double>();

  // Parses section: CentroidalMomentum.
  YAML::Node centroidal_momentum = config["CentroidalMomentum"];
  centroidal_momentum_dot_params_ =
      ParseDesiredMotionParam6(centroidal_momentum, nullptr);
  centroidal_momentum_dot_params_.name = "centroidal_momentum";
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
  contact.mutable_Kd() = param.Kd;

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
    std::vector<Vector6<double>>* Kp, std::vector<Vector6<double>>* Kd) const {
  if (alias_group.has_body_group(group_name)) {
    const std::vector<const RigidBody<double>*> bodies =
        alias_group.get_body_group(group_name);
    int ctr = 0;
    Kp->resize(bodies.size());
    Kd->resize(bodies.size());
    for (const RigidBody<double>* body : bodies) {
      const DesiredMotionParam& param =
          FindParam(body->get_name(), body_motion_params_);
      DRAKE_DEMAND(param.Kp.size() == 6);
      DRAKE_DEMAND(param.Kd.size() == 6);
      (*Kp)[ctr] = param.Kp;
      (*Kd)[ctr] = param.Kd;
      ctr++;
    }
  } else {
    Kp->clear();
    Kd->clear();
  }
}

void ParamSet::LookupDesiredBodyMotionGains(const RigidBody<double>& body,
                                            Vector6<double>* Kp,
                                            Vector6<double>* Kd) const {
  const DesiredMotionParam& param =
      FindParam(body.get_name(), body_motion_params_);
  DRAKE_DEMAND(param.Kp.size() == 6);
  DRAKE_DEMAND(param.Kd.size() == 6);
  *Kp = param.Kp;
  *Kd = param.Kd;
}

void ParamSet::LookupDesiredCentroidalMomentumDotGains(
    Vector6<double>* Kp, Vector6<double>* Kd) const {
  DRAKE_DEMAND(centroidal_momentum_dot_params_.Kp.size() == 6);
  DRAKE_DEMAND(centroidal_momentum_dot_params_.Kd.size() == 6);
  *Kp = centroidal_momentum_dot_params_.Kp;
  *Kd = centroidal_momentum_dot_params_.Kd;
}

void ParamSet::LookupDesiredDofMotionGains(VectorX<double>* Kp,
                                           VectorX<double>* Kd) const {
  int dim = static_cast<int>(dof_motion_params_.size());

  Kp->resize(dim);
  Kd->resize(dim);
  for (int i = 0; i < dim; ++i) {
    (*Kp)(i) = dof_motion_params_.at(i).Kp(0);
    (*Kd)(i) = dof_motion_params_.at(i).Kd(0);
  }
}

}  // namespace param_parsers
}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
