#include "drake/systems/controllers/qp_inverse_dynamics/param_parser_loader.h"

#include <unordered_map>
#include <utility>
#include <vector>

#include "google/protobuf/text_format.h"

#include "drake/common/proto/protobuf.h"
#include "drake/systems/controllers/qp_inverse_dynamics/id_controller_config.pb.h"
#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

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
    const RigidBodyTreeAliasGroups<double>& alias_groups,
    std::function<ParamType(const ConfigType&)> parse_param,
    std::unordered_map<std::string, ParamType>* param_map) {
  for (const auto& config : configs) {
    ParamType param = parse_param(config);
    const std::string& group_name = config.name();
    // Parses the contact parameter for this body group.
    if (alias_groups.has_body_group(group_name)) {
      const std::vector<const RigidBody<double>*> bodies =
        alias_groups.get_body_group(group_name);
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

std::unique_ptr<ParamSet> ParamSetLoadFromFile(
    const std::string& config_path,
    const RigidBodyTreeAliasGroups<double>& alias_groups) {
  auto paramset = std::make_unique<ParamSet>();

  InverseDynamicsControllerConfig id_configs;
  auto istream = MakeFileInputStreamOrThrow(config_path);
  if (!google::protobuf::TextFormat::Parse(istream.get(), &id_configs)) {
    throw std::runtime_error("Error parsing " + config_path);
  }

  paramset->set_name(id_configs.name());

  // Contact force basis regularization weight.
  paramset->set_basis_regularization_weight(
      id_configs.contact_force_basis_weight());

  // Centroidal momentum.
  paramset->set_centroidal_momentum_dot_params(
      ParseDesiredMotionParam(id_configs.centroidal_momentum(), 6));

  // Contacts.
  ContactParam default_contact_param =
      ParseContactParam(id_configs.default_contact());
  std::unordered_map<std::string, ContactParam> contact_params;
  contact_params.emplace(default_contact_param.name, default_contact_param);
  BuildParamMapForBodyGroups<ContactConfig, ContactParam>(
      id_configs.contact(), alias_groups,
      ParseContactParam, &contact_params);
  paramset->set_contact_params(std::move(contact_params));

  // Body motion.
  DesiredMotionParam default_motion_param =
      ParseDesiredMotionParam6(id_configs.default_body_motion());
  std::unordered_map<std::string, DesiredMotionParam> body_motion_params;
  body_motion_params.emplace(default_motion_param.name, default_motion_param);
  BuildParamMapForBodyGroups<AccelerationConfig, DesiredMotionParam>(
      id_configs.body_motion(), alias_groups,
      ParseDesiredMotionParam6, &body_motion_params);
  paramset->set_body_motion_params(std::move(body_motion_params));

  // Dof motion.
  std::vector<DesiredMotionParam> dof_motion_params;
  const RigidBodyTree<double>& robot = alias_groups.get_tree();
  default_motion_param =
      ParseDesiredMotionParam(id_configs.default_dof_motion(), 1);

  // Initializes everything to the default param.
  dof_motion_params.resize(robot.get_num_velocities(), default_motion_param);
  for (int i = 0; i < robot.get_num_velocities(); ++i) {
    dof_motion_params[i].name = get_dof_name(robot, i);
  }

  DesiredMotionParam single_dof_motion_param(1), group_dof_motion_param;
  for (const auto& dof_config : id_configs.dof_motion()) {
    const std::string& group_name = dof_config.name();

    if (alias_groups.has_velocity_group(group_name)) {
      const std::vector<int>& v_indices =
          alias_groups.get_velocity_group(group_name);

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
        if (dof_motion_params.at(dof_idx) != default_motion_param &&
            dof_motion_params.at(dof_idx) != single_dof_motion_param) {
          throw std::runtime_error("Dof motion param for " +
                                   single_dof_motion_param.name +
                                   " cannot be set to different values.");
        } else {
          dof_motion_params.at(dof_idx) = single_dof_motion_param;
        }
      }
    } else {
      throw std::runtime_error("Dof motion param for unknown joint group: " +
          group_name);
    }
  }

  paramset->set_dof_motion_params(std::move(dof_motion_params));

  return paramset;
}

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
