#include "yamlUtil.h"
// #include <regex>

YAML::Node applyDefaults(const YAML::Node& node, const YAML::Node& default_node) {
  YAML::Node result = YAML::Clone(node);
  if (!default_node.IsMap()) {
    throw std::runtime_error("map node expected");
  }
  for (auto field = default_node.begin(); field != default_node.end(); ++field) {
    std::string fieldname = field->first.as<std::string>();
    if (!result[fieldname]) {
      result[fieldname] = YAML::Clone(field->second);
    } else if (field->second.IsMap()) {
      result[fieldname] = applyDefaults(result[fieldname], field->second);
    }
  }
  return result;
}

YAML::Node expandDefaults(const YAML::Node& node) {
  YAML::Node result = YAML::Clone(node);
  if (node.IsMap() && node["DEFAULT"]) {
    const YAML::Node& default_node = result["DEFAULT"];
    for (auto field = result.begin(); field != result.end(); ++field) {
      std::string fieldname = field->first.as<std::string>();
      if (fieldname != "DEFAULT") {
        result[fieldname] = applyDefaults(result[fieldname], default_node);
      }
    }
  }
  if (result.IsMap() || result.IsSequence()) {
    for (auto child = result.begin(); child != result.end(); ++child) {
      std::string child_name = child->first.as<std::string>();
      result[child_name] = expandDefaults(child->second);
    }
  }
  return result;
}

double dampingGain(double Kp, double damping_ratio) {
  return 2 * damping_ratio * sqrt(Kp);
}

// std::regex globToRegex(const std::string& glob) {
//   auto re_escape_pattern = std::regex("[.^$|()\\[\\]{}+?\\\\]");
//   std::string re_escape_replacement = "\\\\$&";
//   std::string escaped_pattern = std::regex_replace(glob, re_escape_pattern, re_escape_replacement);
//   auto re_star_pattern = std::regex("\\*");
//   std::string re_star_replacement = ".*";
//   escaped_pattern = std::regex_replace(escaped_pattern, re_star_pattern, re_star_replacement);
//   return std::regex(escaped_pattern);
// }


YAML::Node get(const YAML::Node& parent, const std::string& key) {
  auto result = parent[key];
  if (result) {
    return result;
  } else {
    if (parent["<<"]) {
      return get(parent["<<"], key);
    } else if (parent["DEFAULT"]) {
      return parent["DEFAULT"];
    } else {
      // No key, and no default, so return the null node as expected
      return result;
    }
  }
}

void loadBodyMotionParams(QPControllerParams &params, const YAML::Node &config, const RigidBodyTree &robot) {
  for (auto body_it = robot.bodies.begin(); body_it != robot.bodies.end(); ++body_it) {
    params.body_motion[body_it - robot.bodies.begin()] = get(config, (*body_it)->linkname).as<BodyMotionParams>();
  }

  // for (auto config_it = config.begin(); config_it != config.end(); ++config_it) {
  //   std::regex body_regex = globToRegex((*config_it)["name"].as<std::string>());
  //   for (auto body_it = robot.bodies.begin(); body_it != robot.bodies.end(); ++body_it) {
  //     if (std::regex_match((*body_it)->linkname, body_regex)) {
  //       params.body_motion[body_it - robot.bodies.begin()] = get(*config_it, "params").as<BodyMotionParams>();
  //       // loadSingleBodyMotionParams(params.body_motion[body_it - robot.bodies.begin()], (*config_it)["params"]);
  //     }
  //   }
  // }
}

void loadSingleJointParams(QPControllerParams &params, Eigen::DenseIndex position_index, const YAML::Node &config, const RigidBodyTree &robot) {
  params.whole_body.Kp(position_index) = get(config, "Kp").as<double>();
  params.whole_body.Kd(position_index) = dampingGain(get(config, "Kp").as<double>(), get(config, "damping_ratio").as<double>());
  params.whole_body.w_qdd(position_index) = get(config, "w_qdd").as<double>();

  const YAML::Node &integrator_config = get(config, "integrator");
  params.whole_body.integrator.gains(position_index) = get(integrator_config, "gain").as<double>();
  params.whole_body.integrator.clamps(position_index) = get(integrator_config, "clamp").as<double>();

  const YAML::Node &qdd_bounds_config = get(config, "qdd_bounds");
  params.whole_body.qdd_bounds.min(position_index) = get(qdd_bounds_config, "min").as<double>();
  params.whole_body.qdd_bounds.max(position_index) = get(qdd_bounds_config, "max").as<double>();

  const YAML::Node &soft_limits_config = get(config, "joint_soft_limits");
  params.joint_soft_limits.enabled(position_index) = get(soft_limits_config, "enabled").as<bool>();
  if (get(soft_limits_config, "disable_when_body_in_support").as<std::string>().size() > 0) {
    // std::regex disable_body_regex = globToRegex(get(soft_limits_config, "disable_when_body_in_support").as<std::string>());
    std::string disable_body_name = get(soft_limits_config, "disable_when_body_in_support").as<std::string>();
    for (auto body_it = robot.bodies.begin(); body_it != robot.bodies.end(); ++body_it) {
      if (disable_body_name == (*body_it)->linkname) {
      // if (std::regex_match((*body_it)->linkname, disable_body_regex)) {
        params.joint_soft_limits.disable_when_body_in_support(position_index) = body_it - robot.bodies.begin() + 1;
        // break;
      }
    }
  }
  params.joint_soft_limits.lb(position_index) = get(soft_limits_config, "lb").as<double>();
  params.joint_soft_limits.ub(position_index) = get(soft_limits_config, "ub").as<double>();
  params.joint_soft_limits.kp(position_index) = get(soft_limits_config, "kp").as<double>();
  params.joint_soft_limits.kd(position_index) = dampingGain(params.joint_soft_limits.kp(position_index), get(soft_limits_config, "damping_ratio").as<double>());
  params.joint_soft_limits.weight(position_index) = get(soft_limits_config, "weight").as<double>();
  params.joint_soft_limits.k_logistic(position_index) = get(soft_limits_config, "k_logistic").as<double>();

}

void loadJointParams(QPControllerParams &params, const YAML::Node &config, const RigidBodyTree &robot) {
  std::map<std::string, int> position_name_to_index = robot.computePositionNameToIndexMap();
  for (auto position_it = position_name_to_index.begin(); position_it != position_name_to_index.end(); ++position_it) {
    loadSingleJointParams(params, position_it->second, get(config, position_it->first), robot);
  }

  // for (auto config_it = config.begin(); config_it != config.end(); ++config_it) {
  //   std::regex joint_regex = globToRegex(get(*config_it, "name").as<std::string>());
  //   for (auto position_it = position_name_to_index.begin(); position_it != position_name_to_index.end(); ++position_it) {
  //     if (std::regex_match(position_it->first, joint_regex)) {
  //       // std::cout << get(*config_it, "name").as<std::string>() << " matches " << position_it->first << std::endl;
  //       loadSingleJointParams(params, position_it->second, get(*config_it, "params"), robot);
  //     }
  //   }
  // }
}

void loadSingleInputParams(QPControllerParams &params, Eigen::DenseIndex position_index, YAML::Node config, const RigidBodyTree &robot) {
  YAML::Node hardware_config = get(config, "hardware");
  params.hardware.gains.k_f_p(position_index) = get(hardware_config, "k_f_p").as<double>();
  params.hardware.gains.k_q_p(position_index) = get(hardware_config, "k_q_p").as<double>();
  params.hardware.gains.k_q_i(position_index) = get(hardware_config, "k_q_i").as<double>();
  params.hardware.gains.k_qd_p(position_index) = get(hardware_config, "k_qd_p").as<double>();
  params.hardware.gains.ff_qd(position_index) = get(hardware_config, "ff_qd").as<double>();
  params.hardware.gains.ff_f_d(position_index) = get(hardware_config, "ff_f_d").as<double>();
  params.hardware.gains.ff_const(position_index) = get(hardware_config, "ff_const").as<double>();
  params.hardware.gains.ff_qd_d(position_index) = get(hardware_config, "ff_qd_d").as<double>();
  params.hardware.joint_is_force_controlled(position_index) = get(hardware_config, "joint_is_force_controlled").as<bool>();
  params.hardware.joint_is_position_controlled(position_index) = get(hardware_config, "joint_is_position_controlled").as<bool>();
}

void loadInputParams(QPControllerParams& params, const YAML::Node &config, const RigidBodyTree& robot) {
  for (auto actuator_it = robot.actuators.begin(); actuator_it != robot.actuators.end(); ++actuator_it) {
    std::cout << "loading input params: " << actuator_it->name << std::endl;
    loadSingleInputParams(params, actuator_it - robot.actuators.begin(), get(config, actuator_it->name), robot);
  }

  // for (auto config_it = config.begin(); config_it != config.end(); ++config_it) {
  //   std::regex input_regex = globToRegex(get(*config_it, "name").as<std::string>());
  //   for (auto actuator_it = robot.actuators.begin(); actuator_it != robot.actuators.end(); ++actuator_it) {
  //     if (std::regex_match((*actuator_it).name, input_regex)) {
  //       // std::cout << get(*config_it, "name").as<std::string>() << " matches " << (*actuator_it).name << std::endl;
  //       loadSingleInputParams(params, actuator_it - robot.actuators.begin(), get(*config_it, "params"), robot);
  //     }
  //   }
  // }

}

namespace YAML {
  template<>
  struct convert<VRefIntegratorParams> {

    static bool decode(const Node& node, VRefIntegratorParams& rhs) {
      if (!node.IsMap()) {
        return false;
      }
      rhs.zero_ankles_on_contact = get(node, "zero_ankles_on_contact").as<bool>();
      rhs.eta = get(node, "eta").as<double>();
      rhs.delta_max = get(node, "delta_max").as<double>();
      return true;
    }
  };

  template<>
  struct convert<BodyMotionParams> {
    static bool decode(const Node& node, BodyMotionParams& params) {
      if (!node.IsMap()) {
        return false;
      }
      params.Kp << get(get(node, "Kp"), "x").as<double>(),
                   get(get(node, "Kp"), "y").as<double>(),
                   get(get(node, "Kp"), "z").as<double>(),
                   get(get(node, "Kp"), "wx").as<double>(),
                   get(get(node, "Kp"), "wy").as<double>(),
                   get(get(node, "Kp"), "wz").as<double>();
      double damping_ratio = get(node, "damping_ratio").as<double>();
      params.Kd = 2 * damping_ratio * params.Kp.array().sqrt();
      params.accel_bounds.min << get(get(get(node, "accel_bounds"), "min"), "x").as<double>(),
                                 get(get(get(node, "accel_bounds"), "min"), "y").as<double>(),
                                 get(get(get(node, "accel_bounds"), "min"), "z").as<double>(),
                                 get(get(get(node, "accel_bounds"), "min"), "wx").as<double>(),
                                 get(get(get(node, "accel_bounds"), "min"), "wy").as<double>(),
                                 get(get(get(node, "accel_bounds"), "min"), "wz").as<double>();
      params.accel_bounds.max << get(get(get(node, "accel_bounds"), "max"), "x").as<double>(),
                                 get(get(get(node, "accel_bounds"), "max"), "y").as<double>(),
                                 get(get(get(node, "accel_bounds"), "max"), "z").as<double>(),
                                 get(get(get(node, "accel_bounds"), "max"), "wx").as<double>(),
                                 get(get(get(node, "accel_bounds"), "max"), "wy").as<double>(),
                                 get(get(get(node, "accel_bounds"), "max"), "wz").as<double>();
      params.weight = get(node, "weight").as<double>();
      return true;
    }
  };
}

QPControllerParams loadSingleParamSet(const YAML::Node& config, const RigidBodyTree &robot) {
  QPControllerParams params(robot);

  loadJointParams(params, get(config, "position_specific"), robot);
  params.whole_body.integrator.eta = get(get(config, "integrator"), "eta").as<double>();
  loadBodyMotionParams(params, get(config, "body_specific"), robot);
  loadInputParams(params, get(config, "input_specific"), robot);
  params.vref_integrator = get(config, "vref_integrator").as<VRefIntegratorParams>();
  params.W_kdot = get(config, "W_kdot").as<double>() * Eigen::Matrix3d::Identity();
  params.Kp_ang = get(config, "Kp_ang").as<double>();
  params.w_slack = get(config, "w_slack").as<double>();
  params.slack_limit = get(config, "slack_limit").as<double>();
  params.w_grf = get(config, "w_grf").as<double>();
  params.Kp_accel = get(config, "Kp_accel").as<double>();
  params.contact_threshold = get(config, "contact_threshold").as<double>();
  params.min_knee_angle = get(config, "min_knee_angle").as<double>();
  YAML::Node com_observer_config = get(config, "center_of_mass_observer");
  params.use_center_of_mass_observer = get(com_observer_config, "enabled").as<bool>();
  double l_zmp = get(com_observer_config, "l_zmp").as<double>();
  double l_com = get(com_observer_config, "l_com").as<double>();
  params.center_of_mass_observer_gain = Eigen::Matrix4d::Zero();
  params.center_of_mass_observer_gain(0,0) = l_zmp;
  params.center_of_mass_observer_gain(1,1) = l_zmp;
  params.center_of_mass_observer_gain(2,2) = l_com;
  params.center_of_mass_observer_gain(3,3) = l_com;

  return params;
}

std::map<std::string, QPControllerParams> loadAllParamSetsFromExpandedConfig(YAML::Node config, const RigidBodyTree &robot) {
  auto param_sets = std::map<std::string, QPControllerParams>();
  for (auto config_it = config.begin(); config_it != config.end(); ++config_it) {
    std::cout << "loading param set: " << config_it->first << std::endl;
    QPControllerParams params = loadSingleParamSet(config_it->second, robot);
    param_sets.insert(std::pair<std::string, QPControllerParams>(config_it->first.as<std::string>(), params));
  }
  return param_sets;
}

std::map<std::string, QPControllerParams> loadAllParamSets(YAML::Node config, const RigidBodyTree &robot) {

  config = expandDefaults(config);
  return loadAllParamSetsFromExpandedConfig(config, robot);
}


std::map<std::string, QPControllerParams> loadAllParamSets(YAML::Node config, const RigidBodyTree &robot, std::ofstream debug_output_file) {

  config = expandDefaults(config);
  debug_output_file << config;
  return loadAllParamSetsFromExpandedConfig(config, robot);
}

