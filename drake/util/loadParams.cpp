#include <iostream>
#include <regex>
#include "yaml-cpp/yaml.h"
#include "QPCommon.h"
#include "RigidBodyTree.h"

double dampingGain(double Kp, double damping_ratio) {
  return 2 * damping_ratio * sqrt(Kp);
}

YAML::Node get(YAML::Node parent, std::string key) {
  // yaml-cpp has a bug, in which it does not use the <<: syntax to merge in
  // the contents of a default node. See: https://github.com/jbeder/yaml-cpp/issues/353 
  // So, instead, we implement our own get() function which also checks for
  // the (wrongly-generated) << key and handles it transparently.
  auto result = parent[key];
  if (result) {
    return result;
  } else {
    auto default_node = parent["<<"];
    if (default_node) {
      return get(default_node, key);
    } else {
      // No key, and no default, so return the null node as expected
      return result;
    }
  }
}

void loadSingleJointParams(WholeBodyParams & params, Eigen::DenseIndex index, const YAML::Node &config) {
  params.Kp(index) = get(config, "Kp").as<double>();
  params.Kd(index) = dampingGain(get(config, "Kp").as<double>(), get(config, "damping_ratio").as<double>());
  params.w_qdd(index) = get(config, "w_qdd").as<double>();
}

void loadSingleBodyMotionParams(BodyMotionParams &params, const YAML::Node & config) {
  params.Kp << get(get(config, "Kp"), "x").as<double>(),
               get(get(config, "Kp"), "y").as<double>(),
               get(get(config, "Kp"), "z").as<double>(),
               get(get(config, "Kp"), "wx").as<double>(),
               get(get(config, "Kp"), "wy").as<double>(),
               get(get(config, "Kp"), "wz").as<double>();
  double damping_ratio = get(config, "damping_ratio").as<double>();
  params.Kd = 2 * damping_ratio * params.Kp.array().sqrt();
  params.accel_bounds.min << get(get(get(config, "accel_bounds"), "min"), "x").as<double>(),
                             get(get(get(config, "accel_bounds"), "min"), "y").as<double>(),
                             get(get(get(config, "accel_bounds"), "min"), "z").as<double>(),
                             get(get(get(config, "accel_bounds"), "min"), "wx").as<double>(),
                             get(get(get(config, "accel_bounds"), "min"), "wy").as<double>(),
                             get(get(get(config, "accel_bounds"), "min"), "wz").as<double>();
  params.accel_bounds.max << get(get(get(config, "accel_bounds"), "max"), "x").as<double>(),
                             get(get(get(config, "accel_bounds"), "max"), "y").as<double>(),
                             get(get(get(config, "accel_bounds"), "max"), "z").as<double>(),
                             get(get(get(config, "accel_bounds"), "max"), "wx").as<double>(),
                             get(get(get(config, "accel_bounds"), "max"), "wy").as<double>(),
                             get(get(get(config, "accel_bounds"), "max"), "wz").as<double>();
  params.weight = get(config, "weight").as<double>();
}

void loadBodyMotionParams(QPControllerParams &params, const YAML::Node &config, const RigidBodyTree &robot) {
  for (auto config_it = config.begin(); config_it != config.end(); ++config_it) {
    auto body_regex = std::regex(config_it->first.as<std::string>());
    for (auto body_it = robot.bodies.begin(); body_it != robot.bodies.end(); ++body_it) {
      if (std::regex_match((*body_it)->linkname.begin(), (*body_it)->linkname.end(), body_regex)) {
        loadSingleBodyMotionParams(params.body_motion[body_it - robot.bodies.begin()], config_it->second);
      }
    }
  }
}


int main(int argc, char** argv) {
  YAML::Node config = YAML::LoadFile("/home/rdeits/locomotion/drake-distro/drake/examples/Atlas/+atlasParams/Base.yaml");

  auto robot = std::make_shared<RigidBodyTree>("/home/rdeits/locomotion/drake-distro/drake/examples/Atlas/urdf/atlas_minimal_contact.urdf");

  std::map<std::string, int> position_name_to_index = robot->computePositionNameToIndexMap();

  QPControllerParams params(*robot);

  YAML::Node whole_body_config = config["whole_body"];
  for (auto config_it = whole_body_config.begin(); config_it != whole_body_config.end(); ++config_it) {
    auto joint_regex = std::regex(config_it->first.as<std::string>());
    for (auto position_it = position_name_to_index.begin(); position_it != position_name_to_index.end(); ++position_it) {
      if (std::regex_match(position_it->first.begin(), position_it->first.end(), joint_regex)) {
        loadSingleJointParams(params.whole_body, position_it->second, config_it->second);
      }
    }
  }

  loadBodyMotionParams(params, config["body_motion"], *robot);

  std::cout << "base_x Kp: " << params.whole_body.Kp(position_name_to_index["base_x"]) << " w_qdd: " << params.whole_body.w_qdd(position_name_to_index["base_x"]) << std::endl;
  std::cout << "body 0 linkanme: " << robot->bodies[0]->linkname << " Kp: " << params.body_motion[0].Kp.transpose() << " accel_max: " << params.body_motion[0].accel_bounds.max.transpose() << std::endl;
  return 0;
}