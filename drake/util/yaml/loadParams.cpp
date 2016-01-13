#include <iostream>
#include <regex>
#include "yamlUtil.h"
#include "Path.h"

int main(int argc, char** argv) {
  YAML::Node config = YAML::LoadFile(Drake::getDrakePath() + "/examples/Atlas/+atlasParams/qp_controller_params.yaml");

  auto robot = std::make_shared<RigidBodyTree>(Drake::getDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf");

  std::map<std::string, QPControllerParams> params_from_yaml = loadAllParamSets(config, *robot);
  // QPControllerParams params = loadSingleParamSet(get(config, "walking"), *robot);

  // std::map<std::string, int> position_name_to_index = robot->computePositionNameToIndexMap();
  // std::cout << "base_x Kp: " << params.whole_body.Kp(position_name_to_index["base_x"]) << " w_qdd: " << params.whole_body.w_qdd(position_name_to_index["base_x"]) << std::endl;
  // std::cout << "body 5 linkanme: " << robot->bodies[5]->linkname << " Kp: " << params.body_motion[5].Kp.transpose() << " accel_max: " << params.body_motion[5].accel_bounds.max.transpose() << std::endl;
  return 0;
}