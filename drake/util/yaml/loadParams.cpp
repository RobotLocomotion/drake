#include <iostream>
#include <fstream>
#include <regex>
#include "yamlUtil.h"
#include "Path.h"

// void applyDefaults(YAML::Node& parent) {
//   if (parent.IsMap() && parent["DEFAULT"]) {
//     for (auto child : parent) {
//       if (child->first != "DEFAULT) {


// template <typename T>
// YAML::Node getDefault(const YAML::Node& parent, std::initializer_list<T> fields) {
//   YAML::Node& child;
//   for (auto field : fields) {
//     child = parent[child];
//     if (!child) {
//       break;
//     }
//   }
//   if (child) {
//     return child;
//   }
//   for (auto field 


int main(int argc, char** argv) {
  YAML::Node config = YAML::LoadFile(Drake::getDrakePath() + "/examples/Atlas/+atlasParams/defaults.yaml");

  findAndApplyDefaults(config);

  std::ofstream out_file;
  out_file.open (Drake::getDrakePath() + "/examples/Atlas/+atlasParams/defaults_out.yaml");
  out_file << config;
  out_file.close();

  // std::cout << "c: " << get(get(config, "body_specific"), "c") << std::endl;
  std::cout << "manip body c: " << get(get(get(config, "manip"), "body_specific"), "c") << std::endl;

  // auto robot = std::make_shared<RigidBodyTree>(Drake::getDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf");

  // YAML::Node config_yaml = YAML::LoadFile(Drake::getDrakePath() + "/examples/Atlas/+atlasParams/params_defaults.yaml");
  // std::map<std::string, QPControllerParams> params_from_yaml = loadAllParamSets(config_yaml, *robot);

  return 0;
}