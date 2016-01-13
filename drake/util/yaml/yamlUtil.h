#include <regex>
#include <map>
#include "yaml-cpp/yaml.h"
#include "QPCommon.h"
#include "RigidBodyTree.h"

// Convert simple glob expressions (using * as wildcard) to regex by:
//    1. Prefixing each regex special chars EXCEPT * with \
//    2. Converting * to .*
std::regex globToRegex(const std::string &glob);


// yaml-cpp has a bug, in which it does not use the <<: syntax to merge in
// the contents of a default node. See: https://github.com/jbeder/yaml-cpp/issues/353 
// So, instead, we implement our own get() function which also checks for
// the << key and uses it to supply any missing fields.
YAML::Node get(YAML::Node parent, std::string key);

// 
// Tools for loading various parameter sets from YAML
//
void loadSingleBodyMotionParams(BodyMotionParams &params, const YAML::Node & config);
void loadBodyMotionParams(QPControllerParams &params, const YAML::Node &config, const RigidBodyTree &robot);
void loadSingleJointParams(QPControllerParams &params, Eigen::DenseIndex position_index, YAML::Node config, const RigidBodyTree &robot);
void loadJointParams(QPControllerParams &params, const YAML::Node &config, const RigidBodyTree &robot);
QPControllerParams loadSingleParamSet(YAML::Node config, const RigidBodyTree &robot);
std::map<std::string, QPControllerParams> loadAllParamSets(YAML::Node config, const RigidBodyTree &robot);