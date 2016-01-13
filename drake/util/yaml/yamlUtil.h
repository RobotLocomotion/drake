#include <regex>
#include <map>
#include "yaml-cpp/yaml.h"
#include "QPCommon.h"
#include "RigidBodyTree.h"

// Convert simple glob expressions (using * as wildcard) to regex by:
//    1. Prefixing each regex special chars EXCEPT * with \
//    2. Converting * to .*
std::regex globToRegex(const std::string& glob);

YAML::Node applyDefaults(const YAML::Node& node, const YAML::Node& default_node);
YAML::Node findAndApplyDefaults(const YAML::Node& node);

// yaml-cpp does not understand the =: syntax for a default value or the <<:
// syntax to merge the contents of a node. This function wraps the map lookup
// to provide the correct behavior.
YAML::Node get(const YAML::Node& parent, const std::string& key);

// 
// Tools for loading various parameter sets from YAML
//
void loadSingleBodyMotionParams(BodyMotionParams &params, const YAML::Node & config);
void loadBodyMotionParams(QPControllerParams &params, const YAML::Node &config, const RigidBodyTree &robot);
void loadSingleJointParams(QPControllerParams &params, Eigen::DenseIndex position_index, const YAML::Node &config, const RigidBodyTree &robot);
void loadJointParams(QPControllerParams &params, const YAML::Node &config, const RigidBodyTree &robot);
QPControllerParams loadSingleParamSet(const YAML::Node &config, const RigidBodyTree &robot);
std::map<std::string, QPControllerParams> loadAllParamSets(YAML::Node config, const RigidBodyTree &robot);