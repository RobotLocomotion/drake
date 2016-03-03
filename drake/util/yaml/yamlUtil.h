#include <map>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/systems/plants/RigidBodyTree.h"

YAML::Node applyDefaults(const YAML::Node& node,
                         const YAML::Node& default_node);
YAML::Node expandDefaults(const YAML::Node& node);

// yaml-cpp does not understand the DEFAULT: syntax for a default value or the
// <<:
// syntax to merge the contents of a node. This function wraps the map lookup
// to provide the correct behavior.
YAML::Node get(const YAML::Node& parent, const std::string& key);

QPControllerParams loadSingleParamSet(const YAML::Node& config,
                                      const RigidBodyTree& robot);
std::map<std::string, QPControllerParams> loadAllParamSets(
    YAML::Node config, const RigidBodyTree& robot);
std::map<std::string, QPControllerParams> loadAllParamSets(
    YAML::Node config, const RigidBodyTree& robot,
    std::ofstream& debug_output_file);
RobotPropertyCache parseKinematicTreeMetadata(const YAML::Node& metadata,
                                              const RigidBodyTree& robot);
KinematicModifications parseKinematicModifications(const YAML::Node& mods);
