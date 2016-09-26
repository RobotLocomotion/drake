#pragma once

#include <fstream>
#include <map>

#include "yaml-cpp/yaml.h"

#include "drake/common/eigen_stl_types.h"
#include "drake/drakeYAMLUtil_export.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/systems/plants/RigidBodyTree.h"

DRAKEYAMLUTIL_EXPORT YAML::Node applyDefaults(const YAML::Node& node,
                                              const YAML::Node& default_node);
DRAKEYAMLUTIL_EXPORT YAML::Node expandDefaults(const YAML::Node& node);

// yaml-cpp does not understand the DEFAULT: syntax for a default value or the
// <<: syntax to merge the contents of a node. This function wraps the map
// lookup to provide the correct behavior.
DRAKEYAMLUTIL_EXPORT YAML::Node get(const YAML::Node& parent,
                                    const std::string& key);

DRAKEYAMLUTIL_EXPORT QPControllerParams
loadSingleParamSet(const YAML::Node& config, const RigidBodyTree& robot);
DRAKEYAMLUTIL_EXPORT
drake::eigen_aligned_std_map<std::string, QPControllerParams> loadAllParamSets(
    YAML::Node config, const RigidBodyTree& robot);
DRAKEYAMLUTIL_EXPORT
drake::eigen_aligned_std_map<std::string, QPControllerParams> loadAllParamSets(
    YAML::Node config, const RigidBodyTree& robot,
    std::ofstream& debug_output_file);
DRAKEYAMLUTIL_EXPORT RobotPropertyCache parseKinematicTreeMetadata(
    const YAML::Node& metadata, const RigidBodyTree& robot);
DRAKEYAMLUTIL_EXPORT KinematicModifications
parseKinematicModifications(const YAML::Node& mods);
