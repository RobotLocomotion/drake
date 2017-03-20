#pragma once

#include <fstream>
#include <map>

#include "yaml-cpp/yaml.h"

#include "drake/common/eigen_stl_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/QPCommon.h"

YAML::Node applyDefaults(const YAML::Node& node,
                         const YAML::Node& default_node);
YAML::Node expandDefaults(const YAML::Node& node);

// yaml-cpp does not understand the DEFAULT: syntax for a default value or the
// <<: syntax to merge the contents of a node. This function wraps the map
// lookup to provide the correct behavior.
YAML::Node get(const YAML::Node& parent, const std::string& key);

QPControllerParams
loadSingleParamSet(const YAML::Node& config,
                   const RigidBodyTree<double>& robot);
drake::eigen_aligned_std_map<std::string, QPControllerParams> loadAllParamSets(
    YAML::Node config, const RigidBodyTree<double>& robot);
drake::eigen_aligned_std_map<std::string, QPControllerParams> loadAllParamSets(
    YAML::Node config, const RigidBodyTree<double>& robot,
    std::ofstream& debug_output_file);
RobotPropertyCache parseKinematicTreeMetadata(
    const YAML::Node& metadata, const RigidBodyTree<double>& robot);
KinematicModifications parseKinematicModifications(const YAML::Node& mods);
