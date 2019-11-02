#pragma once

#include <fstream>
#include <map>
#include <string>

#include "yaml-cpp/yaml.h"

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/QPCommon.h"

DRAKE_DEPRECATED("2020-02-01", "Some attic controllers code is being removed.")
YAML::Node applyDefaults(const YAML::Node& node,
                         const YAML::Node& default_node);

DRAKE_DEPRECATED("2020-02-01", "Some attic controllers code is being removed.")
YAML::Node expandDefaults(const YAML::Node& node);

// yaml-cpp does not understand the DEFAULT: syntax for a default value or the
// <<: syntax to merge the contents of a node. This function wraps the map
// lookup to provide the correct behavior.
DRAKE_DEPRECATED("2020-02-01", "Some attic controllers code is being removed.")
YAML::Node get(const YAML::Node& parent, const std::string& key);

DRAKE_DEPRECATED("2020-02-01", "Some attic controllers code is being removed.")
QPControllerParams
loadSingleParamSet(const YAML::Node& config,
                   const RigidBodyTree<double>& robot);

DRAKE_DEPRECATED("2020-02-01", "Some attic controllers code is being removed.")
drake::eigen_aligned_std_map<std::string, QPControllerParams> loadAllParamSets(
    YAML::Node config, const RigidBodyTree<double>& robot);

DRAKE_DEPRECATED("2020-02-01", "Some attic controllers code is being removed.")
drake::eigen_aligned_std_map<std::string, QPControllerParams> loadAllParamSets(
    YAML::Node config, const RigidBodyTree<double>& robot,
    // NOLINTNEXTLINE(runtime/references)
    std::ofstream& debug_output_file);

DRAKE_DEPRECATED("2020-02-01", "Some attic controllers code is being removed.")
RobotPropertyCache parseKinematicTreeMetadata(
    const YAML::Node& metadata, const RigidBodyTree<double>& robot);

DRAKE_DEPRECATED("2020-02-01", "Some attic controllers code is being removed.")
KinematicModifications parseKinematicModifications(const YAML::Node& mods);
