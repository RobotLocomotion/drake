#pragma once

#include <string>
#include <vector>

#include "drake/multibody/parsing/model_directives.h"
#include "drake/multibody/parsing/model_instance_info.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace parsing {

ModelDirectives LoadModelDirectives(const std::string& filename);

ModelDirectives LoadModelDirectivesFromString(
    const std::string& model_directives);

/// Converts URIs into filesystem absolute paths.
///
/// ModelDirectives refer to their resources by URIs like
/// `package://somepackage/somepath/somefile.sdf`, where somepackage refers to
/// the ROS-style package.xml system.
std::string ResolveModelDirectiveUri(
    const std::string& uri,
    const drake::multibody::PackageMap& package_map);

/// Flatten model directives.
void FlattenModelDirectives(const ModelDirectives& directives,
                            const drake::multibody::PackageMap& package_map,
                            ModelDirectives* out);

/// Parses the given model directives using the given parser.
/// The MultibodyPlant (and optionally SceneGraph) being modified are
/// implicitly associated with the Parser object.
/// Returns the list of added models.
std::vector<ModelInstanceInfo> ProcessModelDirectives(
    const ModelDirectives& directives,
    drake::multibody::Parser* parser);

/// Processes model directives for a given MultibodyPlant.
void ProcessModelDirectives(
    const ModelDirectives& directives,
    drake::multibody::MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models = nullptr,
    drake::multibody::Parser* parser = nullptr);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
