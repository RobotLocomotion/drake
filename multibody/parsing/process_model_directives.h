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

ModelDirectives LoadModelDirectives(const std::filesystem::path& filename);

ModelDirectives LoadModelDirectivesFromString(
    const std::string& model_directives);

/// Converts URIs into filesystem absolute paths.
///
/// ModelDirectives refer to their resources by URIs like
/// `package://somepackage/somepath/somefile.sdf`, where somepackage refers to
/// the ROS-style package.xml system.
std::string ResolveModelDirectiveUri(
    const std::string& uri, const drake::multibody::PackageMap& package_map);

/// Flatten model directives into a single object.
///
/// This function removes all AddDirectives directives from the given
/// `directives`, locating the file via the given `package_map`, parsing it,
/// and updating the names of its items to add any namespace prefix
/// requested by the `model_namespace` of the directive.  The resulting
/// directives are appended to `out`.
///
/// The results of FlattenModelDirectives are semantically identical to
/// `directives`.  FlattenModelDirectives is therefore also idempotent.
///
/// This flattening is intended to assist with creating reproducible
/// simulation scenarios and with hashing; it can also be useful in debugging.
void FlattenModelDirectives(const ModelDirectives& directives,
                            const drake::multibody::PackageMap& package_map,
                            ModelDirectives* out);

/// Parses the given model directives using the given parser.
/// The MultibodyPlant (and optionally SceneGraph) being modified are
/// implicitly associated with the Parser object.
/// Returns the list of added models.
std::vector<ModelInstanceInfo> ProcessModelDirectives(
    const ModelDirectives& directives, drake::multibody::Parser* parser);

/// Processes model directives for a given MultibodyPlant.
void ProcessModelDirectives(
    const ModelDirectives& directives,
    drake::multibody::MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models = nullptr,
    drake::multibody::Parser* parser = nullptr);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
