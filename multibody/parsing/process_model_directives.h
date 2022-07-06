#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/multibody/parsing/model_directives.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace parsing {

ModelDirectives LoadModelDirectives(const std::string& filename);

/// Converts URIs into filesystem absolute paths.
///
/// ModelDirectives refer to their resources by URIs like
/// `package://somepackage/somepath/somefile.sdf`, where somepackage refers to
/// the ROS-style package.xml system.
std::string ResolveModelDirectiveUri(
    const std::string& uri,
    const drake::multibody::PackageMap& package_map);

// TODO(#13074): Burn this in a dumpster fire pending real model
// composition / extraction in Drake.
// TODO(#14084): This structure might combine with the implementation of 14084:
// https://github.com/RobotLocomotion/drake/issues/14084#issuecomment-694394869
/// Convenience structure to hold all of the information to add a model
/// instance from a file.
struct ModelInstanceInfo {
  /// Model name (possibly scoped).
  std::string model_name;
  /// File path.
  std::string model_path;
  /// WARNING: This is the *unscoped* parent frame, assumed to be unique.
  std::string parent_frame_name;
  /// This is the unscoped frame name belonging to `model_instance`.
  std::string child_frame_name;
  drake::math::RigidTransformd X_PC;
  drake::multibody::ModelInstanceIndex model_instance;
};

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
