#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/multibody/parsing/dev/model_directives.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace parsing {

ModelDirectives LoadModelDirectives(const std::string& filename);

/// ModelDirectives refer to their resources by URIs like
/// `package://somepackage/somepath/somefile.sdf`, where somepackage refers to
/// the ROS-style package.xml system.  This method converts those URIs into
/// filesystem absolute paths.
std::string ResolveModelDirectiveUri(
    const std::string& uri,
    const drake::multibody::PackageMap& package_map);

// TODO(eric.cousineau): Rename this to `ModelInstanceInfo` to deconflict with
// `model_info.h`.
// TODO(eric.cousineau): Burn this in a dumpster fire pending real model
// composition / extraction in Drake. This is just dumb.
struct ModelInfo {
  // Model name (possibly scoped).
  std::string model_name;
  // File path.
  std::string model_path;
  // WARNING: This is the *unscoped* parent frame, assumed to be unique.
  std::string parent_frame_name;
  // This is the unscoped frame name belonging to `model_instance`.
  std::string child_frame_name;
  drake::math::RigidTransformd X_PC;
  drake::multibody::ModelInstanceIndex model_instance;
};

ModelInfo MakeModelInfo(
    const std::string& model_name, const std::string& model_path,
    const std::string& parent_frame_name, const std::string& child_frame_name,
    const drake::math::RigidTransformd& X_PC = drake::math::RigidTransformd());

/**
 * If `X_PC` is not identity, a AddFrame directive is made to insert a
 * new frame named "model_name_attachment_frame" that's offset from
 * `parent_frame_name` by `X_PC`, and the subsequent AddModel directive is
 * made to weld `model_name`'s `child_frame_name` to this new frame.
 * Otherwise, a AddModel directive is made to weld `model_name`'s
 * `child_frame_name` to `parent_frame_name` directly. The `model_instance`
 * field is ignored for this purposes.
 */
ModelDirectives MakeModelsAttachedToFrameDirectives(
    const std::vector<ModelInfo>& models_to_add);

// Flatten model directives.
void FlattenModelDirectives(const ModelDirectives& directives,
                            const drake::multibody::PackageMap& package_map,
                            ModelDirectives* out);

/// Provides a magical way to inject error (randomization) for synthesis.
/// Maps from (parent_frame, child_frame) -> X_PCe, where Ce is the perturbed
/// child frame pose w.r.t. parent frame P. If there is no error, then nullopt
/// should be returned.
using ModelWeldErrorFunction =
    std::function<std::optional<drake::math::RigidTransformd>(
        const drake::multibody::Frame<double>&,
        const drake::multibody::Frame<double>&)>;

/// Processes model directives for a given MultibodyPlant.
///
/// Note: The passed-in parser will be mutated to add the jaco_description
/// package to its package map (since model directives and their contents are
/// allowed to refer to the package directly for workaround reasons).
void ProcessModelDirectives(const ModelDirectives& directives,
                            drake::multibody::MultibodyPlant<double>* plant,
                            std::vector<ModelInfo>* added_models = nullptr,
                            drake::multibody::Parser* parser = nullptr,
                            ModelWeldErrorFunction = nullptr);

/// Finds an optionally model-scoped frame according to
/// `internal::ScopedNameParser::Parse`.
const drake::multibody::Frame<double>*
GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name);

/// Required version of `GetScopedFrameByNameMaybe`.
inline const drake::multibody::Frame<double>&
GetScopedFrameByName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name) {
  auto* frame = GetScopedFrameByNameMaybe(plant, full_name);
  if (frame == nullptr) {
    throw std::runtime_error("Could not find frame: " + full_name);
  }
  return *frame;
}

const std::string GetScopedFrameName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::Frame<double>& frame);

namespace internal {

// TODO(eric.cousineau): Consider hoisting this.
std::string FindDirectiveResource(const std::string& name);

struct ScopedName {
  // If empty, implies no scope.
  std::string instance_name;
  std::string name;
};

// Attempts to find a name using the following scoping rules:
// - The delimiter "::" may appear zero or more times.
// - If one more delimiters are present, the full name is split by the *last*
// delimiter. The provided model instance name must exist.
ScopedName ParseScopedName(const std::string& full_name);

}  // namespace internal

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
