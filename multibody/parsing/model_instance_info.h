#pragma once

#include <string>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace parsing {

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

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
