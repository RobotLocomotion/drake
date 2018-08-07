#pragma once
/// @file
///
/// This file assumes a 16-degree of freedom allegro hand.
///
///

#include <string>

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace allegro_hand {

constexpr int kNumDofs = 16;  // degrees of freedom available 

/// Builds a RigidBodyTree at the specified @position and @orientation from
/// the model specified by @model_file_name.
/// This method is a convenience wrapper over `AddModelInstanceFromUrdfFile`.
/// @see drake::parsers::urdf::AddModelInstanceFromUrdfFile
void CreateTreeFromFixedModelAtPose(
    const std::string& model_file_name, RigidBodyTreed* tree,
    const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& orientation = Eigen::Vector3d::Zero());


}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
