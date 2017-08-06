#pragma once

#include <string>

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace double_pendulum {

/// Parses a model from an SDF file into a RigidBodyTree.
/// @remarks if more than one model is described on the given
/// SDF file, only the first one found will be parsed.
/// @note only point mass inertias are supported.
/// @param[in] sdf_path where the model description is.
/// @param[in,out] tree mutable rigid body tree to parse into.
/// @return instance id for the newly added model
int ParseModelFromFile(const std::string& sdf_path,
                        RigidBodyTree<double>* tree);

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake

