#pragma once

#include <string>

#include "drake/multibody/rigid_body_tree.h"

#include "sdf/sdf.hh"

namespace drake {
namespace examples {
namespace double_pendulum {

/// Parses a model from an SDF file into a RigidBodyTree.
/// @param[in] sdf_path where the model description is.
/// @param[in,out] tree mutable rigid body tree to parse into.
void ParseModelFromFile(const std::string& sdf_path,
                        RigidBodyTree<double>* tree);

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake

