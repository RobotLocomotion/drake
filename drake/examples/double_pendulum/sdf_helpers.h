#pragma once

#include "drake/multibody/rigid_body_tree.h"

#include "sdf/sdf.hh"

namespace drake {
namespace examples {
namespace double_pendulum {

/// Parses a model from an SDF description into
/// a tree.
///
/// @param[in] model SDF model description to parse from.
/// @param[in,out] tree rigid body tree to parse into.
void ParseModel(const sdf::ElementPtr& model,
                RigidBodyTree<double>* tree);

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake

