#pragma once

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

/// Check that robot_state_t can unambiguously represent the RigidBodyTree's
/// state. This method is intended to check preconditions inside a constructor's
/// intitializer list.
/// Aborts when the tree is not compatible with robot_state_t.
/// @param tree a RigidBodyTree.
/// @return the same RigidBodyTree that was passed in.
const RigidBodyTree<double>&
CheckTreeIsRobotStateLcmTypeCompatible(const RigidBodyTree<double>& tree);

}  // namespace systems
}  // namespace drake
