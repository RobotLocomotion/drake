#pragma once

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace multibody {

/**
 * Compares a %RigidBodyTree with its clone. Since this method is intended to
 * compare a clone, an *exact* match is performed. This method will only
 * return `true` if the provided %RigidBodyTree is exactly the same as its
 * clone.
 */
bool CompareToClone(const RigidBodyTree<double>& tree);

}  // namespace multibody
}  // namespace drake
