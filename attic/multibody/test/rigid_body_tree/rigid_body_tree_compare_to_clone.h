#pragma once

#include "drake/attic_warning.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_tree {

/**
 * This method clones the provided `tree` and verifies its correctness against
 * the original RigidBodyTree. Since this method is intended to compare a clone,
 * an *exact* match is performed. This method will only return `true` if the
 * provided %RigidBodyTree is exactly the same as its clone.
 */
bool CompareToClone(const RigidBodyTree<double>& tree);

}  // namespace rigid_body_tree
}  // namespace test
}  // namespace multibody
}  // namespace drake
