#pragma once

#include "drake/multibody/RigidBodyTree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * Verify that @p tree matches assumptions about joint indices.
 * Aborts if the tree isn't as expected.
 */
void VerifyIiwaTree(const RigidBodyTree<double>& tree);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
