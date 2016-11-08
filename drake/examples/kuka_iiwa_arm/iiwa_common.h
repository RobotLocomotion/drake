#pragma once

#include "drake/common/drake_export.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * Verify that @p tree matches assumptions about joint indices.
 * Aborts if the tree isn't as expected.
 */
DRAKE_EXPORT
void VerifyIiwaTree(const RigidBodyTree<double>& tree);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
