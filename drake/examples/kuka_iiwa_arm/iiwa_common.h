#pragma once

#include "drake/common/drake_export.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * Adds a ground plane to @p tree. The ground is represented as a box-shaped
 * visual and collision element.
 *
 * @param[in] tree The RigidBodyTree to which to add the ground plane.
 */
DRAKE_EXPORT
void AddGround(RigidBodyTree* tree);

/**
 * Verify that @p tree matches assumptions about joint indices.
 * Aborts if the tree isn't as expected.
 */
DRAKE_EXPORT
void VerifyIiwaTree(const RigidBodyTree& tree);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
