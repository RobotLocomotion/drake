#pragma once

#include <memory>

#include "drake/attic_warning.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
/** Construct a RigidBodyTree containing a four bar linkage. */
std::unique_ptr<RigidBodyTree<double>> ConstructFourBarTree();
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
