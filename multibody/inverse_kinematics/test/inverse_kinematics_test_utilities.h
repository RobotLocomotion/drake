#pragma once

#include <memory>

#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace test {
/**
 * Constructs a MultibodyTree consists of two free bodies. This two free bodies
 * will be used to test kinematic constraints.
 */
template <typename T>
std::unique_ptr<MultibodyTree<T>> ConstructTwoFreeBodies();
}  // namespace test
}  // namespace multibody
}  // namespace drake
