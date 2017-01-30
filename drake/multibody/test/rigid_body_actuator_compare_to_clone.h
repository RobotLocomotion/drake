#pragma once

#include "drake/multibody/rigid_body_actuator.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_actuator {

/**
 * This method compares the provided @p original and @p clone RigidBodyActuator
 * objects, which are clones, and verifies their correctness. Since this method
 * is intended to compare a clone, an *exact* match is performed. This method
 * will only return `true` if the provided %RigidBodyActuator is exactly the
 * same as its clone.
 */
bool CompareToClone(const RigidBodyActuator& original,
                    const RigidBodyActuator& clone);

}  // namespace rigid_body_actuator
}  // namespace test
}  // namespace multibody
}  // namespace drake
