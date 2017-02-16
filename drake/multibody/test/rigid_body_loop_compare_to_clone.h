#pragma once

#include "drake/multibody/rigid_body_loop.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_loop {

/**
 * This method compares the provided @p original and @p other RigidBodyLoop
 * objects, which are clones, and verifies their correctness. Since this method
 * is intended to compare a clone, an *exact* match is performed. This method
 * will only return `true` if the provided %RigidBodyLoop is exactly the
 * same as its clone.
 */
bool CompareToClone(const RigidBodyLoop<double>& original,
                    const RigidBodyLoop<double>& other);

}  // namespace rigid_body_loop
}  // namespace test
}  // namespace multibody
}  // namespace drake
