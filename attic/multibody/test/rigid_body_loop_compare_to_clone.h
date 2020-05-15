#pragma once

#include "drake/attic_warning.h"
#include "drake/multibody/rigid_body_loop.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_loop {

/**
 * Compares the provided `original` object to the provided `clone` and verifies
 * that the clone is correct relative to the original. Since this method is
 * intended to compare an original to its clone, an *exact* match is performed.
 * This method will only return `true` if the provided `original` is exactly the
 * same as the provided clone.
 */
bool CompareToClone(const RigidBodyLoop<double>& original,
                    const RigidBodyLoop<double>& other);

}  // namespace rigid_body_loop
}  // namespace test
}  // namespace multibody
}  // namespace drake
