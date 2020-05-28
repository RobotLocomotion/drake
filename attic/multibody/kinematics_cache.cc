/* clang-format: off */
// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/multibody/kinematics_cache-inl.h"
/* clang-format: on */

#include "drake/common/autodiff.h"

// Explicitly instantiates on the most common scalar types.
template class KinematicsCacheElement<double>;
template class KinematicsCacheElement<drake::AutoDiffXd>;

template class KinematicsCache<double>;
template class KinematicsCache<drake::AutoDiffXd>;
