// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/multibody/kinematics_cache-inl.h"

#include "drake/common/eigen_autodiff_types.h"

// Explicitly instantiates on the most common scalar types.
template class KinematicsCacheElement<double>;
template class KinematicsCacheElement<drake::AutoDiffXd>;
template class KinematicsCacheElement<drake::AutoDiffUpTo73d>;

template class KinematicsCache<double>;
template class KinematicsCache<drake::AutoDiffXd>;
template class KinematicsCache<drake::AutoDiffUpTo73d>;
