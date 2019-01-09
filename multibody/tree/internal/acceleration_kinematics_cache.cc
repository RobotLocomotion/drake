#include "drake/multibody/tree/internal/acceleration_kinematics_cache.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace internal {

// Explicitly instantiates on the most common scalar types.
template class AccelerationKinematicsCache<double>;
template class AccelerationKinematicsCache<AutoDiffXd>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
