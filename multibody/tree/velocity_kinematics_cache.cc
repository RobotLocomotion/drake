#include "drake/multibody/tree/velocity_kinematics_cache.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace internal {

// Explicitly instantiates on the most common scalar types.
template class VelocityKinematicsCache<double>;
template class VelocityKinematicsCache<AutoDiffXd>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
