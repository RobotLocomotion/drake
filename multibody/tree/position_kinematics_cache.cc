#include "drake/multibody/tree/position_kinematics_cache.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace internal {

// Explicitly instantiates on the most common scalar types.
template class PositionKinematicsCache<double>;
template class PositionKinematicsCache<AutoDiffXd>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
