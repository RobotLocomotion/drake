#include "drake/multibody/multibody_tree/position_kinematics_cache.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class PositionKinematicsCache<double>;
template class PositionKinematicsCache<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
