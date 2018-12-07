#include "drake/multibody/multibody_tree/acceleration_kinematics_cache.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class AccelerationKinematicsCache<double>;
template class AccelerationKinematicsCache<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
