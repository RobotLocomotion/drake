#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class VelocityKinematicsCache<double>;
template class VelocityKinematicsCache<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
