#include "drake/multibody/multibody_tree/joints/revolute_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class RevoluteJoint<double>;
template class RevoluteJoint<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
