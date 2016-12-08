#include "drake/multibody/rbt_with_alternates/rigid_body_tree_with_alternates.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {

// Explicitly instantiates on the most common scalar types.
template class RigidBodyTreeWithAlternates<double>;
//template class RigidBodyTreeWithAlternates<AutoDiffXd>;

}  // namespace drake