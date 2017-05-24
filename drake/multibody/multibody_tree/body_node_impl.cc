#include "drake/multibody/multibody_tree/body_node_impl.h"

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
//#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"

namespace drake {
namespace multibody {

// Macro used to explicitly instantiate implementations on all sizes needed.
#define EXPLICITLY_INSTANTIATE_IMPLS(T) \
template class BodyNodeImpl<T, 1, 1>; \
template class BodyNodeImpl<T, 2, 2>; \
template class BodyNodeImpl<T, 3, 3>; \
template class BodyNodeImpl<T, 4, 4>; \
template class BodyNodeImpl<T, 5, 5>; \
template class BodyNodeImpl<T, 6, 6>; \
template class BodyNodeImpl<T, 7, 6>;

// Explicitly instantiates on the most common scalar types.
EXPLICITLY_INSTANTIATE_IMPLS(double);
EXPLICITLY_INSTANTIATE_IMPLS(AutoDiffXd);

}  // namespace multibody
}  // namespace drake
