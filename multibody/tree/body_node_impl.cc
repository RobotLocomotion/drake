#include "drake/multibody/tree/body_node_impl.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T, int num_positions, int num_velocities>
BodyNodeImpl<T, num_positions, num_velocities>::~BodyNodeImpl() = default;

// Macro used to explicitly instantiate implementations on all sizes needed.
#define EXPLICITLY_INSTANTIATE_IMPLS(T) \
template class BodyNodeImpl<T, 0, 0>; \
template class BodyNodeImpl<T, 1, 1>; \
template class BodyNodeImpl<T, 2, 2>; \
template class BodyNodeImpl<T, 3, 3>; \
template class BodyNodeImpl<T, 6, 6>; \
template class BodyNodeImpl<T, 7, 6>;

// Explicitly instantiates on the most common scalar types.
// These should be kept in sync with the list in default_scalars.h.
EXPLICITLY_INSTANTIATE_IMPLS(double);
EXPLICITLY_INSTANTIATE_IMPLS(AutoDiffXd);
EXPLICITLY_INSTANTIATE_IMPLS(symbolic::Expression);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
