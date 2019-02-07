#include "drake/multibody/tree/body_node_impl.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace internal {

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
// These should be kept in sync with the list in default_scalars.h
EXPLICITLY_INSTANTIATE_IMPLS(double);
EXPLICITLY_INSTANTIATE_IMPLS(AutoDiffXd);
EXPLICITLY_INSTANTIATE_IMPLS(symbolic::Expression);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
