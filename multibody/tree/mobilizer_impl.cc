#include "drake/multibody/tree/mobilizer_impl.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T, int nq, int nv>
MobilizerImpl<T, nq, nv>::~MobilizerImpl() = default;

// Macro used to explicitly instantiate implementations on all sizes needed.
#define EXPLICITLY_INSTANTIATE_IMPLS(T)  \
  template class MobilizerImpl<T, 0, 0>; \
  template class MobilizerImpl<T, 1, 1>; \
  template class MobilizerImpl<T, 2, 2>; \
  template class MobilizerImpl<T, 3, 3>; \
  template class MobilizerImpl<T, 6, 6>; \
  template class MobilizerImpl<T, 7, 6>;

// Explicitly instantiates on the supported scalar types.
// These should be kept in sync with the list in default_scalars.h.
EXPLICITLY_INSTANTIATE_IMPLS(double);
EXPLICITLY_INSTANTIATE_IMPLS(AutoDiffXd);
EXPLICITLY_INSTANTIATE_IMPLS(symbolic::Expression);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
