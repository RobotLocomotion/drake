#include "drake/multibody/multibody_tree/mobilizer_impl.h"

#include <memory>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/body_node_impl.h"

namespace drake {
namespace multibody {

template <typename T, int  nq, int nv>
std::unique_ptr<internal::BodyNode<T>> MobilizerImpl<T, nq, nv>::CreateBodyNode(
    const Body<T>& body, const Mobilizer<T>* mobilizer) const {
  return std::make_unique<internal::BodyNodeImpl<T, nq, nv>>(body, mobilizer);
}

// Macro used to explicitly instantiate implementations on all sizes needed.
#define EXPLICITLY_INSTANTIATE_IMPLS(T) \
template class MobilizerImpl<T, 1, 1>;

// Explicitly instantiates on the most common scalar types.
EXPLICITLY_INSTANTIATE_IMPLS(double);
EXPLICITLY_INSTANTIATE_IMPLS(AutoDiffXd);

}  // namespace multibody
}  // namespace drake
