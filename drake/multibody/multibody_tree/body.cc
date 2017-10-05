#include "drake/multibody/multibody_tree/body.h"

#include <memory>

#include "drake/common/autodiff.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> BodyFrame<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Body<ToScalar>& body_clone =
      tree_clone.get_body(this->get_body().get_index());
  // BodyFrame's constructor cannot be called from std::make_unique since it is
  // private and therefore we use "new".
  return std::unique_ptr<BodyFrame<ToScalar>>(
      new BodyFrame<ToScalar>(body_clone));
}

template <typename T>
std::unique_ptr<Frame<double>> BodyFrame<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> BodyFrame<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class BodyFrame<double>;
template class BodyFrame<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
