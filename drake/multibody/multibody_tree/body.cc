#include "drake/multibody/multibody_tree/body.h"

#include <memory>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
std::unique_ptr<Frame<T>> BodyFrame<T>::Clone(
    const MultibodyTree<T>& tree_clone) const {
  const Body<T>& body_clone =
      tree_clone.get_body(this->get_body().get_index());
  // BodyFrame's constructor cannot be called from std::make_unique since it is
  // private and therefore we use "new".
  return std::unique_ptr<BodyFrame<T>>(new BodyFrame<T>(body_clone));
}

// Explicitly instantiates on the most common scalar types.
template class BodyFrame<double>;
template class BodyFrame<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
