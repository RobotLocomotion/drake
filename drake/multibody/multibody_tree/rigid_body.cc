#include "drake/multibody/multibody_tree/rigid_body.h"

#include <memory>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
const RigidBody<T>& RigidBody<T>::Create(MultibodyTree<T>* tree) {
  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating bodies by other means other than calling
  // Create().
  // However we can still create a unique_ptr as below where ownership is clear
  // and an exception would call the destructor.
  return *tree->AddBody(std::unique_ptr<RigidBody<T>>(new RigidBody<T>()));
}

template <typename T>
RigidBody<T>::RigidBody() {}

// Explicitly instantiates on the most common scalar types.
template class RigidBody<double>;
template class RigidBody<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
