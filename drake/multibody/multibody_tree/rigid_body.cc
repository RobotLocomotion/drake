#include "drake/multibody/multibody_tree/rigid_body.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>

namespace drake {
namespace multibody {

template <typename T>
const RigidBody<T>& RigidBody<T>::Create(MultibodyTree<T>* tree) {
  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating bodies by other means other than calling
  // Create().
  // However we can still create a unique_ptr as below where ownership is clear
  // and an exception would call the destructor.
  BodyIndex body_index =
      tree->AddBody(std::unique_ptr<RigidBody<T>>(new RigidBody<T>()));
  RigidBody<T>& body =
      dynamic_cast<RigidBody<T>&>(tree->get_mutable_body(body_index));
  body.set_parent_tree(tree);
  body.set_index(body_index);
  return body;
}


template <typename T>
RigidBody<T>::RigidBody() {}

// Explicitly instantiates on the most common scalar types.
template class RigidBody<double>;
template class RigidBody<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
