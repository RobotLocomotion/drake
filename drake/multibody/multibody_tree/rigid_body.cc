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
  RigidBody<T>* body = new RigidBody<T>();
  // tree takes ownership.
  BodyIndex body_id = tree->AddBody(std::unique_ptr<Body<T>>(body));
  body->set_parent_tree(tree);
  body->set_id(body_id);

  return *body;
}


template <typename T>
RigidBody<T>::RigidBody() {}

// Explicitly instantiates on the most common scalar types.
template class RigidBody<double>;
template class RigidBody<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
