#include "drake/multibody/multibody_tree/frame.h"

#include <memory>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {

template <typename T>
MaterialFrame<T>::MaterialFrame(const Body<T>& body) :
    body_index_(body.get_index()) {}

template <typename T>
const BodyFrame<T>& BodyFrame<T>::Create(
    MultibodyTree<T>* tree, const Body<T>& body) {
  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating bodies by other means other than calling
  // Create().
  // However we can still create a unique_ptr as below where ownership is clear
  return *tree->AddMaterialFrame(
      std::unique_ptr<BodyFrame<T>>(new BodyFrame<T>(body)));
}

template <typename T>
BodyFrame<T>::BodyFrame(const Body<T>& body) : MaterialFrame<T>(body) {}

template <typename T>
RigidBodyFrame<T>& RigidBodyFrame<T>::Create(
    MultibodyTree<T>* tree, const RigidBody<T>& body,
    const Isometry3<T>& X_BM) {
  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating bodies by other means other than calling
  // Create().
  // However we can still create a unique_ptr as below where ownership is clear
  // and an exception would call the destructor.
  return *tree->AddMaterialFrame(
      std::unique_ptr<RigidBodyFrame<T>>(new RigidBodyFrame<T>(body, X_BM)));
}

template <typename T>
RigidBodyFrame<T>::RigidBodyFrame(
    const RigidBody<T>& body, const Isometry3<T>& X_BM) :
    MaterialFrame<T>(body), X_BM_(X_BM) {}

// Explicitly instantiates on the most common scalar types.
template class BodyFrame<double>;
template class BodyFrame<AutoDiffXd>;
template class RigidBodyFrame<double>;
template class RigidBodyFrame<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
