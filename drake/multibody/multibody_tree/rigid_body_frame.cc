#include "drake/multibody/multibody_tree/rigid_body_frame.h"

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
RigidBodyFrame<T>& RigidBodyFrame<T>::Create(
    MultibodyTree<T>* tree,
    const RigidBody<T>& body, const Isometry3<T>& X_BM) {
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
    PhysicalFrame<T>(body.get_index()), X_BM_(X_BM) {}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyFrame<double>;
template class RigidBodyFrame<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
