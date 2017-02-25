#include "drake/multibody/multibody_tree/multibody_tree.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

#include <queue>

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace multibody {

using Eigen::Vector3d;

using std::make_unique;

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // The "world" body has infinite mass.
  RigidBody<T>::Create(this);
}

template <typename T>
BodyIndex MultibodyTree<T>::AddBody(std::unique_ptr<Body<T>> body) {
  DRAKE_DEMAND(body != nullptr);
  // TODO(amcastro-tri): This id will be returned by the MultibodyTreeTopology
  // class in a future PR.
  BodyIndex id(static_cast<int>(bodies_.size()));
  bodies_.push_back(std::move(body));
  return id;
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;

}  // namespace multibody
}  // namespace drake
