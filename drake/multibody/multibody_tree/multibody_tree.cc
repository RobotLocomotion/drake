#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // TODO(amcastro-tri): Assign an infinite mass to the "world" body when rigid
  // body mass properties are implemented.
  RigidBody<T>::Create(this);
}

template <typename T>
void MultibodyTree<T>::Compile() {
  // If the topology is valid it means that this MultibodyTree was already
  // compiled. Since this is an expensive operation, throw an exception to alert
  // users.
  if (topology_is_valid()) {
    throw std::logic_error(
        "Attempting to call MultibodyTree::Compile() on an already compiled "
            "MultibodyTree.");
  }

  // TODO(amcastro-tri): This is a brief list of operations to be added in
  // subsequent PR's:
  //   - Compile non-T dependent topological information.
  //   - Compute degrees of freedom, array sizes and any other information to
  //     allocate a context and request the required cache entries.
  //   - Setup computational structures (BodyNode based).

  // Here, give bodies the chance to perform any compile-time setup.
  for (const auto& body : owned_bodies_) {
    body->Compile();
  }

  validate_topology();
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;
template class MultibodyTree<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
