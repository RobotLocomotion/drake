#include "drake/multibody/multibody_tree/multibody_tree.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

#include <stdexcept>
#include <utility>

namespace drake {
namespace multibody {

using std::make_unique;

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // TODO(amcastro-tri): Assign an infinite mass to the "world" body when rigid
  // body mass properties are implemented.
  RigidBody<T>::Create(this);
}

template <typename T>
BodyIndex MultibodyTree<T>::AddBody(std::unique_ptr<Body<T>> body) {
  if (body == nullptr) {
    throw std::logic_error("Input body is an invalid nullptr.");
  }

  // If the topology is valid it means that this MultibodyTree was already
  // compiled. Thus throw an exception to alert users.
  if (topology_is_valid_) {
    throw std::logic_error(
        "Attempting to add a body to an already compiled MultibodyTree is not "
        "allowed. See MultibodyTree::Compile() for details.");
  }
  // TODO(amcastro-tri): This id will be returned by the MultibodyTreeTopology
  // class in a future PR.
  BodyIndex id(static_cast<int>(bodies_.size()));
  bodies_.push_back(std::move(body));
  return id;
}

template <typename T>
void MultibodyTree<T>::Compile() {
  // TODO(amcastro-tri): This is a brief list of operations to be added in
  // subsequent PR's:
  //   - Compile non-T dependent topological information.
  //   - Compute degrees of freedom, array sizes and any other information to
  //     allocate a context and request the required cache entries.
  //   - Setup computational structures (BodyNode based).

  // Here, give bodies the chance to perform any compile-time setup.
  for (const auto& body : bodies_) {
    body->Compile();
  }

  // Validate topology.
  topology_is_valid_ = true;
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;
template class MultibodyTree<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
