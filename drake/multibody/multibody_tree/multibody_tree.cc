#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // Adds a "world" body to MultibodyTree having a NaN SpatialInertia.
  AddBody<RigidBody>(SpatialInertia<double>());
}

template <typename T>
void MultibodyTree<T>::Compile() {
  // If the topology is valid it means that this MultibodyTree was already
  // compiled. Re-compilation is not allowed.
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

  // Give bodies the chance to perform any compile-time setup.
  for (const auto& body : owned_bodies_) {
    body->Compile(*this);
  }

  // Give frames the chance to perform any compile-time setup.
  for (const auto& frame : owned_frames_) {
    frame->Compile(*this);
  }

  validate_topology();
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;
template class MultibodyTree<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
