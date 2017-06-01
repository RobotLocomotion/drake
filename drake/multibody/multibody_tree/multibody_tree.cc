#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
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
void MultibodyTree<T>::Finalize() {
  // If the topology is valid it means that this MultibodyTree was already
  // finalized. Re-compilation is not allowed.
  if (topology_is_valid()) {
    throw std::logic_error(
        "Attempting to call MultibodyTree::Finalize() on an already finalized "
        "MultibodyTree.");
  }

  // Before performing any setup that depends on the scalar type <T>, compile
  // all the type-T independent topological information.
  topology_.Finalize();

  // TODO(amcastro-tri): This is a brief list of operations to be added in
  // subsequent PR's:
  //   - Compute degrees of freedom, array sizes and any other information to
  //     allocate a context and request the required cache entries.
  //   - Setup computational structures (BodyNode based).

  // Give bodies the chance to perform any finalize-time setup.
  for (const auto& body : owned_bodies_) {
    body->SetTopology(topology_);
  }

  // Give frames the chance to perform any finalize-time setup.
  for (const auto& frame : owned_frames_) {
    frame->SetTopology(topology_);
  }

  // Give mobilizers the chance to perform any finalize-time setup.
  for (const auto& mobilizer : owned_mobilizers_) {
    mobilizer->SetTopology(topology_);
  }
}

template <typename T>
std::unique_ptr<systems::Context<T>>
MultibodyTree<T>::CreateDefaultContext() const {
  if (!topology_is_valid()) {
    throw std::logic_error(
        "Attempting to create a Context for a MultibodyTree with an invalid "
        "topology. MultibodyTree::Finalize() must be called before attempting "
        "to create a context.");
  }
  auto context = std::make_unique<MultibodyTreeContext<T>>(topology_);
  SetDefaults(context.get());
  return std::move(context);
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;
template class MultibodyTree<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
