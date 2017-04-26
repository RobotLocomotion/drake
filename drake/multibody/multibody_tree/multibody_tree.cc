#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
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
void MultibodyTree<T>::Finalize() {
  // If the topology is valid it means that this MultibodyTree was already
  // finalized. Re-compilation is not allowed.
  if (topology_is_valid()) {
    throw std::logic_error(
        "Attempting to call MultibodyTree::Finalize() on an already finalized "
        "MultibodyTree.");
  }

  CompileTopology();

  // TODO(amcastro-tri): This is a brief list of operations to be added in
  // subsequent PR's:
  //   - Finalize non-T dependent topological information.
  //   - Compute degrees of freedom, array sizes and any other information to
  //     allocate a context and request the required cache entries.
  //   - Setup computational structures (BodyNode based).

  // Give bodies the chance to perform any finalize-time setup.
  for (const auto& body : owned_bodies_) {
    body->Finalize(*this);
  }

  // Give frames the chance to perform any finalize-time setup.
  for (const auto& frame : owned_frames_) {
    frame->Finalize(*this);
  }

  set_valid_topology();
}

template <typename T>
void MultibodyTree<T>::CompileTopology() {
  // TODO(amcastro-tri): BodyNode objects will be actual tree nodes ordered by a
  // breadth first traversal. Therefore body node indexes will be appropriately
  // initialized in a following PR that introduces Mobilizer objects connecting
  // frames in a meaningful way. The code below is now introduced so that bodies
  // can retrieve Context entries.
  BodyNodeIndex body_node(0);
  for (auto& body_topology : topology_.bodies) {
    body_topology.body_node = body_node++;
  }
}

template <typename T>
std::unique_ptr<systems::Context<T>>
MultibodyTree<T>::CreateDefaultContext() const {
  if (!topology_is_valid()) {
    throw std::logic_error(
        "Attempting to create a Context for a  MultibodyTree with an invalid "
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
