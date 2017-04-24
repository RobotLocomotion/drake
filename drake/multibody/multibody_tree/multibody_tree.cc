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
  // By default allow to perform dynamic allocations even in Debug builds.
  // We can disable dynamic memory allocation in Debug builds to verify we do
  // not re-allocate memory unnecessarily after CreateDefaultContext().
#ifndef NDEBUG
  Eigen::internal::set_is_malloc_allowed(true);
#endif

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
std::unique_ptr<systems::Context<T>>
MultibodyTree<T>::CreateDefaultContext() const {
  if (!topology_is_valid()) {
    throw std::logic_error(
        "Attempting to create a Context for a  MultibodyTree with an invalid "
        "topology. MultibodyTree::Finalize() must be called before attempting "
        "to create a context.");
  }

  // TODO(amcastro-tri): pass the MultibodyTreeTopology instead of just the
  // number of bodies.
  auto context = std::make_unique<MultibodyTreeContext<T>>(get_num_bodies());
  SetDefaults(context.get());
  return context;
}

template <typename T>
const Isometry3<T>& MultibodyTree<T>::get_body_pose_in_world(
    const systems::Context<T>& context, BodyIndex body_index) const {
  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  // TODO(amcastro-tri): body_node_index will come from the
  // MultibodyTreeTopology as:
  //    body_node_index = topology_.bodies[body_index].body_node;
  BodyNodeIndex body_node_index = BodyNodeIndex((int)body_index);
  // TODO(amcastro-tri): Check cache validity.
  //                     Check topology validity.
  return mbt_context.get_position_kinematics().get_X_WB(body_node_index);
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;
template class MultibodyTree<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
