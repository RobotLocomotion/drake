#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/body_node_welded.h"
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

  // Create a list of body nodes organized by levels.
  body_node_levels_.reserve(topology_.get_num_levels());
  for (BodyNodeIndex body_node_index(1);
       body_node_index < topology_.get_num_body_nodes(); ++body_node_index) {
    const BodyNodeTopology& node_topology =
        topology_.get_body_node(body_node_index);
    body_node_levels_[node_topology.level].push_back(body_node_index);
  }

  // Creates BodyNode's:
  for (BodyNodeIndex body_node_index(1);
       body_node_index < topology_.get_num_body_nodes(); ++body_node_index) {
    CreateBodyNode(body_node_index);
  }
}

template <typename T>
void MultibodyTree<T>::CreateBodyNode(BodyNodeIndex body_node_index) {
  const BodyNodeTopology& node_topology =
      topology_.get_body_node(body_node_index);
  const BodyIndex body_index = node_topology.body;

  const Body<T>* body = owned_bodies_[node_topology.body].get();

  std::unique_ptr<BodyNode<T>> body_node;
  if (body_index == world_index()) {
    body_node = std::make_unique<BodyNodeWelded<T>>(&get_world_body());
  } else {
    // The mobilizer should be valid if not at the root (the world).
    DRAKE_ASSERT(node_topology.mobilizer.is_valid());
    const Mobilizer<T>* mobilizer =
        owned_mobilizers_[node_topology.mobilizer].get();

    // Only the mobilizer knows how to create a body node with compile-time
    // fixed sizes.
    body_node = mobilizer->CreateBodyNode(body, mobilizer);
  }
  body_node->set_parent_tree(this, body_node_index);
  body_node->SetTopology(topology_);

  body_nodes_.push_back(std::move(body_node));
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

template <typename T>
void MultibodyTree<T>::CalcPositionKinematicsCache(
    const MultibodyTreeContext<T>& context,
    PositionKinematicsCache<T>* pc) const {
  // Loop over all mobilizers to update their position dependent kinematics.
  // This updates the kinematics quantities only dependent on the rigid degrees
  // of freedom qr. These are: X_FM(qr), H_FM(qr), HdotTimesV(qr), N(qr).
  // Notice this loop can be performed in any order, even in parallel.
  for (const auto& mobilizer: owned_mobilizers_)
    mobilizer->CalcPositionKinematicsCache(context, pc);

  // With the kinematics information across mobilizer's and the kinematics
  // information for each body, we are now in position to perform a base-to-tip
  // recursion to update world positions and parent to child body transforms.
  // This skips the world, level = 0.
  for (int level = 1; level < get_num_levels(); ++level) {
    for (BodyNodeIndex body_node_index: body_node_levels_[level]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];
      
      DRAKE_ASSERT(node.get_topology().level == level);
      DRAKE_ASSERT(node.get_index() == body_node_index);
      
      // Update per-node kinematics.
      node.UpdatePositionKinematicsCache_BaseToTip(context);
    }
  }
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;
template class MultibodyTree<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
