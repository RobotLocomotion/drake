#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node_welded.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

using internal::BodyNode;
using internal::BodyNodeWelded;

namespace internal {
template <typename T>
class JointModelBuilder {
 public:
  JointModelBuilder() = delete;
  static void BuildJointModel(Joint<T>* joint, MultibodyTree<T>* tree) {
    std::unique_ptr<JointBluePrint> blue_print = joint->MakeModelBlueprint();
    auto model = std::make_unique<JointModel>(*blue_print);
    DRAKE_DEMAND(static_cast<int>(model->mobilizers_.size()) != 0);
    for (auto& mobilizer : blue_print->mobilizers_) {
      tree->AddMobilizer(std::move(mobilizer));
    }
    // TODO(amcastro-tri): add force elements, bodies, constraints, etc.
    joint->OwnModel(std::move(model));
  }
 private:
  typedef typename Joint<T>::BluePrint JointBluePrint;
  typedef typename Joint<T>::JointModel JointModel;
};
}  // namespace internal

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // Adds a "world" body to MultibodyTree having a NaN SpatialInertia.
  world_body_ = &AddBody<RigidBody>(SpatialInertia<double>());
}

template <typename T>
void MultibodyTree<T>::FinalizeTopology() {
  // If the topology is valid it means that this MultibodyTree was already
  // finalized. Re-compilation is not allowed.
  if (topology_is_valid()) {
    throw std::logic_error(
        "Attempting to call MultibodyTree::FinalizeTopology() on a tree with"
        " an already finalized topology.");
  }

  // Before performing any setup that depends on the scalar type <T>, compile
  // all the type-T independent topological information.
  topology_.Finalize();
}

template <typename T>
void MultibodyTree<T>::FinalizeInternals() {
  if (!topology_is_valid()) {
    throw std::logic_error(
        "MultibodyTree::FinalizeTopology() must be called before "
        "MultibodyTree::FinalizeInternals().");
  }

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
  body_node_levels_.resize(topology_.get_tree_height());
  for (BodyNodeIndex body_node_index(1);
       body_node_index < topology_.get_num_body_nodes(); ++body_node_index) {
    const BodyNodeTopology& node_topology =
        topology_.get_body_node(body_node_index);
    body_node_levels_[node_topology.level].push_back(body_node_index);
  }

  // Creates BodyNode's:
  // This recursion order ensures that a BodyNode's parent is created before the
  // node itself, since BodyNode objects are in Breadth First Traversal order.
  for (BodyNodeIndex body_node_index(0);
       body_node_index < topology_.get_num_body_nodes(); ++body_node_index) {
    CreateBodyNode(body_node_index);
  }
}

template <typename T>
void MultibodyTree<T>::Finalize() {
  // Create Joint objects's implementation.
  // This must happen before FinalizeTopology() so that the model, in terms of
  // basic MBT elements such as frames, bodies, mobilizers, force elements and
  // constraints, is complete.
  for (auto& joint : owned_joints_) {
    internal::JointModelBuilder<T>::BuildJointModel(joint.get(), this);
  }
  FinalizeTopology();
  FinalizeInternals();
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

    BodyNode<T>* parent_node =
        body_nodes_[node_topology.parent_body_node].get();

    // Only the mobilizer knows how to create a body node with compile-time
    // fixed sizes.
    body_node = mobilizer->CreateBodyNode(parent_node, body, mobilizer);
    parent_node->add_child_node(body_node.get());
  }
  body_node->set_parent_tree(this, body_node_index);
  body_node->SetTopology(topology_);

  body_nodes_.push_back(std::move(body_node));
}

template <typename T>
std::unique_ptr<systems::LeafContext<T>>
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
void MultibodyTree<T>::SetDefaults(systems::Context<T>* context) const {
  for (const auto& mobilizer : owned_mobilizers_) {
    mobilizer->set_zero_configuration(context);
  }
}

template <typename T>
void MultibodyTree<T>::CalcPositionKinematicsCache(
    const systems::Context<T>& context,
    PositionKinematicsCache<T>* pc) const {
  DRAKE_DEMAND(pc != nullptr);
  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  // TODO(amcastro-tri): Loop over bodies to update their position dependent
  // kinematics. This gives the chance to flexible bodies to update the pose
  // X_BQ(qb_B) of each frame Q that is attached to the body.
  // Notice this loop can be performed in any order and each X_BQ(qf_B) is
  // independent of all others. This could even be performed in parallel.

  // With the kinematics information across mobilizer's and the kinematics
  // information for each body, we are now in position to perform a base-to-tip
  // recursion to update world positions and parent to child body transforms.
  // This skips the world, level = 0.
  for (int level = 1; level < get_tree_height(); ++level) {
    for (BodyNodeIndex body_node_index : body_node_levels_[level]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == level);
      DRAKE_ASSERT(node.get_index() == body_node_index);

      // Update per-node kinematics.
      node.CalcPositionKinematicsCache_BaseToTip(mbt_context, pc);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcVelocityKinematicsCache(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    VelocityKinematicsCache<T>* vc) const {
  DRAKE_DEMAND(vc != nullptr);
  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  // TODO(amcastro-tri): Loop over bodies to compute velocity kinematics updates
  // corresponding to flexible bodies.

  // Performs a base-to-tip recursion computing body velocities.
  // This skips the world, depth = 0.
  for (int depth = 1; depth < get_tree_height(); ++depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.get_index() == body_node_index);

      // Update per-node kinematics.
      node.CalcVelocityKinematicsCache_BaseToTip(mbt_context, pc, vc);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    const VectorX<T>& known_vdot,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  DRAKE_DEMAND(A_WB_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(A_WB_array->size()) == get_num_bodies());

  DRAKE_DEMAND(known_vdot.size() == topology_.get_num_velocities());

  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  // TODO(amcastro-tri): Loop over bodies to compute acceleration kinematics
  // updates corresponding to flexible bodies.

  // The world's spatial acceleration is always zero.
  A_WB_array->at(world_index()) = SpatialAcceleration<T>::Zero();

  // Performs a base-to-tip recursion computing body accelerations.
  // This skips the world, depth = 0.
  for (int depth = 1; depth < get_tree_height(); ++depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.get_index() == body_node_index);

      // Update per-node kinematics.
      node.CalcSpatialAcceleration_BaseToTip(
          mbt_context, pc, vc, known_vdot, A_WB_array);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcAccelerationKinematicsCache(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    const VectorX<T>& known_vdot,
    AccelerationKinematicsCache<T>* ac) const {
  DRAKE_DEMAND(ac != nullptr);
  DRAKE_DEMAND(known_vdot.size() == topology_.get_num_velocities());

  // TODO(amcastro-tri): Loop over bodies to compute velocity kinematics updates
  // corresponding to flexible bodies.

  std::vector<SpatialAcceleration<T>>& A_WB_array = ac->get_mutable_A_WB_pool();

  CalcSpatialAccelerationsFromVdot(context, pc, vc, known_vdot, &A_WB_array);
}

template <typename T>
void MultibodyTree<T>::CalcInverseDynamics(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    const VectorX<T>& known_vdot,
    const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
    const Eigen::Ref<const VectorX<T>>& tau_applied_array,
    std::vector<SpatialAcceleration<T>>* A_WB_array,
    std::vector<SpatialForce<T>>* F_BMo_W_array,
    EigenPtr<VectorX<T>> tau_array) const {
  DRAKE_DEMAND(known_vdot.size() == get_num_velocities());
  const int Fapplied_size = static_cast<int>(Fapplied_Bo_W_array.size());
  DRAKE_DEMAND(Fapplied_size == get_num_bodies() || Fapplied_size == 0);
  const int tau_applied_size = tau_applied_array.size();
  DRAKE_DEMAND(
      tau_applied_size == get_num_velocities() || tau_applied_size == 0);

  DRAKE_DEMAND(A_WB_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(A_WB_array->size()) == get_num_bodies());

  DRAKE_DEMAND(F_BMo_W_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(F_BMo_W_array->size()) == get_num_bodies());

  DRAKE_DEMAND(tau_array != nullptr);
  DRAKE_DEMAND(tau_array->size() == get_num_velocities());

  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  // Compute body spatial accelerations given the generalized accelerations are
  // known.
  CalcSpatialAccelerationsFromVdot(context, pc, vc, known_vdot, A_WB_array);

  // Vector of generalized forces per mobilizer.
  // It has zero size if no forces are applied.
  VectorUpTo6<T> tau_applied_mobilizer(0);

  // Spatial force applied on B at Bo.
  // It is left initialized to zero if no forces are applied.
  SpatialForce<T> Fapplied_Bo_W = SpatialForce<T>::Zero();

  // Performs a tip-to-base recursion computing the total spatial force F_BMo_W
  // acting on body B, about point Mo, expressed in the world frame W.
  // This includes the world (depth = 0) so that F_BMo_W_array[world_index()]
  // contains the total force of the bodies connected to the world by a
  // mobilizer.
  for (int depth = get_tree_height() - 1; depth >= 0; --depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.get_index() == body_node_index);

      // Make a copy to the total applied forces since the call to
      // CalcInverseDynamics_TipToBase() below could overwrite the entry for the
      // current body node if the input applied forces arrays are the same
      // in-memory object as the output arrays.
      // This allows users to specify the same input and output arrays if
      // desired to minimize memory footprint.
      // Leave them initialized to zero if no applied forces were provided.
      if (tau_applied_size != 0) {
        tau_applied_mobilizer =
            node.get_mobilizer().get_generalized_forces_from_array(
                tau_applied_array);
      }
      if (Fapplied_size != 0) {
        Fapplied_Bo_W = Fapplied_Bo_W_array[body_node_index];
      }

      // Compute F_BMo_W for the body associated with this node and project it
      // onto the space of generalized forces for the associated mobilizer.
      node.CalcInverseDynamics_TipToBase(
          mbt_context, pc, vc, *A_WB_array,
          Fapplied_Bo_W, tau_applied_mobilizer,
          F_BMo_W_array, tau_array);
    }
  }
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;
template class MultibodyTree<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
