#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node_welded.h"
#include "drake/multibody/multibody_tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

using internal::BodyNode;
using internal::BodyNodeWelded;

#define MBT_THROW_IF_NOT_FINALIZED ThrowIfNotFinalized(__FUNCTION__);

namespace internal {
template <typename T>
class JointImplementationBuilder {
 public:
  JointImplementationBuilder() = delete;
  static void Build(Joint<T>* joint, MultibodyTree<T>* tree) {
    std::unique_ptr<JointBluePrint> blue_print =
        joint->MakeImplementationBlueprint();
    auto implementation = std::make_unique<JointImplementation>(*blue_print);
    DRAKE_DEMAND(implementation->get_num_mobilizers() != 0);
    for (auto& mobilizer : blue_print->mobilizers_) {
      tree->AddMobilizer(std::move(mobilizer));
    }
    // TODO(amcastro-tri): add force elements, bodies, constraints, etc.
    joint->OwnImplementation(std::move(implementation));
  }
 private:
  typedef typename Joint<T>::BluePrint JointBluePrint;
  typedef typename Joint<T>::JointImplementation JointImplementation;
};
}  // namespace internal

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // Adds a "world" body to MultibodyTree having a NaN SpatialInertia.
  world_body_ = &AddBody<RigidBody>(SpatialInertia<double>());
}

template <typename T>
void MultibodyTree<T>:: AddQuaternionFreeMobilizerToAllBodiesWithNoMobilizer() {
  // Do not call on not finalized tree, thought it should be a no-op in that
  // case?
  DRAKE_DEMAND(!topology_is_valid());
  // Skip the world.
  for (BodyIndex body_index(1); body_index < get_num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    const BodyTopology& body_topology =
        get_topology().get_body(body.get_index());
    if (!body_topology.inboard_mobilizer.is_valid()) {
      this->template AddMobilizer<QuaternionFloatingMobilizer>(
          get_world_body().get_body_frame(), body.get_body_frame());
    }
  }
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
MultibodyTree<T>::GetFreeBodyMobilizerOrThrow(
    const Body<T>& body) const {
  DRAKE_DEMAND(body.get_index() != world_index());
  const BodyTopology& body_topology = get_topology().get_body(body.get_index());
  const QuaternionFloatingMobilizer<T>* mobilizer =
      dynamic_cast<const QuaternionFloatingMobilizer<T>*>(
          &get_mobilizer(body_topology.inboard_mobilizer));
  // TODO: just do an if and throw with nice message.
  DRAKE_THROW_UNLESS(mobilizer != nullptr);
  return *mobilizer;
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

  // TODO(amcastro-tri): Remove when MultibodyCachingEvaluatorInterface lands.
  AllocateFakeCacheEntries();
}

template <typename T>
void MultibodyTree<T>::Finalize() {
  // Create Joint objects's implementation. Joints are implemented using a
  // combination of MultibodyTree's building blocks such as Body, Mobilizer,
  // ForceElement and Constraint. For a same physical Joint, several
  // implementations could be created (for instance, a Constraint instead of a
  // Mobilizer). The decision on what implementation to create is performed by
  // MultibodyTree at Finalize() time. Then, JointImplementationBuilder below
  // can request MultibodyTree for these choices when building the Joint
  // implementation. Since a Joint's implementation is built upon
  // MultibodyTree's building blocks, notice that creating a Joint's
  // implementation will therefore change the tree topology. Since topology
  // changes are NOT allowed after Finalize(), joint implementations MUST be
  // assembled BEFORE the tree's topology is finalized.
  for (auto& joint : owned_joints_) {
    internal::JointImplementationBuilder<T>::Build(joint.get(), this);
  }
  // It is VERY important to add quaternions if needed only AFTER joints had a
  // chance to get implemented with mobilizers. Therefore, do not change this
  // order!
  AddQuaternionFreeMobilizerToAllBodiesWithNoMobilizer();
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
  SetDefaultContext(context.get());
  return std::move(context);
}

template <typename T>
void MultibodyTree<T>::SetDefaultContext(systems::Context<T> *context) const {
  for (const auto& mobilizer : owned_mobilizers_) {
    mobilizer->set_zero_configuration(context);
  }
}

template <typename T>
void MultibodyTree<T>::SetDefaultState(
    const systems::Context<T>& context, systems::State<T>* state) const {
  for (const auto& mobilizer : owned_mobilizers_) {
    mobilizer->set_zero_state(context, state);
  }
}

template <typename T>
void MultibodyTree<T>::CalcAllBodyPosesInWorld(
    const systems::Context<T>& context,
    std::vector<Isometry3<T>>* X_WB) const {
  DRAKE_THROW_UNLESS(X_WB != nullptr);
  if (static_cast<int>(X_WB->size()) != get_num_bodies()) {
    X_WB->resize(get_num_bodies(), Isometry3<T>::Identity());
  }
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  for (BodyIndex body_index(0); body_index < get_num_bodies(); ++body_index) {
    const BodyNodeIndex node_index = get_body(body_index).get_node_index();
    X_WB->at(body_index) = pc.get_X_WB(node_index);
  }
}

template <typename T>
void MultibodyTree<T>::CalcAllBodySpatialVelocitiesInWorld(
    const systems::Context<T>& context,
    std::vector<SpatialVelocity<T>>* V_WB) const {
  DRAKE_THROW_UNLESS(V_WB != nullptr);
  if (static_cast<int>(V_WB->size()) != get_num_bodies()) {
    V_WB->resize(get_num_bodies(), SpatialVelocity<T>::Zero());
  }
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  for (BodyIndex body_index(0); body_index < get_num_bodies(); ++body_index) {
    const BodyNodeIndex node_index = get_body(body_index).get_node_index();
    V_WB->at(body_index) = vc.get_V_WB(node_index);
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

  // TODO(amcastro-tri): Eval H_PB_W from the cache.
  std::vector<Vector6<T>> H_PB_W_cache(get_num_velocities());
  CalcAcrossNodeGeometricJacobianExpressedInWorld(context, pc, &H_PB_W_cache);

  // Performs a base-to-tip recursion computing body velocities.
  // This skips the world, depth = 0.
  for (int depth = 1; depth < get_tree_height(); ++depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.get_index() == body_node_index);

      // Jacobian matrix for this node. H_PB_W ∈ ℝ⁶ˣⁿᵐ with nm ∈ [0; 6] the
      // number of mobilities for this node. Therefore, the return is a
      // MatrixUpTo6 since the number of columns generally changes with the
      // node.
      // It is returned as an Eigen::Map to the memory allocated in the
      // std::vector H_PB_W_cache so that we can work with H_PB_W as with any
      // other Eigen matrix object.
      Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
          node.GetJacobianFromArray(H_PB_W_cache);

      // Update per-node kinematics.
      node.CalcVelocityKinematicsCache_BaseToTip(mbt_context, pc, H_PB_W, vc);
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

template <typename T>
void MultibodyTree<T>::CalcForceElementsContribution(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(*this));

  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  forces->SetZero();
  // Add contributions from force elements.
  for (const auto& force_element : owned_force_elements_) {
    force_element->CalcAndAddForceContribution(mbt_context, pc, vc, forces);
  }
}

template <typename T>
void MultibodyTree<T>::MapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_DEMAND(qdot.size() == get_num_positions());
  DRAKE_DEMAND(v != nullptr);
  DRAKE_DEMAND(v->size() == get_num_velocities());
  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);
  VectorUpTo6<T> v_mobilizer;
  for (const auto& mobilizer : owned_mobilizers_) {
    const auto qdot_mobilizer = mobilizer->get_positions_from_array(qdot);
    v_mobilizer.resize(mobilizer->get_num_velocities());
    mobilizer->MapQDotToVelocity(mbt_context, qdot_mobilizer, &v_mobilizer);
    mobilizer->get_mutable_velocities_from_array(v) = v_mobilizer;
  }
}

template <typename T>
void MultibodyTree<T>::MapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_DEMAND(v.size() == get_num_velocities());
  DRAKE_DEMAND(qdot != nullptr);
  DRAKE_DEMAND(qdot->size() == get_num_positions());
  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);
  const int kMaxQdot = 7;
  Eigen::Matrix<T, kMaxQdot, 1> qdot_mobilizer;
  for (const auto& mobilizer : owned_mobilizers_) {
    const auto v_mobilizer = mobilizer->get_velocities_from_array(v);
    DRAKE_DEMAND(mobilizer->get_num_positions() <= kMaxQdot);
    qdot_mobilizer.resize(mobilizer->get_num_positions());
    mobilizer->MapVelocityToQDot(mbt_context, v_mobilizer, &qdot_mobilizer);
    mobilizer->get_mutable_positions_from_array(qdot) = qdot_mobilizer;
  }
}

template <typename T>
void MultibodyTree<T>::CalcMassMatrixViaInverseDynamics(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> H) const {
  DRAKE_DEMAND(H != nullptr);
  DRAKE_DEMAND(H->rows() == get_num_velocities());
  DRAKE_DEMAND(H->cols() == get_num_velocities());
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  DoCalcMassMatrixViaInverseDynamics(context, pc, H);
}

template <typename T>
void MultibodyTree<T>::DoCalcMassMatrixViaInverseDynamics(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    EigenPtr<MatrixX<T>> H) const {
  // TODO(amcastro-tri): Consider passing a boolean flag to tell
  // CalcInverseDynamics() to ignore velocity dependent terms.
  VelocityKinematicsCache<T> vc(get_topology());
  vc.InitializeToZero();

  // Compute one column of the mass matrix via inverse dynamics at a time.
  const int nv = get_num_velocities();
  VectorX<T> vdot(nv);
  VectorX<T> tau(nv);
  // Auxiliary arrays used by inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB_array(get_num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(get_num_bodies());

  for (int j = 0; j < nv; ++j) {
    vdot = VectorX<T>::Unit(nv, j);
    tau.setZero();
    CalcInverseDynamics(context, pc, vc, vdot, {}, VectorX<T>(),
                        &A_WB_array, &F_BMo_W_array, &tau);
    H->col(j) = tau;
  }
}

template <typename T>
void MultibodyTree<T>::CalcBiasTerm(
    const systems::Context<T>& context, EigenPtr<VectorX<T>> Cv) const {
  DRAKE_DEMAND(Cv != nullptr);
  DRAKE_DEMAND(Cv->rows() == get_num_velocities());
  DRAKE_DEMAND(Cv->cols() == 1);
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  DoCalcBiasTerm(context, pc, vc, Cv);
}

template <typename T>
void MultibodyTree<T>::DoCalcBiasTerm(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    EigenPtr<VectorX<T>> Cv) const {
  const int nv = get_num_velocities();
  const VectorX<T> vdot = VectorX<T>::Zero(nv);

  // Auxiliary arrays used by inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB_array(get_num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(get_num_bodies());

  // TODO(amcastro-tri): provide specific API for when vdot = 0.
  CalcInverseDynamics(context, pc, vc, vdot, {}, VectorX<T>(),
                      &A_WB_array, &F_BMo_W_array, Cv);
}

template <typename T>
Isometry3<T> MultibodyTree<T>::CalcRelativeTransform(
    const systems::Context<T>& context,
    const Frame<T>& frame_A, const Frame<T>& frame_B) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const Isometry3<T>& X_WA =
      pc.get_X_WB(frame_A.get_body().get_node_index());
  const Isometry3<T>& X_WB =
      pc.get_X_WB(frame_B.get_body().get_node_index());
  return X_WA.inverse() * X_WB;
}

template <typename T>
void MultibodyTree<T>::CalcPointsPositions(
    const systems::Context<T>& context,
    const Frame<T>& frame_B,
    const Eigen::Ref<const MatrixX<T>>& p_BQi,
    const Frame<T>& frame_A,
    EigenPtr<MatrixX<T>> p_AQi) const {
  DRAKE_THROW_UNLESS(p_BQi.rows() == 3);
  DRAKE_THROW_UNLESS(p_AQi != nullptr);
  DRAKE_THROW_UNLESS(p_AQi->rows() == 3);
  DRAKE_THROW_UNLESS(p_AQi->cols() == p_BQi.cols());
  const Isometry3<T> X_AB =
      CalcRelativeTransform(context, frame_A, frame_B);
  // We demanded above that these matrices have three rows. Therefore we tell
  // Eigen so.
  p_AQi->template topRows<3>() = X_AB * p_BQi.template topRows<3>();
}

template <typename T>
const Isometry3<T>& MultibodyTree<T>::EvalBodyPoseInWorld(
    const systems::Context<T>& context,
    const Body<T>& body_B) const {
  MBT_THROW_IF_NOT_FINALIZED
  body_B.HasThisParentTreeOrThrow(this);
  return EvalPositionKinematics(context).get_X_WB(body_B.get_node_index());
}

template <typename T>
const SpatialVelocity<T>& MultibodyTree<T>::EvalBodySpatialVelocityInWorld(
    const systems::Context<T>& context,
    const Body<T>& body_B) const {
  MBT_THROW_IF_NOT_FINALIZED
  body_B.HasThisParentTreeOrThrow(this);
  return EvalVelocityKinematics(context).get_V_WB(body_B.get_node_index());
}

template <typename T>
void MultibodyTree<T>::CalcAcrossNodeGeometricJacobianExpressedInWorld(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    std::vector<Vector6<T>>* H_PB_W_cache) const {
  DRAKE_DEMAND(H_PB_W_cache != nullptr);
  DRAKE_DEMAND(static_cast<int>(H_PB_W_cache->size()) == get_num_velocities());

  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  for (BodyNodeIndex node_index(1);
       node_index < get_num_bodies(); ++node_index) {
    const BodyNode<T>& node = *body_nodes_[node_index];

    // Jacobian matrix for this node. H_PB_W ∈ ℝ⁶ˣⁿᵐ with nm ∈ [0; 6] the number
    // of mobilities for this node. Therefore, the return is a MatrixUpTo6 since
    // the number of columns generally changes with the node.
    // It is returned as an Eigen::Map to the memory allocated in the
    // std::vector H_PB_W_cache so that we can work with H_PB_W as with any
    // other Eigen matrix object.
    Eigen::Map<MatrixUpTo6<T>> H_PB_W =
        node.GetMutableJacobianFromArray(H_PB_W_cache);

    node.CalcAcrossNodeGeometricJacobianExpressedInWorld(
        mbt_context, pc, &H_PB_W);
  }
}

template <typename T>
void MultibodyTree<T>::CalcPointsGeometricJacobianExpressedInWorld(
    const systems::Context<T>& context,
    const Frame<T>& frame_B, const Eigen::Ref<const MatrixX<T>>& p_BQi_set,
    EigenPtr<MatrixX<T>> p_WQi_set, EigenPtr<MatrixX<T>> Jv_WQi) const {
  DRAKE_THROW_UNLESS(p_BQi_set.rows() == 3);
  const int num_points = p_BQi_set.cols();
  DRAKE_THROW_UNLESS(p_WQi_set != nullptr);
  DRAKE_THROW_UNLESS(p_WQi_set->cols() == num_points);
  DRAKE_THROW_UNLESS(Jv_WQi != nullptr);
  DRAKE_THROW_UNLESS(Jv_WQi->rows() == 3 * num_points);
  DRAKE_THROW_UNLESS(Jv_WQi->cols() == get_num_velocities());

  // Body to which frame B is attached to:
  const Body<T>& body_B = frame_B.get_body();

  // Compute kinematic path from body B to the world:
  std::vector<BodyNodeIndex> path_to_world;
  topology_.GetKinematicPathToWorld(body_B.get_node_index(), &path_to_world);

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  // TODO(amcastro-tri): Eval H_PB_W from the cache.
  std::vector<Vector6<T>> H_PB_W_cache(get_num_velocities());
  CalcAcrossNodeGeometricJacobianExpressedInWorld(context, pc, &H_PB_W_cache);

  CalcPointsPositions(context,
                      frame_B, p_BQi_set,            /* From frame B */
                      get_world_frame(), p_WQi_set); /* To world frame W */

  // Performs a scan of all bodies in the kinematic path from body_B to the
  // world computing each node's contribution to Jv_WQi.
  const int Jnrows = 3 * num_points;  // Number of rows in Jv_WQi.
  // Skip the world (ilevel = 0).
  for (size_t ilevel = 1; ilevel < path_to_world.size(); ++ilevel) {
    BodyNodeIndex body_node_index = path_to_world[ilevel];
    const BodyNode<T>& node = *body_nodes_[body_node_index];
    const BodyNodeTopology& node_topology = node.get_topology();
    const int start_index_in_v = node_topology.mobilizer_velocities_start_in_v;
    const int num_velocities = node_topology.num_mobilizer_velocities;

    // Across-node Jacobian.
    Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
        node.GetJacobianFromArray(H_PB_W_cache);
    auto J_PBq_W = Jv_WQi->block(0, start_index_in_v, Jnrows, num_velocities);

    // Position of this node's body Bi in the world W.
    const Vector3<T>& p_WBi = pc.get_X_WB(node.get_index()).translation();

    for (int ipoint = 0; ipoint < num_points; ++ipoint) {
      const auto p_WQi = p_WQi_set->col(ipoint);
      // Position of point Qi measured from Bi, expressed in the world W.
      const Vector3<T> p_BiQi_W = p_WQi - p_WBi;

      // Alias to Hv_PBqi_W for the ipoint-th point. Hv denotes the
      // translational components of the entire geometric Jacobian H.
      auto Hv_PBqi_W = J_PBq_W.block(3 * ipoint, 0, 3, num_velocities);

      // Aliases to angular and translational components in H_PB_W:
      const auto Hw_PB_W = H_PB_W.template topRows<3>();
      const auto Hv_PB_W = H_PB_W.template bottomRows<3>();

      // Now "shift" H_PB_W to H_PBqi_W.
      // We do it by shifting one column at a time:
      Hv_PBqi_W = Hv_PB_W + Hw_PB_W.colwise().cross(p_BiQi_W);
    }  // ipoint.
  }  // body_node_index
}

template <typename T>
void MultibodyTree<T>::CalcFrameGeometricJacobianExpressedInWorld(
    const systems::Context<T>& context,
    const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BoFo_B,
    EigenPtr<MatrixX<T>> Jv_WF) const {
  DRAKE_THROW_UNLESS(Jv_WF != nullptr);
  DRAKE_THROW_UNLESS(Jv_WF->rows() == 6);
  DRAKE_THROW_UNLESS(Jv_WF->cols() == get_num_velocities());

  // Body to which frame B is attached to:
  const Body<T>& body_B = frame_B.get_body();

  // Compute kinematic path from body B to the world:
  std::vector<BodyNodeIndex> path_to_world;
  topology_.GetKinematicPathToWorld(body_B.get_node_index(), &path_to_world);

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  // TODO(amcastro-tri): Eval H_PB_W from the cache.
  std::vector<Vector6<T>> H_PB_W_cache(get_num_velocities());
  CalcAcrossNodeGeometricJacobianExpressedInWorld(context, pc, &H_PB_W_cache);

  // Compute the position of F's origin in the world frame.
  Vector3<T> p_WoFo_W;
  CalcPointsPositions(context,
                      frame_B, p_BoFo_B,             /* From frame B */
                      get_world_frame(), &p_WoFo_W);  /* To world frame W */

  // Performs a scan of all bodies in the kinematic path from the world to
  // body_B, computing each node's contribution to Jv_WF.
  // Skip the world (ilevel = 0).
  for (size_t ilevel = 1; ilevel < path_to_world.size(); ++ilevel) {
    BodyNodeIndex body_node_index = path_to_world[ilevel];
    const BodyNode<T>& node = *body_nodes_[body_node_index];
    const BodyNodeTopology& node_topology = node.get_topology();
    const int start_index_in_v = node_topology.mobilizer_velocities_start_in_v;
    const int num_velocities = node_topology.num_mobilizer_velocities;

    // Across-node Jacobian.
    Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
        node.GetJacobianFromArray(H_PB_W_cache);

    // Output block corresponding to mobilities in the current node.
    // This correspond to the geometric Jacobian to compute the spatial velocity
    // of frame Bf (frame B shifted to Fo) measured in the inboard body frame P
    // and expressed in world. That is, V_PBf_W = J_PBf_W * v(B), with v(B) the
    // mobilities that correspond to the current node.
    auto J_PBf_W = Jv_WF->block(0, start_index_in_v, 6, num_velocities);

    // Position of this node's body Bi in the world W.
    const Vector3<T>& p_WBi = pc.get_X_WB(node.get_index()).translation();

    // Position of origin Fo measured from Bi, expressed in the world W.
    const Vector3<T> p_BiFo_W = p_WoFo_W - p_WBi;

    // Mutable aliases to Hw_PBf_W and Hv_PBf_W. Hw (Hv) denotes the
    // rotational (translational) components of the full geometric Jacobian.
    auto Hw_PBf_W = J_PBf_W.template topRows<3>();
    auto Hv_PBf_W = J_PBf_W.template bottomRows<3>();

    // Aliases to angular and translational components in H_PB_W:
    const auto Hw_PB_W = H_PB_W.template topRows<3>();
    const auto Hv_PB_W = H_PB_W.template bottomRows<3>();

    // Now "shift" (See SpatialVelocity::Shift()) H_PB_W to H_PBf_W.
    // We do it by shifting one column at a time:
    Hw_PBf_W = Hw_PB_W;  // angular component statys the same.
    Hv_PBf_W = Hv_PB_W + Hw_PB_W.colwise().cross(p_BiFo_W);
  }  // body_node_index
}

template <typename T>
T MultibodyTree<T>::CalcPotentialEnergy(
    const systems::Context<T>& context) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  return DoCalcPotentialEnergy(context, pc);
}

template <typename T>
T MultibodyTree<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc) const {
  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  T potential_energy = 0.0;
  // Add contributions from force elements.
  for (const auto& force_element : owned_force_elements_) {
    potential_energy += force_element->CalcPotentialEnergy(mbt_context, pc);
  }
  return potential_energy;
}

template <typename T>
T MultibodyTree<T>::CalcConservativePower(
    const systems::Context<T>& context) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  return DoCalcConservativePower(context, pc, vc);
}

template <typename T>
T MultibodyTree<T>::DoCalcConservativePower(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  T conservative_power = 0.0;
  // Add contributions from force elements.
  for (const auto& force_element : owned_force_elements_) {
    conservative_power +=
        force_element->CalcConservativePower(mbt_context, pc, vc);
  }
  return conservative_power;
}

template<typename T>
void MultibodyTree<T>::AllocateFakeCacheEntries() {
  // Temporary hack before MultibodyCachingEvaluoatorInterface lands.
  pc_ = std::make_unique<PositionKinematicsCache<T>>(get_topology());
  vc_ = std::make_unique<VelocityKinematicsCache<T>>(get_topology());
}

template<typename T>
const PositionKinematicsCache<T>& MultibodyTree<T>::EvalPositionKinematics(
    const systems::Context<T>& context) const {
  // TODO(amcastro-tri): Replace by cache_evaluator_->EvalPositionKinematics()
  // when MultibodyCachingEvaluatorInterface lands.
  CalcPositionKinematicsCache(context, pc_.get());
  return *pc_;
}

template<typename T>
const VelocityKinematicsCache<T>& MultibodyTree<T>::EvalVelocityKinematics(
    const systems::Context<T>& context) const {
  // TODO(amcastro-tri): Replace by cache_evaluator_->EvalVelocityKinematics()
  // when MultibodyCachingEvaluatorInterface lands.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  CalcVelocityKinematicsCache(context, pc, vc_.get());
  return *vc_;
}

template <typename T>
void MultibodyTree<T>::ThrowIfNotFinalized(
    const char* source_method) const {
  if (!topology_is_valid()) {
    throw std::logic_error(
        "The call to '" + std::string(source_method) + "' is invalid; "
        " You must call Finalize() first. ");
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyInertiaCache(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    ArticulatedBodyInertiaCache<T>* abc) const {
  DRAKE_DEMAND(abc != nullptr);

  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);

  // TODO(bobbyluig): Eval H_PB_W from the cache.
  std::vector<Vector6<T>> H_PB_W_cache(get_num_velocities());
  CalcAcrossNodeGeometricJacobianExpressedInWorld(context, pc, &H_PB_W_cache);

  // Perform tip-to-base recursion, skipping the world.
  for (int depth = get_tree_height() - 1; depth > 0; depth--) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      // Get hinge mapping matrix.
      const MatrixUpTo6<T> H_PB_W = node.GetJacobianFromArray(H_PB_W_cache);

      node.CalcArticulatedBodyInertiaCache_TipToBase(
          mbt_context, pc, H_PB_W, abc);
    }
  }
}

// Explicitly instantiates on the most common scalar types.
template class MultibodyTree<double>;
template class MultibodyTree<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
