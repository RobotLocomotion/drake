#include "drake/multibody/tree/multibody_tree.h"

#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/body_node_welded.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace internal {

using internal::BodyNode;
using internal::BodyNodeWelded;
using math::RigidTransform;
using math::RotationMatrix;

// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBT_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBT_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)

template <typename T>
class JointImplementationBuilder {
 public:
  JointImplementationBuilder() = delete;
  static std::vector<Mobilizer<T>*> Build(
      Joint<T>* joint, MultibodyTree<T>* tree) {
    std::vector<Mobilizer<T>*> mobilizers;
    std::unique_ptr<JointBluePrint> blue_print =
        joint->MakeImplementationBlueprint();
    auto implementation = std::make_unique<JointImplementation>(*blue_print);
    DRAKE_DEMAND(implementation->num_mobilizers() != 0);
    for (auto& mobilizer : blue_print->mobilizers_) {
      mobilizers.push_back(mobilizer.get());
      tree->AddMobilizer(std::move(mobilizer));
    }
    // TODO(amcastro-tri): add force elements, bodies, constraints, etc.
    joint->OwnImplementation(std::move(implementation));
    return mobilizers;
  }
 private:
  typedef typename Joint<T>::BluePrint JointBluePrint;
  typedef typename Joint<T>::JointImplementation JointImplementation;
};

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // Adds a "world" body to MultibodyTree having a NaN SpatialInertia.
  ModelInstanceIndex world_instance = AddModelInstance("WorldModelInstance");

  // `world_model_instance()` hardcodes the returned index.  Make sure it's
  // correct.
  DRAKE_DEMAND(world_instance == world_model_instance());
  world_body_ = &AddRigidBody("WorldBody", world_model_instance(),
                              SpatialInertia<double>());

  // `default_model_instance()` hardcodes the returned index.  Make sure it's
  // correct.
  ModelInstanceIndex default_instance =
      AddModelInstance("DefaultModelInstance");
  DRAKE_DEMAND(default_instance == default_model_instance());

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // TODO(sammy-tri) Move the custom handling logic for adding a
  // UniformGravityFieldElement here once we remove the deprecated overload.
  const ForceElement<T>& new_field =
      AddForceElement<UniformGravityFieldElement>();
#pragma GCC diagnostic pop
  DRAKE_DEMAND(num_force_elements() == 1);
  DRAKE_DEMAND(owned_force_elements_[0].get() == &new_field);
}

template <typename T>
void MultibodyTree<T>::SetActuationInArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& u_instance,
    EigenPtr<VectorX<T>> u) const {
  model_instances_.at(model_instance)->SetActuationInArray(u_instance, u);
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetPositionsFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& q) const {
  return model_instances_.at(model_instance)->GetPositionsFromArray(q);
}

template <class T>
void MultibodyTree<T>::SetPositionsInArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& q_instance,
    EigenPtr<VectorX<T>> q) const {
  model_instances_.at(model_instance)->SetPositionsInArray(q_instance, q);
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetVelocitiesFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& v) const {
  return model_instances_.at(model_instance)->GetVelocitiesFromArray(v);
}

template <class T>
void MultibodyTree<T>::SetVelocitiesInArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& v_instance,
    EigenPtr<VectorX<T>> v) const {
  model_instances_.at(model_instance)->SetVelocitiesInArray(v_instance, v);
}

template <typename T>
void MultibodyTree<T>::AddQuaternionFreeMobilizerToAllBodiesWithNoMobilizer() {
  DRAKE_DEMAND(!topology_is_valid());
  // Skip the world.
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    const BodyTopology& body_topology =
        get_topology().get_body(body.index());
    if (!body_topology.inboard_mobilizer.is_valid()) {
      std::unique_ptr<QuaternionFloatingMobilizer<T>> mobilizer =
          std::make_unique<QuaternionFloatingMobilizer<T>>(
              world_body().body_frame(), body.body_frame());
      mobilizer->set_model_instance(body.model_instance());
      this->AddMobilizer(std::move(mobilizer));
    }
  }
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
MultibodyTree<T>::GetFreeBodyMobilizerOrThrow(
    const Body<T>& body) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  DRAKE_DEMAND(body.index() != world_index());
  const BodyTopology& body_topology = get_topology().get_body(body.index());
  const QuaternionFloatingMobilizer<T>* mobilizer =
      dynamic_cast<const QuaternionFloatingMobilizer<T>*>(
          &get_mobilizer(body_topology.inboard_mobilizer));
  if (mobilizer == nullptr) {
    throw std::logic_error(
        "Body '" + body.name() + "' is not a free floating body.");
  }
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

  // Give different multiobody elements the chance to perform any finalize-time
  // setup.
  for (const auto& body : owned_bodies_) {
    body->SetTopology(topology_);
  }
  for (const auto& frame : owned_frames_) {
    frame->SetTopology(topology_);
  }
  for (const auto& mobilizer : owned_mobilizers_) {
    mobilizer->SetTopology(topology_);
  }
  for (const auto& force_element : owned_force_elements_) {
    force_element->SetTopology(topology_);
  }
  for (const auto& actuator : owned_actuators_) {
    actuator->SetTopology(topology_);
  }

  body_node_levels_.resize(topology_.tree_height());
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

  CreateModelInstances();
}

template <typename T>
void MultibodyTree<T>::Finalize() {
  DRAKE_MBT_THROW_IF_FINALIZED();
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
    std::vector<Mobilizer<T>*> mobilizers =
        internal::JointImplementationBuilder<T>::Build(joint.get(), this);
    for (Mobilizer<T>* mobilizer : mobilizers) {
      mobilizer->set_model_instance(joint->model_instance());
    }
  }
  // It is VERY important to add quaternions if needed only AFTER joints had a
  // chance to get implemented with mobilizers. This is because joints's
  // implementations change the topology of the tree. Therefore, do not change
  // this order!
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
    body_node = std::make_unique<BodyNodeWelded<T>>(&world_body());
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
void MultibodyTree<T>::CreateModelInstances() {
  DRAKE_ASSERT(model_instances_.empty());

  // First create the pool of instances.
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    std::unique_ptr<internal::ModelInstance<T>> model_instance =
        std::make_unique<internal::ModelInstance<T>>(model_instance_index);
    model_instance->set_parent_tree(this, model_instance_index);
    model_instances_.push_back(std::move(model_instance));
  }

  // Add all of our mobilizers and joint actuators to the appropriate instance.
  // The order of the mobilizers should match the order in which the bodies were
  // added to the tree, which may not be the order in which the mobilizers were
  // added, so we get the mobilizer through the BodyNode.
  for (const auto& body_node : body_nodes_) {
    if (body_node->get_num_mobilizer_positions() > 0 ||
        body_node->get_num_mobilizer_velocities() > 0) {
      model_instances_.at(body_node->model_instance())->add_mobilizer(
          &body_node->get_mobilizer());
    }
  }

  for (const auto& joint_actuator : owned_actuators_) {
    model_instances_.at(joint_actuator->model_instance())->add_joint_actuator(
        joint_actuator.get());
  }
}

template <typename T>
void MultibodyTree<T>::SetDefaultState(
    const systems::Context<T>& context, systems::State<T>* state) const {
  for (const auto& mobilizer : owned_mobilizers_) {
    mobilizer->set_default_state(context, state);
  }
}

template <typename T>
void MultibodyTree<T>::SetRandomState(const systems::Context<T>& context,
                                      systems::State<T>* state,
                                      RandomGenerator* generator) const {
  for (const auto& mobilizer : owned_mobilizers_) {
    mobilizer->set_random_state(context, state, generator);
  }
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>>
MultibodyTree<T>::GetPositionsAndVelocities(
    const systems::Context<T>& context) const {
  return get_state_vector(context);
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetPositionsAndVelocities(
    const systems::Context<T>& context,
    ModelInstanceIndex model_instance) const {
  Eigen::VectorBlock<const VectorX<T>> state_vector =
      get_state_vector(context);

  VectorX<T> instance_state_vector(num_states(model_instance));
  instance_state_vector.head(num_positions(model_instance)) =
      GetPositionsFromArray(
          model_instance, state_vector.head(num_positions()));
  instance_state_vector.tail(num_velocities(model_instance)) =
      GetVelocitiesFromArray(
          model_instance, state_vector.tail(num_velocities()));

  return instance_state_vector;
}

template <typename T>
Eigen::VectorBlock<VectorX<T>>
MultibodyTree<T>::GetMutablePositionsAndVelocities(
    const systems::Context<T>&, systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  return get_mutable_state_vector(state);
}

template <typename T>
void MultibodyTree<T>::SetPositionsAndVelocities(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& instance_state,
    systems::Context<T>* context) const {
  Eigen::VectorBlock<VectorX<T>> state_vector =
      GetMutablePositionsAndVelocities(context);
  Eigen::VectorBlock<VectorX<T>> q = state_vector.nestedExpression().head(
      num_positions());
  Eigen::VectorBlock<VectorX<T>> v = state_vector.nestedExpression().tail(
      num_velocities());
  SetPositionsInArray(model_instance,
                      instance_state.head(num_positions(model_instance)), &q);
  SetVelocitiesInArray(model_instance,
                       instance_state.tail(num_velocities(model_instance)), &v);
}

template <typename T>
RigidTransform<T> MultibodyTree<T>::GetFreeBodyPoseOrThrow(
    const systems::Context<T>& context, const Body<T>& body) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  const QuaternionFloatingMobilizer<T>& mobilizer =
      GetFreeBodyMobilizerOrThrow(body);
  return RigidTransform<T>(mobilizer.get_quaternion(context),
                                 mobilizer.get_position(context));
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyPoseOrThrow(
    const Body<T>& body, const RigidTransform<T>& X_WB,
    systems::Context<T>* context) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  SetFreeBodyPoseOrThrow(body, X_WB, *context, &context->get_mutable_state());
}

template <typename T>
void MultibodyTree<T>::SetFreeBodySpatialVelocityOrThrow(
    const Body<T>& body, const SpatialVelocity<T>& V_WB,
    systems::Context<T>* context) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  SetFreeBodySpatialVelocityOrThrow(
      body, V_WB, *context, &context->get_mutable_state());
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyPoseOrThrow(
    const Body<T>& body, const RigidTransform<T>& X_WB,
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  const QuaternionFloatingMobilizer<T>& mobilizer =
      GetFreeBodyMobilizerOrThrow(body);
  const RotationMatrix<T>& R_WB = X_WB.rotation();
  mobilizer.set_quaternion(context, R_WB.ToQuaternion(), state);
  mobilizer.set_position(context, X_WB.translation(), state);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodySpatialVelocityOrThrow(
    const Body<T>& body, const SpatialVelocity<T>& V_WB,
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  const QuaternionFloatingMobilizer<T>& mobilizer =
      GetFreeBodyMobilizerOrThrow(body);
  mobilizer.set_angular_velocity(context, V_WB.rotational(), state);
  mobilizer.set_translational_velocity(context, V_WB.translational(), state);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyRandomPositionDistributionOrThrow(
    const Body<T>& body, const Vector3<symbolic::Expression>& position) {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  QuaternionFloatingMobilizer<T>& mobilizer =
      get_mutable_variant(GetFreeBodyMobilizerOrThrow(body));
  mobilizer.set_random_position_distribution(position);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyRandomRotationDistributionOrThrow(
    const Body<T>& body,
    const Eigen::Quaternion<symbolic::Expression>& rotation) {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  QuaternionFloatingMobilizer<T>& mobilizer =
      get_mutable_variant(GetFreeBodyMobilizerOrThrow(body));
  mobilizer.set_random_quaternion_distribution(rotation);
}

template <typename T>
void MultibodyTree<T>::CalcAllBodyPosesInWorld(
    const systems::Context<T>& context,
    std::vector<RigidTransform<T>>* X_WB) const {
  DRAKE_THROW_UNLESS(X_WB != nullptr);
  if (static_cast<int>(X_WB->size()) != num_bodies()) {
    X_WB->resize(num_bodies(), RigidTransform<T>::Identity());
  }
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const BodyNodeIndex node_index = get_body(body_index).node_index();
    X_WB->at(body_index) = pc.get_X_WB(node_index);
  }
}

template <typename T>
void MultibodyTree<T>::CalcAllBodySpatialVelocitiesInWorld(
    const systems::Context<T>& context,
    std::vector<SpatialVelocity<T>>* V_WB) const {
  DRAKE_THROW_UNLESS(V_WB != nullptr);
  if (static_cast<int>(V_WB->size()) != num_bodies()) {
    V_WB->resize(num_bodies(), SpatialVelocity<T>::Zero());
  }
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const BodyNodeIndex node_index = get_body(body_index).node_index();
    V_WB->at(body_index) = vc.get_V_WB(node_index);
  }
}

template <typename T>
void MultibodyTree<T>::CalcPositionKinematicsCache(
    const systems::Context<T>& context,
    PositionKinematicsCache<T>* pc) const {
  DRAKE_DEMAND(pc != nullptr);

  // TODO(amcastro-tri): Loop over bodies to update their position dependent
  // kinematics. This gives the chance to flexible bodies to update the pose
  // X_BQ(qb_B) of each frame Q that is attached to the body.
  // Notice this loop can be performed in any order and each X_BQ(qf_B) is
  // independent of all others. This could even be performed in parallel.

  // With the kinematics information across mobilizer's and the kinematics
  // information for each body, we are now in position to perform a base-to-tip
  // recursion to update world positions and parent to child body transforms.
  // This skips the world, level = 0.
  for (int level = 1; level < tree_height(); ++level) {
    for (BodyNodeIndex body_node_index : body_node_levels_[level]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == level);
      DRAKE_ASSERT(node.index() == body_node_index);

      // Update per-node kinematics.
      node.CalcPositionKinematicsCache_BaseToTip(context, pc);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcVelocityKinematicsCache(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    VelocityKinematicsCache<T>* vc) const {
  DRAKE_DEMAND(vc != nullptr);

  // TODO(amcastro-tri): Loop over bodies to compute velocity kinematics updates
  // corresponding to flexible bodies.

  const std::vector<Vector6<T>>& H_PB_W_cache =
      tree_system_->EvalAcrossNodeGeometricJacobianExpressedInWorld(context);

  // Performs a base-to-tip recursion computing body velocities.
  // This skips the world, depth = 0.
  for (int depth = 1; depth < tree_height(); ++depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.index() == body_node_index);

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
      node.CalcVelocityKinematicsCache_BaseToTip(context, pc, H_PB_W, vc);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcSpatialInertiaInWorldCache(
    const systems::Context<T>& context,
    std::vector<SpatialInertia<T>>* M_B_W_cache) const {
  DRAKE_THROW_UNLESS(M_B_W_cache != nullptr);
  DRAKE_THROW_UNLESS(static_cast<int>(M_B_W_cache->size()) == num_bodies());

  const PositionKinematicsCache<T>& pc = this->EvalPositionKinematics(context);

  // Skip the world.
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    const RigidTransform<T>& X_WB = pc.get_X_WB(body.node_index());

    // Orientation of B in W.
    const RotationMatrix<T>& R_WB = X_WB.rotation();

    // Spatial inertia of body B about Bo and expressed in the body frame B.
    // This call has zero cost for rigid bodies.
    const SpatialInertia<T> M_B = body.CalcSpatialInertiaInBodyFrame(context);

    // Re-express body B's spatial inertia in the world frame W.
    SpatialInertia<T>& M_B_W = (*M_B_W_cache)[body.node_index()];
    M_B_W = M_B.ReExpress(R_WB);
  }
}

template <typename T>
void MultibodyTree<T>::CalcDynamicBiasCache(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* b_Bo_W_cache) const {
  DRAKE_THROW_UNLESS(b_Bo_W_cache != nullptr);
  DRAKE_THROW_UNLESS(static_cast<int>(b_Bo_W_cache->size()) == num_bodies());

  const std::vector<SpatialInertia<T>>& spatial_inertia_in_world_cache =
      EvalSpatialInertiaInWorldCache(context);

  const VelocityKinematicsCache<T>& vc = this->EvalVelocityKinematics(context);

  // Skip the world.
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);

    const SpatialInertia<T>& M_B_W =
        spatial_inertia_in_world_cache[body.node_index()];

    const T& mass = M_B_W.get_mass();
    // B's center of mass measured in B and expressed in W.
    const Vector3<T>& p_BoBcm_W = M_B_W.get_com();
    // B's unit rotational inertia about Bo, expressed in W.
    const UnitInertia<T>& G_B_W = M_B_W.get_unit_inertia();

    // Gyroscopic spatial force b_Bo_W(q, v) on body B about Bo, expressed in W.
    const SpatialVelocity<T>& V_WB = vc.get_V_WB(body.node_index());
    const Vector3<T>& w_WB = V_WB.rotational();
    SpatialForce<T>& b_Bo_W = (*b_Bo_W_cache)[body.node_index()];
    b_Bo_W = mass * SpatialForce<T>(
                        w_WB.cross(G_B_W * w_WB), /* rotational */
                        w_WB.cross(w_WB.cross(p_BoBcm_W)) /* translational */);
  }
}

template <typename T>
void MultibodyTree<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&,
    const VectorX<T>& known_vdot,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  const bool ignore_velocities = false;
  CalcSpatialAccelerationsFromVdot(context, known_vdot, ignore_velocities,
                                   A_WB_array);
}

template <typename T>
void MultibodyTree<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context,
    const VectorX<T>& known_vdot,
    bool ignore_velocities,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  DRAKE_DEMAND(A_WB_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(A_WB_array->size()) == num_bodies());

  DRAKE_DEMAND(known_vdot.size() == topology_.num_velocities());

  const auto& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>* vc =
      ignore_velocities ? nullptr : &EvalVelocityKinematics(context);

  // TODO(amcastro-tri): Loop over bodies to compute acceleration kinematics
  // updates corresponding to flexible bodies.

  // The world's spatial acceleration is always zero.
  A_WB_array->at(world_index()) = SpatialAcceleration<T>::Zero();

  // Performs a base-to-tip recursion computing body accelerations.
  // This skips the world, depth = 0.
  for (int depth = 1; depth < tree_height(); ++depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.index() == body_node_index);

      // Update per-node kinematics.
      node.CalcSpatialAcceleration_BaseToTip(
          context, pc, vc, known_vdot, A_WB_array);
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
  DRAKE_DEMAND(known_vdot.size() == topology_.num_velocities());

  // TODO(amcastro-tri): Loop over bodies to compute velocity kinematics updates
  // corresponding to flexible bodies.

  std::vector<SpatialAcceleration<T>>& A_WB_array = ac->get_mutable_A_WB_pool();

  CalcSpatialAccelerationsFromVdot(context, pc, vc, known_vdot, &A_WB_array);
}

template <typename T>
VectorX<T> MultibodyTree<T>::CalcInverseDynamics(
    const systems::Context<T>& context,
    const VectorX<T>& known_vdot,
    const MultibodyForces<T>& external_forces) const {
  // Temporary storage used in the computation of inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W(num_bodies());
  VectorX<T> tau(num_velocities());
  CalcInverseDynamics(
      context, known_vdot,
      external_forces.body_forces(), external_forces.generalized_forces(),
      &A_WB, &F_BMo_W, &tau);
  return tau;
}

template <typename T>
void MultibodyTree<T>::CalcInverseDynamics(
    const systems::Context<T>& context, const VectorX<T>& known_vdot,
    const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
    const Eigen::Ref<const VectorX<T>>& tau_applied_array,
    std::vector<SpatialAcceleration<T>>* A_WB_array,
    std::vector<SpatialForce<T>>* F_BMo_W_array,
    EigenPtr<VectorX<T>> tau_array) const {
  const bool ignore_velocities = false;
  CalcInverseDynamics(context, known_vdot, Fapplied_Bo_W_array,
                      tau_applied_array, ignore_velocities, A_WB_array,
                      F_BMo_W_array, tau_array);
}

template <typename T>
void MultibodyTree<T>::CalcInverseDynamics(
    const systems::Context<T>& context,
    const VectorX<T>& known_vdot,
    const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
    const Eigen::Ref<const VectorX<T>>& tau_applied_array,
    bool ignore_velocities,
    std::vector<SpatialAcceleration<T>>* A_WB_array,
    std::vector<SpatialForce<T>>* F_BMo_W_array,
    EigenPtr<VectorX<T>> tau_array) const {
  DRAKE_DEMAND(known_vdot.size() == num_velocities());
  const int Fapplied_size = static_cast<int>(Fapplied_Bo_W_array.size());
  DRAKE_DEMAND(Fapplied_size == num_bodies() || Fapplied_size == 0);
  const int tau_applied_size = tau_applied_array.size();
  DRAKE_DEMAND(
      tau_applied_size == num_velocities() || tau_applied_size == 0);

  DRAKE_DEMAND(A_WB_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(A_WB_array->size()) == num_bodies());

  DRAKE_DEMAND(F_BMo_W_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(F_BMo_W_array->size()) == num_bodies());

  DRAKE_DEMAND(tau_array->size() == num_velocities());

  // Compute body spatial accelerations given the generalized accelerations are
  // known.
  CalcSpatialAccelerationsFromVdot(context, known_vdot, ignore_velocities,
                                   A_WB_array);

  // Vector of generalized forces per mobilizer.
  // It has zero size if no forces are applied.
  VectorUpTo6<T> tau_applied_mobilizer(0);

  // Spatial force applied on B at Bo.
  // It is left initialized to zero if no forces are applied.
  SpatialForce<T> Fapplied_Bo_W = SpatialForce<T>::Zero();

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  // Eval M_Bo_W(q).
  const std::vector<SpatialInertia<T>>& spatial_inertia_in_world_cache =
      EvalSpatialInertiaInWorldCache(context);

  // Eval b_Bo_W(q, v). b_Bo_W = 0 if v = 0.
  const std::vector<SpatialForce<T>>* dynamic_bias_cache =
      ignore_velocities ? nullptr : &EvalDynamicBiasCache(context);

  // Performs a tip-to-base recursion computing the total spatial force F_BMo_W
  // acting on body B, about point Mo, expressed in the world frame W.
  // This includes the world (depth = 0) so that F_BMo_W_array[world_index()]
  // contains the total force of the bodies connected to the world by a
  // mobilizer.
  for (int depth = tree_height() - 1; depth >= 0; --depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.index() == body_node_index);

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
          context, pc, spatial_inertia_in_world_cache, dynamic_bias_cache,
          *A_WB_array, Fapplied_Bo_W, tau_applied_mobilizer, F_BMo_W_array,
          tau_array);
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

  forces->SetZero();
  // Add contributions from force elements.
  for (const auto& force_element : owned_force_elements_) {
    force_element->CalcAndAddForceContribution(context, pc, vc, forces);
  }

  // TODO(amcastro-tri): Remove this call once damping is implemented in terms
  // of force elements.
  AddJointDampingForces(context, forces);
}

template<typename T>
void MultibodyTree<T>::AddJointDampingForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  for (const auto& joint : owned_joints_) {
    joint->AddInDamping(context, forces);
  }
}

template <typename T>
void MultibodyTree<T>::MapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_DEMAND(qdot.size() == num_positions());
  DRAKE_DEMAND(v != nullptr);
  DRAKE_DEMAND(v->size() == num_velocities());

  VectorUpTo6<T> v_mobilizer;
  for (const auto& mobilizer : owned_mobilizers_) {
    const auto qdot_mobilizer = mobilizer->get_positions_from_array(qdot);
    v_mobilizer.resize(mobilizer->num_velocities());
    mobilizer->MapQDotToVelocity(context, qdot_mobilizer, &v_mobilizer);
    mobilizer->get_mutable_velocities_from_array(v) = v_mobilizer;
  }
}

template <typename T>
void MultibodyTree<T>::MapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_DEMAND(v.size() == num_velocities());
  DRAKE_DEMAND(qdot != nullptr);
  DRAKE_DEMAND(qdot->size() == num_positions());

  const int kMaxQdot = 7;
  // qdot_mobilizer is a dynamic sized vector of max size equal to seven.
  Eigen::Matrix<T, Eigen::Dynamic, 1, 0, kMaxQdot, 1> qdot_mobilizer;
  for (const auto& mobilizer : owned_mobilizers_) {
    const auto v_mobilizer = mobilizer->get_velocities_from_array(v);
    DRAKE_DEMAND(mobilizer->num_positions() <= kMaxQdot);
    qdot_mobilizer.resize(mobilizer->num_positions());
    mobilizer->MapVelocityToQDot(context, v_mobilizer, &qdot_mobilizer);
    mobilizer->get_mutable_positions_from_array(qdot) = qdot_mobilizer;
  }
}

template <typename T>
void MultibodyTree<T>::CalcMassMatrixViaInverseDynamics(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> H) const {
  DRAKE_DEMAND(H != nullptr);
  DRAKE_DEMAND(H->rows() == num_velocities());
  DRAKE_DEMAND(H->cols() == num_velocities());

  // Compute one column of the mass matrix via inverse dynamics at a time.
  const int nv = num_velocities();
  VectorX<T> vdot(nv);
  VectorX<T> tau(nv);
  // Auxiliary arrays used by inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(num_bodies());

  // The mass matrix is only a function of configuration q. Therefore velocity
  // terms are not considered.
  const bool ignore_velocities = true;
  for (int j = 0; j < nv; ++j) {
    // N.B. VectorX<T>::Unit() does not perform any heap allocation but rather
    // returns a functor-like object that fills the entries in vdot.
    vdot = VectorX<T>::Unit(nv, j);
    tau.setZero();
    CalcInverseDynamics(context, vdot, {}, VectorX<T>(), ignore_velocities,
                        &A_WB_array, &F_BMo_W_array, &tau);
    H->col(j) = tau;
  }
}

template <typename T>
void MultibodyTree<T>::CalcBiasTerm(
    const systems::Context<T>& context, EigenPtr<VectorX<T>> Cv) const {
  DRAKE_DEMAND(Cv != nullptr);
  DRAKE_DEMAND(Cv->rows() == num_velocities());
  DRAKE_DEMAND(Cv->cols() == 1);
  const int nv = num_velocities();
  const VectorX<T> vdot = VectorX<T>::Zero(nv);
  // Auxiliary arrays used by inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(num_bodies());
  // TODO(amcastro-tri): provide specific API for when vdot = 0.
  CalcInverseDynamics(context, vdot, {}, VectorX<T>(),
                      &A_WB_array, &F_BMo_W_array, Cv);
}

template <typename T>
VectorX<T> MultibodyTree<T>::CalcGravityGeneralizedForces(
    const systems::Context<T>& context) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  if (gravity_field_) {
    return gravity_field_->CalcGravityGeneralizedForces(context);
  }
  return VectorX<T>::Zero(num_velocities());
}

template <typename T>
RigidTransform<T> MultibodyTree<T>::CalcRelativeTransform(
    const systems::Context<T>& context,
    const Frame<T>& frame_F,
    const Frame<T>& frame_G) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const Body<T>& A = frame_F.body();
  const Body<T>& B = frame_G.body();
  const RigidTransform<T>& X_WA = pc.get_X_WB(A.node_index());
  const RigidTransform<T>& X_WB = pc.get_X_WB(B.node_index());
  const RigidTransform<T> X_WF = X_WA * frame_F.CalcPoseInBodyFrame(context);
  const RigidTransform<T> X_WG = X_WB * frame_G.CalcPoseInBodyFrame(context);
  return X_WF.inverse() * X_WG;  // X_FG = X_FW * X_WG;
}

template <typename T>
RotationMatrix<T> MultibodyTree<T>::CalcRelativeRotationMatrix(
    const systems::Context<T>& context,
    const Frame<T>& frame_F,
    const Frame<T>& frame_G) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const Body<T>& A = frame_F.body();
  const Body<T>& B = frame_G.body();
  const RotationMatrix<T>& R_WA = pc.get_X_WB(A.node_index()).rotation();
  const RotationMatrix<T>& R_WB = pc.get_X_WB(B.node_index()).rotation();
  const RotationMatrix<T> R_WF =
      R_WA * frame_F.CalcPoseInBodyFrame(context).rotation();
  const RotationMatrix<T> R_WG =
      R_WB * frame_G.CalcPoseInBodyFrame(context).rotation();
  return R_WF.inverse() * R_WG;  // R_FG = R_FW * R_WG;
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
  const RigidTransform<T> X_AB =
      CalcRelativeTransform(context, frame_A, frame_B);
  p_AQi->template topRows<3>() = X_AB * p_BQi.template topRows<3>();
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassPosition(
    const systems::Context<T>& context) const {
  if (!(num_bodies() > 1)) {
    throw std::runtime_error(
        "CalcCenterOfMassPosition(): this MultibodyPlant contains only "
        "world_body() so its center of mass is undefined.");
  }

  std::vector<ModelInstanceIndex> model_instances;
  for (ModelInstanceIndex model_instance_index(1);
       model_instance_index < num_model_instances(); ++model_instance_index)
    model_instances.push_back(model_instance_index);

  return CalcCenterOfMassPosition(context, model_instances);
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassPosition(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances) const {
  if (!(num_model_instances() > 1)) {
    throw std::runtime_error(
        "CalcCenterOfMassPosition(): this MultibodyPlant contains only "
        "world_body() so its center of mass is undefined.");
  }

  std::vector<BodyIndex> body_indexes;
  for (auto model_instance : model_instances) {
    const std::vector<BodyIndex> body_index_in_instance =
        GetBodyIndices(model_instance);
    for (BodyIndex body_index : body_index_in_instance)
      body_indexes.push_back(body_index);
  }

  return CalcCenterOfMassPosition(context, body_indexes);
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassPosition(
    const systems::Context<T>& context,
    const std::vector<BodyIndex>& body_indexes) const {
  if (!(num_bodies() > 1)) {
    throw std::runtime_error(
        "CalcCenterOfMassPosition(): this MultibodyPlant contains only "
        "world_body() so its center of mass is undefined.");
  }
  if (body_indexes.empty()) {
    throw std::runtime_error(
        "CalcCenterOfMassPosition(): you must provide at least one selected "
        "body.");
  }

  Vector3<T> Mp = Vector3<T>::Zero();
  T composite_mass = 0;

  for (BodyIndex body_index : body_indexes) {
    if (body_index == 0) continue;

    const Body<T>& body = get_body(body_index);

    const Vector3<T> pi_BoBcm = body.CalcCenterOfMassInBodyFrame(context);
    Vector3<T> pi_WBcm = body.EvalPoseInWorld(context) * pi_BoBcm;

    // Calculate composite_mass and M * p in world frame.
    const T& body_mass = body.get_mass(context);
    // Mp = ∑ mᵢ * pi_WBcm
    Mp += body_mass * pi_WBcm;
    // composite_mass = ∑ mᵢ
    composite_mass += body_mass;
  }

  if (!(composite_mass > 0)) {
    throw std::runtime_error(
        "CalcCenterOfMassPosition(): the total mass must larger than zero.");
  }
  return Mp / composite_mass;
}

template <typename T>
const RigidTransform<T>& MultibodyTree<T>::EvalBodyPoseInWorld(
    const systems::Context<T>& context,
    const Body<T>& body_B) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  body_B.HasThisParentTreeOrThrow(this);
  return EvalPositionKinematics(context).get_X_WB(body_B.node_index());
}

template <typename T>
const SpatialVelocity<T>& MultibodyTree<T>::EvalBodySpatialVelocityInWorld(
    const systems::Context<T>& context,
    const Body<T>& body_B) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  body_B.HasThisParentTreeOrThrow(this);
  return EvalVelocityKinematics(context).get_V_WB(body_B.node_index());
}

template <typename T>
void MultibodyTree<T>::CalcAcrossNodeGeometricJacobianExpressedInWorld(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    std::vector<Vector6<T>>* H_PB_W_cache) const {
  DRAKE_DEMAND(H_PB_W_cache != nullptr);
  DRAKE_DEMAND(static_cast<int>(H_PB_W_cache->size()) == num_velocities());

  for (BodyNodeIndex node_index(1);
       node_index < num_bodies(); ++node_index) {
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
        context, pc, &H_PB_W);
  }
}

// TODO(Mitiguy) Delete this method as per issue #10155.
// DRAKE_DEPRECATED("2019-10-01", "Use CalcJacobianTranslationalVelocity().")
template <typename T>
void MultibodyTree<T>::CalcPointsGeometricJacobianExpressedInWorld(
    const systems::Context<T>& context,
    const Frame<T>& frame_F,
    const Eigen::Ref<const MatrixX<T>>& p_FP_list,
    EigenPtr<MatrixX<T>> p_WP_list,
    EigenPtr<MatrixX<T>> Jv_WFp) const {
  const int num_points = p_FP_list.cols();
  DRAKE_THROW_UNLESS(p_WP_list != nullptr);
  DRAKE_THROW_UNLESS(p_WP_list->cols() == num_points);

  // For each point Fi, calculate Fi's position from Wo (World origin),
  // expressed in world W.
  const Frame<T>& frame_W = world_frame();
  CalcPointsPositions(context, frame_F, p_FP_list,      /* From frame F */
                      frame_W, p_WP_list);          /* To world frame W */
  CalcJacobianTranslationalVelocity(context,
                                    JacobianWrtVariable::kV,
                                    frame_F,
                                    frame_W,
                                    *p_WP_list,
                                    frame_W,
                                    frame_W,
                                    Jv_WFp);
}

// TODO(Mitiguy) Delete this method as per issue #10155.
// DRAKE_DEPRECATED("2019-10-01", "Use CalcJacobianTranslationalVelocity().")
template <typename T>
void MultibodyTree<T>::CalcPointsAnalyticalJacobianExpressedInWorld(
    const systems::Context<T>& context,
    const Frame<T>& frame_F,
    const Eigen::Ref<const MatrixX<T>>& p_FP_list,
    EigenPtr<MatrixX<T>> p_WP_list,
    EigenPtr<MatrixX<T>> Jq_WFp) const {
  const int num_points = p_FP_list.cols();
  DRAKE_THROW_UNLESS(p_WP_list != nullptr);
  DRAKE_THROW_UNLESS(p_WP_list->cols() == num_points);

  // For each point Fi, calculate Fi's position from Wo (World origin),
  // expressed in world W.
  const Frame<T>& frame_W = world_frame();
  CalcPointsPositions(context, frame_F, p_FP_list,     /* From frame F */
                      frame_W, p_WP_list);         /* To world frame W */

  CalcJacobianTranslationalVelocity(context,
                                    JacobianWrtVariable::kQDot,
                                    frame_F,
                                    frame_W,
                                    *p_WP_list,
                                    frame_W,
                                    frame_W,
                                    Jq_WFp);
}

template <typename T>
SpatialAcceleration<T> MultibodyTree<T>::CalcSpatialAccelerationBiasShift(
    const systems::Context<T>& context,
    const Frame<T>& frame_F,
    const math::RigidTransform<T>& X_BF,
    const Vector3<T>& p_FoFp_F,
    const SpatialAcceleration<T>& Abias_WBo_W,
    const Frame<T>& frame_E) const {

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Get body B's rotation matrix in world W and angular velocity in world W.
  const Body<T>& body_B = frame_F.body();
  const RotationMatrix<T>& R_WB = pc.get_X_WB(body_B.node_index()).rotation();
  const Vector3<T>& w_WB_W = vc.get_V_WB(body_B.node_index()).rotational();

  // We need to compute p_BoFp_W, the position from Bo to Fp, expressed in W.
  const Vector3<T> p_BoFp_B = X_BF * p_FoFp_F;
  const Vector3<T> p_BoFp_W = R_WB * p_BoFp_B;

  // Shift spatial acceleration bias term from point Bo to point Fp.
  // See SpatialAcceleration::Shift() for details.
  SpatialAcceleration<T> Abias_WFp = Abias_WBo_W.Shift(p_BoFp_W, w_WB_W);

  // Express the resulting vectors in frame_E (rather than the world frame).
  if (&frame_E != &world_frame()) {
    const RigidTransform<T> X_WE = frame_E.CalcPoseInWorld(context);
    const RotationMatrix<T> R_EW = X_WE.rotation().inverse();

    // Abias_WFp_E = R_EW * Abias_WFp_W.
    Abias_WFp = R_EW * Abias_WFp;
  }

  return Abias_WFp;
}

template <typename T>
VectorX<T> MultibodyTree<T>::CalcBiasForJacobianTranslationalVelocity(
    const systems::Context<T>& context,
    JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_F,
    const Eigen::Ref<const MatrixX<T>>& p_FP_list,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E) const {
  DRAKE_THROW_UNLESS(p_FP_list.rows() == 3);

  // TODO(mitiguy) Allow with_respect_to be JacobianWrtVariable::kQDot.
  DRAKE_THROW_UNLESS(with_respect_to == JacobianWrtVariable::kV);

  // TODO(mitiguy) Allow frame_A to be something other than world frame W.
  DRAKE_THROW_UNLESS(&frame_A == &world_frame());

  // This algorithm is explained in CalcBiasForJacobianSpatialVelocity().
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Set ṡ = 0 and calculate point Bo's spatial acceleration bias in world W.
  std::vector<SpatialAcceleration<T>> Abias_WB_array(num_bodies());
  const VectorX<T> vdot = VectorX<T>::Zero(num_velocities());
  CalcSpatialAccelerationsFromVdot(context, pc, vc, vdot, &Abias_WB_array);

  // Extract point Bo's spatial acceleration bias from the large array.
  const Body<T>& body_B = frame_F.body();
  const SpatialAcceleration<T>& Abias_WBo = Abias_WB_array[body_B.node_index()];

  // Get transform from body B to frame F.
  const RigidTransform<T> X_BF = frame_F.GetFixedPoseInBodyFrame();

  // Allocate the output vector.
  const int num_points = p_FP_list.cols();
  VectorX<T> Abias_WFp_array(3 * num_points);

  for (int ipoint = 0; ipoint < num_points; ++ipoint) {
    const Vector3<T> p_FoPi_F = p_FP_list.col(ipoint);

    // Shift spatial acceleration bias term from point Bo to point Fp.
    const SpatialAcceleration<T> Abias_WFp = CalcSpatialAccelerationBiasShift(
        context, frame_F, X_BF, p_FoPi_F, Abias_WBo, frame_E);

    // Output translational component only.
    Abias_WFp_array.template segment<3>(3 * ipoint) = Abias_WFp.translational();
  }

  return Abias_WFp_array;
}

// TODO(Mitiguy) Delete this method as per issue #10155.
// DRAKE_DEPRECATED("2019-10-01", "Use CalcJacobianTranslationalVelocity().")
template <typename T>
void MultibodyTree<T>::CalcPointsGeometricJacobianExpressedInWorld(
    const systems::Context<T>& context,
    const Frame<T>& frame_F,
    const Eigen::Ref<const MatrixX<T>>& p_WP_list,
    EigenPtr<MatrixX<T>> Jv_WFp) const {
  const Frame<T>& frame_W = world_frame();
  CalcJacobianTranslationalVelocity(context,
                                    JacobianWrtVariable::kV,
                                    frame_F,
                                    frame_W,
                                    p_WP_list,
                                    frame_W,
                                    frame_W,
                                    Jv_WFp);
}

template <typename T>
void MultibodyTree<T>::CalcJacobianSpatialVelocity(
    const systems::Context<T>& context,
    const JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Eigen::Ref<const Vector3<T>>& p_BP,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E,
    EigenPtr<MatrixX<T>> Jw_V_ABp_E) const {
  DRAKE_THROW_UNLESS(Jw_V_ABp_E != nullptr);
  DRAKE_THROW_UNLESS(Jw_V_ABp_E->rows() == 6);

  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                           num_positions() : num_velocities();
  DRAKE_THROW_UNLESS(Jw_V_ABp_E->cols() == num_columns);

  // The spatial velocity V_WBp can be obtained by composing the spatial
  // velocities V_WAp and V_ABp. Expressed in the world frame W this composition
  // is V_WBp_W = V_WAp_W + V_ABp_W
  // Therefore, V_ABp_W = (Jw_WBp - Jw_WAp)⋅w.
  //
  // If with_respect_to = JacobianWrtVariable::kQDot, w = q̇ and
  // Jw_W{Ap,Bp} = Jq_W{Ap,Bp},
  // If with_respect_to == JacobianWrtVariable::kV,  w = v and
  // Jw_W{Ap,Bp} = Jv_W{Ap,Bp}.
  //
  // Expressed in frame E, this becomes
  //   V_ABp_E = R_EW⋅(Jw_WBp - Jw_WAp)⋅w.
  // Thus, Jw_V_ABp_E = R_EW⋅(Jw_WBp - Jw_WAp).

  Vector3<T> p_WP;
  CalcPointsPositions(context, frame_B, p_BP, /* From frame B */
                      world_frame(), &p_WP);  /* To world frame W */

  // TODO(amcastro-tri): When performance becomes an issue, implement this
  // method so that we only consider the kinematic path from A to B.

  Matrix6X<T> Jw_WAp(6, num_columns);
  auto Jr_WAp = Jw_WAp.template topRows<3>();     // rotational part.
  auto Jt_WAp = Jw_WAp.template bottomRows<3>();  // translational part.
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
      with_respect_to, frame_A,  p_WP,  &Jr_WAp, &Jt_WAp);

  Matrix6X<T> Jw_WBp(6, num_columns);
  auto Jr_WBp = Jw_WBp.template topRows<3>();     // rotational part.
  auto Jt_WBp = Jw_WBp.template bottomRows<3>();  // translational part.
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
      with_respect_to, frame_B,  p_WP,  &Jr_WBp, &Jt_WBp);

  // Jacobian Jw_ABp_W when E is the world frame W.
  Jw_V_ABp_E->template topRows<3>() = Jr_WBp - Jr_WAp;
  Jw_V_ABp_E->template bottomRows<3>() = Jt_WBp - Jt_WAp;

  // If the expressed-in frame E is not the world frame, we need to perform
  // an additional operation.
  if (frame_E.index() != world_frame().index()) {
    const RotationMatrix<T> R_EW =
        CalcRelativeRotationMatrix(context, frame_E, world_frame());
    Jw_V_ABp_E->template topRows<3>() =
        R_EW * Jw_V_ABp_E->template topRows<3>();
    Jw_V_ABp_E->template bottomRows<3>() =
        R_EW * Jw_V_ABp_E->template bottomRows<3>();
  }
}

template <typename T>
void MultibodyTree<T>::CalcJacobianAngularVelocity(
    const systems::Context<T>& context,
    const JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E,
    EigenPtr<Matrix3X<T>> Js_w_AB_E) const {
  DRAKE_THROW_UNLESS(Js_w_AB_E != nullptr);
  DRAKE_THROW_UNLESS(Js_w_AB_E->rows() == 3);
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                          num_positions() : num_velocities();
  DRAKE_THROW_UNLESS(Js_w_AB_E->cols() == num_columns);

  // The angular velocity addition theorem, gives w_WB = w_WA + w_AB, where
  // w_WB is frame B's angular velocity in world W,
  // w_WA is frame A's angular velocity in world W, and
  // w_AB is frame B's angular velocity in frame A.
  // Rearrange to calculate B's angular velocity in A as w_AB = w_WB - w_WA.
  // So B's angular velocity Jacobian in A, expressed in frame E is
  // Js_w_AB_E = R_EW * (Js_w_WB_W - Js_w_WA_W).

  // TODO(Mitiguy): When performance becomes an issue, optimize this method by
  // only using the kinematics path from A to B.

  // Create dummy position list for signature requirements of next method.
  const Eigen::Matrix<T, 3, 0> empty_position_list;

  // TODO(Mitiguy) One way to avoid memory allocation and speed this up is to
  // be clever and use the input argument as follows:
  // Eigen::Ref<MatrixX<T>> Js_w_WA_W = *Js_w_AB_E;
  // Also modify CalcJacobianAngularAndOrTranslationalVelocityInWorld() so
  // it can add or subtract to the Jacobian that is passed to it.
  Matrix3X<T> Js_w_WA_W(3, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
      with_respect_to, frame_A, empty_position_list, &Js_w_WA_W, nullptr);

  Matrix3X<T> Js_w_WB_W(3, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
      with_respect_to, frame_B, empty_position_list, &Js_w_WB_W, nullptr);

  const Frame<T>& frame_W = world_frame();
  if (frame_E.index() == frame_W.index()) {
    // Calculate B's angular velocity Jacobian in A, expressed in W.
    *Js_w_AB_E = Js_w_WB_W - Js_w_WA_W;  // This calculates Js_w_AB_W.
  } else {
    // When frame E is not the world frame:
    // 1. Calculate B's angular velocity Jacobian in A, expressed in W.
    // 2. Re-express that Jacobian in frame_E (rather than frame_W).
    const RotationMatrix<T> R_EW =
        CalcRelativeRotationMatrix(context, frame_E, frame_W);
    *Js_w_AB_E = R_EW * (Js_w_WB_W - Js_w_WA_W);
  }
}

template <typename T>
void MultibodyTree<T>::CalcJacobianTranslationalVelocityHelper(
    const systems::Context<T>& context,
    const JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Eigen::Ref<const Matrix3X<T>>& p_WoBi_W,
    const Frame<T>& frame_A,
    EigenPtr<MatrixX<T>> Js_v_ABi_W) const {
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                          num_positions() : num_velocities();
  const int num_points = p_WoBi_W.cols();
  DRAKE_THROW_UNLESS(num_points > 0);
  DRAKE_THROW_UNLESS(Js_v_ABi_W != nullptr);
  DRAKE_THROW_UNLESS(Js_v_ABi_W->rows() == 3 * num_points);
  DRAKE_THROW_UNLESS(Js_v_ABi_W->cols() == num_columns);

  // Bi's velocity in W can be calculated v_WBi = v_WAi + v_ABi, where
  // v_WBi is point Bi's translational velocity in world W,
  // v_WAi is point Ai's translational velocity in world W
  //         (where Ai is the point of A coincident with Bi),
  // v_ABi is point Bi's translational velocity in frame A.
  // Rearrange to calculate Bi's velocity in A as v_ABi = v_WBi - v_WAi.

  // TODO(Mitiguy): When performance becomes an issue, optimize this method by
  // only using the kinematics path from A to B.

  // Calculate each point Ai's translational velocity Jacobian in world W.
  MatrixX<T> Js_v_WAi_W(3 * num_points, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
    with_respect_to, frame_A, p_WoBi_W, nullptr, &Js_v_WAi_W);

  // Calculate each point Bi's translational velocity Jacobian in world W.
  MatrixX<T> Js_v_WBi_W(3 * num_points, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
    with_respect_to, frame_B, p_WoBi_W, nullptr, &Js_v_WBi_W);

  // Calculate each point Bi's translational velocity Jacobian in frame A,
  // expressed in world W.
  *Js_v_ABi_W = Js_v_WBi_W - Js_v_WAi_W;  // This calculates Js_v_ABi_W.
}

template <typename T>
void MultibodyTree<T>::CalcJacobianTranslationalVelocity(
    const systems::Context<T>& context,
    const JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Frame<T>& frame_F,
    const Eigen::Ref<const Matrix3X<T>>& p_FoBi_F,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E,
    EigenPtr<MatrixX<T>> Js_v_ABi_E) const {
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                          num_positions() : num_velocities();
  const int num_points = p_FoBi_F.cols();
  DRAKE_THROW_UNLESS(num_points > 0);
  DRAKE_THROW_UNLESS(p_FoBi_F.rows() == 3);
  DRAKE_THROW_UNLESS(Js_v_ABi_E != nullptr);
  DRAKE_THROW_UNLESS(Js_v_ABi_E->rows() == 3 * num_points);
  DRAKE_THROW_UNLESS(Js_v_ABi_E->cols() == num_columns);

  // If frame_F == frame_W (World), then just call helper method.
  // The helper method returns each point Bi's translational velocity Jacobian
  // in frame A, expressed in world W, i.e., helper method returns Js_v_ABi_W.
  const Frame<T>& frame_W = world_frame();
  if (&frame_F == &frame_W) {
    CalcJacobianTranslationalVelocityHelper(context, with_respect_to, frame_B,
                                            p_FoBi_F, frame_A, Js_v_ABi_E);
  } else {
    // If frame_F != frame_W, then for each point Bi, determine Bi's position
    // from Wo (World origin), expressed in world W and call helper method.
    Matrix3X<T> p_WoBi_W(3, num_points);
    CalcPointsPositions(context, frame_F, p_FoBi_F,  /* From frame F */
                        world_frame(), &p_WoBi_W);   /* To world frame W */
    CalcJacobianTranslationalVelocityHelper(context, with_respect_to, frame_B,
                                            p_WoBi_W, frame_A, Js_v_ABi_E);
  }

  // If frame_E is not the world frame, re-express Js_v_ABi_W in frame_E as
  // Js_v_ABi_E = R_EW * (Js_v_WBi_W - Js_v_WAi_W).
  if (&frame_E != &frame_W) {
    const RotationMatrix<T> R_EW =
        CalcRelativeRotationMatrix(context, frame_E, frame_W);
    // Extract the 3 x num_columns block that starts at row = 3 * i, column = 0.
    for (int i = 0;  i < num_points; ++i) {
      Js_v_ABi_E->template block<3, Eigen::Dynamic>(3 * i, 0, 3, num_columns) =
      R_EW *
      Js_v_ABi_E->template block<3, Eigen::Dynamic>(3 * i, 0, 3, num_columns);
    }
  }
}

template <typename T>
Vector6<T> MultibodyTree<T>::CalcBiasForJacobianSpatialVelocity(
    const systems::Context<T>& context,
    JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_F,
    const Eigen::Ref<const Vector3<T>>& p_FoFp_F,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E) const {
  // TODO(mitiguy) Allow with_respect_to be JacobianWrtVariable::kQDot.
  DRAKE_THROW_UNLESS(with_respect_to == JacobianWrtVariable::kV);

  // TODO(mitiguy) Allow frame_A to be something other than world frame W.
  DRAKE_THROW_UNLESS(&frame_A == &world_frame());

  // Consider a point Fp of (fixed/welded to) a frame F, where frame F is
  // regarded as fixed/welded to a body frame B.  Fp's spatial acceleration in
  // an arbitrary frame A can be written as:
  //   A_AFp = Js_V_AFp ⋅ ṡ + Abias_AFp,
  // where Js_V_AFp is Fp's spatial velocity Jacobian in frame A with respect to
  // "speeds" 𝑠, where 𝑠 is either
  // q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of generalized positions) or
  // v ≜ [v₁ ... vₖ]ᵀ (generalized velocities), and
  // Abias_AFp is the associated spatial acceleration bias term, equal to
  //   Abias_AFp = J̇s_V_AFp ⋅ s
  // Evident in this formula is Abias_AFp can contribute to A_AFp when s != 0
  // (e.g., there are centripetal, Coriolis, or gyroscopic terms).
  // One way to calculate this bias term is to observe that it is equal to the
  // spatial acceleration when ṡ = 0, that is:
  //   Abias_AFp = A_AFp(q, s, ṡ = 0)
  // After setting ṡ = 0, Fp's spatial acceleration bias can be calculated from
  // Bo's spatial acceleration bias with a shift operation as
  //   Abias_AFp = Abias_ABo.Shift(p_BoFp_A, w_AB_A)
  // where p_BoBp_A is the position from Bo (body frame B's origin) to point Fp
  // and w_AB_A is frame B's angular velocity in frame A, expressed in frame A.
  // See SpatialAcceleration::Shift() for details.
  // TODO(amcastro-tri): Consider caching each body's bias term Abias_ABo(q, s).
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Set ṡ = 0 and calculate point Bo's spatial acceleration bias in world W.
  std::vector<SpatialAcceleration<T>> Abias_WB_array(num_bodies());
  const VectorX<T> vdot = VectorX<T>::Zero(num_velocities());
  CalcSpatialAccelerationsFromVdot(context, pc, vc, vdot, &Abias_WB_array);

  // Extract point Bo's spatial acceleration bias from the large array.
  const Body<T>& body_B = frame_F.body();
  const SpatialAcceleration<T>& Abias_WBo = Abias_WB_array[body_B.node_index()];

  // Get transform from body B to frame F.
  const RigidTransform<T> X_BF = frame_F.GetFixedPoseInBodyFrame();

  // Shift spatial acceleration bias term from point Bo to point Fp.
  const SpatialAcceleration<T> Abias_WFp = CalcSpatialAccelerationBiasShift(
  context, frame_F, X_BF, p_FoFp_F, Abias_WBo, frame_E);

  return Abias_WFp.get_coeffs();
}

template <typename T>
void MultibodyTree<T>::CalcJacobianAngularAndOrTranslationalVelocityInWorld(
    const systems::Context<T>& context,
    JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_F,
    const Eigen::Ref<const Matrix3X<T>>& p_WoFpi_W,
    EigenPtr<Matrix3X<T>> Js_w_WF_W,
    EigenPtr<MatrixX<T>> Js_v_WFpi_W) const {
  // At least one of the Jacobian output terms must be nullptr.
  DRAKE_THROW_UNLESS(Js_w_WF_W != nullptr || Js_v_WFpi_W != nullptr);

  const bool is_wrt_qdot = (with_respect_to == JacobianWrtVariable::kQDot);
  const int num_columns = is_wrt_qdot ? num_positions() : num_velocities();
  const int num_points = p_WoFpi_W.cols();

  // If non-nullptr, check the proper size of the output Jacobian matrices and
  // initialize the contents to zero.
  if (Js_w_WF_W) {
    DRAKE_THROW_UNLESS(Js_w_WF_W->rows() == 3);
    DRAKE_THROW_UNLESS(Js_w_WF_W->cols() == num_columns);
    Js_w_WF_W->setZero();
  }
  if (Js_v_WFpi_W) {
    DRAKE_THROW_UNLESS(Js_v_WFpi_W->rows() == 3 * num_points);
    DRAKE_THROW_UNLESS(Js_v_WFpi_W->cols() == num_columns);
    Js_v_WFpi_W->setZero();
  }

  // Body to which frame_F is welded/attached.
  const Body<T>& body_F = frame_F.body();

  // Return zero Jacobians for bodies anchored to the world, since for anchored
  // bodies, w_wF = Js_w_WF * v = 0  and  v_WFpi = Js_v_WFpi * v = 0.
  if (body_F.index() == world_index()) return;

  // Form kinematic path from body_F to the world.
  std::vector<BodyNodeIndex> path_to_world;
  topology_.GetKinematicPathToWorld(body_F.node_index(), &path_to_world);
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  const std::vector<Vector6<T>>& H_PB_W_cache =
      tree_system_->EvalAcrossNodeGeometricJacobianExpressedInWorld(context);

  // A statically allocated matrix with a maximum number of rows and columns.
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 7> Nplus;

  // For all bodies in the kinematic path from the world to body_F, compute
  // each node's contribution to the Jacobians.
  // Skip the world (ilevel = 0).
  for (size_t ilevel = 1; ilevel < path_to_world.size(); ++ilevel) {
    const BodyNodeIndex body_node_index = path_to_world[ilevel];
    const BodyNode<T>& node = *body_nodes_[body_node_index];
    const BodyNodeTopology& node_topology = node.get_topology();
    const Mobilizer<T>& mobilizer = node.get_mobilizer();
    const int start_index_in_v = node_topology.mobilizer_velocities_start_in_v;
    const int start_index_in_q = node_topology.mobilizer_positions_start;
    const int mobilizer_num_velocities =
        node_topology.num_mobilizer_velocities;
    const int mobilizer_num_positions =
        node_topology.num_mobilizer_positions;

    // "Hinge matrix" H for across-node Jacobian.
    // Herein P designates the inboard (parent) body frame P.
    // B designates the current outboard body in this outward sweep.
    Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
        node.GetJacobianFromArray(H_PB_W_cache);

    // Aliases to angular and translational components in H_PB_W.
    const auto Hw_PB_W = H_PB_W.template topRows<3>();
    const auto Hv_PB_W = H_PB_W.template bottomRows<3>();

    const int start_index = is_wrt_qdot ? start_index_in_q : start_index_in_v;
    const int mobilizer_jacobian_ncols =
        is_wrt_qdot ? mobilizer_num_positions : mobilizer_num_velocities;

    // Mapping defined by v = N⁺(q)⋅q̇.
    if (is_wrt_qdot) {
      // TODO(amcastro-tri): consider using an operator version instead only
      // if/when the computational cost of multiplying with Nplus from the
      // right becomes a bottleneck.
      // TODO(amcastro-tri): cache Nplus to avoid memory allocations.
      Nplus.resize(mobilizer_num_velocities, mobilizer_num_positions);
      mobilizer.CalcNplusMatrix(context, &Nplus);
    } else {
      Nplus.setIdentity(mobilizer_num_velocities, mobilizer_num_velocities);
    }

    // The Jacobian angular velocity term is the same for all points Fpi since
    // all these are points of (fixed/welded to) the same body_F.
    if (Js_w_WF_W) {
      // Get memory address in the output Jacobian angular velocity Js_w_WF_W
      // corresponding to the contribution of the mobilities in level ilevel.
      auto Js_w_PB_W = Js_w_WF_W->block(0, start_index, 3,
                                        mobilizer_jacobian_ncols);
      Js_w_PB_W = Hw_PB_W * Nplus;
    }

    if (Js_v_WFpi_W) {
      // Get memory address in the output block Jacobian translational velocity
      // Js_v_PFpi_W corresponding to the contribution of the mobilities in
      // level ilevel.  This address corresponds to point Fpi's Jacobian
      // translational velocity in the inboard (parent) body frame P, expressed
      // in world frame W.  That is, v_PFpi_W = Js_v_PFpi_W * v(B), where v(B)
      // are the mobilities that correspond to the current node.
      auto Js_v_PFpi_W = Js_v_WFpi_W->block(0, start_index, 3 * num_points,
                                            mobilizer_jacobian_ncols);

      // Position from Wo (world origin) to Bo (origin of body associated with
      // node at level ilevel), expressed in world frame W.
      const Vector3<T>& p_WoBo = pc.get_X_WB(node.index()).translation();

      for (int ipoint = 0; ipoint < num_points; ++ipoint) {
        // Position from Wo to Fp (ith point of Fpi), expressed in world W.
        const Vector3<T>& p_WoFp = p_WoFpi_W.col(ipoint);

        // Position from Bo to Fp, expressed in world W.
        const Vector3<T> p_BoFp_W = p_WoFp - p_WoBo;

        // Point Fp's Jacobian translational velocity is placed in the output
        // memory block in the same order input points Fpi are listed on input.
        // Get a mutable alias into Js_v_PFpi_W for the Jacobian translational
        // velocity term for the currently indexed (ipoint) point.
        const int ipoint_row = 3 * ipoint;
        auto Hv_PFpi_W =
            Js_v_PFpi_W.block(ipoint_row, 0, 3, mobilizer_jacobian_ncols);

        // Now "shift" H_PB_W to H_PFqi_W one column at a time.
        // Reminder: frame_F is fixed/welded to body_F so its angular velocity
        // in world W is the same as body_F's angular velocity in W.
        Hv_PFpi_W = (Hv_PB_W + Hw_PB_W.colwise().cross(p_BoFp_W)) * Nplus;
      }  // ipoint.
    }
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

  T potential_energy = 0.0;
  // Add contributions from force elements.
  for (const auto& force_element : owned_force_elements_) {
    potential_energy += force_element->CalcPotentialEnergy(context, pc);
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

  T conservative_power = 0.0;
  // Add contributions from force elements.
  for (const auto& force_element : owned_force_elements_) {
    conservative_power +=
        force_element->CalcConservativePower(context, pc, vc);
  }
  return conservative_power;
}

template <typename T>
void MultibodyTree<T>::ThrowIfFinalized(const char* source_method) const {
  if (topology_is_valid()) {
    throw std::logic_error(
        "Post-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; calls to this method must happen before Finalize().");
  }
}

template <typename T>
void MultibodyTree<T>::ThrowIfNotFinalized(const char* source_method) const {
  if (!topology_is_valid()) {
    throw std::logic_error(
        "Pre-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; you must call Finalize() first.");
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyInertiaCache(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    ArticulatedBodyInertiaCache<T>* abc) const {
  DRAKE_DEMAND(abc != nullptr);

  const std::vector<Vector6<T>>& H_PB_W_cache =
      tree_system_->EvalAcrossNodeGeometricJacobianExpressedInWorld(context);

  // Perform tip-to-base recursion, skipping the world.
  for (int depth = tree_height() - 1; depth > 0; depth--) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      // Get hinge mapping matrix.
      const MatrixUpTo6<T> H_PB_W = node.GetJacobianFromArray(H_PB_W_cache);

      node.CalcArticulatedBodyInertiaCache_TipToBase(
          context, pc, H_PB_W, abc);
    }
  }
}

template <typename T>
MatrixX<double> MultibodyTree<T>::MakeStateSelectorMatrix(
    const std::vector<JointIndex>& user_to_joint_index_map) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  // We create a set in order to verify that joint indexes appear only once.
  std::unordered_set<JointIndex> already_selected_joints;
  for (const auto& joint_index : user_to_joint_index_map) {
    const bool inserted = already_selected_joints.insert(joint_index).second;
    if (!inserted) {
      throw std::logic_error(
          "Joint named '" + get_joint(joint_index).name() +
              "' is repeated multiple times.");
    }
  }

  // Determine the size of the vector of "selected" states xₛ.
  int num_selected_positions = 0;
  int num_selected_velocities = 0;
  for (JointIndex joint_index : user_to_joint_index_map) {
    num_selected_positions += get_joint(joint_index).num_positions();
    num_selected_velocities += get_joint(joint_index).num_velocities();
  }
  const int num_selected_states =
      num_selected_positions + num_selected_velocities;

  // With state x of size n and selected state xₛ of size nₛ, Sx has size
  // nₛ x n so that xₛ = Sx⋅x.
  MatrixX<double> Sx =
      MatrixX<double>::Zero(num_selected_states, num_states());

  const int nq = num_positions();
  // We place all selected positions first, followed by all the selected
  // velocities, as in the original state x.
  int selected_positions_index = 0;
  int selected_velocities_index = num_selected_positions;
  for (JointIndex joint_index : user_to_joint_index_map) {
    const auto& joint = get_joint(joint_index);

    const int pos_start = joint.position_start();
    const int num_pos = joint.num_positions();
    const int vel_start = joint.velocity_start();
    const int num_vel = joint.num_velocities();

    Sx.block(selected_positions_index, pos_start, num_pos, num_pos) =
        MatrixX<double>::Identity(num_pos, num_pos);

    Sx.block(selected_velocities_index, nq + vel_start, num_vel, num_vel) =
        MatrixX<double>::Identity(num_vel, num_vel);

    selected_positions_index += num_pos;
    selected_velocities_index += num_vel;
  }

  return Sx;
}

template <typename T>
MatrixX<double> MultibodyTree<T>::MakeStateSelectorMatrixFromJointNames(
    const std::vector<std::string>& selected_joints) const {
  std::vector<JointIndex> selected_joints_indexes;
  for (const auto& joint_name : selected_joints) {
    selected_joints_indexes.push_back(GetJointByName(joint_name).index());
  }
  return MakeStateSelectorMatrix(selected_joints_indexes);
}

template <typename T>
MatrixX<double> MultibodyTree<T>::MakeActuatorSelectorMatrix(
    const std::vector<JointActuatorIndex>& user_to_actuator_index_map) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  const int num_selected_actuators = user_to_actuator_index_map.size();

  // The actuation selector matrix maps the vector of "selected" actuators to
  // the full vector of actuators: u = Sᵤ⋅uₛ.
  MatrixX<double> Su =
      MatrixX<double>::Zero(num_actuated_dofs(), num_selected_actuators);
  int user_index = 0;
  for (JointActuatorIndex actuator_index : user_to_actuator_index_map) {
    Su(actuator_index, user_index) = 1.0;
    ++user_index;
  }

  return Su;
}

template <typename T>
MatrixX<double> MultibodyTree<T>::MakeActuatorSelectorMatrix(
    const std::vector<JointIndex>& user_to_joint_index_map) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  std::vector<JointActuatorIndex> joint_to_actuator_index(num_joints());
  for (JointActuatorIndex actuator_index(0);
       actuator_index < num_actuators(); ++actuator_index) {
    const auto& actuator = get_joint_actuator(actuator_index);
    joint_to_actuator_index[actuator.joint().index()] = actuator_index;
  }

  // Build a list of actuators in the order given by user_to_joint_index_map,
  // which must contain actuated joints. We verify this.
  std::vector<JointActuatorIndex> user_to_actuator_index_map;
  for (JointIndex joint_index : user_to_joint_index_map) {
    const auto& joint = get_joint(joint_index);

    // If the map has an invalid index then this joint does not have an
    // actuator.
    if (!joint_to_actuator_index[joint_index].is_valid()) {
      throw std::logic_error(
          "Joint '" + joint.name() + "' does not have an actuator.");
    }

    user_to_actuator_index_map.push_back(joint_to_actuator_index[joint_index]);
  }

  return MakeActuatorSelectorMatrix(user_to_actuator_index_map);
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetPositionLowerLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd q_lower = Eigen::VectorXd::Constant(
      num_positions(), -std::numeric_limits<double>::infinity());
  for (JointIndex i{0}; i < num_joints(); ++i) {
    const auto& joint = get_joint(i);
    q_lower.segment(joint.position_start(), joint.num_positions()) =
        joint.position_lower_limits();
  }
  return q_lower;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetPositionUpperLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd q_upper = Eigen::VectorXd::Constant(
      num_positions(), std::numeric_limits<double>::infinity());
  for (JointIndex i{0}; i < num_joints(); ++i) {
    const auto& joint = get_joint(i);
    q_upper.segment(joint.position_start(), joint.num_positions()) =
        joint.position_upper_limits();
  }
  return q_upper;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetVelocityLowerLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd v_lower = Eigen::VectorXd::Constant(
      num_velocities(), -std::numeric_limits<double>::infinity());
  for (JointIndex i{0}; i < num_joints(); ++i) {
    const auto& joint = get_joint(i);
    v_lower.segment(joint.velocity_start(), joint.num_velocities()) =
        joint.velocity_lower_limits();
  }
  return v_lower;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetVelocityUpperLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd v_upper = Eigen::VectorXd::Constant(
      num_velocities(), std::numeric_limits<double>::infinity());
  for (JointIndex i{0}; i < num_joints(); ++i) {
    const auto& joint = get_joint(i);
    v_upper.segment(joint.velocity_start(), joint.num_velocities()) =
        joint.velocity_upper_limits();
  }
  return v_upper;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetAccelerationLowerLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd vd_lower = Eigen::VectorXd::Constant(
      num_velocities(), -std::numeric_limits<double>::infinity());
  for (JointIndex i{0}; i < num_joints(); ++i) {
    const auto& joint = get_joint(i);
    vd_lower.segment(joint.velocity_start(), joint.num_velocities()) =
        joint.acceleration_lower_limits();
  }
  return vd_lower;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetAccelerationUpperLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd vd_upper = Eigen::VectorXd::Constant(
      num_velocities(), std::numeric_limits<double>::infinity());
  for (JointIndex i{0}; i < num_joints(); ++i) {
    const auto& joint = get_joint(i);
    vd_upper.segment(joint.velocity_start(), joint.num_velocities()) =
        joint.acceleration_upper_limits();
  }
  return vd_upper;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MultibodyTree)
