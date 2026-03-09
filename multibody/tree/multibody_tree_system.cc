#include "drake/multibody/tree/multibody_tree_system.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/tree/multibody_tree-inl.h"

namespace drake {
using systems::BasicVector;
using systems::Context;
using systems::DependencyTicket;
using systems::LeafSystem;
using systems::Parameters;
using systems::State;

namespace multibody {
namespace internal {

template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(
    std::unique_ptr<MultibodyTree<T>> tree, bool is_discrete)
    : MultibodyTreeSystem(systems::SystemTypeTag<MultibodyTreeSystem>{},
                          false,  // Null tree is not allowed here.
                          std::move(tree), is_discrete) {}

template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(bool is_discrete)
    : MultibodyTreeSystem(systems::SystemTypeTag<MultibodyTreeSystem>{},
                          true,  // Null tree is OK.
                          nullptr, is_discrete) {}

template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(
    systems::SystemScalarConverter converter,
    std::unique_ptr<MultibodyTree<T>> tree, bool is_discrete)
    : MultibodyTreeSystem(std::move(converter),
                          true,  // Null tree is OK.
                          std::move(tree), is_discrete) {}

template <typename T>
template <typename U>
MultibodyTreeSystem<T>::MultibodyTreeSystem(const MultibodyTreeSystem<U>& other)
    : MultibodyTreeSystem(systems::SystemTypeTag<MultibodyTreeSystem>{},
                          false,  // Null tree isn't allowed (or possible).
                          other.internal_tree().template CloneToScalar<T>(),
                          other.is_discrete()) {}

// This is the one true constructor.
template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(
    systems::SystemScalarConverter converter, bool null_tree_is_ok,
    std::unique_ptr<MultibodyTree<T>> tree, bool is_discrete)
    : LeafSystem<T>(std::move(converter)), is_discrete_(is_discrete) {
  if (tree == nullptr) {
    if (!null_tree_is_ok) {
      throw std::logic_error(
          "MultibodyTreeSystem(): the supplied MultibodyTree was null.");
    }
    tree_ = std::make_unique<MultibodyTree<T>>();
    tree_->set_tree_system(this);
    // Don't finalize.
    return;
  }

  // We were given an already-built tree.
  tree_ = std::move(tree);
  tree_->set_tree_system(this);
  Finalize();
}

template <typename T>
void MultibodyTreeSystem<T>::SetDefaultParameters(
    const Context<T>& context, Parameters<T>* parameters) const {
  LeafSystem<T>::SetDefaultParameters(context, parameters);

  if (!already_finalized_) {
    throw std::logic_error(
        "MultibodyPlant cannot SetDefaultParameters or CreateDefaultContext "
        "until after MultibodyPlant::Finalize() has been called.");
  }

  // Mobilizers.
  for (MobodIndex mobilizer_index(0); mobilizer_index < tree_->num_mobilizers();
       ++mobilizer_index) {
    internal_tree()
        .get_mobilizer(mobilizer_index)
        .SetDefaultParameters(parameters);
  }
  // Joints.
  for (JointIndex joint_index : tree_->GetJointIndices()) {
    internal_tree().get_joint(joint_index).SetDefaultParameters(parameters);
  }
  // JointActuators.
  for (JointActuatorIndex joint_actuator_index :
       tree_->GetJointActuatorIndices()) {
    internal_tree()
        .get_joint_actuator(joint_actuator_index)
        .SetDefaultParameters(parameters);
  }
  // Bodies.
  for (BodyIndex body_index(0); body_index < tree_->num_bodies();
       ++body_index) {
    internal_tree().get_body(body_index).SetDefaultParameters(parameters);
  }
  // Frames.
  for (FrameIndex frame_index(0); frame_index < tree_->num_frames();
       ++frame_index) {
    internal_tree().get_frame(frame_index).SetDefaultParameters(parameters);
  }
  // Force Elements.
  for (ForceElementIndex force_element_index(0);
       force_element_index < tree_->num_force_elements();
       ++force_element_index) {
    internal_tree()
        .get_force_element(force_element_index)
        .SetDefaultParameters(parameters);
  }
}

template <typename T>
void MultibodyTreeSystem<T>::SetDefaultState(const Context<T>& context,
                                             State<T>* state) const {
  LeafSystem<T>::SetDefaultState(context, state);
  tree_->SetDefaultState(context, state);
}

template <typename T>
MultibodyTreeSystem<T>::~MultibodyTreeSystem() = default;

template <typename T>
MultibodyTree<T>& MultibodyTreeSystem<T>::mutable_tree() {
  DRAKE_DEMAND(tree_ != nullptr);
  return *tree_;
}

template <typename T>
void MultibodyTreeSystem<T>::DeclareMultibodyElementParameters(
    int* num_frame_body_pose_slots_needed) {
  // Mobilizers.
  for (MobodIndex mobilizer_index(0); mobilizer_index < tree_->num_mobilizers();
       ++mobilizer_index) {
    mutable_tree()
        .get_mutable_mobilizer(mobilizer_index)
        .DeclareParameters(this);
  }
  // Joints.
  for (JointIndex joint_index : tree_->GetJointIndices()) {
    mutable_tree().get_mutable_joint(joint_index).DeclareParameters(this);
  }
  // JointActuators.
  for (JointActuatorIndex joint_actuator_index :
       tree_->GetJointActuatorIndices()) {
    mutable_tree()
        .get_mutable_joint_actuator(joint_actuator_index)
        .DeclareParameters(this);
  }
  // Bodies.
  for (BodyIndex body_index(0); body_index < tree_->num_bodies();
       ++body_index) {
    mutable_tree().get_mutable_body(body_index).DeclareParameters(this);
  }
  // Frames.
  *num_frame_body_pose_slots_needed = 1;  // 0th is for an identity transform.
  for (FrameIndex frame_index(0); frame_index < tree_->num_frames();
       ++frame_index) {
    Frame<T>& frame = mutable_tree().get_mutable_frame(frame_index);
    frame.DeclareParameters(this);
    // This is where the extracted, reformatted, and composed body pose X_BF for
    // this Frame will be stored in the frame body poses cache entry.
    frame.set_body_pose_index_in_cache(
        frame.is_body_frame() ? 0 : (*num_frame_body_pose_slots_needed)++);
  }
  // Force Elements.
  for (ForceElementIndex force_element_index(0);
       force_element_index < tree_->num_force_elements();
       ++force_element_index) {
    mutable_tree()
        .get_mutable_force_element(force_element_index)
        .DeclareParameters(this);
  }
}

template <typename T>
void MultibodyTreeSystem<T>::Finalize() {
  if (already_finalized_) {
    throw std::logic_error(
        "MultibodyTreeSystem::Finalize(): repeated calls not allowed.");
  }
  if (!tree_->is_finalized()) {
    tree_->Finalize();
  }

  int num_frame_body_poses_needed{-1};
  DeclareMultibodyElementParameters(&num_frame_body_poses_needed);
  DRAKE_DEMAND(num_frame_body_poses_needed > 0);  // Always at least 1.

  // Declare state.
  if (is_discrete_) {
    tree_->set_discrete_state_index(
        this->DeclareDiscreteState(tree_->num_states()));
  } else {
    this->DeclareContinuousState(BasicVector<T>(tree_->num_states()),
                                 tree_->num_positions(),
                                 tree_->num_velocities(), 0 /* num_z */);
  }

  // Declare cache entries dependent only on parameters.

  // TODO(joemasterjohn): Create more granular parameter tickets for finer
  //  control over cache dependencies on parameters. For example,
  //  all_rigid_body_parameters, etc.

  cache_indexes_.reflected_inertia =
      this->DeclareCacheEntry(std::string("reflected inertia"),
                              VectorX<T>(internal_tree().num_velocities()),
                              &MultibodyTreeSystem<T>::CalcReflectedInertia,
                              {this->all_parameters_ticket()})
          .cache_index();

  cache_indexes_.joint_damping =
      this->DeclareCacheEntry(std::string("joint damping"),
                              VectorX<T>(internal_tree().num_velocities()),
                              &MultibodyTreeSystem<T>::CalcJointDamping,
                              {this->all_parameters_ticket()})
          .cache_index();

  cache_indexes_.frame_body_poses =
      this->DeclareCacheEntry(
              std::string("frame pose in body frame"),
              FrameBodyPoseCache<T>(internal_tree().num_mobods(),
                                    num_frame_body_poses_needed),
              &MultibodyTreeSystem<T>::CalcFrameBodyPoses,
              {this->all_parameters_ticket()})
          .cache_index();

  const DependencyTicket position_ticket =
      is_discrete_ ? this->xd_ticket() : this->q_ticket();
  const DependencyTicket velocity_ticket =
      is_discrete_ ? this->xd_ticket() : this->v_ticket();

  // Declare cache entries dependent on positions (and parameters).

  // Allocate position cache.
  cache_indexes_.position_kinematics =
      this->DeclareCacheEntry(
              std::string("position kinematics"),
              PositionKinematicsCache<T>(internal_tree().forest()),
              &MultibodyTreeSystem<T>::CalcPositionKinematicsCache,
              {position_ticket, this->all_parameters_ticket()})
          .cache_index();

  // Allocate system Jacobian cache.
  cache_indexes_.block_system_jacobian =
      this->DeclareCacheEntry(
              std::string("system Jacobian"),
              BlockSystemJacobianCache<T>(internal_tree().forest()),
              &MultibodyTreeSystem<T>::CalcBlockSystemJacobianCache,
              {position_kinematics_cache_entry().ticket()})
          .cache_index();

  // Allocate cache entry to store spatial inertia M_B_W(q) for each body.
  cache_indexes_.spatial_inertia_in_world =
      this->DeclareCacheEntry(
              std::string("spatial inertia in world (M_B_W)"),
              std::vector<SpatialInertia<T>>(internal_tree().num_bodies(),
                                             SpatialInertia<T>::NaN()),
              &MultibodyTreeSystem<T>::CalcSpatialInertiasInWorld,
              {position_kinematics_cache_entry().ticket()})
          .cache_index();

  // Allocate cache entry for composite-body inertias K_BBo_W(q) for each body.
  cache_indexes_.composite_body_inertia_in_world =
      this->DeclareCacheEntry(
              std::string("composite body inertia in world (K_BBo_W)"),
              std::vector<SpatialInertia<T>>(internal_tree().num_bodies(),
                                             SpatialInertia<T>::NaN()),
              &MultibodyTreeSystem<T>::CalcCompositeBodyInertiasInWorld,
              {position_kinematics_cache_entry().ticket()})
          .cache_index();

  // Declare cache entries dependent on velocities (and parameters & positions).

  // Allocate velocity cache.
  cache_indexes_.velocity_kinematics =
      this->DeclareCacheEntry(
              std::string("velocity kinematics"),
              VelocityKinematicsCache<T>(internal_tree().forest()),
              &MultibodyTreeSystem<T>::CalcVelocityKinematicsCache,
              {position_ticket, velocity_ticket, this->all_parameters_ticket()})
          .cache_index();

  // Allocate cache entry to store Fb_Bo_W(q, v) for each body.
  cache_indexes_.dynamic_bias =
      this->DeclareCacheEntry(
              std::string("dynamic bias (Fb_Bo_W)"),
              std::vector<SpatialForce<T>>(internal_tree().num_bodies()),
              &MultibodyTreeSystem<T>::CalcDynamicBiasForces,
              // The computation of Fb_Bo_W(q, v) requires updated values of
              // M_Bo_W(q) and V_WB(q, v). We make these prerequisites explicit.
              // Another alternative would be to state the dependence on q and
              // v. However this option is not optimal until #9171 gets
              // resolved.
              {this->cache_entry_ticket(
                   cache_indexes_.spatial_inertia_in_world),
               velocity_kinematics_cache_entry().ticket()})
          .cache_index();

  // Declare cache entry for H_PB_W(q).
  // The type of this cache value is std::vector<Vector6<T>>.
  cache_indexes_.across_node_jacobians =
      this->DeclareCacheEntry(
              std::string("H_PB_W(q)"),
              std::vector<Vector6<T>>(internal_tree().num_velocities()),
              &MultibodyTreeSystem<
                  T>::CalcAcrossNodeJacobianWrtVExpressedInWorld,
              {position_kinematics_cache_entry().ticket()})
          .cache_index();

  // Allocate articulated body inertia cache.
  cache_indexes_.abi_cache_index =
      this->DeclareCacheEntry(
              std::string("Articulated Body Inertia"),
              ArticulatedBodyInertiaCache<T>(internal_tree().forest()),
              &MultibodyTreeSystem<T>::CalcArticulatedBodyInertiaCache,
              {position_ticket, this->all_parameters_ticket()})
          .cache_index();

  cache_indexes_.spatial_acceleration_bias =
      this->DeclareCacheEntry(
              std::string("spatial acceleration bias (Ab_WB)"),
              std::vector<SpatialAcceleration<T>>(internal_tree().num_bodies()),
              &MultibodyTreeSystem<T>::CalcSpatialAccelerationBias,
              {position_ticket, velocity_ticket, this->all_parameters_ticket()})
          .cache_index();

  cache_indexes_.articulated_body_force_bias =
      this->DeclareCacheEntry(
              std::string("ABI force bias cache (Zb_Bo_W)"),
              std::vector<SpatialForce<T>>(internal_tree().num_bodies()),
              &MultibodyTreeSystem<T>::CalcArticulatedBodyForceBias,
              {position_ticket, velocity_ticket, this->all_parameters_ticket()})
          .cache_index();

  // Declare cache entries dependent on forces and accelerations (and
  // parameters, positions, and velocities).

  // Forces, and thus accelerations, are functions not only of state but also
  // inputs. In addition, the forces and accelerations can have extra
  // user-injected dependencies through MultibodyElement and
  // ForceDensityFieldBase, so we must include tickets that users might depend
  // on.
  const std::set<DependencyTicket> force_and_acceleration_prereqs = {
      position_ticket,
      velocity_ticket,
      this->all_parameters_ticket(),
      this->time_ticket(),
      this->accuracy_ticket(),
      this->all_input_ports_ticket()};

  // Articulated Body Algorithm (ABA) force cache.
  const auto& articulated_body_forces_cache_entry = this->DeclareCacheEntry(
      std::string("ABA force cache"),
      ArticulatedBodyForceCache<T>(internal_tree().forest()),
      &MultibodyTreeSystem<T>::CalcArticulatedBodyForceCache,
      force_and_acceleration_prereqs);
  cache_indexes_.articulated_body_forces =
      articulated_body_forces_cache_entry.cache_index();

  // Acceleration kinematics must be calculated for forward dynamics,
  // regardless of whether that is done in continuous mode (as the last pass
  // of ABA) or in discrete mode (explicitly by MultibodyPlant).
  cache_indexes_.acceleration_kinematics =
      this->DeclareCacheEntry(
              std::string("Accelerations"),
              AccelerationKinematicsCache<T>(internal_tree().forest()),
              &MultibodyTreeSystem<T>::CalcForwardDynamics,
              force_and_acceleration_prereqs)
          .cache_index();

  already_finalized_ = true;
}

template <typename T>
void MultibodyTreeSystem<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // No derivatives to compute if state is discrete.
  if (is_discrete()) return;
  // No derivatives to compute if state is empty. (Will segfault otherwise.)
  // TODO(amcastro-tri): When nv = 0 we should not declare state or cache
  // entries at all and the system framework will never call this.
  if (internal_tree().num_states() == 0) return;

  const VectorX<T>& x = dynamic_cast<const systems::BasicVector<T>&>(
                            context.get_continuous_state_vector())
                            .value();
  const auto v = x.bottomRows(internal_tree().num_velocities());

  const VectorX<T>& vdot = this->EvalForwardDynamics(context).get_vdot();

  // TODO(sherm1) Heap allocation here. Get rid of it.
  VectorX<T> xdot(internal_tree().num_states());
  VectorX<T> qdot(internal_tree().num_positions());
  internal_tree().MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template <typename T>
void MultibodyTreeSystem<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    systems::VectorBase<T>* generalized_velocity) const {
  const int nq = internal_tree().num_positions();
  const int nv = internal_tree().num_velocities();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_DEMAND(generalized_velocity != nullptr);
  DRAKE_DEMAND(generalized_velocity->size() == nv);

  // TODO(sherm1) Heap allocation. Make this go away.
  VectorX<T> v(nv);
  internal_tree().MapQDotToVelocity(context, qdot, &v);
  generalized_velocity->SetFromVector(v);
}

template <typename T>
void MultibodyTreeSystem<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    systems::VectorBase<T>* positions_derivative) const {
  const int nq = internal_tree().num_positions();
  const int nv = internal_tree().num_velocities();

  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);

  // TODO(sherm1) Heap allocation. Make this go away.
  VectorX<T> qdot(nq);
  internal_tree().MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template <typename T>
void MultibodyTreeSystem<T>::DoCalcImplicitTimeDerivativesResidual(
    const systems::Context<T>& context,
    const systems::ContinuousState<T>& proposed_derivatives,
    EigenPtr<VectorX<T>> residual) const {
  // No residuals to compute if state is discrete.
  if (is_discrete()) return;

  DRAKE_DEMAND(residual->size() ==
               this->implicit_time_derivatives_residual_size());

  const int nq = internal_tree().num_positions();
  const int nv = internal_tree().num_velocities();

  // TODO(sherm1) Heap allocation here. Get rid of it.
  MultibodyForces<T> forces(*this);

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Compute forces applied by force elements. Note that this resets forces
  // to empty so must come first.
  internal_tree().CalcForceElementsContribution(context, pc, vc, &forces);

  // Compute forces applied by the derived class (likely MultibodyPlant).
  AddInForcesContinuous(context, &forces);

  // TODO(sherm1) This dynamic_cast is likely too expensive -- replace with
  //              static_cast in Release builds.
  const VectorX<T>& qvdot_proposed =
      dynamic_cast<const systems::BasicVector<T>&>(
          proposed_derivatives.get_vector())
          .value();
  DRAKE_ASSERT(qvdot_proposed.size() == nq + nv);

  auto qdot_residual = residual->head(nq);
  // N(q)⋅v
  internal_tree().MapVelocityToQDot(
      context, internal_tree().get_velocities(context), &qdot_residual);
  // q̇_proposed - N(q)⋅v
  qdot_residual = qvdot_proposed.head(nq) - qdot_residual;
  // InverseDynamics(context, v_proposed)
  residual->tail(nv) = internal_tree().CalcInverseDynamics(
      context, qvdot_proposed.tail(nv), forces);
}

template <typename T>
void MultibodyTreeSystem<T>::CalcArticulatedBodyForceCache(
    const systems::Context<T>& context,
    ArticulatedBodyForceCache<T>* aba_force_cache) const {
  DRAKE_DEMAND(aba_force_cache != nullptr);

  // TODO(sherm1) Heap allocation here. Get rid of it.
  MultibodyForces<T> forces(*this);

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Compute forces applied by force elements. Note that this resets forces
  // to empty so must come first.
  internal_tree().CalcForceElementsContribution(context, pc, vc, &forces);

  // Compute forces applied by the derived class (likely MultibodyPlant).
  AddInForcesContinuous(context, &forces);

  // Perform the tip-to-base pass to compute the force bias terms needed by ABA.
  internal_tree().CalcArticulatedBodyForceCache(context, forces,
                                                aba_force_cache);
}

template <typename T>
void MultibodyTreeSystem<T>::CalcForwardDynamicsContinuous(
    const systems::Context<T>& context,
    AccelerationKinematicsCache<T>* ac) const {
  DRAKE_DEMAND(ac != nullptr);

  // Collect forces from all sources and propagate tip-to-base.
  const ArticulatedBodyForceCache<T>& aba_force_cache =
      EvalArticulatedBodyForceCache(context);

  // Perform the last base-to-tip pass to compute accelerations using the O(n)
  // ABA.
  internal_tree().CalcArticulatedBodyAccelerations(context, aba_force_cache,
                                                   ac);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::MultibodyTreeSystem);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::MultibodyTreeSystemElementAttorney);
