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
                          std::move(tree), is_discrete,
                          0 /* num_misc_continuous_states */) {}

template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(bool is_discrete)
    : MultibodyTreeSystem(systems::SystemTypeTag<MultibodyTreeSystem>{},
                          true,  // Null tree is OK.
                          nullptr, is_discrete,
                          0 /* num_misc_continuous_states */) {}

template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(
    systems::SystemScalarConverter converter,
    std::unique_ptr<MultibodyTree<T>> tree, bool is_discrete,
    int num_misc_continuous_states)
    : MultibodyTreeSystem(std::move(converter),
                          true,  // Null tree is OK.
                          std::move(tree), is_discrete,
                          num_misc_continuous_states) {}

template <typename T>
template <typename U>
MultibodyTreeSystem<T>::MultibodyTreeSystem(const MultibodyTreeSystem<U>& other)
    : MultibodyTreeSystem(systems::SystemTypeTag<MultibodyTreeSystem>{},
                          false,  // Null tree isn't allowed (or possible).
                          other.internal_tree().template CloneToScalar<T>(),
                          other.is_discrete(),
                          other.num_misc_continuous_states_) {}

// This is the one true constructor.
template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(
    systems::SystemScalarConverter converter, bool null_tree_is_ok,
    std::unique_ptr<MultibodyTree<T>> tree, bool is_discrete,
    int num_misc_continuous_states)
    : LeafSystem<T>(std::move(converter)), is_discrete_(is_discrete) {
  DRAKE_THROW_UNLESS(num_misc_continuous_states >= 0);
  DRAKE_THROW_UNLESS(!is_discrete_ || num_misc_continuous_states == 0);

  num_misc_continuous_states_ = num_misc_continuous_states;

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
void MultibodyTreeSystem<T>::DoCalcMiscDerivatives(
    const systems::Context<T>&, systems::VectorBase<T>*) const {
  throw std::logic_error(
      "MultibodyTreeSystem::DoCalcMiscDerivatives(): derived class must "
      "override this method to compute derivatives for the misc continuous "
      "state.");
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
  // Links.
  for (LinkIndex link_index(0); link_index < tree_->num_links(); ++link_index) {
    internal_tree().get_link(link_index).SetDefaultParameters(parameters);
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
int MultibodyTreeSystem<T>::DeclareMiscContinuousState(
    int num_state_variables) {
  if (already_finalized_) {
    throw std::logic_error(
        "DeclareMiscContinuousState(): calls after Finalize() are not "
        "allowed.");
  }
  DRAKE_THROW_UNLESS(num_state_variables >= 0);
  if (is_discrete_ && num_state_variables > 0) {
    throw std::logic_error(
        "DeclareMiscContinuousState(): cannot declare continuous state for a "
        "discrete MultibodyTreeSystem.");
  }
  const int result = num_misc_continuous_states_;
  num_misc_continuous_states_ += num_state_variables;
  return result;
}

template <typename T>
void MultibodyTreeSystem<T>::DeclareMultibodyElementParameters() {
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
  // Links.
  for (LinkIndex link_index(0); link_index < tree_->num_links(); ++link_index) {
    mutable_tree().get_mutable_link(link_index).DeclareParameters(this);
  }
  // Frames.
  for (FrameIndex frame_index(0); frame_index < tree_->num_frames();
       ++frame_index) {
    Frame<T>& frame = mutable_tree().get_mutable_frame(frame_index);
    frame.DeclareParameters(this);
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

  DeclareMultibodyElementParameters();

  // Declare state.
  if (is_discrete_) {
    tree_->set_discrete_state_index(
        this->DeclareDiscreteState(tree_->num_states()));
  } else {
    const int num_z = num_misc_continuous_states_;
    BasicVector<T> model_continuous_state(tree_->num_states() + num_z);
    model_continuous_state.get_mutable_value().tail(num_z).setZero();
    this->DeclareContinuousState(model_continuous_state, tree_->num_positions(),
                                 tree_->num_velocities(), num_z);
  }

  // Declare cache entries dependent only on parameters.

  // TODO(joemasterjohn): Create more granular parameter tickets for finer
  //  control over cache dependencies on parameters. For example,
  //  all_rigid_body_parameters, etc.

  cache_indexes_.reflected_inertia =
      this->DeclareCacheEntry(std::string("reflected inertia"),
                              VectorX<T>(tree_->num_velocities()),
                              &MultibodyTreeSystem<T>::CalcReflectedInertia,
                              {this->all_parameters_ticket()})
          .cache_index();

  cache_indexes_.joint_damping =
      this->DeclareCacheEntry(std::string("joint damping"),
                              VectorX<T>(tree_->num_velocities()),
                              &MultibodyTreeSystem<T>::CalcJointDamping,
                              {this->all_parameters_ticket()})
          .cache_index();

  cache_indexes_.frame_body_poses =
      this->DeclareCacheEntry(
              std::string("frame pose in link and body frames"),
              FrameBodyPoseCache<T>(tree_->num_links(), tree_->num_frames(),
                                    tree_->num_mobods()),
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

  // Allocate cache entry to store spatial inertia M_B_W(q) for each mobod.
  cache_indexes_.spatial_inertia_in_world =
      this->DeclareCacheEntry(
              std::string("mobod spatial inertia in world (M_B_W)"),
              std::vector<SpatialInertia<T>>(internal_tree().num_mobods(),
                                             SpatialInertia<T>::NaN()),
              &MultibodyTreeSystem<T>::CalcSpatialInertiasInWorld,
              {position_kinematics_cache_entry().ticket()})
          .cache_index();

  // Allocate cache entry for composite-body inertias K_BBo_W(q) for each body.
  cache_indexes_.composite_body_inertia_in_world =
      this->DeclareCacheEntry(
              std::string("composite mobod inertia in world (K_BBo_W)"),
              std::vector<SpatialInertia<T>>(internal_tree().num_mobods(),
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
              std::string("mobod dynamic bias (Fb_Bo_W)"),
              std::vector<SpatialForce<T>>(internal_tree().num_mobods()),
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
              std::vector<SpatialAcceleration<T>>(internal_tree().num_mobods()),
              &MultibodyTreeSystem<T>::CalcSpatialAccelerationBias,
              {position_ticket, velocity_ticket, this->all_parameters_ticket()})
          .cache_index();

  cache_indexes_.articulated_body_force_bias =
      this->DeclareCacheEntry(
              std::string("ABI force bias cache (Zb_Bo_W)"),
              std::vector<SpatialForce<T>>(internal_tree().num_mobods()),
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
  // No derivatives to compute if state is discrete or there are no derivatives.
  if (is_discrete()) return;
  if (derivatives->size() == 0) return;

  const VectorX<T>& x = dynamic_cast<const systems::BasicVector<T>&>(
                            context.get_continuous_state_vector())
                            .value();
  const int nq = internal_tree().num_positions();
  const int nv = internal_tree().num_velocities();
  const auto v = x.segment(nq, nv);

  const VectorX<T>& vdot = this->EvalForwardDynamics(context).get_vdot();

  // TODO(sherm1) Heap allocation here. Get rid of it.
  VectorX<T> qdot(nq);
  internal_tree().MapVelocityToQDot(context, v, &qdot);
  derivatives->get_mutable_generalized_position().SetFromVector(qdot);
  derivatives->get_mutable_generalized_velocity().SetFromVector(vdot);
  if (num_misc_continuous_states_ > 0) {
    DoCalcMiscDerivatives(context,
                          &derivatives->get_mutable_misc_continuous_state());
  }
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

  const VectorX<T>& xdot_proposed =
      dynamic_cast<const systems::BasicVector<T>&>(
          proposed_derivatives.get_vector())
          .value();
  const int nz = num_misc_continuous_states_;
  DRAKE_DEMAND(xdot_proposed.size() == nq + nv + nz);

  auto qdot_residual = residual->head(nq);
  // N(q)⋅v
  internal_tree().MapVelocityToQDot(
      context, internal_tree().get_velocities(context), &qdot_residual);
  // q̇_proposed - N(q)⋅v
  qdot_residual = xdot_proposed.head(nq) - qdot_residual;
  // InverseDynamics(context, v_proposed)
  residual->segment(nq, nv) = internal_tree().CalcInverseDynamics(
      context, xdot_proposed.segment(nq, nv), forces);

  if (num_misc_continuous_states_ == 0) return;
  BasicVector<T> zdot_residual(nz);
  DoCalcMiscDerivatives(context, &zdot_residual);
  residual->tail(nz) = xdot_proposed.tail(nz) - zdot_residual.get_value();
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
