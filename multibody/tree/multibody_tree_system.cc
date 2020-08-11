#include "drake/multibody/tree/multibody_tree_system.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/tree/multibody_tree-inl.h"

namespace drake {
using systems::BasicVector;
using systems::Context;
using systems::LeafSystem;
using systems::State;

namespace multibody {
namespace internal {

template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(
    std::unique_ptr<MultibodyTree<T>> tree,
    bool is_discrete)
    : MultibodyTreeSystem(
          systems::SystemTypeTag<MultibodyTreeSystem>{},
          false,  // Null tree is not allowed here.
          std::move(tree), is_discrete) {}

template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(bool is_discrete)
    : MultibodyTreeSystem(
          systems::SystemTypeTag<MultibodyTreeSystem>{},
          true,  // Null tree is OK.
          nullptr, is_discrete) {}

template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(
    systems::SystemScalarConverter converter,
    std::unique_ptr<MultibodyTree<T>> tree,
    bool is_discrete)
    : MultibodyTreeSystem(
          std::move(converter),
          true,  // Null tree is OK.
          std::move(tree), is_discrete) {}

template <typename T>
template <typename U>
MultibodyTreeSystem<T>::MultibodyTreeSystem(const MultibodyTreeSystem<U>& other)
    : MultibodyTreeSystem(
          systems::SystemTypeTag<MultibodyTreeSystem>{},
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
void MultibodyTreeSystem<T>::SetDefaultState(const Context<T>& context,
                                             State<T>* state) const {
  LeafSystem<T>::SetDefaultState(context, state);
  tree_->SetDefaultState(context, state);
}

template <typename T>
MultibodyTreeSystem<T>::~MultibodyTreeSystem() = default;

template <typename T>
MultibodyTree<T>& MultibodyTreeSystem<T>::mutable_tree() const {
  DRAKE_DEMAND(tree_ != nullptr);
  return *tree_;
}

template <typename T>
void MultibodyTreeSystem<T>::Finalize() {
  if (already_finalized_) {
    throw std::logic_error(
        "MultibodyTreeSystem::Finalize(): repeated calls not allowed.");
  }
  if (!tree_->topology_is_valid()) {
    tree_->Finalize();
  }

  // Declare state.
  if (is_discrete_) {
    this->DeclareDiscreteState(tree_->num_states());
  } else {
    this->DeclareContinuousState(BasicVector<T>(tree_->num_states()),
                                 tree_->num_positions(),
                                 tree_->num_velocities(),
                                 0 /* num_z */);
  }

  // Allocate position cache.
  auto& position_kinematics_cache_entry = this->DeclareCacheEntry(
      std::string("position kinematics"),
      [tree = tree_.get()]() {
        return AbstractValue::Make(
            PositionKinematicsCache<T>(tree->get_topology()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& position_cache =
            cache_value->get_mutable_value<PositionKinematicsCache<T>>();
        tree->CalcPositionKinematicsCache(context, &position_cache);
      },
      {this->configuration_ticket()});
  cache_indexes_.position_kinematics =
      position_kinematics_cache_entry.cache_index();

  // Allocate cache entry to store M_B_W(q) for each body.
  auto& spatial_inertia_in_world_cache_entry = this->DeclareCacheEntry(
      std::string("spatial inertia in world (M_B_W)"),
      [tree = tree_.get()]() {
        return AbstractValue::Make(
            std::vector<SpatialInertia<T>>(tree->num_bodies()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& spatial_inertia_in_world_cache =
            cache_value->get_mutable_value<std::vector<SpatialInertia<T>>>();
        tree->CalcSpatialInertiaInWorldCache(context,
                                             &spatial_inertia_in_world_cache);
      },
      {this->cache_entry_ticket(cache_indexes_.position_kinematics)});
  cache_indexes_.spatial_inertia_in_world =
      spatial_inertia_in_world_cache_entry.cache_index();

  // Allocate velocity cache.
  auto& velocity_kinematics_cache_entry = this->DeclareCacheEntry(
      std::string("velocity kinematics"),
      [tree = tree_.get()]() {
        return AbstractValue::Make(
            VelocityKinematicsCache<T>(tree->get_topology()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& velocity_cache =
            cache_value->get_mutable_value<VelocityKinematicsCache<T>>();
        tree->CalcVelocityKinematicsCache(
            context, tree->EvalPositionKinematics(context), &velocity_cache);
      },
      {this->kinematics_ticket()});
  cache_indexes_.velocity_kinematics =
      velocity_kinematics_cache_entry.cache_index();

  // Allocate cache entry to store Fb_Bo_W(q, v) for each body.
  auto& dynamic_bias_cache_entry = this->DeclareCacheEntry(
      std::string("dynamic bias (Fb_Bo_W)"),
      [tree = tree_.get()]() {
        return AbstractValue::Make(
            std::vector<SpatialForce<T>>(tree->num_bodies()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& dynamic_bias_cache =
            cache_value->get_mutable_value<std::vector<SpatialForce<T>>>();
        tree->CalcDynamicBiasCache(context, &dynamic_bias_cache);
      },
      // The computation of Fb_Bo_W(q, v) requires updated values of M_Bo_W(q)
      // and V_WB(q, v). We make these prerequisites explicit.
      // Another alternative would be to state the dependence on q and v.
      // However this option is not optimal until #9171 gets resolved.
      {this->cache_entry_ticket(cache_indexes_.spatial_inertia_in_world),
       this->cache_entry_ticket(cache_indexes_.velocity_kinematics)});
  cache_indexes_.dynamic_bias = dynamic_bias_cache_entry.cache_index();

  // Declare cache entry for H_PB_W(q).
  // The type of this cache value is std::vector<Vector6<T>>.
  auto& H_PB_W_cache_entry = this->DeclareCacheEntry(
      std::string("H_PB_W(q)"),
      [tree = tree_.get()]() {
        return AbstractValue::Make(
            std::vector<Vector6<T>>(tree->num_velocities()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& H_PB_W_cache =
            cache_value->get_mutable_value<std::vector<Vector6<T>>>();
        tree->CalcAcrossNodeJacobianWrtVExpressedInWorld(
            context, tree->EvalPositionKinematics(context), &H_PB_W_cache);
      },
      {this->cache_entry_ticket(cache_indexes_.position_kinematics)});
  cache_indexes_.across_node_jacobians = H_PB_W_cache_entry.cache_index();

  // Allocate articulated body inertia cache.
  auto& abi_cache_entry = this->DeclareCacheEntry(
      std::string("Articulated Body Inertia"),
      [tree = tree_.get()]() {
        return AbstractValue::Make(
            ArticulatedBodyInertiaCache<T>(tree->get_topology()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& abi_cache =
            cache_value->get_mutable_value<ArticulatedBodyInertiaCache<T>>();
        tree->CalcArticulatedBodyInertiaCache(context, &abi_cache);
      },
      {this->configuration_ticket()});
  cache_indexes_.abi_cache_index = abi_cache_entry.cache_index();

  auto& Ab_WB_cache_entry = this->DeclareCacheEntry(
      std::string("spatial acceleration bias (Ab_WB)"),
      [tree = tree_.get()]() {
        return AbstractValue::Make(
            std::vector<SpatialAcceleration<T>>(tree->num_bodies()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& Ab_WB_cache =
            cache_value
                ->get_mutable_value<std::vector<SpatialAcceleration<T>>>();
        tree->CalcSpatialAccelerationBiasCache(context, &Ab_WB_cache);
      },
      {this->kinematics_ticket()});
  cache_indexes_.spatial_acceleration_bias = Ab_WB_cache_entry.cache_index();

  auto& Zb_Bo_W_cache_entry = this->DeclareCacheEntry(
      std::string("ABI force bias cache (Zb_Bo_W)"),
      [tree = tree_.get()]() {
        return AbstractValue::Make(
            std::vector<SpatialForce<T>>(tree->num_bodies()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& Zb_Bo_W_cache =
            cache_value->get_mutable_value<std::vector<SpatialForce<T>>>();
        tree->CalcArticulatedBodyForceBiasCache(context, &Zb_Bo_W_cache);
      },
      {this->kinematics_ticket()});
  cache_indexes_.articulated_body_force_bias =
      Zb_Bo_W_cache_entry.cache_index();

  // Articulated Body Algorithm (ABA) force cache.
  auto& aba_force_cache_entry = this->DeclareCacheEntry(
      std::string("ABA force cache"),
      ArticulatedBodyForceCache<T>(internal_tree().get_topology()),
      &MultibodyTreeSystem<T>::CalcArticulatedBodyForceCache,
      {this->all_sources_ticket()});
  cache_indexes_.articulated_body_forces =
      aba_force_cache_entry.cache_index();

  // Acceleration kinematics must be calculated for forward dynamics,
  // regardless of whether that is done in continuous mode (as the last pass
  // of ABA) or in discrete mode (explicitly by MultibodyPlant).
  auto& acceleration_kinematics_cache_entry = this->DeclareCacheEntry(
      std::string("Accelerations"),
      AccelerationKinematicsCache<T>(internal_tree().get_topology()),
      &MultibodyTreeSystem<T>::CalcForwardDynamics,
      {this->all_sources_ticket()});
  cache_indexes_.acceleration_kinematics =
      acceleration_kinematics_cache_entry.cache_index();

  already_finalized_ = true;
}

template<typename T>
void MultibodyTreeSystem<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // No derivatives to compute if state is discrete.
  if (is_discrete()) return;
  // No derivatives to compute if state is empty. (Will segfault otherwise.)
  // TODO(amcastro-tri): When nv = 0 we should not declare state or cache
  // entries at all and the system framework will never call this.
  if (internal_tree().num_states() == 0) return;

  // N.B. get_value() here is inexplicably returning a VectorBlock
  // rather than a reference to the stored VectorX.
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const auto v = x.bottomRows(internal_tree().num_velocities());

  const VectorX<T>& vdot = this->EvalForwardDynamics(context).get_vdot();

  // TODO(sherm1) Heap allocation here. Get rid of it.
  VectorX<T> xdot(internal_tree().num_states());
  VectorX<T> qdot(internal_tree().num_positions());
  internal_tree().MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template<typename T>
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

template<typename T>
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
  internal_tree().CalcArticulatedBodyAccelerations(context,
                                                   aba_force_cache, ac);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::MultibodyTreeSystem)
