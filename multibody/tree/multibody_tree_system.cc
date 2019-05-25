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
          systems::SystemTypeTag<internal::MultibodyTreeSystem>{},
          false,  // Null tree is not allowed here.
          std::move(tree), is_discrete) {}

template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(bool is_discrete)
    : MultibodyTreeSystem(
          systems::SystemTypeTag<internal::MultibodyTreeSystem>{},
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
          systems::SystemTypeTag<multibody::internal::MultibodyTreeSystem>{},
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

  // Allocate cache entry to store b_Bo_W(q, v) for each body.
  auto& dynamic_bias_cache_entry = this->DeclareCacheEntry(
      std::string("dynamic bias (b_Bo_W)"),
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
      // The computation of b_Bo_W(q, v) requires updated values of M_Bo_W(q)
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
        tree->CalcAcrossNodeGeometricJacobianExpressedInWorld(
            context, tree->EvalPositionKinematics(context), &H_PB_W_cache);
      },
      {this->cache_entry_ticket(cache_indexes_.position_kinematics)});
  cache_indexes_.across_node_jacobians = H_PB_W_cache_entry.cache_index();

  // TODO(sherm1) Allocate articulated body inertia cache.

  already_finalized_ = true;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::MultibodyTreeSystem)
