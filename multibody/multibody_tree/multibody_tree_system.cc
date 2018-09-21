#include "drake/multibody/multibody_tree/multibody_tree_system.h"

#include <memory>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename U>
MultibodyTreeSystem<T>::MultibodyTreeSystem(const MultibodyTreeSystem<U>& other)
    : MultibodyTreeSystem(
          systems::SystemTypeTag<multibody::MultibodyTreeSystem>{},
          other.tree().template CloneToScalar<T>(), other.is_discrete()) {}

// This is the one true constructor.
template <typename T>
MultibodyTreeSystem<T>::MultibodyTreeSystem(
    systems::SystemScalarConverter converter,
    std::unique_ptr<MultibodyTree<T>> tree,
    bool is_discrete)
    : LeafSystem<T>(std::move(converter)),
      is_discrete_(is_discrete) {
  if (tree != nullptr) {
    tree_ = std::move(tree);
    tree_->set_tree_system(this);
    FinalizeMultibodyTreeSystem();
  } else {
    tree_ = std::make_unique<MultibodyTree<T>>();
    tree_->set_tree_system(this);
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
MultibodyTree<T>& MultibodyTreeSystem<T>::mutable_tree() const {
  DRAKE_DEMAND(tree_ != nullptr);
  if (tree_->topology_is_valid())
    throw std::logic_error(
        "MultibodyTreeSystem::mutable_tree(): "
        "the contained MultibodyTree is finalized already.");\
  return *tree_;
}

template <typename T>
void MultibodyTreeSystem<T>::FinalizeMultibodyTreeSystem() {
  if (!tree_->topology_is_valid())
    tree_->Finalize();

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
        return systems::AbstractValue::Make(
            PositionKinematicsCache<T>(tree->get_topology()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           systems::AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& position_cache =
            cache_value->GetMutableValue<PositionKinematicsCache<T>>();
        tree->CalcPositionKinematicsCache(context, &position_cache);
      },
      {this->configuration_ticket()});
  position_kinematics_cache_index_ =
      position_kinematics_cache_entry.cache_index();

  // Allocate velocity cache.
  auto& velocity_kinematics_cache_entry = this->DeclareCacheEntry(
      std::string("velocity kinematics"),
      [tree = tree_.get()]() {
        return systems::AbstractValue::Make(
            VelocityKinematicsCache<T>(tree->get_topology()));
      },
      [tree = tree_.get()](const systems::ContextBase& context_base,
                           systems::AbstractValue* cache_value) {
        auto& context = dynamic_cast<const Context<T>&>(context_base);
        auto& velocity_cache =
            cache_value->GetMutableValue<VelocityKinematicsCache<T>>();
        tree->CalcVelocityKinematicsCache(
            context, tree->EvalPositionKinematics(context), &velocity_cache);
      },
      {this->kinematics_ticket()});
  velocity_kinematics_cache_index_ =
      velocity_kinematics_cache_entry.cache_index();

  // TODO(sherm1) Allocate articulated body inertia cache.
}

template <typename T>
std::unique_ptr<systems::LeafContext<T>>
MultibodyTreeSystem<T>::DoMakeLeafContext() const {
  return std::make_unique<MultibodyTreeContext<T>>(tree_->get_topology(),
                                                   is_discrete_);
}

// Instantiate supported conversion methods.
template MultibodyTreeSystem<AutoDiffXd>::MultibodyTreeSystem(
    const MultibodyTreeSystem<double>& other);

template MultibodyTreeSystem<double>::MultibodyTreeSystem(
    const MultibodyTreeSystem<AutoDiffXd>& other);

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::MultibodyTreeSystem)
