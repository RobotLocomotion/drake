#include "drake/multibody/tree/multibody_element.h"

#include <utility>

namespace drake {
namespace multibody {

using internal::MultibodyTreeSystem;
using systems::Parameters;

template <typename T>
MultibodyElement<T>::~MultibodyElement() {
  // Clear the tree to help fail-fast in case of accidental use-after-free of
  // this MultibodyElement by end users who accidentally kept around a stale
  // point to us.
  parent_tree_ = nullptr;
}

template <typename T>
void MultibodyElement<T>::DeclareParameters(
    MultibodyTreeSystem<T>* tree_system) {
  DRAKE_DEMAND(tree_system == &GetParentTreeSystem());
  DoDeclareParameters(tree_system);
}

template <typename T>
void MultibodyElement<T>::SetDefaultParameters(
    systems::Parameters<T>* parameters) const {
  DRAKE_DEMAND(parameters != nullptr);
  DoSetDefaultParameters(parameters);
}

template <typename T>
void MultibodyElement<T>::DeclareDiscreteState(
    MultibodyTreeSystem<T>* tree_system) {
  DRAKE_DEMAND(tree_system == &GetParentTreeSystem());
  DoDeclareDiscreteState(tree_system);
}

template <typename T>
void MultibodyElement<T>::DeclareCacheEntries(
    MultibodyTreeSystem<T>* tree_system) {
  DRAKE_DEMAND(tree_system == &GetParentTreeSystem());
  DoDeclareCacheEntries(tree_system);
}

template <typename T>
MultibodyElement<T>::MultibodyElement() {}

template <typename T>
MultibodyElement<T>::MultibodyElement(ModelInstanceIndex model_instance)
    : model_instance_(model_instance) {}

template <typename T>
MultibodyElement<T>::MultibodyElement(ModelInstanceIndex model_instance,
                                      int64_t index)
    : MultibodyElement(model_instance) {
  index_ = index;
}

template <typename T>
void MultibodyElement<T>::DoDeclareParameters(MultibodyTreeSystem<T>*) {}

template <typename T>
void MultibodyElement<T>::DoSetDefaultParameters(Parameters<T>*) const {}

template <typename T>
void MultibodyElement<T>::DoDeclareDiscreteState(MultibodyTreeSystem<T>*) {}

template <typename T>
void MultibodyElement<T>::DoDeclareCacheEntries(MultibodyTreeSystem<T>*) {}

template <typename T>
systems::NumericParameterIndex MultibodyElement<T>::DeclareNumericParameter(
    MultibodyTreeSystem<T>* tree_system,
    const systems::BasicVector<T>& model_vector) {
  return internal::MultibodyTreeSystemElementAttorney<
      T>::DeclareNumericParameter(tree_system, model_vector);
}

template <typename T>
systems::AbstractParameterIndex MultibodyElement<T>::DeclareAbstractParameter(
    MultibodyTreeSystem<T>* tree_system, const AbstractValue& model_value) {
  return internal::MultibodyTreeSystemElementAttorney<
      T>::DeclareAbstractParameter(tree_system, model_value);
}

template <typename T>
systems::DiscreteStateIndex MultibodyElement<T>::DeclareDiscreteState(
    MultibodyTreeSystem<T>* tree_system, const VectorX<T>& model_value) {
  return internal::MultibodyTreeSystemElementAttorney<T>::DeclareDiscreteState(
      tree_system, model_value);
}

template <typename T>
systems::CacheEntry& MultibodyElement<T>::DeclareCacheEntry(
    MultibodyTreeSystem<T>* tree_system, std::string description,
    systems::ValueProducer value_producer,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  return internal::MultibodyTreeSystemElementAttorney<T>::DeclareCacheEntry(
      tree_system, std::move(description), std::move(value_producer),
      std::move(prerequisites_of_calc));
}

template <typename T>
void MultibodyElement<T>::ThrowNoParentTree() const {
  throw std::logic_error(
      "This multibody element was not added to a MultibodyTree.");
}

template <typename T>
void MultibodyElement<T>::HasThisParentTreeOrThrow(
    const internal::MultibodyTree<T>* tree) const {
  DRAKE_ASSERT(tree != nullptr);
  if (parent_tree_ != tree) {
    throw std::logic_error(
        "This multibody element does not belong to the "
        "supplied MultibodyTree.");
  }
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyElement);
