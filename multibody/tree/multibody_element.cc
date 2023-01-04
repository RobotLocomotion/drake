#include "drake/multibody/tree/multibody_element.h"

namespace drake {
namespace multibody {

using internal::MultibodyTreeSystem;

template <typename T>
MultibodyElement<T>::~MultibodyElement() {}

template <typename T>
void MultibodyElement<T>::DeclareParameters(
    MultibodyTreeSystem<T>* tree_system) {
  DRAKE_DEMAND(tree_system == &GetParentTreeSystem());
  DoDeclareParameters(tree_system);
}

template <typename T>
MultibodyElement<T>::MultibodyElement() {}

template <typename T>
MultibodyElement<T>::MultibodyElement(ModelInstanceIndex model_instance)
    : model_instance_(model_instance) {}

template <typename T>
void MultibodyElement<T>::DoDeclareParameters(MultibodyTreeSystem<T>*) {}

template <typename T>
systems::NumericParameterIndex MultibodyElement<T>::DeclareNumericParameter(
    MultibodyTreeSystem<T>* tree_system,
    const systems::BasicVector<T>& model_vector) {
  return internal::MultibodyTreeSystemElementAttorney<T>
      ::DeclareNumericParameter(tree_system, model_vector);
}

template <typename T>
systems::AbstractParameterIndex MultibodyElement<T>::DeclareAbstractParameter(
    MultibodyTreeSystem<T>* tree_system,
    const AbstractValue& model_value) {
  return internal::MultibodyTreeSystemElementAttorney<T>
      ::DeclareAbstractParameter(tree_system, model_value);
}

template <typename T>
void MultibodyElement<T>::HasParentTreeOrThrow() const {
  if (!has_parent_tree()) {
    throw std::logic_error(
        "This multibody element was not added to a MultibodyTree.");
  }
}

template <typename T>
void MultibodyElement<T>::HasThisParentTreeOrThrow(
    const internal::MultibodyTree<T>* tree) const {
  DRAKE_ASSERT(tree != nullptr);
  if (parent_tree_ != tree) {
    throw std::logic_error("This multibody element does not belong to the "
                           "supplied MultibodyTree.");
  }
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyElement)
