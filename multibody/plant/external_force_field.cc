#include "drake/multibody/plant/external_force_field.h"

namespace drake {
namespace multibody {

template <typename T>
systems::CacheEntry& ExternalForceField<T>::DeclareCacheEntry(
    internal::MultibodyTreeSystem<T>* tree_system, std::string description,
    systems::ValueProducer value_producer,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  return internal::MultibodyTreeSystemElementAttorney<T>::DeclareCacheEntry(
      tree_system, std::move(description), std::move(value_producer),
      std::move(prerequisites_of_calc));
}

template <typename T>
systems::InputPort<T>& ExternalForceField<T>::DeclareAbstractInputPort(
    internal::MultibodyTreeSystem<T>* tree_system, std::string name,
    const AbstractValue& model_value) {
  return internal::MultibodyTreeSystemElementAttorney<
      T>::DeclareAbstractInputPort(tree_system, std::move(name), model_value);
}

template <typename T>
systems::InputPort<T>& ExternalForceField<T>::DeclareVectorInputPort(
    internal::MultibodyTreeSystem<T>* tree_system, std::string name,
    const systems::BasicVector<T>& model_vector) {
  return internal::MultibodyTreeSystemElementAttorney<
      T>::DeclareVectorInputPort(tree_system, std::move(name), model_vector);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ExternalForceField)
