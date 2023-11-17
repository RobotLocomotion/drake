#include "drake/multibody/plant/force_density_field.h"

namespace drake {
namespace multibody {

template <typename T>
systems::CacheEntry& ForceDensityField<T>::DeclareCacheEntry(
    internal::MultibodyTreeSystem<T>* plant, std::string description,
    systems::ValueProducer value_producer,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  return internal::MultibodyTreeSystemElementAttorney<T>::DeclareCacheEntry(
      plant, std::move(description), std::move(value_producer),
      std::move(prerequisites_of_calc));
}

template <typename T>
systems::InputPort<T>& ForceDensityField<T>::DeclareAbstractInputPort(
    internal::MultibodyTreeSystem<T>* plant, std::string name,
    const AbstractValue& model_value) {
  return internal::MultibodyTreeSystemElementAttorney<
      T>::DeclareAbstractInputPort(plant, std::move(name), model_value);
}

template <typename T>
systems::InputPort<T>& ForceDensityField<T>::DeclareVectorInputPort(
    internal::MultibodyTreeSystem<T>* plant, std::string name,
    const systems::BasicVector<T>& model_vector) {
  return internal::MultibodyTreeSystemElementAttorney<
      T>::DeclareVectorInputPort(plant, std::move(name), model_vector);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ForceDensityField)
