#include "drake/multibody/plant/physical_model.h"

#include <utility>

#include "drake/multibody/plant/multibody_plant_model_attorney.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
systems::DiscreteStateIndex PhysicalModel<T>::DeclareDiscreteState(
    MultibodyPlant<T>* plant, const VectorX<T>& model_value) {
  return MultibodyPlantModelAttorney<T>::DeclareDiscreteState(plant,
                                                              model_value);
}

template <typename T>
systems::LeafOutputPort<T>& PhysicalModel<T>::DeclareAbstractOutputPort(
    MultibodyPlant<T>* plant, std::string name,
    typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
    typename systems::LeafOutputPort<T>::CalcCallback calc_function,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  return MultibodyPlantModelAttorney<T>::DeclareAbstractOutputPort(
      plant, std::move(name), std::move(alloc_function),
      std::move(calc_function), std::move(prerequisites_of_calc));
}

template <typename T>
systems::LeafOutputPort<T>& PhysicalModel<T>::DeclareVectorOutputPort(
    MultibodyPlant<T>* plant, std::string name,
    const systems::BasicVector<T>& model_vector,
    typename systems::LeafOutputPort<T>::CalcVectorCallback
        vector_calc_function,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  return MultibodyPlantModelAttorney<T>::DeclareVectorOutputPort(
      plant, std::move(name), model_vector, std::move(vector_calc_function),
      std::move(prerequisites_of_calc));
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PhysicalModel);
