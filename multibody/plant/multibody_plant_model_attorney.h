#pragma once

#include <set>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
class PhysicalModel;

/* This class is used to grant access to a selected collection of
 MultibodyPlant's private methods to PhysicalModel.

 @tparam_default_scalar */
template <typename T>
class MultibodyPlantModelAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantModelAttorney);

  friend class PhysicalModel<T>;

  static systems::DiscreteStateIndex DeclareDiscreteState(
      MultibodyPlant<T>* plant, const VectorX<T>& model_value) {
    DRAKE_DEMAND(plant != nullptr);
    return plant->DeclareDiscreteState(model_value);
  }

  static systems::LeafOutputPort<T>& DeclareAbstractOutputPort(
      MultibodyPlant<T>* plant, std::string name,
      typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
      typename systems::LeafOutputPort<T>::CalcCallback calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()}) {
    DRAKE_DEMAND(plant != nullptr);
    return plant->DeclareAbstractOutputPort(
        std::move(name), std::move(alloc_function), std::move(calc_function),
        std::move(prerequisites_of_calc));
  }

  static systems::LeafOutputPort<T>& DeclareVectorOutputPort(
      MultibodyPlant<T>* plant, std::string name,
      const systems::BasicVector<T>& model_vector,
      typename systems::LeafOutputPort<T>::CalcVectorCallback
          vector_calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()}) {
    DRAKE_DEMAND(plant != nullptr);
    return plant->DeclareVectorOutputPort(std::move(name), model_vector,
                                          std::move(vector_calc_function),
                                          std::move(prerequisites_of_calc));
  }
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
