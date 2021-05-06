#pragma once

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

// This class is used to grant access to a selected collection of
// MultibodyPlant's private members and/or methods to ContactComputationManager.
template <typename T>
class MultibodyPlantContactComputationManagerAttorney {
 private:
  friend class ContactComputationMangerImpl;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(
      MultibodyPlantContactComputationManagerAttorney);
  MultibodyPlantContactComputationManagerAttorney() = delete;

  static inline const MultibodyTree<T>& internal_tree(
      const MultibodyPlant<T>& plant) {
    return plant.internal_tree();
  }

  static inline systems::DiscreteStateIndex DeclareDiscreteState(
      MultibodyPlant<T>* plant, const VectorX<T>& model_value) {
    return plant->DeclareDiscreteState(model_value);
  }

  systems::LeafOutputPort<T>& DeclareAbstractOutputPort(
    MultibodyPlant<T>* plant,
      const std::string& name,
      typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
      typename systems::LeafOutputPort<T>::CalcCallback calc_function) {
    return plant->DeclareAbstractOutputPort(name, alloc_function, calc_function);
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
