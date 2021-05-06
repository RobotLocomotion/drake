#pragma once

#include "drake/multibody/plant/contact_computation_manager.h"
#include "drake/multibody/plant/multibody_plant_access.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace internal {
// This class forwards the call to private methods of MultibodyPlant associated
// with the derived ContactComputationManager to the attorney.
// @tparam_default_scalar
template <typename T>
class ContactComputationManagerImpl : public ContactComputationManager<T> {
 public:
  ContactComputationManagerImpl(MultibodyPlant<T>* plant) : plant_(plant) {
    DRAKE_DEMAND(plant_ != nullptr);
  }

  virtual ~ContactComputationManagerImpl() = 0;

 protected:
  const MultibodyTree<T>& internal_tree() const {
    return MultibodyPlantContactComputationManagerAttorney<T>::internal_tree(
        plant_);
  }

  systems::DiscreteStateIndex DeclareDiscreteState(
      const VectorX<T>& model_value) const {
    return MultibodyPlantContactComputationManagerAttorney<
        T>::DeclareDiscreteState(&plant_, model_value);
  }

  systems::LeafOutputPort<T>& DeclareAbstractOutputPort(
      const std::string& name,
      typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
      typename systems::LeafOutputPort<T>::CalcCallback calc_function) {
    return MultibodyPlantContactComputationManagerAttorney<
        T>::DeclareAbstractOutputPort(&plant_, name, alloc_function,
                                      calc_function);
  }

 private:
  MultibodyPlant<T>* plant_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
