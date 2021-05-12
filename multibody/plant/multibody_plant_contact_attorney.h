#pragma once

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
class ContactComputationManager;

/* This class is used to grant access to a selected collection of
 MultibodyPlant's private methods to ContactComputationManager.

 @tparam_default_scalar */
template <typename T>
class MultibodyPlantContactAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantContactAttorney);

  friend class ContactComputationManager<T>;

  static const MultibodyTree<T>& internal_tree(const MultibodyPlant<T>& plant) {
    return plant.internal_tree();
  }

  static const contact_solvers::internal::ContactSolverResults<T>&
  EvalContactSolverResults(const MultibodyPlant<T>& plant,
                           const systems::Context<T>& context) {
    return plant.EvalContactSolverResults(context);
  }
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
