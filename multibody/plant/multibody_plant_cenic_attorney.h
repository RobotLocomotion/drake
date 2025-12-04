#pragma once

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {

// Forward declaration for friendship, below.
template <typename T>
class CenicIntegrator;

namespace internal {

/* This class is used to grant access to a selected collection of
MultibodyPlant's private methods to CenicIntegrator.

@tparam_default_scalar */
template <typename T>
class MultibodyPlantCenicAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantCenicAttorney);

  friend class CenicIntegrator<T>;

  static void AddAppliedExternalGeneralizedForces(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context,
      MultibodyForces<T>* forces) {
    return plant.AddAppliedExternalGeneralizedForces(context, forces);
  }

  static void AddAppliedExternalSpatialForces(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context,
      MultibodyForces<T>* forces) {
    return plant.AddAppliedExternalSpatialForces(context, forces);
  }

  static void AddJointActuationForces(const MultibodyPlant<T>& plant,
                                      const systems::Context<T>& context,
                                      VectorX<T>* forces) {
    return plant.AddJointActuationForces(context, forces);
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
