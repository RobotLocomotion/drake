#pragma once

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {

namespace contact_solvers {
namespace icf {
namespace internal {
// Forward declarations for friendship, below.
template <typename T>
class IcfBuilder;
template <typename T>
class IcfExternalSystemsLinearizer;
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers

namespace internal {

/* This class is used to grant access to a selected collection of
MultibodyPlant's private methods to //multibody/contact_solvers/icf.

@tparam_default_scalar */
template <typename T>
class MultibodyPlantIcfAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantIcfAttorney);

  friend class contact_solvers::icf::internal::IcfBuilder<T>;
  friend class contact_solvers::icf::internal::IcfExternalSystemsLinearizer<T>;

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

  static const ContactByPenaltyMethodParameters&
  GetContactByPenaltyMethodParameters(const MultibodyPlant<T>& plant) {
    return plant.penalty_method_contact_parameters_;
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
