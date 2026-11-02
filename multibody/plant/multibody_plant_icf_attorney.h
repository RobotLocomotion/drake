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

// Forward declaration for friendship, below.
template <typename T>
class CenicIntegrator;

namespace internal {

/* This class is used to grant access to a selected collection of
MultibodyPlant's private methods to //multibody/contact_solvers/icf and to
//multibody/cenic.

@tparam_default_scalar */
template <typename T>
class MultibodyPlantIcfAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantIcfAttorney);

  friend class contact_solvers::icf::internal::IcfBuilder<T>;
  friend class contact_solvers::icf::internal::IcfExternalSystemsLinearizer<T>;
  friend class CenicIntegrator<T>;

  // Returns the plant's cached geometry contact data (point pairs and contact
  // surfaces) at the configuration stored in `context`.
  static const GeometryContactData<T>& EvalGeometryContactData(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalGeometryContactData(context);
  }

  // See MultibodyPlant::CalcStepMemoryFromAppliedForces().
  static void CalcStepMemoryFromAppliedForces(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context,
      const MultibodyForces<T>& applied_forces, const VectorX<T>& vdot,
      DiscreteStepMemory::Data<T>* memory) {
    plant.CalcStepMemoryFromAppliedForces(context, applied_forces, vdot,
                                          memory);
  }

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

  static const VectorX<T>& EvalActuationInput(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context,
      bool apply_effort_limit) {
    return plant.EvalActuationInput(context, apply_effort_limit);
  }

  static const DesiredStateInput<T>& EvalDesiredStateInput(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalDesiredStateInput(context);
  }

  static const ContactByPenaltyMethodParameters&
  GetContactByPenaltyMethodParameters(const MultibodyPlant<T>& plant) {
    return plant.penalty_method_contact_parameters_;
  }

  static void AddSurfaceVelocityBias(const MultibodyPlant<T>& plant,
                                     const systems::Context<T>& context,
                                     BodyIndex bodyA_index,
                                     BodyIndex bodyB_index,
                                     const Vector3<T>& nhat_BA_W,
                                     Vector3<T>* v_AcBc_W) {
    plant.AddSurfaceVelocityBias(context, bodyA_index, bodyB_index, nhat_BA_W,
                                 v_AcBc_W);
  }

  static const internal::JointLockingCacheData<T>& EvalJointLocking(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalJointLocking(context);
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
