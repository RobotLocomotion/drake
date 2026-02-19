#pragma once

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
class DiscreteUpdateManager;

/* This class is used to grant access to a selected collection of
 MultibodyPlant's private methods to DiscreteUpdateManager.

 @tparam_default_scalar */
template <typename T>
class MultibodyPlantDiscreteUpdateManagerAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantDiscreteUpdateManagerAttorney);

  friend class DiscreteUpdateManager<T>;

  // N.B. Keep the spelling and order of declarations here identical to the
  // DiscreteUpdateManager protected section's spelling and order of same.

  static const MultibodyTree<T>& internal_tree(const MultibodyPlant<T>& plant) {
    return plant.internal_tree();
  }

  static systems::CacheEntry& DeclareCacheEntry(
      MultibodyPlant<T>* plant, std::string description,
      systems::ValueProducer value_producer,
      std::set<systems::DependencyTicket> prerequisites_of_calc) {
    return plant->DeclareCacheEntry(std::move(description),
                                    std::move(value_producer),
                                    std::move(prerequisites_of_calc));
  }

  static const GeometryContactData<T>& EvalGeometryContactData(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalGeometryContactData(context);
  }

  static void AddJointLimitsPenaltyForces(const MultibodyPlant<T>& plant,
                                          const systems::Context<T>& context,
                                          MultibodyForces<T>* forces) {
    plant.AddJointLimitsPenaltyForces(context, forces);
  }

  static void AddAppliedExternalGeneralizedForces(
      const MultibodyPlant<T>& plant, const drake::systems::Context<T>& context,
      MultibodyForces<T>* forces) {
    plant.AddAppliedExternalGeneralizedForces(context, forces);
  }

  static void AddAppliedExternalSpatialForces(
      const MultibodyPlant<T>& plant, const drake::systems::Context<T>& context,
      MultibodyForces<T>* forces) {
    plant.AddAppliedExternalSpatialForces(context, forces);
  }

  static void CalcForceElementsContribution(
      const MultibodyPlant<T>& plant, const drake::systems::Context<T>& context,
      MultibodyForces<T>* forces) {
    return plant.CalcForceElementsContribution(context, forces);
  }

  static const VectorX<T>& EvalActuationInput(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context,
      bool effort_limit) {
    return plant.EvalActuationInput(context, effort_limit);
  }

  static const DesiredStateInput<T>& EvalDesiredStateInput(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalDesiredStateInput(context);
  }

  // TODO(xuchenhan-tri): Remove this when SceneGraph takes control of all
  //  geometries.
  /* Returns the per-body arrays of collision geometries indexed by BodyIndex
   for the given `plant`. */
  static const std::vector<std::vector<geometry::GeometryId>>&
  collision_geometries(const MultibodyPlant<T>& plant) {
    return plant.collision_geometries_;
  }

  static double default_contact_stiffness(const MultibodyPlant<T>& plant) {
    return plant.penalty_method_contact_parameters_.geometry_stiffness;
  }

  static double default_contact_dissipation(const MultibodyPlant<T>& plant) {
    return plant.penalty_method_contact_parameters_.dissipation;
  }

  static const internal::JointLockingCacheData<T>& EvalJointLocking(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalJointLocking(context);
  }

  static const std::map<MultibodyConstraintId, internal::CouplerConstraintSpec>&
  coupler_constraints_specs(const MultibodyPlant<T>& plant) {
    return plant.coupler_constraints_specs_;
  }

  static const std::map<MultibodyConstraintId, DistanceConstraintParams>&
  GetDistanceConstraintParams(const MultibodyPlant<T>& plant,
                              const systems::Context<T>& context) {
    return plant.GetDistanceConstraintParams(context);
  }

  static const std::map<MultibodyConstraintId, internal::BallConstraintSpec>&
  ball_constraints_specs(const MultibodyPlant<T>& plant) {
    return plant.ball_constraints_specs_;
  }

  static const std::map<MultibodyConstraintId, internal::WeldConstraintSpec>&
  weld_constraints_specs(const MultibodyPlant<T>& plant) {
    return plant.weld_constraints_specs_;
  }

  static const std::map<MultibodyConstraintId, internal::TendonConstraintSpec>&
  tendon_constraints_specs(const MultibodyPlant<T>& plant) {
    return plant.tendon_constraints_specs_;
  }

  static const std::map<MultibodyConstraintId, bool>& GetConstraintActiveStatus(
      const systems::Context<T>& context, const MultibodyPlant<T>& plant) {
    return plant.GetConstraintActiveStatus(context);
  }

  static BodyIndex FindBodyByGeometryId(const MultibodyPlant<T>& plant,
                                        geometry::GeometryId geometry_id) {
    return plant.FindBodyByGeometryId(geometry_id);
  }
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
