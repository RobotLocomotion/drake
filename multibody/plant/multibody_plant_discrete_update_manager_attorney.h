#pragma once

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

  static const contact_solvers::internal::ContactSolverResults<T>&
  EvalContactSolverResults(const MultibodyPlant<T>& plant,
                           const systems::Context<T>& context) {
    return plant.EvalContactSolverResults(context);
  }

  static const internal::ContactJacobians<T>& EvalContactJacobians(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalContactJacobians(context);
  }

  static const std::vector<internal::DiscreteContactPair<T>>&
  EvalDiscreteContactPairs(const MultibodyPlant<T>& plant,
                           const systems::Context<T>& context) {
    return plant.EvalDiscreteContactPairs(context);
  }

  static const std::vector<geometry::ContactSurface<T>>& EvalContactSurfaces(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalContactSurfaces(context);
  }

  static std::vector<CoulombFriction<double>> CalcCombinedFrictionCoefficients(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context,
      const std::vector<internal::DiscreteContactPair<T>>& contact_pairs) {
    return plant.CalcCombinedFrictionCoefficients(context, contact_pairs);
  }

  static void AddInForcesFromInputPorts(const MultibodyPlant<T>& plant,
                                        const systems::Context<T>& context,
                                        MultibodyForces<T>* forces) {
    plant.AddInForcesFromInputPorts(context, forces);
  }

  static void CalcNonContactForces(const MultibodyPlant<T>& plant,
                            const drake::systems::Context<T>& context,
                            MultibodyForces<T>* forces) {
    return plant.CalcNonContactForces(context, true /* is discrete */, forces);
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

  static const std::unordered_map<geometry::GeometryId, BodyIndex>&
  geometry_id_to_body_index(const MultibodyPlant<T>& plant) {
    return plant.geometry_id_to_body_index_;
  }
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
