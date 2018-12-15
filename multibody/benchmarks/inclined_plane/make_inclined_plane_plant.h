#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace inclined_plane {

/// This method adds a sphere rolling down an inclined plane.
///
/// @param[in] radius
///   The radius in meters of the sphere.
/// @param[in] mass
///   The mass in kilograms of the sphere.
/// @param[in] slope
///   The slope in radians of the inclined plane.
/// @param[in] surface_friction
///   The Coulomb's law coefficients of friction. This model uses the same
///   surface properties for both the inclined plane and the sphere.
/// @param[in] gravity
///   The acceleration of gravity, in m/s². Points in the minus z direction.
/// @param[out] plant
///   Plant to contain the sphere and plane.
/// @throws std::exception if plant is nullptr.
/// @pre plant as registered with a scene graph.
void AddInclinedPlanePlant(
    double radius, double mass, double slope,
    const CoulombFriction<double>& surface_friction,
    double gravity,
    MultibodyPlant<double>* plant);

// TODO(eric.cousineau): Remove on or about 2018/03/01.
/// Deprecated method. Use `AddInclinedPlanePlant` instead.
DRAKE_DEPRECATED("Use `AddInclinedPlanePlant` instead.")
inline std::unique_ptr<MultibodyPlant<double>> MakeInclinedPlanePlant(
    double radius, double mass, double slope,
    const CoulombFriction<double>& surface_friction,
    double gravity, double time_step,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_DEMAND(scene_graph != nullptr);
  DRAKE_DEMAND(time_step >= 0);
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  plant->RegisterAsSourceForSceneGraph(scene_graph);
  AddInclinedPlanePlant(
      radius, mass, slope, surface_friction, gravity, plant.get());
  plant->Finalize();
  return plant;
}

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
