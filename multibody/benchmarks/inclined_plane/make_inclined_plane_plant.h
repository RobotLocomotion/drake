#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace inclined_plane {

/// This method makes a MultibodyPlant model for a sphere rolling down an
/// inclined plane.
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
/// @param scene_graph
///   This factory method will register the new multibody plant to be a source
///   for this geometry system and it will also register geometry for contact
///   modeling.
/// @throws std::exception if scene_graph is nullptr.
std::unique_ptr<multibody_plant::MultibodyPlant<double>> MakeInclinedPlanePlant(
    double radius, double mass, double slope,
    const multibody_plant::CoulombFriction<double>& surface_friction,
    double gravity, geometry::SceneGraph<double>* scene_graph);

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
