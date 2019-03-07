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
/// @pre plant must be registered with a scene graph.
void AddInclinedPlaneToPlant(
    double radius, double mass, double slope,
    const CoulombFriction<double>& surface_friction,
    double gravity,
    MultibodyPlant<double>* plant);

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
