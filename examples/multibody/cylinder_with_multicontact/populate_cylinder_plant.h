#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cylinder_with_multicontact {

/// This function populates the given MultibodyPlant with a model of a cylinder
/// free to fall onto the ground.
/// The model adds a fixed number of spheres (10) around each rim of the
/// cylinder as a way of emulating multicontact so that we can evaluate
/// MultibodyPlant's contact solver.
/// MultibodyPlant models the contact of the ball with the ground as a perfectly
/// inelastic collision (zero coefficient of restitution), i.e. energy is lost
/// due to the collision.
///
/// @param[in] radius
///   The radius of the cylinder.
/// @param[in] length
///   The length of the cylinder.
/// @param[in] mass
///   The mass of the cylinder.
/// @param[in] surface_friction
///   The Coulomb's law coefficients of friction.
/// @param[in] gravity_W
///   The acceleration of gravity vector, expressed in the world frame W.
/// @param plant
///   A valid pointer to a MultibodyPlant. This function will register
///   geometry for contact modeling.
/// @pre `plant` is not null, finalized, and is registered with a SceneGraph.
/// @note The given plant is not finalized. You must call Finalize() on the new
///   model once you are done populating it.
void PopulateCylinderPlant(
    double radius, double length, double mass,
    const drake::multibody::CoulombFriction<double>& surface_friction,
    const Vector3<double>& gravity_W,
    drake::multibody::MultibodyPlant<double>* plant);

}  // namespace cylinder_with_multicontact
}  // namespace multibody
}  // namespace examples
}  // namespace drake
