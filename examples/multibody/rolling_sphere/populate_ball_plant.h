#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {

/// This function populates the given MultibodyPlant with a model of a ball
/// falling onto a plane.
/// MultibodyPlant models the contact of the ball with the ground as a perfectly
/// inelastic collision (zero coefficient of restitution), i.e. energy is lost
/// due to the collision.
///
/// The hydroelastic compliance type of the ground and ball can be independently
/// configured. Not every compliance configuration is compatible with every
/// contact model; contact between strict hydroelastic contact will fail for
/// two contacting objects with the same compliance type. So, it is important to
/// use the resulting plant with a contact model consistent with the
/// configuration of the ball and ground.
///
/// @param[in] radius
///   The radius of the ball.
/// @param[in] mass
///   The mass of the ball.
/// @param[in] hydroelastic_modulus
///   The modulus of elasticity for the ball. Only used when modeled with the
///   hydroelastic model. See @ref hydro_model_parameters
///   "hydroelastic contact parameters" documentation for details.
/// @param[in] dissipation
///   The Hunt & Crossley dissipation constant for the ball. Only used with the
///   hydroelastic model.  See @ref hydro_model_parameters
///   "hydroelastic contact parameters" documentation for details.
/// @param[in] surface_friction
///   The Coulomb's law coefficients of friction.
/// @param[in] gravity_W
///   The acceleration of gravity vector, expressed in the world frame W.
/// @param[in] rigid_sphere
///   If `true`, the sphere will have a _rigid_ hydroelastic representation
///   (soft otherwise).
/// @param[in] compliant_ground
///   If 'true', the ground will have a _soft_ hydroelastic representation
///   (rigid otherwise).
/// @param[in,out] plant
///   A valid pointer to a MultibodyPlant. This function will register
///   geometry for contact modeling.
/// @pre `plant` is not null, unfinalized, and is registered with a SceneGraph.
/// @note The given plant is not finalized. You must call Finalize() on the new
///   model once you are done populating it.
void PopulateBallPlant(
    double radius, double mass, double hydroelastic_modulus, double dissipation,
    const drake::multibody::CoulombFriction<double>& surface_friction,
    const Vector3<double>& gravity_W, bool rigid_sphere, bool compliant_ground,
    drake::multibody::MultibodyPlant<double>* plant);

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
