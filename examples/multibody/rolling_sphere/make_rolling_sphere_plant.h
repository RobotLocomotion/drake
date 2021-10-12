#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {

/// This method makes a MultibodyPlant model of a ball falling into a plane.
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
/// @param[in] mbp_dt
///   The discrete update period when MultibodyPlant is modeled as a discrete
///   system. If zero, the plant is modeled as a continuous system.
/// @param[in] radius
///   The radius of the ball.
/// @param[in] mass
///   The mass of the ball.
/// @param[in] hydroelastic_modulus
///   The modulus of elasticity for the ball. Only used when modeled with the
///   hydroelastic model. See @ref mbp_hydroelastic_materials_properties
///   "Hydroelastic contact" documentation for details.
/// @param[in] dissipation
///   The Hunt & Crossley dissipation constant for the ball. Only used with the
///   hydroelastic model.  See @ref mbp_hydroelastic_materials_properties
///   "Hydroelastic contact" documentation for details.
/// @param[in] surface_friction
///   The Coulomb's law coefficients of friction.
/// @param[in] gravity_W
///   The acceleration of gravity vector, expressed in the world frame W.
/// @param[in] rigid_sphere
///   If `true`, the sphere will have a _rigid_ hydroelastic representation
///   (soft otherwise).
/// @param[in] soft_ground
///   If 'true', the ground will have a _soft_ hydroelastic representation
///   (rigid otherwise).
/// @param scene_graph
///   If a SceneGraph is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for collision.
///   If this argument is omitted, no geometry will be registered.
/// @note The MultibodyPlant model is not finalized. You must call Finalize() on
/// the new model once you are done creating it.
std::unique_ptr<drake::multibody::MultibodyPlant<double>>
MakeBouncingBallPlant(
    double mbp_dt,
    double radius, double mass,
    double hydroelastic_modulus, double dissipation,
    const drake::multibody::CoulombFriction<double>& surface_friction,
    const Vector3<double>& gravity_W, bool rigid_sphere, bool soft_ground,
    geometry::SceneGraph<double>* scene_graph = nullptr);

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
