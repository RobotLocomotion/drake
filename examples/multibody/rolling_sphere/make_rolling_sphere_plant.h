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
/// @param[in] radius
///   The radius of the ball.
/// @param[in] mass
///   The mass of the ball.
/// @param[in] elastic_modulus
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
/// @param scene_graph
///   If a SceneGraph is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for collision.
///   If this argument is omitted, no geometry will be registered.
/// @note The MultibodyPlant model is not finalized. You must call Finalize() on
/// the new model once you are done creating it.
std::unique_ptr<drake::multibody::MultibodyPlant<double>>
MakeBouncingBallPlant(
    double radius, double mass,
    double elastic_modulus, double dissipation,
    const drake::multibody::CoulombFriction<double>& surface_friction,
    const Vector3<double>& gravity_W,
    geometry::SceneGraph<double>* scene_graph = nullptr);

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
