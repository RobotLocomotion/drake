#pragma once

#include <memory>

#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

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
/// @param[in] gravity_W
///   The acceleration of gravity vector, expressed in the world frame W.
/// @param geometry_system
///   If a GeometrySystem is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for collision.
///   If this argument is omitted, no geometry will be registered.
std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeBouncingBallPlant(
    double radius, double mass, const Vector3<double>& gravity_W,
    geometry::GeometrySystem<double>* geometry_system = nullptr);

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
