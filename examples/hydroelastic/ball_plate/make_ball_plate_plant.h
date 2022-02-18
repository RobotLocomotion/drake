#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace multibody {
namespace ball_plate {

/// This method makes a MultibodyPlant model of a ball falling on a dinner
/// plate. MultibodyPlant models the contact of the ball with the ground as a
/// inelastic collision (near zero coefficient of restitution), i.e.
/// energy is lost due to the collision.
///
/// @param[in] mbp_dt
///   The discrete update period (seconds) for MultibodyPlant modeled as a
//    discrete system.
/// @param[in] radius
///   The radius (meters) of the ball.
/// @param[in] mass
///   The mass (kg) of the ball.
/// @param[in] hydroelastic_modulus
///   The hydroelastic modulus of elasticity (Pa) of the ball and the floor.
/// @param[in] dissipation
///   The Hunt & Crossley dissipation constant (s/m) for the ball.
/// @param[in] surface_friction
///   The Coulomb's law coefficients (dimensionless) of friction.
/// @param[in] gravity_W
///   The acceleration (m/s²) of gravity vector, expressed in the world frame W.
/// @param[in] resolution_hint_factor
///   This scaling factor (dimensionless) multiplying the radius of the ball
///   gives the target edge length of the mesh on the surface of the ball.
///   It controls the ball's mesh resolution. The smaller number gives a finer
///   mesh with more tetrahedral elements.
/// @param scene_graph
///   If a SceneGraph is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for collision.
///   If this argument is omitted, no geometry will be registered.
/// @note The MultibodyPlant model is not finalized. You must call Finalize() on
/// the new model once you are done creating it.
///
/// See also
/// https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html
std::unique_ptr<drake::multibody::MultibodyPlant<double>>
MakeBallPlatePlant(
    double mbp_dt,
    double radius, double mass,
    double hydroelastic_modulus, double dissipation,
    const drake::multibody::CoulombFriction<double>& surface_friction,
    const Vector3<double>& gravity_W,
    double resolution_hint_factor,
    geometry::SceneGraph<double>* scene_graph = nullptr);

}  // namespace ball_plate
}  // namespace multibody
}  // namespace examples
}  // namespace drake
