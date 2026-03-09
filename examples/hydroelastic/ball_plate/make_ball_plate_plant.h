#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace ball_plate {

/** This function modifies a MultibodyPlant by adding a ball falling on a dinner
 plate. The plate and floor are read from sdf files but the ball is
 constructed programmatically.

 @param[in] radius
   The radius (meters) of the ball.
 @param[in] mass
   The mass (kg) of the ball.
 @param[in] hydroelastic_modulus
   The hydroelastic modulus (Pa) of the ball.
 @param[in] dissipation
   The Hunt & Crossley dissipation constant (s/m) for the ball.
 @param[in] surface_friction
   The Coulomb's law coefficients (unitless) of friction of the ball.
 @param[in] resolution_hint_factor
   This scaling factor (unitless) multiplied by the radius of the ball
   gives the target edge length of the mesh on the surface of the ball.
   The smaller number gives a finer mesh with more tetrahedral elements.
 @param[in,out] plant
   The bodies will be added here.

 @pre `plant` is not null.

 See also
 https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html
*/
void AddBallPlateBodies(
    double radius, double mass, double hydroelastic_modulus, double dissipation,
    const multibody::CoulombFriction<double>& surface_friction,
    double resolution_hint_factor, multibody::MultibodyPlant<double>* plant);

}  // namespace ball_plate
}  // namespace examples
}  // namespace drake
