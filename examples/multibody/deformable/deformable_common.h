#pragma once

#include <string>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace deformable {

/* Adds collision and visual box geometry serving as the world's floor. The
 floor is placed just below the XY plane and assigned rigid proximity
 properties. */
void RegisterRigidGround(drake::multibody::MultibodyPlant<double>* plant);

}  // namespace deformable
}  // namespace examples
}  // namespace drake
