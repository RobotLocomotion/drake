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

/* Adds a deformable torus model with the given properties.
 @param[in,out] model             The deformable model to which the torus will
                                  be added.
 @param[in]     model_name        A unique name assigned to the torus instance.
 @param[in]     X_WB              The initial pose of the torus expressed in the
                                  world frame.
 @param[in]     deformable_config Deformable properties of the torus.
 @param[in]     scale             Scaling factor applied to the loaded torus
                                  mesh.
 @param[in]     contact_damping   Damping value assigned to the torus material.
 */
multibody::DeformableBodyId RegisterDeformableTorus(
    multibody::DeformableModel<double>* model, const std::string& model_name,
    const math::RigidTransformd& X_WB,
    const multibody::fem::DeformableBodyConfig<double>& deformable_config,
    double scale, double contact_damping);

}  // namespace deformable
}  // namespace examples
}  // namespace drake
