#include "drake/examples/multibody/deformable/deformable_common.h"

#include <memory>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/plant/multibody_plant.h"

using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::Mesh;
using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::DeformableBodyId;
using drake::multibody::DeformableModel;
using drake::multibody::MultibodyPlant;
using drake::multibody::fem::DeformableBodyConfig;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace drake {
namespace examples {
namespace deformable {

// TODO(xuchenhan-tri): Instead of defining the ground model in code, it should
// come from a data file, e.g. a "ground.sdf" in this directory, which is loaded
// via the parser.
void RegisterRigidGround(MultibodyPlant<double>* plant) {
  /* Minimum required proximity properties for rigid bodies to interact with
   deformable bodies.
   1. A valid Coulomb friction coefficient, and
   2. A resolution hint. (Rigid bodies need to be tessellated so that collision
   queries can be performed against deformable geometries.) The value dictates
   how fine the mesh used to represent the rigid collision geometry is. */
  ProximityProperties rigid_proximity_props;
  /* Set the friction coefficient close to that of rubber against rubber. */
  const CoulombFriction<double> surface_friction(1.15, 1.15);
  AddContactMaterial({}, {}, surface_friction, &rigid_proximity_props);
  rigid_proximity_props.AddProperty(geometry::internal::kHydroGroup,
                                    geometry::internal::kRezHint, 0.01);
  /* Set up a ground. */
  Box ground{4, 4, 4};
  const RigidTransformd X_WG(Eigen::Vector3d{0, 0, -2});
  plant->RegisterCollisionGeometry(plant->world_body(), X_WG, ground,
                                   "ground_collision", rigid_proximity_props);
  IllustrationProperties illustration_props;
  illustration_props.AddProperty("phong", "diffuse",
                                 Vector4d(0.7, 0.5, 0.4, 0.8));
  plant->RegisterVisualGeometry(plant->world_body(), X_WG, ground,
                                "ground_visual", std::move(illustration_props));
}

DeformableBodyId RegisterDeformableTorus(
    DeformableModel<double>* model, const std::string& model_name,
    const RigidTransformd& X_WB,
    const DeformableBodyConfig<double>& deformable_config, double scale,
    double contact_damping) {
  /* Load the torus mesh and apply scaling. */
  const std::string torus_vtk = FindResourceOrThrow(
      "drake/examples/multibody/deformable/models/torus.vtk");
  auto torus_mesh = std::make_unique<Mesh>(torus_vtk, scale);
  auto torus_instance = std::make_unique<GeometryInstance>(
      X_WB, std::move(torus_mesh), model_name);

  /* Minimally required proximity properties for deformable bodies: A valid
   Coulomb friction coefficient. */
  ProximityProperties deformable_proximity_props;
  const CoulombFriction<double> surface_friction(1.15, 1.15);
  AddContactMaterial(contact_damping, {}, surface_friction,
                     &deformable_proximity_props);
  torus_instance->set_proximity_properties(deformable_proximity_props);

  /* Registration of all deformable geometries ostensibly requires a resolution
   hint parameter that dictates how the shape is tessellated. In the case of a
   `Mesh` shape, the resolution hint is unused because the shape is already
   tessellated. */
  // TODO(xuchenhan-tri): Though unused, we still asserts the resolution hint is
  // positive. Remove the requirement of a resolution hint for meshed shapes.
  const double unused_resolution_hint = 1.0;
  return model->RegisterDeformableBody(
      std::move(torus_instance), deformable_config, unused_resolution_hint);
}

}  // namespace deformable
}  // namespace examples
}  // namespace drake
