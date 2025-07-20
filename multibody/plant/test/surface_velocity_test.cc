#include <iostream>
#include <gtest/gtest.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(SurfaceVelocityTest, CheckParameterRegistration) {
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

  CoulombFriction<double> friction(0.5, 0.3);

  // Create ground
  geometry::GeometryId ground_id = plant.RegisterCollisionGeometry(
      plant.world_body(),
      geometry::HalfSpace::MakePose(Eigen::Vector3d::UnitY(),
                                    Eigen::Vector3d::Zero()),
      geometry::HalfSpace(), "floor", friction);

  // Crate an thin, elongated box that ressembles a conveyor belt
  const RigidBody<double>& belt =
      plant.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());
  const double belt_stiffness = 980;
  const double belt_dissipation = 3.2;

  geometry::ProximityProperties belt_props;
  belt_props.AddProperty(geometry::internal::kMaterialGroup,
                         geometry::internal::kFriction, friction);
  belt_props.AddProperty(geometry::internal::kMaterialGroup,
                         geometry::internal::kPointStiffness, belt_stiffness);
  belt_props.AddProperty(geometry::internal::kMaterialGroup,
                         geometry::internal::kHcDissipation, belt_dissipation);
  belt_props.AddProperty(geometry::internal::kSurfaceVelocityGroup,
                         geometry::internal::kSurfaceSpeed, 0.5);
  belt_props.AddProperty(geometry::internal::kSurfaceVelocityGroup,
                         geometry::internal::kSurfaceVelocityNormal,
                         Eigen::Vector3d(1.0, 0.0, 0.0));
  geometry::GeometryId belt_geom_id = plant.RegisterCollisionGeometry(
      belt, math::RigidTransformd(Eigen::Vector3d(0., 0., 1.)),
      geometry::Box(Eigen::Vector3d(3, 0.5, 0.1)), "belt_collision",
      std::move(belt_props));

//  const double pz_WS1 =
//             plant_->GetFreeBodyPose

  plant.Finalize();
  std::unique_ptr<drake::systems::Context<double>> context = plant.CreateDefaultContext();
  auto diagram = builder.Build();

  const geometry::VolumeMesh<double>* volume_mesh =
      scene_graph.model_inspector().GetReferenceMesh(belt_geom_id);
  if (volume_mesh) {
    std::cout << "Elements: " << volume_mesh->num_elements() << std::endl;
    std::cout << "Vertices: " << volume_mesh->num_vertices() << std::endl;
  } else {
    std::cout << "No volume Mesh" << std::endl;
  }
  (void)ground_id;
  (void)belt_geom_id;
}

}  // namespace
}  // namespace multibody
}  // namespace drake