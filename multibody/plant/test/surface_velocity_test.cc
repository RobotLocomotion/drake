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
  const double surface_speed = 0.5;
  const Eigen::Vector3d velocity_n(1.0, 0.0, 0.0);

  geometry::ProximityProperties belt_props;
  belt_props.AddProperty(geometry::internal::kMaterialGroup,
                         geometry::internal::kFriction, friction);
  belt_props.AddProperty(geometry::internal::kMaterialGroup,
                         geometry::internal::kPointStiffness, belt_stiffness);
  belt_props.AddProperty(geometry::internal::kMaterialGroup,
                         geometry::internal::kHcDissipation, belt_dissipation);
  belt_props.AddProperty(geometry::internal::kSurfaceVelocityGroup,
                         geometry::internal::kSurfaceSpeed, surface_speed);
  belt_props.AddProperty(geometry::internal::kSurfaceVelocityGroup,
                         geometry::internal::kSurfaceVelocityNormal,
                         velocity_n);
  const double w = 3;
  const double d = 0.5;
  const double h = 0.1;
  geometry::GeometryId belt_geom_id = plant.RegisterCollisionGeometry(
      belt, math::RigidTransformd(Eigen::Vector3d(0., 0., 0.)),
      geometry::Box(Eigen::Vector3d(w, d, h)), "belt_collision",
      std::move(belt_props));

  plant.Finalize();
  std::unique_ptr<drake::systems::Context<double>> context =
      plant.CreateDefaultContext();
  auto diagram = builder.Build();

  // Set pose of body (and read it again just to make sure this
  // is doing what it is supposed to do).
  math::RigidTransformd body_pose(math::RollPitchYaw(0., 0., 0.),
                                  Eigen::Vector3d(0., 0., 1.));
  plant.SetFreeBodyPoseInWorldFrame(&(*context), belt, body_pose);
  math::RigidTransformd pose = plant.GetFreeBodyPose(*context, belt);

  // Assume there are some contacts on each face of the conveyor belt.
  // For ease of reading, these are expressed in coordinates of its body
  // frame and later will be transformed to world coordinates
  std::vector<Eigen::Vector3d> contacts_G = {
      {w / 2, 0., 0.},    // Contact at +x face
      {-w / 2, 0., 0.},   // Contact at -x face
      {0., d / 2, 0.},    // Contact at +y face
      {0., -d / 2, 0.},   // Contact at -y face
      {0., 0., h / 2},    // Contact at +z face
      {0., 0., -h / 2}};  // Contact at -z face

  for (const Eigen::Vector3d& c_G : contacts_G) {
    const Eigen::Vector3d c_W = pose * c_G;
    std::cout << "c_w " << c_W.x() << ", " << c_W.y() << ", " << c_W.z()
              << std::endl;
    Eigen::Vector3d surface_v = plant.GetSurfaceVelocity(
        belt_geom_id, scene_graph.model_inspector(), pose, c_W);
    // Verify the direction of surface velocity is equal to cross product
    // between the surface normal at each contact point and the velocity
    // normal vector
    EXPECT_LT(
        (surface_v.normalized() - (velocity_n.cross(c_G.normalized()))).norm(),
        1e-5);
    EXPECT_EQ(surface_v.norm(), surface_speed);
  }

  (void)ground_id;
  (void)belt_geom_id;
}

}  // namespace
}  // namespace multibody
}  // namespace drake