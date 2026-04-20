#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::internal::kSurfaceSpeed;
using geometry::internal::kSurfaceVelocityGroup;
using geometry::internal::kSurfaceVelocityNormal;

// Simply exercises the MbP::GetSurfaceVelocity() API. Given arbitrary normal
// directions, we'll confirm we get the expected surface velocity, based on
// the registered geometry properties. Essentially, we're testing the property
// look up and calculations. No contact logic is tested here.
GTEST_TEST(MultibodyPlantTest, GetSurfaceVelocity) {
  constexpr double tol = 1e-5;

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

  const CoulombFriction<double> friction(0.5, 0.3);

  // The "conveyor belt". We don't actually need a body or a geometry registered
  // to it -- GetSurfaceVelocity() is provided the id and an inspector. However,
  // we use a body and the MbP API to register the collision geometry for
  // convenience.
  const RigidBody<double>& belt =
      plant.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());

  geometry::ProximityProperties belt_props;
  const double surface_speed = 0.5;
  const Vector3d velocity_n_G(1.0, 0.0, 0.0);
  belt_props.AddProperty(kSurfaceVelocityGroup, kSurfaceSpeed, surface_speed);
  belt_props.AddProperty(kSurfaceVelocityGroup, kSurfaceVelocityNormal,
                         velocity_n_G);

  // Note: the actual *shape* doesn't matter. We just need something that can
  // carry the surface velocity proximity properties.
  const GeometryId belt_geom_id = plant.RegisterCollisionGeometry(
      belt, math::RigidTransformd(), geometry::Sphere(1.0), "belt_collision",
      std::move(belt_props));

  // GetSurfaceVelocity() doesn't find the body pose for a geometry id, it is
  // provided. So, we can simply provide an arbitrary pose, as long as n_G is
  // consistent with X_WG, the results are meaningful.
  const double yaw = 0.78;
  math::RigidTransformd X_WG(math::RollPitchYaw(0.0, 0.0, yaw),
                             Vector3d(0.0, 0.0, 1.0));

  // A normal and the expected surface velocity for that normal.
  struct TestData {
    Vector3d n_G;
    Vector3d v_G_expect;
  };

  // Produce multiple contact points: one in the middle of each face. Normals
  // parallel with velocity_n_G (i.e., Gx) produce a zero surface velocity.
  const std::vector<TestData> test_data{
      {.n_G = Vector3d(1, 0, 0), .v_G_expect = Vector3d(0, 0, 0)},
      {.n_G = Vector3d(-1, 0, 0), .v_G_expect = Vector3d(0, 0, 0)},
      {.n_G = Vector3d(0, 1, 0), .v_G_expect = Vector3d(0, 0, surface_speed)},
      {.n_G = Vector3d(0, -1, 0), .v_G_expect = Vector3d(0, 0, -surface_speed)},
      {.n_G = Vector3d(0, 0, 1), .v_G_expect = Vector3d(0, -surface_speed, 0)},
      {.n_G = Vector3d(0, 0, -1), .v_G_expect = Vector3d(0, surface_speed, 0)},
  };
  for (const auto& data : test_data) {
    const Vector3d n_W = X_WG.rotation() * data.n_G;
    const Vector3d v_ss_G = plant.GetSurfaceVelocity(
        belt_geom_id, scene_graph.model_inspector(), X_WG, n_W);

    EXPECT_TRUE(CompareMatrices(v_ss_G, data.v_G_expect, tol))
        << fmt::format("with n_G = [{}]", fmt_eigen(data.n_G.transpose()));
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
