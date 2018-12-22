#include "drake/examples/multibody/cylinder_with_multicontact/make_cylinder_plant.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cylinder_with_multicontact {
namespace {

using drake::geometry::SceneGraph;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using Eigen::Vector3d;

GTEST_TEST(MakeCylinderPlant, VerifyPlant) {
  // Plant's parameters.
  const double radius = 0.05;          // The cylinder's radius, m
  const double mass = 0.1;             // The cylinder's mass, kg
  const double g = 9.81;               // Acceleration of gravity, m/s^2
  const double length = 4.0 * radius;  // The cylinder's length, m.
  const CoulombFriction<double> coulomb_friction(
      0.5 /* static friction */,
      0.5 /* dynamic friction */);

  // Time stepping step size.
  const double time_step = 1.0e-3;

  SceneGraph<double> scene_graph;

  auto plant = MakeCylinderPlant(
      radius, length, mass, coulomb_friction, -g * Vector3d::UnitZ(),
      time_step, &scene_graph);

  EXPECT_EQ(plant->num_velocities(), 6);
  EXPECT_EQ(plant->num_positions(), 7);
  EXPECT_EQ(plant->time_step(), time_step);
  EXPECT_TRUE(plant->geometry_source_is_registered());

  ASSERT_TRUE(plant->HasBodyNamed("Cylinder"));
  const RigidBody<double>& cylinder =
      plant->GetRigidBodyByName("Cylinder");
  EXPECT_EQ(cylinder.get_default_mass(), mass);

  // Verify the value of the inertial properties for the cylinder.
  const SpatialInertia<double>& M_Bcm_B = cylinder.default_spatial_inertia();

  // Unit inertia about the revolute axis, the z axis, of the cylinder.
  const double G_axis = radius * radius / 2.0;
  // Unit inertia about any axis perpendicular to the revolute axis passing
  // through the COM.
  const double G_perp = (3.0 * radius * radius + length * length) / 12.0;

  EXPECT_EQ(M_Bcm_B.get_mass(), mass);
  EXPECT_EQ(M_Bcm_B.get_com(), Vector3d::Zero());
  EXPECT_EQ(M_Bcm_B.get_unit_inertia().get_moments(),
            Vector3d(G_perp, G_perp, G_axis));
  EXPECT_EQ(M_Bcm_B.get_unit_inertia().get_products(), Vector3d::Zero());

  // TODO(amcastro-tri): Verify geometry including:
  //  - The number of registered geometries.
  //  - Whether it is for visualization or contact modeling.
  //  - Obtain geometry specifics; type, radius, length, etc.
  //  - Obtain friction coefficients for each geometry.
  // However, we need proper SceneGraph/MBP APIs to enable this.

  // TODO(amcastro-tri): verify gravity is correct.
  // We need an API to retrieve the gravity force element, #9253.
}

}  // namespace
}  // namespace cylinder_with_multicontact
}  // namespace multibody
}  // namespace examples
}  // namespace drake
