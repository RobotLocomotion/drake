#include "drake/examples/multibody/deformable/point_source_force_field.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace examples {
namespace deformable {
namespace {

using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::systems::Context;
using Eigen::Vector3d;

GTEST_TEST(PointSourceForceFieldTest, EvaluateAt) {
  MultibodyPlant<double> plant(0.01);
  const RigidBody<double>& box = plant.AddRigidBody(
      "box", SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1));
  /* The fixed offset from the body origin B to the point source of
   the force field C. */
  const Vector3d p_BC(0, 0, 0.123);
  const double max_distance = 0.2;
  PointSourceForceField force_field(plant, box, p_BC, max_distance);
  force_field.DeclareSystemResources(&plant);
  plant.Finalize();

  auto context = plant.CreateDefaultContext();
  const RigidTransformd X_WB(RollPitchYawd(1, 2, 3), Vector3d(3, 4, 5));
  plant.SetFreeBodyPose(context.get(), box, X_WB);
  const double max_force_density = 42.0;
  /* Turn the force on. */
  force_field.maximum_force_density_input_port().FixValue(context.get(),
                                                          max_force_density);
  const Vector3d p_WC = X_WB * p_BC;
  /* Inside the non-zero range. */
  const Vector3d p_CQ1_W = Vector3d(0, 0, 0.1);
  const Vector3d p_WQ1 = p_WC + p_CQ1_W;
  Vector3d expected_force =
      -0.1 / max_distance * max_force_density * p_CQ1_W.normalized();
  EXPECT_TRUE(CompareMatrices(force_field.EvaluateAt(*context, p_WQ1),
                              expected_force, 1e-13));
  /* Outside the non-zero range. */
  const Vector3d p_CQ2_W = Vector3d(0, 0, 0.3);
  const Vector3d p_WQ2 = p_WC + p_CQ2_W;
  EXPECT_TRUE(CompareMatrices(force_field.EvaluateAt(*context, p_WQ2),
                              Vector3d::Zero()));
  /* Turn the force off. */
  force_field.maximum_force_density_input_port().FixValue(context.get(), 0.0);
  EXPECT_TRUE(CompareMatrices(force_field.EvaluateAt(*context, p_WQ1),
                              Vector3d::Zero()));
}

}  // namespace
}  // namespace deformable
}  // namespace examples
}  // namespace drake
