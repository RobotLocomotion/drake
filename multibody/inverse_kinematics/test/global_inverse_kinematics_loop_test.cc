#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/inverse_kinematics/global_inverse_kinematics.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;

GTEST_TEST(GlobalInverseKinematicsTest, RejectsKinematicLoops) {
  MultibodyPlant<double> plant(0.0);
  const auto inertia = SpatialInertia<double>::MakeUnitary();
  const auto& ground = plant.AddRigidBody("ground", inertia);
  const auto& body_a = plant.AddRigidBody("a", inertia);
  const auto& body_b = plant.AddRigidBody("b", inertia);
  plant.WeldFrames(plant.world_frame(), ground.body_frame());

  // Three revolute joints form a triangle assembled at the default positions.
  plant.AddJoint<RevoluteJoint>("ground_a", ground,
                                RigidTransformd(Vector3d(-1, 0, 0)), body_a,
                                RigidTransformd(), Vector3d::UnitZ());
  plant.AddJoint<RevoluteJoint>("ground_b", ground,
                                RigidTransformd(Vector3d(1, 0, 0)), body_b,
                                RigidTransformd(), Vector3d::UnitZ());
  plant.AddJoint<RevoluteJoint>(
      "a_b", body_a, RigidTransformd(Vector3d(1, 1, 0)), body_b,
      RigidTransformd(Vector3d(-1, 1, 0)), Vector3d::UnitZ());
  plant.Finalize();
  ASSERT_EQ(plant.num_loop_constraints(), 1);

  DRAKE_EXPECT_THROWS_MESSAGE(
      GlobalInverseKinematics(plant),
      "GlobalInverseKinematics does not support kinematic loops\\.");
}

}  // namespace
}  // namespace multibody
}  // namespace drake
