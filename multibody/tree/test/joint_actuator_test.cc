#include "drake/multibody/tree/joint_actuator.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace {

constexpr double kPostiveEffortLimit = 1e3;
constexpr double kZeroEffortLimit = 0;
constexpr double kNegativeEffortLimit = -2;

GTEST_TEST(JointActuatorTest, JointAcutatorLimitTest) {
  auto tree_pointer = std::make_unique<internal::MultibodyTree<double>>();
  internal::MultibodyTree<double>& tree = *tree_pointer;

  // Spatial inertia for adding body. The actual value is not important for
  // these tests and therefore we do not initialize it.
  const SpatialInertia<double> M_B;  // Default construction is ok for this.

  // Add bodies so we can add joints to them.
  const auto body1 = &tree.AddBody<RigidBody>(M_B);
  const auto body2 = &tree.AddBody<RigidBody>(M_B);
  const auto body3 = &tree.AddBody<RigidBody>(M_B);

  // Add a prismatic joint between the world and body1:
  const Joint<double>& body1_world =
      tree.AddJoint(std::make_unique<PrismaticJoint<double>>(
          "prism1", tree.world_body().body_frame(), body1->body_frame(),
          Eigen::Vector3d(0, 0, 1)));

  tree.AddJointActuator("act1", body1_world, kPostiveEffortLimit);
  // Validate the actuator effort limit has been setup correctly.
  const auto& actuator1 = tree.GetJointActuatorByName("act1");
  EXPECT_EQ(actuator1.effort_limit(), kPostiveEffortLimit);

  // Throw is the effort limit is set to 0.
  const Joint<double>& body2_body1 =
      tree.AddJoint(std::make_unique<PrismaticJoint<double>>(
          "prism2", body1->body_frame(), body2->body_frame(),
          Eigen::Vector3d(0, 0, 1)));
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree.AddJointActuator("act2", body2_body1, kNegativeEffortLimit),
      std::runtime_error, "Effort limit must be strictly positive!");

  // Throw is the effort limit is set to be negative.
  const Joint<double>& body3_body2 =
      tree.AddJoint(std::make_unique<PrismaticJoint<double>>(
          "prism3", body2->body_frame(), body3->body_frame(),
          Eigen::Vector3d(0, 0, 1)));
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree.AddJointActuator("act3", body3_body2, kZeroEffortLimit),
      std::runtime_error, "Effort limit must be strictly positive!");

  tree.Finalize();
}

}  // namespace
}  // namespace multibody
}  // namespace drake
