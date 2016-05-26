#include <memory>
#include <stdexcept>
#include <string>

#include "drake/Path.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/util/testUtil.h"

//#include "drake/systems/plants/BotVisualizer.h"
//#include "drake/systems/plants/RigidBodySystem.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;
//using Drake::BotVisualizer;
//using Drake::RigidBodySystem;

class RBTCollisionTest: public ::testing::Test {
 protected:
  void SetUp() override {
    tree_.addRobotFromSDF(
        Drake::getDrakePath() + "/systems/plants/test/collision_test.sdf",
        DrakeJoint::QUATERNION);
  }

  RigidBodyTree tree_;
};

TEST_F(RBTCollisionTest, FindAndComputeContactPoints) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  const double tolerance = 2.0e-9;

  int nq = tree_.number_of_positions();
  int nv = tree_.number_of_velocities();
  int num_states = nq + nv;
  VectorXd x = VectorXd::Zero(num_states);
  x.head(nq) = tree_.getZeroConfiguration();

  auto q = x.topRows(nq);
  auto v = x.bottomRows(nv);

  auto kinsol = tree_.doKinematics(q, v);

  VectorXd phi;
  Matrix3Xd normal, xA, xB;
  std::vector<int> bodyA_idx, bodyB_idx;

  tree_.ComputeMaximumDepthCollisionPoints(
      kinsol, phi, normal, xA, xB, bodyA_idx, bodyB_idx, false);

#if 0
  auto lcm = std::make_shared<lcm::LCM>();
  auto tree_ptr = std::shared_ptr<RigidBodyTree>(&tree_);
  auto visualizer = std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(
      lcm, tree_ptr);
  visualizer->output(0.0, x, x);
#endif

  ASSERT_EQ(1, phi.size());
  EXPECT_NEAR(-0.1, phi(0), tolerance);
  EXPECT_TRUE(normal.col(0).isApprox(Vector3d(0.0, -1.0, 0.0)));
  // Collision points are reported on each of the respective bodies' frames.
  // Only test for vertical position.
  EXPECT_NEAR(xA.col(0).y(), 2.5, tolerance);
  // In body's frame, which is rotated 90 degrees in pitch from
  // collision_test.sdf, the collision point is on the z-axis.
  // In addition, is not at z=0.5 but at z=0.6 since there is an offset of 0.1
  // in body's z-axis for the collision element as set from collision_test.sdf.
  EXPECT_NEAR(xB.col(0).z(), 0.6, tolerance);

  for(int i=0;i<phi.size();++i) {
    PRINT_VAR(i);
    PRINT_VAR(tree_.bodies[bodyA_idx[i]]->name());
    PRINT_VAR(tree_.bodies[bodyB_idx[i]]->name());
    PRINT_VAR(phi(i));
    PRINT_VAR(normal.col(i).transpose());
    PRINT_VAR(xA.col(i).transpose());
    PRINT_VAR(xB.col(i).transpose());
  }

}

} // namespace
} // namespace drake