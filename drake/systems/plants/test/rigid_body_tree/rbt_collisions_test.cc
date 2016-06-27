#include <memory>

#include "drake/Path.h"
#include "drake/systems/plants/RigidBodyTree.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace rigid_body_tree {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;
using DrakeCollision::ElementId;

// Structure used to hold the analytical solution of the tests.
// It stores the collision point on the surface of a collision body in both
// world and body frames.
// See systems/plants/collision/test/model_test.cc.
struct SurfacePoint {
  SurfacePoint() { }
  SurfacePoint(Vector3d wf, Vector3d bf) : world_frame(wf), body_frame(bf) { }
  // Eigen variables are left uninitalized by default.
  Vector3d world_frame;
  Vector3d body_frame;
};

// Solutions are accessed by collision element id using an std::unordered_set.
// See detailed explanation in systems/plants/collision/test/model_test.cc.
typedef std::unordered_map<DrakeCollision::ElementId, SurfacePoint>
    ElementToSurfacePointMap;

class RBTCollisionTest: public ::testing::Test {
 protected:
  void SetUp() override {
    tree_.addRobotFromSDF(
        Drake::getDrakePath() +
            "/systems/plants/test/rigid_body_tree/small_box_on_large_box.sdf",
        DrakeJoint::QUATERNION);

    small_box_id_ = tree_.FindBody("small_box")->collision_element_ids[0];
    large_box_id_ = tree_.FindBody("large_box")->collision_element_ids[0];

    // Access the analytical solution to the contact point on the surface of
    // each collision element by element id.
    // Solutions are expressed in world and body frames.
    solution_ = {
        /*              world frame    , body frame  */
        {large_box_id_, {{0.0, 5.0, 0.0}, {0.0, 2.5, 0.0}}},
        {small_box_id_, {{0.0, 4.9, 0.0}, {0.0, 0.0, 0.6}}}};
  }

  double tolerance_;
  RigidBodyTree tree_;
  ElementId small_box_id_, large_box_id_;
  ElementToSurfacePointMap solution_;
};

TEST_F(RBTCollisionTest, FindAndComputeContactPoints) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 4.0e-16;

  int nq = tree_.number_of_positions();
  int nv = tree_.number_of_velocities();
  int num_states = nq + nv;
  VectorXd x = VectorXd::Zero(num_states);
  x.head(nq) = tree_.getZeroConfiguration();

  auto q = x.topRows(nq);
  auto v = x.bottomRows(nv);

  KinematicsCache<double> kinsol = tree_.doKinematics(q, v);

  std::vector<RigidBodyCollisionPair> collision_pairs =
      tree_.ComputeMaximumDepthCollisionPoints(
      kinsol, false);

  // RigidBodyTree::ComputeMaximumDepthCollisionPoints returns only one point,
  // the maximum depth collision point.
  ASSERT_EQ(1, collision_pairs.size());

  ElementId bodyA_collision_element_id =
      collision_pairs[0].bodyA_.collision_element_ids[0];
  ElementId bodyB_collision_element_id =
      collision_pairs[0].bodyB_.collision_element_ids[0];

  EXPECT_NEAR(-0.1, collision_pairs[0].distance_, tolerance_);
  EXPECT_TRUE(collision_pairs[0].normal_.isApprox(Vector3d(0.0, -1.0, 0.0)));
  // Collision points are reported on each of the respective bodies' frames.
  // Only test for vertical position.
  EXPECT_NEAR(collision_pairs[0].ptA_.y(),
              solution_[bodyA_collision_element_id].body_frame.y(), tolerance_);
  // In body's frame, which is rotated 90 degrees in pitch from
  // collision_test.sdf, the collision point is on the z-axis.
  // In addition, is not at z=0.5 but at z=0.6 since there is an offset of 0.1
  // in body's z-axis for the collision element as set from collision_test.sdf.
  EXPECT_NEAR(collision_pairs[0].ptB_.z(),
              solution_[bodyB_collision_element_id].body_frame.z(), tolerance_);
}

}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
