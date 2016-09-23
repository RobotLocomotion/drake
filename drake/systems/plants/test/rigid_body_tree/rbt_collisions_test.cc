#include <memory>

#include "drake/common/drake_path.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_sdf.h"

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

// Solutions are accessed by rigid body pointer using an std::unordered_map.
// DrakeCollision::Model returns the collision detection results as a vector of
// DrakeCollision::PointPair entries. Each entry holds a reference to the pair
// of collision elements taking part in the collision.
// To provide a simple access to the appropriate solution for the collision
// point on each body, a map is used to allow referencing the corresponding
// solution by body pointer.
typedef std::unordered_map<const RigidBody*, SurfacePoint>
    BodyToSurfacePointMap;

class RBTCollisionTest: public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void SetUp() override {
    drake::parsers::sdf::AddModelInstancesFromSdfFileInWorldFrame(
        drake::GetDrakePath() +
        "/systems/plants/test/rigid_body_tree/small_sphere_on_large_box.sdf",
            drake::systems::plants::joints::kQuaternion, &tree_);

    small_sphere_ = tree_.FindBody("small_sphere");
    large_box_ = tree_.FindBody("large_box");

    // Access the analytical solution to the contact point on the surface of
    // each collision element by element id.
    // Solutions are expressed in world and body frames.
    solution_ = {
        /*              world frame    , body frame  */
        {large_box_,    {{0.0, 5.0, 0.0}, {0.0, 2.5, 0.0}}},
        {small_sphere_, {{0.0, 4.9, 0.0}, {0.0, 0.0, 0.6}}}};
  }

  double tolerance_;
  RigidBodyTree tree_;
  const RigidBody *small_sphere_, *large_box_;
  BodyToSurfacePointMap solution_;
};

// This unit test assesses the correct return from
// RigidBodyTree::ComputeMaximumDepthCollisionPoints.
// The test consists on finding the maximum depth penetration point between a
// sphere and a box.
TEST_F(RBTCollisionTest, FindAndComputeContactPoints) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 4.0*Eigen::NumTraits<double>::epsilon();

  int nq = tree_.get_num_positions();
  int nv = tree_.get_num_velocities();
  int num_states = nq + nv;
  VectorXd x = VectorXd::Zero(num_states);
  x.head(nq) = tree_.getZeroConfiguration();

  auto q = x.topRows(nq);
  auto v = x.bottomRows(nv);

  KinematicsCache<double> kinsol = tree_.doKinematics(q, v);

  std::vector<DrakeCollision::PointPair> collision_pairs =
      tree_.ComputeMaximumDepthCollisionPoints(
      kinsol, false);

  // RigidBodyTree::ComputeMaximumDepthCollisionPoints returns only one point,
  // the maximum depth collision point.
  ASSERT_EQ(1u, collision_pairs.size());

  const RigidBody* bodyA = collision_pairs[0].elementA->get_body();
  const RigidBody* bodyB = collision_pairs[0].elementB->get_body();

  EXPECT_NEAR(-0.1, collision_pairs[0].distance, tolerance_);
  EXPECT_TRUE(collision_pairs[0].normal.isApprox(Vector3d(0.0, -1.0, 0.0)));

  // Collision points are reported on each of the respective bodies' frames.
  EXPECT_TRUE(collision_pairs[0].ptA.isApprox(
      solution_[bodyA].body_frame, tolerance_));

  // In body's frame, which is rotated 90 degrees in pitch from
  // collision_test.sdf, the collision point is on the z-axis.
  // In addition, is not at z=0.5 but at z=0.6 since there is an offset of 0.1
  // in body's z-axis for the collision element as set from collision_test.sdf.
  EXPECT_TRUE(collision_pairs[0].ptB.isApprox(
      solution_[bodyB].body_frame, tolerance_));
}

}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
