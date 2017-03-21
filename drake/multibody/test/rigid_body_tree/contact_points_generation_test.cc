/* clang-format off */
#include "drake/multibody/rigid_body_tree.h"
/* clang-format on */

#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/urdf_parser.h"

// Tests the functionality that computes the contact points for each body.
// A body's contact points depend on its collision elements.  These tests
// assume correct parsing and simply evaluate the final state of the body
// contact points, confirming they are correctly evaluated.
namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_tree {
namespace {

// Base class for the common functionality between the various tests.  Provides
// functionality for loading a urdf file and coordinating the expected answers.
class RBTContactPointsTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  // Populates an ordered set of expected contact points.  The number and order
  // of the values are directly related to the structure of the URDF.  A change
  // in the URDF *may* require a change in these values.
  virtual void initialize_expected_points() = 0;

  // Returns the drake-relative path to the test urdf file.
  virtual std::string get_urdf_file() const = 0;

  void SetUp() override {
    initialize_expected_points();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        drake::GetDrakePath() + get_urdf_file(),
        drake::multibody::joints::kQuaternion, &tree_);
  }

  RigidBodyTree<double> tree_;
  Eigen::Matrix3Xd expected_points_;
};

// Infrastructure for testing the *types* of geometries which create contact
// points.
class ContactGenerationTest : public RBTContactPointsTest {
 public:
  std::string get_urdf_file() const override {
    return "/multibody/test/rigid_body_tree/contact_points_generation.urdf";
  }

  void initialize_expected_points() override {
    const int kPointCount = 9;
    expected_points_.resize(Eigen::NoChange, kPointCount);
    Eigen::RowVectorXd cx(kPointCount), cy(kPointCount), cz(kPointCount);
    // The first eight columns represent the box's corners, the ninth is the
    // sphere with radius < 1e-6.
    cx << -1, 1, 1, -1, -1, 1, 1, -1, -1;
    cy << 1, 1, 1, 1, -1, -1, -1, -1, 0;
    cz << 1, 1, -1, -1, -1, -1, 1, 1, 0;
    expected_points_ << cx, cy, cz;
  }
};

// This tests the case where only the right types of collision elements create
// contact points.
TEST_F(ContactGenerationTest, GeneratingElementsTest) {
  RigidBody<double>* body = tree_.FindBody("body");
  const auto& contact_points = body->get_contact_points();
  ASSERT_EQ(expected_points_.cols(), contact_points.cols());
  ASSERT_TRUE(expected_points_.isApprox(contact_points));
}

// Infrastructure for testing the the group name filters for contact points.
class ContactGroupNameTest : public RBTContactPointsTest {
 public:
  std::string get_urdf_file() const override {
    return "/multibody/test/rigid_body_tree/contact_points_group_name.urdf";
  }

  void initialize_expected_points() override {
    const int kPointCount = 4;
    expected_points_.resize(Eigen::NoChange, kPointCount);
    Eigen::RowVectorXd cx(kPointCount), cy(kPointCount), cz(kPointCount);
    // There are four spheres placed along the x-axis at 0, 1, 2, & 3,
    // respectively.  The first is in group "zero", the second in "one", and
    // the last *two* in "two".
    cx << 0, 1, 2, 3;
    cy << 0, 0, 0, 0;
    cz << 0, 0, 0, 0;
    expected_points_ << cx, cy, cz;
  }
};

// This tests the case where only the right types of collision elements create
// contact points.
TEST_F(ContactGroupNameTest, GroupNameContactPointsTest) {
  RigidBody<double>* body = tree_.FindBody("body");

  // Test the default case; should get three points, one for each small sphere.
  const auto& contact_points = body->get_contact_points();
  ASSERT_EQ(expected_points_.cols(), contact_points.cols());
  EXPECT_TRUE(expected_points_.isApprox(contact_points));

  // Tests for individual groups.
  Eigen::Matrix3Xd group_points;

  tree_.getTerrainContactPoints(*body, &group_points, "zero");
  ASSERT_EQ(1, group_points.cols());
  EXPECT_TRUE(expected_points_.col(0).isApprox(group_points));

  tree_.getTerrainContactPoints(*body, &group_points, "one");
  ASSERT_EQ(1, group_points.cols());
  EXPECT_TRUE(expected_points_.col(1).isApprox(group_points));

  tree_.getTerrainContactPoints(*body, &group_points, "two");
  ASSERT_EQ(2, group_points.cols());
  EXPECT_TRUE(expected_points_.block(0, 2, 3, 2).isApprox(group_points));
}
}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace multibody
}  // namespace drake
