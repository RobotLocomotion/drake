#include "drake/multibody/rigid_body_tree.h"

#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/parser_urdf.h"

// Tests the functionality that computes the contact points for each body.
// A body's contact points depend on its collision elements.  These tests
// assume correct parsing and simply evaluate the final state of the body
// contact points, confirming they are correctly evaluated.
namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace rigid_body_tree {
namespace {

using drake::parsers::urdf::AddModelInstanceFromUrdfFile;

// Tests
// 1. Test that only the right *type* of elements are giving contact points.
//    Single link with collision elements of as many types as possible
//    confirm that I have only the requisite number and values of points.
// 2. Confirm that the group_name value works appropriately.
//    Another urdf with elements selected by

// Base class for the common functionality between the various tests.  Provides
// functionality for loading a urdf file and coordinating the expected answers.
class RBTContactPointsTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Populates an ordered set of expected contact points.  The number and order
  // of the values are directly related to the structure of the URDF.  A change
  // in the URDF *may* require a change in these values.
  virtual void initialize_expected_points() = 0;

  // Returns the drake-relative path to the test urdf file.
  virtual std::string get_urdf_file() const = 0;

 protected:
  void SetUp() override {
    initialize_expected_points();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        drake::GetDrakePath() + get_urdf_file(),
        drake::multibody::joints::kQuaternion, &tree_);
  }

  RigidBodyTree<double> tree_;
  Eigen::Matrix3Xd expected_points_;
};

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
    // sphere.
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

//GTEST_TEST()
}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
