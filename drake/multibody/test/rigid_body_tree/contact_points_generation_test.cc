#include "drake/multibody/rigid_body_tree.h"

#include <string>

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

  // Returns the drake-relative path to the test urdf file.
  virtual std::string get_urdf_file() const = 0;
 protected:
  void SetUp() override {
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + get_urdf_file(),
        drake::multibody::joints::kQuaternion, &tree_);
  }

  RigidBodyTree<double> tree_;
};

class ContactGenerationTest : public RBTContactPointsTest {
 public:
  std::string get_urdef_file() const override {
    return "/multibody/test/rigid_body_tree/contact_points_generation.urdf";
  }
};

// This tests the case where only the right types of collision elements create
// contact points.
TEST_F(ContactGenerationTest, GeneratingElementsTest) {
}

//GTEST_TEST()
}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake

}