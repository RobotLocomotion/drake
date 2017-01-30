#include "drake/multibody/rigid_body_tree.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/test/rigid_body_tree/rigid_body_tree_compare_to_clone.h"

using std::unique_ptr;

namespace drake {

using multibody::CompareToClone;
using parsers::ModelInstanceIdTable;
using parsers::sdf::AddModelInstancesFromSdfFileToWorld;
using parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace systems {
namespace plants {
namespace test {
namespace {

class RigidBodyTreeCloneTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree_ = std::make_unique<RigidBodyTree<double>>();
  }
  unique_ptr<RigidBodyTree<double>> tree_;
};

// Tests RigidBodyTree::Clone() using a simple two DOF robot.
TEST_F(RigidBodyTreeCloneTest, CloneTwoDofRobot) {
  std::string filename = drake::GetDrakePath() +
      "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
  AddModelInstanceFromUrdfFileWithRpyJointToWorld(filename, tree_.get());
  EXPECT_TRUE(CompareToClone(*tree_));
}

// Tests RigidBodyTree::Clone() using Atlas
TEST_F(RigidBodyTreeCloneTest, CloneAtlas) {
  std::string filename = drake::GetDrakePath() +
      "/examples/Atlas/urdf/atlas_convex_hull.urdf";
  AddModelInstanceFromUrdfFileToWorld(filename, multibody::joints::kQuaternion,
      tree_.get());
  EXPECT_TRUE(CompareToClone(*tree_));
}

// Tests RigidBodyTree::Clone() using a Prius with LIDAR sensors.
TEST_F(RigidBodyTreeCloneTest, ClonePrius) {
  std::string filename = drake::GetDrakePath() +
     "/automotive/models/prius/prius_with_lidar.sdf";
  AddModelInstancesFromSdfFileToWorld(filename, multibody::joints::kQuaternion,
      tree_.get());
  EXPECT_TRUE(CompareToClone(*tree_));
}

// Tests RigidBodyTree::Clone() using Valkyrie
TEST_F(RigidBodyTreeCloneTest, CloneValkyrie) {
  std::string filename = drake::GetDrakePath() +
      "/examples/Valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf";
  // While it may seem odd to use a fixed floating joint with Valkyrie, it is
  // used in this case just to confirm that RigidBodyTree::Clone() works with
  // this type of joint. Previous unit tests already cover the quaternion
  // floating joint type.
  AddModelInstanceFromUrdfFileToWorld(filename, multibody::joints::kFixed,
      tree_.get());
  EXPECT_TRUE(CompareToClone(*tree_));
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
