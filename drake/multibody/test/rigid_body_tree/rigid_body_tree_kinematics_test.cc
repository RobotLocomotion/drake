#include "drake/multibody/rigid_body_tree.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/urdf_parser.h"

using Eigen::VectorXd;

namespace drake {

using drake::parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld;

namespace {

class RigidBodyTreeKinematicsTests : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree_ = std::make_unique<RigidBodyTree<double>>();
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
};

// Tests that RigidBodyTree::doKinematics() will not throw an exception if
// provided a valid KinematicsCache.
TEST_F(RigidBodyTreeKinematicsTests, TestDoKinematicWithValidCache) {
  const std::string filename =
      drake::GetDrakePath() +
      "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
  AddModelInstanceFromUrdfFileWithRpyJointToWorld(filename, tree_.get());
  KinematicsCache<double> cache = tree_->CreateKinematicsCache();
  const VectorXd q = Eigen::VectorXd::Zero(tree_->get_num_positions());
  cache.initialize(q);
  EXPECT_NO_THROW(tree_->doKinematics(cache));
}

// Tests that RigidBodyTree::doKinematics() will throw an exception if provided
// an invalid KinematicsCache. In this case, the cache is not valid because the
// number of KinematicCacheElement objects within it does not equal the number
// of RigidBody objects within the RigidBodyTree.
TEST_F(RigidBodyTreeKinematicsTests, TestDoKinematicWithBadCache1) {
  const std::string filename =
      drake::GetDrakePath() +
      "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
  AddModelInstanceFromUrdfFileWithRpyJointToWorld(filename, tree_.get());
  const std::vector<int> num_joint_positions;
  const std::vector<int> num_joint_velocities;
  KinematicsCache<double> cache(tree_->get_num_positions(),
                                tree_->get_num_velocities(),
                                num_joint_positions, num_joint_velocities);
  const VectorXd q = Eigen::VectorXd::Zero(tree_->get_num_positions());
  cache.initialize(q);
  EXPECT_THROW(tree_->doKinematics(cache), std::runtime_error);
}

// Tests that RigidBodyTree::doKinematics() will throw an exception if provided
// an invalid KinematicsCache. In this case, the cache is not valid because the
// number of joint DOFs within the KinematicCacheElement objects are incorrect.
// They do not match the number of joint DOFs within the RigidBodyTree.
TEST_F(RigidBodyTreeKinematicsTests, TestDoKinematicWithBadCache2) {
  const std::string filename =
      drake::GetDrakePath() +
      "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
  AddModelInstanceFromUrdfFileWithRpyJointToWorld(filename, tree_.get());
  std::vector<int> num_joint_positions;
  std::vector<int> num_joint_velocities;
  for (int i = 0; i < tree_->get_num_bodies(); ++i) {
    num_joint_positions.push_back(i);
    num_joint_velocities.push_back(i);
  }
  KinematicsCache<double> cache(tree_->get_num_positions(),
                                tree_->get_num_velocities(),
                                num_joint_positions, num_joint_velocities);
  const VectorXd q = Eigen::VectorXd::Zero(tree_->get_num_positions());
  cache.initialize(q);
  EXPECT_THROW(tree_->doKinematics(cache), std::runtime_error);
}

}  // namespace
}  // namespace drake
