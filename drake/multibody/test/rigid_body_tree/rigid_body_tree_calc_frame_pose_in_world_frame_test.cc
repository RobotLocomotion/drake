#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::make_unique;

namespace drake {

using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace multibody {

// Tests that RigidBodyTree::doKinematics() will not throw an exception if
// provided a valid KinematicsCache.
GTEST_TEST(RigidBodyTreeCalcFramePoseInWorldFrameTests, BoxAtOriginTest) {
  auto tree = make_unique<RigidBodyTree<double>>();

  // Adds a box to the RigidBodyTree.
  AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/multibody/models/box.urdf",
      drake::multibody::joints::kQuaternion, tree.get());

  // Adds a frame to the RigidBodyTree called "box frame" that is coincident
  // with the "box" body within the RigidBodyTree.
  auto frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "box frame",
      tree->FindBody("box"), Eigen::Isometry3d::Identity());
  tree->addFrame(frame);

  const VectorXd q = VectorXd::Zero(tree->get_num_positions());
  const VectorXd v = VectorXd::Zero(tree->get_num_velocities());
  const KinematicsCache<double> cache = tree->doKinematics(q, v);
  drake::Isometry3<double> X_WF =
      tree->CalcFramePoseInWorldFrame(cache, *frame);

  // The following is a debug print statement.
  std::cout << "Accelerometer::DoCalcOutput:\n"
            << "  - frame transform =\n"
            << frame->get_transform_to_body().matrix() << "\n"
            << "  - X_WF =\n"
            << X_WF.matrix() << std::endl;

  EXPECT_TRUE(CompareMatrices(X_WF.linear(), MatrixXd::Identity(3, 3),
                              1e-10 /* tolerance */,
                              drake::MatrixCompareType::absolute));
}

}  // namespace multibody
}  // namespace drake
