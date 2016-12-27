#include "drake/multibody/parsers/urdf_parser.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_tree.h"

using std::endl;
using std::ifstream;
using std::make_unique;
using std::string;
using std::stringstream;
using std::unique_ptr;
using std::vector;

using Eigen::Vector3d;
using Eigen::VectorXd;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {

using multibody::joints::kQuaternion;
using parsers::urdf::AddModelInstanceFromUrdfFile;
using parsers::urdf::AddModelInstanceFromUrdfFileSearchingInRosPackages;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfStringWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfString;

namespace parsers {
namespace {

class DoublePendulumFramesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // One entry per body in the tree.
    expected_Bo_W_.resize(4);
    expected_Bcm_B_.resize(4);
    expected_Bcm_W_.resize(4);
    q.resize(4);
  }

  void LoadTreeFrom(const string& file_name) {
    const string full_name = GetDrakePath() +
        "/multibody/parsers/test/" + file_name;

    tree_ = std::make_unique<RigidBodyTree<double>>();
    AddModelInstanceFromUrdfFileToWorld(
        full_name, drake::multibody::joints::kFixed, tree_.get());

    world_id_ = tree_->FindBody("world")->get_body_index();
    base_id_ = tree_->FindBody("base")->get_body_index();
    upper_arm_id_ = tree_->FindBody("upper_arm")->get_body_index();
    lower_arm_id_ = tree_->FindBody("lower_arm")->get_body_index();

    axis1_index_ =
        tree_->FindBody("upper_arm")->get_position_start_index();
    axis2_index_ =
        tree_->FindBody("lower_arm")->get_position_start_index();

    // Body "world" is always at the origin.
    expected_Bo_W_[world_id_] = Vector3d(0.0, 0.0, 0.0);
    expected_Bcm_B_[world_id_] = Vector3d(0.0, 0.0, 0.0);
    expected_Bcm_W_[world_id_] = Vector3d(0.0, 0.0, 0.0);

    // Body "base" is fixed to the world. Located at the origin.
    expected_Bo_W_[base_id_] = Vector3d(0.0, 0.0, 0.0);
    expected_Bcm_B_[base_id_] = Vector3d(0.0, 0.0, 0.0);
    expected_Bcm_W_[base_id_] = Vector3d(0.0, 0.0, 0.0);
  }

  void SetState(double theta1, double theta2) {
    const double deg_to_rad = M_PI / 180.0;
    q(axis1_index_) = theta1 * deg_to_rad;
    q(axis2_index_) = theta2 * deg_to_rad;
  }

  void ComputeAnalyticalSolution() {
    double theta1 = q(axis1_index_);
    double theta2 = q(axis2_index_);
    // Body "upper_arm".
    expected_Bo_W_[upper_arm_id_] = Vector3d(0.0, 0.0, 0.0);
    expected_Bcm_B_[upper_arm_id_] = Vector3d(0.0, -0.5, 0.0);
    expected_Bcm_W_[upper_arm_id_] =
        0.5 * Vector3d(sin(theta1), -cos(theta1), 0.0);
    // Body "lower_arm".
    expected_Bo_W_[lower_arm_id_] = Vector3d(sin(theta1), -cos(theta1), 0.0);
    expected_Bcm_B_[lower_arm_id_] = Vector3d(0.0, -0.5, 0.0);
    expected_Bcm_W_[lower_arm_id_] =
        0.5 * Vector3d(sin(theta1 + theta2), -cos(theta1 + theta2), 0.0) +
        expected_Bo_W_[lower_arm_id_];
  }

  void TestPoses(
      const vector<Vector3d> &expected_Bo_W,
      const vector<Vector3d> &expected_Bcm_B,
      const vector<Vector3d> &expected_Bcm_W) {
    VectorXd v = VectorXd::Zero(tree_->get_num_velocities());

    KinematicsCache<double> cache = tree_->doKinematics(q, v);

    for (int i = 2; i < tree_->get_num_bodies(); ++i) {
      auto Bo_W = tree_->transformPoints(cache, Vector3d::Zero(), i, 0);
      auto Bcm_B = tree_->get_body(i).get_center_of_mass_in_B();
      auto Bcm_W = tree_->transformPoints(cache, Bcm_B, i, 0);

      PRINT_VAR(tree_->get_body(i).get_name());
      PRINT_VAR(Bo_W.transpose());
      PRINT_VAR(Bcm_B.transpose());
      PRINT_VAR(Bcm_W.transpose());

      EXPECT_TRUE(Bo_W.isApprox(expected_Bo_W[i]));
      EXPECT_TRUE(Bcm_B.isApprox(expected_Bcm_B[i]));
      EXPECT_TRUE(Bcm_W.isApprox(expected_Bcm_W[i]));
    }
  }

  void RunTest(double theta1, double theta2) {
    SetState(theta1, theta2);
    ComputeAnalyticalSolution();
    TestPoses(expected_Bo_W_, expected_Bcm_B_, expected_Bcm_W_);
  }

  unique_ptr<RigidBodyTree<double>> tree_;
  int world_id_, base_id_, upper_arm_id_, lower_arm_id_;
  int axis1_index_, axis2_index_;
  vector<Vector3d> expected_Bo_W_;
  vector<Vector3d> expected_Bcm_B_;
  vector<Vector3d> expected_Bcm_W_;
  VectorXd q;
};

TEST_F(DoublePendulumFramesTest, URDFTest) {
  LoadTreeFrom("simple_pendulum.urdf");

  EXPECT_EQ(tree_->get_num_bodies(), 4);
  EXPECT_EQ(tree_->get_num_positions(), 2);
  EXPECT_EQ(tree_->get_num_velocities(), 2);

  // Expected poses for the zero state configuration.
  RunTest(0.0, 0.0);
  RunTest(0.0, 45.0);
  RunTest(45.0, 0.0);
  RunTest(12.0, -18.0);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
