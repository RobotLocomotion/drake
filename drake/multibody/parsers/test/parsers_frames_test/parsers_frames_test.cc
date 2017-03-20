#include <memory>
#include <string>

#include <gtest/gtest.h>
#include "spruce.hh"

#include "drake/common/drake_path.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using std::make_unique;
using std::string;
using std::unique_ptr;
using std::vector;

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {

using multibody::joints::kQuaternion;

namespace parsers {

using sdf::AddModelInstancesFromSdfFileToWorld;
using urdf::AddModelInstanceFromUrdfFileToWorld;

namespace {

class DoublePendulumFramesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // One entry per body in the tree.
    const int kNumBodiesInTree = 4;
    expected_Bo_W_.resize(kNumBodiesInTree);
    expected_Bcm_B_.resize(kNumBodiesInTree);
    expected_Bcm_W_.resize(kNumBodiesInTree);
  }

  void LoadAndRunTests(const string& file_name) {
    const string full_name = GetDrakePath() +
                             "/multibody/parsers/test/parsers_frames_test/" +
                             file_name;

    tree_ = make_unique<RigidBodyTree<double>>();

    spruce::path spruce_path(file_name);
    auto extension = spruce_path.extension();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   ::tolower);
    DRAKE_DEMAND(extension == ".urdf" || extension == ".sdf");

    if (extension == ".urdf") {
      AddModelInstanceFromUrdfFileToWorld(
          full_name, drake::multibody::joints::kFixed, tree_.get());
    } else {
      AddModelInstancesFromSdfFileToWorld(
          full_name, drake::multibody::joints::kFixed, tree_.get());
    }

    ASSERT_EQ(tree_->get_num_bodies(), 4);
    ASSERT_EQ(tree_->get_num_positions(), 2);
    ASSERT_EQ(tree_->get_num_velocities(), 2);

    q_ = VectorXd::Zero(tree_->get_num_positions());

    world_id_ = tree_->FindBody("world")->get_body_index();
    base_id_ = tree_->FindBody("base")->get_body_index();
    upper_arm_id_ = tree_->FindBody("upper_arm")->get_body_index();
    lower_arm_id_ = tree_->FindBody("lower_arm")->get_body_index();

    axis1_index_ = tree_->FindBody("upper_arm")->get_position_start_index();
    axis2_index_ = tree_->FindBody("lower_arm")->get_position_start_index();

    // Body "world" is always at the origin.
    expected_Bo_W_[world_id_] = Vector3d(0.0, 0.0, 0.0);
    expected_Bcm_B_[world_id_] = Vector3d(0.0, 0.0, 0.0);
    expected_Bcm_W_[world_id_] = Vector3d(0.0, 0.0, 0.0);

    // Body "base" is fixed to the world and located at the origin.
    expected_Bo_W_[base_id_] = Vector3d(0.0, 0.0, 0.0);
    expected_Bcm_B_[base_id_] = Vector3d(0.0, 0.0, 0.0);
    expected_Bcm_W_[base_id_] = Vector3d(0.0, 0.0, 0.0);

    // Runs a number of tests for different joint angles. The joint angles are
    // in units of degrees.
    RunTest(0.0, 0.0);
    RunTest(0.0, 45.0);
    RunTest(45.0, 0.0);
    RunTest(12.0, -18.0);
  }

  // Sets the state of the double pendulum model described by its joint
  // angles theta1 (joint1 dof) and theta2 (joint2 dof).
  // Input angles are in degrees.
  void SetState(double theta1_degrees, double theta2_degrees) {
    const double deg_to_rad = M_PI / 180.0;
    q_(axis1_index_) = theta1_degrees * deg_to_rad;
    q_(axis2_index_) = theta2_degrees * deg_to_rad;
  }

  // Analytically computes the pose of the frame of each link in the double
  // pendulum model for the state set with SetState().
  void ComputeAnalyticalSolution() {
    double theta1 = q_(axis1_index_);
    double theta2 = q_(axis2_index_);
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

  // Compares the analytical solution obtained with
  // DoublePendulumFramesTest::ComputeAnalyticalSolution(), and saved as
  // DoublePendulumFramesTest members, with the poses computed with a
  // RigidBodyTree model constructed by parsing a URDF or SDF file.
  void TestPoses() {
    VectorXd v = VectorXd::Zero(tree_->get_num_velocities());
    KinematicsCache<double> cache = tree_->doKinematics(q_, v);

    for (int i = 2; i < tree_->get_num_bodies(); ++i) {
      auto Bo_W = tree_->transformPoints(cache, Vector3d::Zero(), i, 0);
      auto Bcm_B = tree_->get_body(i).get_center_of_mass();
      auto Bcm_W = tree_->transformPoints(cache, Bcm_B, i, 0);

      EXPECT_TRUE(Bo_W.isApprox(expected_Bo_W_[i]));
      EXPECT_TRUE(Bcm_B.isApprox(expected_Bcm_B_[i]));
      EXPECT_TRUE(Bcm_W.isApprox(expected_Bcm_W_[i]));
    }
  }

  void RunTest(double theta1_deg, double theta2_deg) {
    SetState(theta1_deg, theta2_deg);
    ComputeAnalyticalSolution();
    TestPoses();
  }

  unique_ptr<RigidBodyTree<double>> tree_;
  int world_id_, base_id_, upper_arm_id_, lower_arm_id_;
  int axis1_index_, axis2_index_;
  vector<Vector3d> expected_Bo_W_;
  vector<Vector3d> expected_Bcm_B_;
  vector<Vector3d> expected_Bcm_W_;
  VectorXd q_;
};

// In this test the simple double pendulum model is loaded from a URDF file
// into a RigidBodyTree. The correctness in the position of each body frame
// as computed with the RigidBodyTree model is then verified against an
// analytical solution.
TEST_F(DoublePendulumFramesTest, UrdfTest) {
  LoadAndRunTests("simple_double_pendulum_urdf/simple_double_pendulum.urdf");
}

// In this test the simple double pendulum model is loaded from an SDF file
// explicitly specifying the link frames L to be located at the mobile frames
// B (recall B = M in Drake). In this case, <joint> does not need to specify its
// pose with respect to the parent body, i.e. no <pose> is given in <joint>.
// The SDF parser determines X_PF by assuming F = B for the initial
// configuration (q = 0).
// Inertial frames, I, need to be specified as done in URDF files.
TEST_F(DoublePendulumFramesTest, SdfTestWhereLequalsB) {
  LoadAndRunTests(
      "simple_double_pendulum_l_equals_b_sdf/"
      "simple_double_pendulum_l_equals_b.sdf");
}

// In this test, the lower_arm's link frame L is not specified (no <pose>
// entry is given for this link). Therefore the parser makes L = M where M is
// the joint's inboard frame specified by a <pose> entry for joint "joint2" in
// the file. The pose of frame M is expressed in the model frame, D, i.e.,
// <pose> is giving X_DM for "joint2".
TEST_F(DoublePendulumFramesTest, SdfTestLisNotSpecified) {
  LoadAndRunTests(
      "simple_double_pendulum_l_is_not_specified_sdf/"
      "simple_double_pendulum_l_is_not_specified.sdf");
}

// The following tests are disabled because they fail to pass. See #4641.

// TODO(liang.fok) Enable this test once #4641 is resolved.
//
// In this test the frame L for the lower arm is specified to be half way
// between the body frame B and the inertial frame I. Since the link frame L was
// specified, the <pose> entry for joint "joint2" specifies X_LM, and therefore
// the pose of the joint outboard frame M is measured and expressed in the link
// frame L.
TEST_F(DoublePendulumFramesTest, DISABLED_SdfTestLBetweenBandI) {
  LoadAndRunTests(
      "simple_double_pendulum_l_between_b_and_i_sdf/"
      "simple_double_pendulum_l_between_b_and_i.sdf");
}

// TODO(liang.fok) Enable this test once #4641 is resolved.
//
// In this test the link frame L of the lower arm is defined to be coincident
// with the inertial frame I of the link. The <pose> entry for joint "joint2"
// specifies X_LM, and therefore the pose of the joint outboard frame M is
// measured and expressed in the link frame L.
TEST_F(DoublePendulumFramesTest, DISABLED_SdfTestLequalsI) {
  LoadAndRunTests(
      "simple_double_pendulum_l_equals_i_sdf/"
      "simple_double_pendulum_l_equals_i.sdf");
}

}  // namespace
}  // namespace parsers
}  // namespace drake
