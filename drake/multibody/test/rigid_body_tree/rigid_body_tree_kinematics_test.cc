#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace multibody {

// Test the following utility functions assuming their underlying functions
// are correct:
// CalcPoseInWorld,
// CalcTwistInWorld,
// CalcTwistInWorldAlignedBody,
// CalcTwistInWorldAlignedBody,
// CalcJacobianDotTimesVForWorldAlignedBody.
//
// A fixed frame named "test_frame" is added to
// "multibody/test/rigid_body_tree/two_dof_robot.urdf".
// All the following tests are essentially checking the equality between:
// robot.CalcPoseInWorld(cache, body, offset) and
// robot.CalcPoseInWorld(cache, frame), where "test_frame" is attached to
// "body" with an "offset".
class RBTDifferentialKinematicsHelperTest : public ::testing::Test {
 protected:
  void SetUp() {
    std::string file_name =
        GetDrakePath() + "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        file_name, multibody::joints::kRollPitchYaw, robot_.get());
    cache_ = std::make_unique<KinematicsCache<double>>(
        robot_->CreateKinematicsCache());

    frame_ptr_ = robot_->findFrame("test_frame").get();
    body_ptr_ = &(frame_ptr_->get_rigid_body());
    offset_ = frame_ptr_->get_transform_to_body();
    frame_id_ = frame_ptr_->get_frame_index();

    q_ = VectorX<double>(robot_->get_num_positions());
    v_ = VectorX<double>(robot_->get_num_velocities());

    for (int i = 0; i < q_.size(); ++i) q_[i] = 0.2 * i;
    for (int i = 0; i < v_.size(); ++i) v_[i] = -0.2 * i;

    cache_->initialize(q_, v_);
    robot_->doKinematics(*cache_, true);
  }

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<KinematicsCache<double>> cache_;
  const RigidBodyFrame<double>* frame_ptr_;
  const RigidBody<double>* body_ptr_;
  int frame_id_;

  VectorX<double> q_;
  VectorX<double> v_;
  Isometry3<double> offset_;
};

// Tests CalcPoseInWorld(body, offset) == CalcPoseInWorld(frame),
// assuming the underlying function relativeTransform is correct.
TEST_F(RBTDifferentialKinematicsHelperTest, PoseTest) {
  Isometry3<double> pose =
      robot_->CalcPoseInWorld(*cache_, *body_ptr_, offset_);
  Isometry3<double> pose_as_frame =
      robot_->CalcPoseInWorld(*cache_, *frame_ptr_);

  EXPECT_TRUE(drake::CompareMatrices(pose.linear(), pose_as_frame.linear(),
                                     1e-14,
                                     drake::MatrixCompareType::absolute));

  EXPECT_TRUE(drake::CompareMatrices(pose.translation(),
                                     pose_as_frame.translation(), 1e-14,
                                     drake::MatrixCompareType::absolute));
}

// Tests CalcTwistInWorld(body, offset) == CalcTwistInWorld(frame),
// assuming the underlying function relativeTwist is correct.
TEST_F(RBTDifferentialKinematicsHelperTest, TwistInWorldTest) {
  TwistVector<double> twist =
      robot_->CalcTwistInWorld(*cache_, *body_ptr_, offset_);
  TwistVector<double> twist_as_frame =
      robot_->CalcTwistInWorld(*cache_, *frame_ptr_);

  EXPECT_TRUE(drake::CompareMatrices(twist, twist_as_frame, 1e-14,
                                     drake::MatrixCompareType::absolute));
}

// Tests
// CalcTwistInWorldAlignedBody(body, offset) ==
// CalcTwistInWorldAlignedBody(frame),
// and
// CalcTwistInWorldAlignedBody(frame) ==
// [R, 0; 0 R] * relativeTwist(world, frame, frame), where R is the rotation
// from frame to world.
// This assumes relativeTwist and CalcPoseInWorld are correct.
TEST_F(RBTDifferentialKinematicsHelperTest, TwistInWorldAlignedBodyTest) {
  TwistVector<double> xdot =
      robot_->CalcTwistInWorldAlignedBody(*cache_, *body_ptr_, offset_);
  TwistVector<double> xdot_as_frame =
      robot_->CalcTwistInWorldAlignedBody(*cache_, *frame_ptr_);

  EXPECT_TRUE(drake::CompareMatrices(xdot, xdot_as_frame, 1e-14,
                                     drake::MatrixCompareType::absolute));

  TwistVector<double> xdot_b = robot_->relativeTwist(
      *cache_, robot_->world().get_body_index(), frame_id_, frame_id_);
  Isometry3<double> pose = robot_->CalcPoseInWorld(*cache_, *frame_ptr_);
  TwistVector<double> xdot_w;
  xdot_w.head<3>() = pose.linear() * xdot_b.head<3>();
  xdot_w.tail<3>() = pose.linear() * xdot_b.tail<3>();

  EXPECT_TRUE(drake::CompareMatrices(xdot, xdot_w, 1e-14,
                                     drake::MatrixCompareType::absolute));
}

// Tests
// CalcTwistInWorldAlignedBody(body, offset) ==
// CalcTwistInWorldAlignedBody(frame),
// and
// CalcTwistInWorldAlignedBody(frame) * v ==
// CalcTwistInWorldAlignedBody(frame),
// assuming geometricJacobian and CalcTwistInWorldAlignedBody are correct.
TEST_F(RBTDifferentialKinematicsHelperTest, JacobianTest) {
  TwistVector<double> xdot =
      robot_->CalcTwistInWorldAlignedBody(*cache_, *body_ptr_, offset_);
  MatrixX<double> J =
      robot_->CalcJacobianForWorldAlignedBody(*cache_, *body_ptr_, offset_);
  MatrixX<double> J_as_frame =
      robot_->CalcJacobianForWorldAlignedBody(*cache_, *frame_ptr_);

  EXPECT_TRUE(drake::CompareMatrices(J, J_as_frame, 1e-14,
                                     drake::MatrixCompareType::absolute));

  EXPECT_TRUE(drake::CompareMatrices(xdot, J * v_, 1e-14,
                                     drake::MatrixCompareType::absolute));
}

// Tests
// CalcJacobianDotTimesVForWorldAlignedBody(body, offset) ==
// CalcJacobianDotTimesVForWorldAlignedBody(frame),
// and
// CalcJacobianDotTimesVForWorldAlignedBody(frame) ==
// transformPointsJacobianDotTimesV(Vec3::Zero, frame, world),
// assuming geometricJacobianDotTimesV and transformPointsJacobianDotTimesV
// are correct.
TEST_F(RBTDifferentialKinematicsHelperTest, JacobianDotTimeVTest) {
  TwistVector<double> Jdv = robot_->CalcJacobianDotTimesVForWorldAlignedBody(
      *cache_, *body_ptr_, offset_);
  TwistVector<double> Jdv_as_frame =
      robot_->CalcJacobianDotTimesVForWorldAlignedBody(*cache_, *frame_ptr_);
  EXPECT_TRUE(drake::CompareMatrices(Jdv, Jdv_as_frame, 1e-14,
                                     drake::MatrixCompareType::absolute));

  Vector3<double> off = offset_.translation();
  VectorX<double> Jdv1 = robot_->transformPointsJacobianDotTimesV(
      *cache_, off, body_ptr_->get_body_index(),
      robot_->world().get_body_index());
  VectorX<double> Jdv2 = robot_->transformPointsJacobianDotTimesV(
      *cache_, Vector3<double>::Zero(), frame_ptr_->get_frame_index(),
      robot_->world().get_body_index());

  EXPECT_TRUE(drake::CompareMatrices(Jdv2, Jdv1, 1e-14,
                                     drake::MatrixCompareType::absolute));

  EXPECT_TRUE(drake::CompareMatrices(Jdv.tail<3>(), Jdv1, 1e-14,
                                     drake::MatrixCompareType::absolute));
}

}  // namespace multibody
}  // namespace drake
