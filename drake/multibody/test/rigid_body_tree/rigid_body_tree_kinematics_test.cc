#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {

class RigidBodyTreeKinematicsTests : public ::testing::Test {
 protected:
  virtual void SetUp() { tree_ = std::make_unique<RigidBodyTree<double>>(); }

  std::unique_ptr<RigidBodyTree<double>> tree_;
};

// Tests that RigidBodyTree::doKinematics() will not throw an exception if
// provided a valid KinematicsCache.
TEST_F(RigidBodyTreeKinematicsTests, TestDoKinematicWithValidCache) {
  const std::string filename =
      drake::GetDrakePath() +
      "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
  parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld(filename,
                                                                 tree_.get());
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
  parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld(filename,
                                                                 tree_.get());
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
  parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld(filename,
                                                                 tree_.get());
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

class AcrobotTests : public ::testing::Test {
 protected:
  void SetUp() {
    std::string file_name =
        GetDrakePath() + "/multibody/test/rigid_body_tree/double_pendulum.urdf";
    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        file_name, multibody::joints::kFixed, robot_.get());
    cache_ = std::make_unique<KinematicsCache<double>>(
        robot_->CreateKinematicsCache());

    link1_ = robot_->FindBody("link1");
    link2_ = robot_->FindBody("link2");
    frame1_ = robot_->findFrame("frame_on_link1").get();
    link1_com_ = robot_->findFrame("link1_com").get();
    link2_com_ = robot_->findFrame("link2_com").get();
    X_L1F1_ = frame1_->get_transform_to_body();

    world_id_ = robot_->FindBody("world")->get_body_index();
    base_id_ = robot_->FindBody("base")->get_body_index();
    link1_id_ = link1_->get_body_index();
    link2_id_ = link2_->get_body_index();

    axis1_index_ = link1_->get_position_start_index();
    axis2_index_ = link2_->get_position_start_index();

    q_.resize(robot_->get_num_positions());
    v_.resize(robot_->get_num_velocities());
  }

  // Sets the state of the acrobot given by its joint angles measured in
  // degrees.
  void SetState(double theta1_rad, double theta2_rad) {
    q_(axis1_index_) = theta1_rad;
    q_(axis2_index_) = theta2_rad;
    cache_->initialize(q_);
    robot_->doKinematics(*cache_);
  }

  // Sets the state of the acrobot given by its joint angles measured in
  // degrees and its joint angular velocities measured in radians per second.
  void SetState(double theta1_rad, double theta2_rad, double theta1dot,
                double theta2dot) {
    q_(axis1_index_) = theta1_rad;
    q_(axis2_index_) = theta2_rad;
    v_(axis1_index_) = theta1dot;
    v_(axis2_index_) = theta2dot;
    cache_->initialize(q_, v_);
    robot_->doKinematics(*cache_);
  }

  // Given a set of joint angles, this test computes the pose of each link in
  // the model and verifies it against an analytical benchmark.
  void RunPoseTest(double theta1_deg, double theta2_deg) {
    const double deg_to_rad = M_PI / 180.0;
    const double theta1_rad = theta1_deg * deg_to_rad;
    const double theta2_rad = theta2_deg * deg_to_rad;
    SetState(theta1_rad, theta2_rad);

    Isometry3<double> X_WL1cm =
        robot_->CalcFramePoseInWorldFrame(*cache_, *link1_com_);
    Isometry3<double> X_WL1cm_exact =
        acrobot_benchmark_.CalcLink1PoseInWorldFrame(theta1_rad, theta2_rad);
    EXPECT_TRUE(
        X_WL1cm.isApprox(X_WL1cm_exact, Eigen::NumTraits<double>::epsilon()));
    Isometry3<double> X_WL2cm =
        robot_->CalcFramePoseInWorldFrame(*cache_, *link2_com_);
    Isometry3<double> X_WL2cm_exact =
        acrobot_benchmark_.CalcLink2PoseInWorldFrame(theta1_rad, theta2_rad);
    EXPECT_TRUE(
        X_WL2cm.isApprox(X_WL2cm_exact, Eigen::NumTraits<double>::epsilon()));
  }

  void RunSpatialVelocityTest(double theta1_deg, double theta2_deg,
                              double theta1dot, double theta2dot) {
    const double deg_to_rad = M_PI / 180.0;
    const double theta1_rad = theta1_deg * deg_to_rad;
    const double theta2_rad = theta2_deg * deg_to_rad;
    SetState(theta1_rad, theta2_rad, theta1dot, theta2dot);

    Vector6<double> V_WL1cm =
        robot_->CalcFrameSpatialVelocityInWorldFrame(*cache_, *link1_com_);
    Vector6<double> V_WL1cm_exact =
        acrobot_benchmark_.CalcLink1SpatialVelocityInWorldFrame(
            theta1_rad, theta2_rad, theta1dot, theta2dot);
    EXPECT_TRUE(
        V_WL1cm.isApprox(V_WL1cm_exact, Eigen::NumTraits<double>::epsilon()));

    Vector6<double> V_WL2cm =
        robot_->CalcFrameSpatialVelocityInWorldFrame(*cache_, *link2_com_);
    Vector6<double> V_WL2cm_exact =
        acrobot_benchmark_.CalcLink2SpatialVelocityInWorldFrame(
            theta1_rad, theta2_rad, theta1dot, theta2dot);
    EXPECT_TRUE(V_WL2cm.isApprox(V_WL2cm_exact,
                                 2 * Eigen::NumTraits<double>::epsilon()));
  }

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<KinematicsCache<double>> cache_;
  const RigidBody<double> *link1_, *link2_;
  const RigidBodyFrame<double> *frame1_, *link1_com_, *link2_com_;
  int world_id_, base_id_, link1_id_, link2_id_;
  int axis1_index_, axis2_index_;
  int frame_id_;
  VectorX<double> q_;
  VectorX<double> v_;
  Isometry3<double> X_L1F1_;  // Transform from frame F1 to body frame of link1.
  benchmarks::Acrobot<double> acrobot_benchmark_{Vector3d::UnitZ(),
                                                 Vector3d::UnitY()};
};

// Tests RigidBodyTree::CalcFrameSpatialVelocityInWorldFrame() by comparing
// against an analytical solution.
TEST_F(AcrobotTests, PoseTests) {
  const double theta_max = 45.0;
  const int ntheta = 5;
  const double dtheta = 2 * theta_max / ntheta;
  for (double theta1 = -theta_max; theta1 <= theta_max; theta1 += dtheta) {
    for (double theta2 = -theta_max; theta2 <= theta_max; theta2 += dtheta) {
      RunPoseTest(theta1, theta2);
    }
  }
}

// Tests RigidBodyTree::CalcFrameSpatialVelocityInWorldFrame() by comparing
// against an analytical solution.
TEST_F(AcrobotTests, SpatialVelocityTests) {
  const double theta_max = 45.0;
  const int ntheta = 5;
  const double dtheta = 2 * theta_max / ntheta;
  for (double theta1 = -theta_max; theta1 <= theta_max; theta1 += dtheta) {
    for (double theta2 = -theta_max; theta2 <= theta_max; theta2 += dtheta) {
      RunSpatialVelocityTest(theta1, theta2, 0.2, 0.0);
      RunSpatialVelocityTest(theta1, theta2, 0.0, 0.2);
      RunSpatialVelocityTest(theta1, theta2, 0.3, -0.1);
      RunSpatialVelocityTest(theta1, theta2, -0.1, 0.3);
      RunSpatialVelocityTest(theta1, theta2, 0.2, -0.2);
    }
  }
}

// Test the following utility functions assuming their underlying functions
// are correct:
// CalcFramePoseInWorldFrame,
// CalcFrameSpatialVelocityInWorldFrame,
// CalcFrameSpatialVeclocityJacobianInWorldFrame,
// CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame
//
// A fixed frame named "test_frame" is added to
// "multibody/test/rigid_body_tree/two_dof_robot.urdf".
// All the following tests are essentially checking the equality between:
// robot.CalcPoseInWorld(cache, body, offset) and
// robot.CalcPoseInWorld(cache, frame), where "test_frame" is attached to
// "body" with an "offset".
class RBTDifferentialKinematicsHelperTest : public ::testing::Test {
 protected:
  void SetUpWithType(drake::multibody::joints::FloatingBaseType type) {
    std::string file_name =
        GetDrakePath() + "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(file_name, type,
                                                       robot_.get());
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

  // Tests:
  // CalcFramePoseInWorldFrame(body, offset) ==
  // CalcFramePoseInWorldFrame(frame),
  // assuming the underlying function relativeTransform is correct.
  void TestPose() {
    Isometry3<double> pose =
        robot_->CalcFramePoseInWorldFrame(*cache_, *body_ptr_, offset_);
    Isometry3<double> pose_as_frame =
        robot_->CalcFramePoseInWorldFrame(*cache_, *frame_ptr_);

    EXPECT_TRUE(drake::CompareMatrices(pose.linear(), pose_as_frame.linear(),
                                       1e-14,
                                       drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(pose.translation(),
                                       pose_as_frame.translation(), 1e-14,
                                       drake::MatrixCompareType::absolute));
  }

  // Tests:
  // CalcFrameSpatialVelocityInWorldFrame(body, offset) ==
  // CalcFrameSpatialVelocityInWorldFrame(frame),
  // and
  // CalcFrameSpatialVelocityInWorldFrame(frame) ==
  // [R, 0; 0, R] * relativeTwist(world, frame, frame), where R is the rotation
  // from frame to world.
  void TestSpatialVelocity() {
    TwistVector<double> xdot = robot_->CalcFrameSpatialVelocityInWorldFrame(
        *cache_, *body_ptr_, offset_);
    TwistVector<double> xdot_as_frame =
        robot_->CalcFrameSpatialVelocityInWorldFrame(*cache_, *frame_ptr_);

    EXPECT_TRUE(drake::CompareMatrices(xdot, xdot_as_frame, 1e-14,
                                       drake::MatrixCompareType::absolute));

    TwistVector<double> xdot_b = robot_->relativeTwist(
        *cache_, robot_->world().get_body_index(), frame_id_, frame_id_);
    Isometry3<double> pose =
        robot_->CalcFramePoseInWorldFrame(*cache_, *frame_ptr_);
    TwistVector<double> xdot_w;
    xdot_w.head<3>() = pose.linear() * xdot_b.head<3>();
    xdot_w.tail<3>() = pose.linear() * xdot_b.tail<3>();

    EXPECT_TRUE(drake::CompareMatrices(xdot, xdot_w, 1e-14,
                                       drake::MatrixCompareType::absolute));
  }

  // Tests:
  // CalcFrameSpatialVelocityInWorldFrame(body, offset) ==
  // CalcFrameSpatialVelocityInWorldFrame(frame),
  // and
  // CalcFrameSpatialVelocityInWorldFrame(frame) * v ==
  // CalcFrameSpatialVelocityInWorldFrame(frame),
  // [R, 0; 0, R] * geometricJacobian(world, frame, frame) ==
  // CalcFrameSpatialVelocityInWorldFrame(frame)
  void TestSpatialVelocityJacobian(bool use_qdot) {
    TwistVector<double> xdot = robot_->CalcFrameSpatialVelocityInWorldFrame(
        *cache_, *body_ptr_, offset_);
    MatrixX<double> J = robot_->CalcFrameSpatialVeclocityJacobianInWorldFrame(
        *cache_, *body_ptr_, offset_, use_qdot);
    MatrixX<double> J_as_frame =
        robot_->CalcFrameSpatialVeclocityJacobianInWorldFrame(
            *cache_, *frame_ptr_, use_qdot);

    EXPECT_TRUE(drake::CompareMatrices(J, J_as_frame, 1e-14,
                                       drake::MatrixCompareType::absolute));

    if (!use_qdot) {
      EXPECT_TRUE(drake::CompareMatrices(xdot, J * v_, 1e-14,
                                         drake::MatrixCompareType::absolute));
    } else {
      VectorX<double> qd = robot_->GetVelocityToQDotMapping(*cache_) * v_;
      EXPECT_TRUE(drake::CompareMatrices(xdot, J * qd, 1e-14,
                                         drake::MatrixCompareType::absolute));
    }

    Isometry3<double> pose =
        robot_->CalcFramePoseInWorldFrame(*cache_, *frame_ptr_);
    KinematicPath kinematic_path = robot_->findKinematicPath(
        robot_->world().get_body_index(), frame_ptr_->get_frame_index());
    MatrixX<double> Jg = robot_->geometricJacobian(
        *cache_, robot_->world().get_body_index(),
        frame_ptr_->get_frame_index(), frame_ptr_->get_frame_index(), use_qdot);
    Jg = robot_->compactToFull(Jg, kinematic_path.joint_path, use_qdot);
    Jg.topRows<3>() = pose.linear() * Jg.topRows<3>();
    Jg.bottomRows<3>() = pose.linear() * Jg.bottomRows<3>();

    EXPECT_TRUE(drake::CompareMatrices(Jg, J, 1e-14,
                                       drake::MatrixCompareType::absolute));
  }

  // Tests:
  // CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(body, offset) ==
  // CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(frame),
  // and
  // CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(frame).tail<3> ==
  // transformPointsJacobianDotTimesV(Vec3::Zero, frame, world),
  void TestSpatialVelocityJacobianDotTimesV() {
    TwistVector<double> Jdv =
        robot_->CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
            *cache_, *body_ptr_, offset_);
    TwistVector<double> Jdv_as_frame =
        robot_->CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
            *cache_, *frame_ptr_);
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

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<KinematicsCache<double>> cache_;
  const RigidBodyFrame<double>* frame_ptr_;
  const RigidBody<double>* body_ptr_;
  int frame_id_;

  VectorX<double> q_;
  VectorX<double> v_;
  Isometry3<double> offset_;
};

TEST_F(RBTDifferentialKinematicsHelperTest, RPYPoseTest) {
  SetUpWithType(multibody::joints::kRollPitchYaw);
  TestPose();
}

TEST_F(RBTDifferentialKinematicsHelperTest, RPYTwistInWorldAlignedBodyTest) {
  SetUpWithType(multibody::joints::kRollPitchYaw);
  TestSpatialVelocity();
}

TEST_F(RBTDifferentialKinematicsHelperTest, RPYJacobianTest) {
  SetUpWithType(multibody::joints::kRollPitchYaw);
  TestSpatialVelocityJacobian(true);
  TestSpatialVelocityJacobian(false);
}

TEST_F(RBTDifferentialKinematicsHelperTest, RPYJacobianDotTimeVTest) {
  SetUpWithType(multibody::joints::kRollPitchYaw);
  TestSpatialVelocityJacobianDotTimesV();
}

TEST_F(RBTDifferentialKinematicsHelperTest, QuatPoseTest) {
  SetUpWithType(multibody::joints::kQuaternion);
  TestPose();
}

TEST_F(RBTDifferentialKinematicsHelperTest, QuatTwistInWorldAlignedBodyTest) {
  SetUpWithType(multibody::joints::kQuaternion);
  TestSpatialVelocity();
}

TEST_F(RBTDifferentialKinematicsHelperTest, QuatJacobianTest) {
  SetUpWithType(multibody::joints::kQuaternion);
  TestSpatialVelocityJacobian(true);
  TestSpatialVelocityJacobian(false);
}

TEST_F(RBTDifferentialKinematicsHelperTest, QuatJacobianDotTimeVTest) {
  SetUpWithType(multibody::joints::kQuaternion);
  TestSpatialVelocityJacobianDotTimesV();
}

}  // namespace multibody
}  // namespace drake
