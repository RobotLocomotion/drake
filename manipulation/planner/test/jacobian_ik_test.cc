#include "drake/manipulation/planner/jacobian_ik.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace manipulation {
namespace planner {

class JacobianIkTest : public ::testing::Test {
 protected:
  void SetUp() {
    const std::string kModelPath = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf");
    iiwa_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        kModelPath, multibody::joints::kFixed, nullptr, iiwa_.get());

    VectorX<double> q0(iiwa_->get_num_positions());
    q0 << 0, 0, 0.3, -M_PI / 2., 0, M_PI / 2., 0;
    cache_ = std::make_unique<KinematicsCache<double>>(iiwa_->doKinematics(q0));

    frame_E_ = std::make_unique<RigidBodyFrame<double>>(
        "frame_E", iiwa_->FindBody("iiwa_link_7"),
        Isometry3<double>::Identity());

    ik_ = std::make_unique<JacobianIk>(iiwa_.get());
    q_nominal_ = iiwa_->getZeroConfiguration();
  }

  std::unique_ptr<RigidBodyTree<double>> iiwa_;
  std::unique_ptr<KinematicsCache<double>> cache_;
  std::unique_ptr<RigidBodyFrame<double>> frame_E_;
  std::unique_ptr<JacobianIk> ik_;

  Vector6<double> V_WE_desired_{Vector6<double>::Zero()};
  Vector6<double> gain_E_{Vector6<double>::Constant(1)};
  VectorX<double> q_nominal_;
  VectorX<double> v_;
  bool is_stuck_{false};
  double dt_{1e-3};
};

// Test solve for 1 tick.
TEST_F(JacobianIkTest, SolveV) {
  V_WE_desired_(0) = 0.2;
  bool solved =
      ik_->CalcJointVelocity(*cache_, *frame_E_, V_WE_desired_, gain_E_, dt_,
                             q_nominal_, &v_, &is_stuck_);

  *cache_ = iiwa_->doKinematics(cache_->getQ(), v_);
  Vector6<double> V_WE =
      iiwa_->CalcFrameSpatialVelocityInWorldFrame(*cache_, *frame_E_);

  EXPECT_TRUE(solved);
  EXPECT_TRUE(
      CompareMatrices(V_WE, V_WE_desired_, 1e-4, MatrixCompareType::absolute));
}

// Test solver by disabling more dof in the Cartesian tracking objective.
TEST_F(JacobianIkTest, Gain) {
  V_WE_desired_ << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  Isometry3<double> X_WE = iiwa_->CalcFramePoseInWorldFrame(*cache_, *frame_E_);
  MatrixX<double> J =
      iiwa_->CalcFrameSpatialVelocityJacobianInWorldFrame(*cache_, *frame_E_);
  Vector6<double> V_WE, V_WE_E, V_WE_E_desired;

  for (int i = 0; i < 6; i++) {
    gain_E_(i) = 0;

    bool solved =
        ik_->CalcJointVelocity(*cache_, *frame_E_, V_WE_desired_, gain_E_, dt_,
                               q_nominal_, &v_, &is_stuck_);
    EXPECT_TRUE(solved);

    // Transform the resulting end effector frame's velocity into body frame.
    V_WE = J * v_;
    V_WE_E.head<3>() = X_WE.linear().transpose() * V_WE.head<3>();
    V_WE_E.tail<3>() = X_WE.linear().transpose() * V_WE.tail<3>();

    // Transform the desired end effector velocity into body frame.
    V_WE_E_desired.head<3>() =
        X_WE.linear().transpose() * V_WE_desired_.head<3>();
    V_WE_E_desired.tail<3>() =
        X_WE.linear().transpose() * V_WE_desired_.tail<3>();

    for (int j = 0; j < 6; j++) {
      // For the constrained dof, the velocity should match.
      if (gain_E_(j) > 0) {
        EXPECT_NEAR(V_WE_E(j), V_WE_E_desired(j), 1e-4);
      }
    }
  }

  // All Cartesian tracking has been disabled, the resulting velocity should be
  // tracking q_nominal only.
  VectorX<double> v_desired = (q_nominal_ - cache_->getQ()) / dt_;
  VectorX<double> v_upper = ik_->get_joint_velocity_upper_limit();
  VectorX<double> v_lower = ik_->get_joint_velocity_lower_limit();
  for (int i = 0; i < v_desired.size(); i++) {
    v_desired(i) = std::max(v_desired(i), v_lower(i));
    v_desired(i) = std::min(v_desired(i), v_upper(i));
  }
  EXPECT_TRUE(
      CompareMatrices(v_, v_desired, 1e-8, MatrixCompareType::absolute));
}

// Use the solver to track a fixed end effector pose.
TEST_F(JacobianIkTest, SimpleTracker) {
  Isometry3<double> X_WE_desired, X_WE;
  X_WE = iiwa_->CalcFramePoseInWorldFrame(*cache_, *frame_E_);
  X_WE_desired =
      Eigen::Translation3d(Vector3<double>(0.02, -0.01, -0.03)) * X_WE;

  VectorX<double> q;
  do {
    X_WE = iiwa_->CalcFramePoseInWorldFrame(*cache_, *frame_E_);
    V_WE_desired_ = JacobianIk::CalcPoseDifference(X_WE, X_WE_desired) * 10;

    bool solved =
        ik_->CalcJointVelocity(*cache_, *frame_E_, V_WE_desired_, gain_E_, dt_,
                               q_nominal_, &v_, &is_stuck_);
    EXPECT_TRUE(solved);

    q = cache_->getQ() + v_ * dt_;
    *cache_ = iiwa_->doKinematics(q, v_);
  } while (v_.norm() > 1e-6);
  EXPECT_TRUE(CompareMatrices(X_WE.matrix(), X_WE_desired.matrix(), 1e-5,
                              MatrixCompareType::absolute));
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
