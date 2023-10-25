#include "drake/multibody/inverse_kinematics/polyhedron_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/math/autodiff.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::systems::Context;

namespace drake {
namespace multibody {
const double kInf = std::numeric_limits<double>::infinity();
TEST_F(TwoFreeBodiesConstraintTest, PolyhedronConstraint) {
  // Given two free bodies with some given (arbitrary) poses, check if the
  // evaluation result matches with the expected value.
  const Eigen::Quaterniond body1_quaternion(
      Eigen::Vector4d(0.2, 0.5, -1, 2).normalized());
  const Eigen::Quaterniond body2_quaternion(
      Eigen::Vector4d(0.5, -1.5, -0.1, 2.3).normalized());
  const Eigen::Vector3d body1_position(0.2, -1.5, 0.3);
  const Eigen::Vector3d body2_position(0.5, 0.2, 1.5);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(body1_quaternion), body1_position,
      QuaternionToVectorWxyz(body2_quaternion), body2_position;
  plant_->SetPositions(plant_context_, q);

  // Set the gradient of q to arbitrary value.
  Eigen::MatrixXd q_grad(14, 2);
  for (int i = 0; i < q_grad.rows(); ++i) {
    for (int j = 0; j < q_grad.cols(); ++j) {
      q_grad(i, j) = 0.5 * std::sin(i) + j;
    }
  }
  const auto q_ad = math::InitializeAutoDiff(Eigen::VectorXd(q), q_grad);
  plant_autodiff_->SetPositions(plant_context_autodiff_.get(), q_ad);

  Eigen::Matrix<double, 3, 2> p_GP;
  p_GP.col(0) << 0.5, -0.3, 0.1;
  p_GP.col(1) << 0.2, -2.3, 1.1;

  Eigen::MatrixXd A(2, 6);
  // clang-format off
  A << 0.2, -0.5, 0.1, 1.2, 3.1, -1,
       0.4, 0.5, -1, 0.4, 1.2, 0.5;
  // clang-format on
  const Eigen::Vector2d b(3, 4);

  const FrameIndex frameF_index = body1_index_;
  const FrameIndex frameG_index = body2_index_;
  Eigen::Matrix<double, 3, 2> p_FP;
  plant_->CalcPointsPositions(*plant_context_, plant_->get_frame(frameG_index),
                              p_GP, plant_->get_frame(frameF_index), &p_FP);
  Vector6d p_FP_stack;
  p_FP_stack << p_FP.col(0), p_FP.col(1);
  const Eigen::Vector2d y_expected = A * p_FP_stack;
  PolyhedronConstraint dut(plant_, plant_->get_frame(frameF_index),
                           plant_->get_frame(frameG_index), p_GP, A, b,
                           plant_context_);
  EXPECT_TRUE(CompareMatrices(dut.lower_bound(),
                              Eigen::VectorXd::Constant(b.rows(), -kInf)));
  EXPECT_TRUE(CompareMatrices(dut.upper_bound(), b));
  PolyhedronConstraint dut_ad(plant_autodiff_.get(),
                              plant_autodiff_->get_frame(frameF_index),
                              plant_autodiff_->get_frame(frameG_index), p_GP, A,
                              b, plant_context_autodiff_.get());
  // Test Eval for dut and dut_ad.
  Eigen::VectorXd y;

  // MultibodyPlant double, Eval double.
  dut.Eval(q, &y);
  const double tol = 1E-12;
  EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

  // MultibodyPlant AutoDiffXd, Eval double.
  dut_ad.Eval(q, &y);
  EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

  // MultibodyPlant double, Eval AutoDiffXd.
  AutoDiffVecXd y_ad;
  dut.Eval(q_ad, &y_ad);

  // MultibodyPlant AutoDiffXd, Eval AutoDiffXd.
  AutoDiffVecXd y_ad_expected;
  dut_ad.Eval(q_ad, &y_ad_expected);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_ad),
                              math::ExtractValue(y_ad_expected), tol));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_ad),
                              math::ExtractGradient(y_ad_expected), tol));
}

TEST_F(TwoFreeBodiesConstraintTest, SquareGazeCone) {
  // Motivated by the stackoverflow question
  // https://stackoverflow.com/questions/72984441/general-cone-gaze-constraint,
  // we provide an implementation that the camera gaze cone (with rectangle
  // cross section) covers a certain point.

  // We denote the camera frame as C, and the target frame as T. We want the
  // rectangle cone originated from the camera origin to cover a point P fixed
  // in the target frame T.
  FrameIndex camera_frame = body1_index_;
  FrameIndex target_frame = body2_index_;
  const Eigen::Vector3d p_TP(0.2, 0.3, 0);
  // The rectangle cone is described as A * x <= 0, where each row of A is the
  // outward normal of the cone face.
  Eigen::Matrix<double, 4, 3> A;
  // clang-format off
  A << -0.2, 0, -0.1,
       0.2, 0, -0.1,
       0, -0.3, -0.1,
       0, 0.3, -0.1;
  // clang-format on
  PolyhedronConstraint dut(plant_, plant_->get_frame(camera_frame),
                           plant_->get_frame(target_frame), p_TP, A,
                           Eigen::Vector4d::Zero(), plant_context_);

  // Test a configuration that the target point is inside the cone. The
  // constraint bounds should be satisfied.
  Eigen::VectorXd q(14, 1);
  // Put the camera in the world origin.
  q.head<4>() << 1, 0, 0, 0;
  q.segment<3>(4) << 0, 0, 0;
  q.segment<4>(7) << 1, 0, 0, 0;
  q.tail<3>() << -0.2, -0.3, 0.5;
  Eigen::VectorXd y;
  dut.Eval(q, &y);
  EXPECT_TRUE((y.array() <= dut.upper_bound().array()).all());

  // Test a configuration that the target point is outside the cone.
  // The target is behind the camera.
  q.tail<3>() << 0.2, -0.3, -0.5;
  dut.Eval(q, &y);
  EXPECT_FALSE((y.array() <= dut.upper_bound().array()).all());
}

TEST_F(TwoFreeBodiesConstraintTest, AlignVector) {
  // Motivated by the stackoverflow question
  // https://stackoverflow.com/questions/72999101/slightly-different-angle-between-vectors-constraint,
  // we want that after projecting a vector `a` (attached to frame G) to the
  // world xy-plane, the projected vector aligns with another vector b. We show
  // that this kinematic constraint can be modelled using PolyhedronConstraint.

  // If we denote two points P1 and P2 attached to frame G, with the line
  // segment P1P2 along the vector `a` direction, the constraint is that the
  // vector a = p_WP2 - P_WP1 should satisfy dot(a, b_perp) = 0
  // where b_perp is the vector perpendicular to b in the xy plane.
  const Eigen::Vector3d b_W(1, 2, 0);
  const Eigen::Vector3d b_perp_W(2, -1, 0);
  Eigen::Matrix<double, 3, 2> p_GP;
  p_GP.col(0) << 0, 0, 0;
  p_GP.col(1) << 0, 0, 1;

  // The constraint dot(a, b_perp) = 0 is equivalent to
  // [b_perp.T  -b_perp.T] * [p_WP1] <= 0
  // [-b_perp.T  b_perp.T]   [p_WP2]
  Eigen::Matrix<double, 2, 6> A;
  A.row(0) << b_perp_W.transpose(), -b_perp_W.transpose();
  A.row(1) << -b_perp_W.transpose(), b_perp_W.transpose();
  PolyhedronConstraint dut(plant_, plant_->world_frame(),
                           plant_->get_frame(body1_index_), p_GP, A,
                           Eigen::Vector2d::Zero(), plant_context_);

  // Evaluate a configuration where the projection of a aligns with b.
  Eigen::VectorXd q(14);
  // Set the orientation such that p_WP2 - p_WP1 is along (1/3, 2/3, 2/3)
  // direction.
  const math::RotationMatrixd R_WG =
      math::RotationMatrixd::MakeFromOneUnitVector(
          Eigen::Vector3d(1.0 / 3, 2. / 3, 2. / 3), 2);
  const auto quat_WG = R_WG.ToQuaternion();
  q.head<4>() << quat_WG.w(), quat_WG.x(), quat_WG.y(), quat_WG.z();
  q.segment<3>(4) << 0, 0, 0;
  q.segment<4>(7) << 1, 0, 0, 0;
  q.tail<3>() << 0, 0, 0;
  Eigen::VectorXd y;
  dut.Eval(q, &y);
  EXPECT_TRUE((y.array() <= dut.upper_bound().array() + 1E-10).all());
  EXPECT_TRUE((y.array() >= dut.lower_bound().array() + 1E-10).all());

  // Evaluate a configuration where the projection of a doesn't align with b.
  q.head<4>() << std::sqrt(2) / 2, std::sqrt(2) / 2, 0, 0;
  dut.Eval(q, &y);
  EXPECT_FALSE((y.array() <= dut.upper_bound().array() + 1E-10).all());
}
}  // namespace multibody
}  // namespace drake
