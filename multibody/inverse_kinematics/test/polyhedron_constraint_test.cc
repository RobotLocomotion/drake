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
  plant_->GetMutablePositions(plant_context_) = q;

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
}  // namespace multibody
}  // namespace drake
