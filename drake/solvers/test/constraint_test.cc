#include "drake/solvers/constraint.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace solvers {
namespace {
// Tests if the Lorentz Cone constraint is imposed correctly.
void TestLorentzConeEval(const Eigen::Ref<const Eigen::MatrixXd> A,
                         const Eigen::Ref<const Eigen::VectorXd> b,
                         const VectorXd& x_test, bool is_in_cone) {
  LorentzConeConstraint cnstr(A, b);
  VectorXd y;
  // Test Eval with VectorXd.
  cnstr.Eval(x_test, y);
  Vector2d y_expected;
  VectorXd z = A * x_test + b;
  y_expected(0) = z(0);
  y_expected(1) = z(0) * z(0) - z.tail(z.size() - 1).squaredNorm();
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected = (y(0) >= 0) & (y(1) >= 0);
  EXPECT_EQ(is_in_cone, is_in_cone_expected);

  auto tx = drake::math::initializeAutoDiff(x_test);
  TaylorVecXd x_taylor = tx;
  TaylorVecXd y_taylor;
  // Test Eval with TaylorVar.
  cnstr.Eval(x_taylor, y_taylor);

  EXPECT_TRUE(CompareMatrices(y, math::autoDiffToValueMatrix(y_taylor)));
}

void TestRotatedLorentzConeEval(const Eigen::Ref<const Eigen::MatrixXd> A,
                                const Eigen::Ref<const Eigen::VectorXd> b,
                                const VectorXd& x_test, bool is_in_cone) {
  RotatedLorentzConeConstraint cnstr(A, b);
  VectorXd y;
  cnstr.Eval(x_test, y);
  Eigen::VectorXd z = A * x_test + b;
  Vector3d y_expected(
      z(0),
      z(1),
      z(0) * z(1) - z.tail(z.size() - 2).squaredNorm());
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected =
      (z(0) >= 0) & (z(1) >= 0) & (z(0) * z(1) >= z.tail(z.size() - 2).norm());
  EXPECT_EQ(is_in_cone, is_in_cone_expected);

  // Eval with taylor var.
  auto tx = drake::math::initializeAutoDiff(x_test);
  TaylorVecXd x_taylor = tx;
  TaylorVecXd y_taylor;
  cnstr.Eval(x_taylor, y_taylor);

  EXPECT_TRUE(CompareMatrices(y, math::autoDiffToValueMatrix(y_taylor)));
}

GTEST_TEST(testConstraint, testLorentzConeConstraint) {
  // [3;1;1] is in the interior of the Lorentz cone.
  Eigen::Vector3d x1(3.0, 1.0, 1.0);
  TestLorentzConeEval(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), x1,
                      true);

  // [3;2;2;1] is on the boundary of the Lorentz cone.
  Eigen::Vector2d x2(1, 3);
  Eigen::Matrix<double, 4, 2> A2;
  // clang-format off
  A2 << 1, 0,
       1, 1,
       -1, 1,
       1, -2;
  // clang-format on
  Eigen::Vector4d b2(2, -2, 0, 6);
  TestLorentzConeEval(A2, b2, x2, true);

  // [3; 3; 1] is outside of the Lorentz cone.
  Eigen::Vector4d x3(1, -1, 2, 3);
  Eigen::Matrix<double, 3, 4> A3;
  // clang-format off
  A3 << 1, 0, -1, 2,
        -1, 2, 0, 1,
        0, -2, 3, 1;
  // clang-format on
  Eigen::Vector3d b3 = Eigen::Vector3d(3, 3, 1) - A3 * x3;
  TestLorentzConeEval(A3, b3, x3, false);

  // [-3; 1; 1] is outside of the Lorentz cone.
  Vector1d x4 = Vector1d::Constant(4);
  Eigen::Vector3d A4(-1, 3, 2);
  Eigen::Vector3d b4 = Eigen::Vector3d(-3, 1, 1) - A4 * x4;
  TestLorentzConeEval(A4, b4, x4, false);
}

GTEST_TEST(testConstraint, testRotatedLorentzConeConstraint) {
  // [1;2;1] is in the interior of the rotated lorentz cone.
  TestRotatedLorentzConeEval(Eigen::Matrix3d::Identity(),
                             Eigen::Vector3d::Zero(), Vector3d(1, 2, 1), true);

  // [1;2;1;1] is on the boundary of the rotated Lorentz cone.
  Eigen::Vector2d x2(1, 2);
  Eigen::Matrix<double, 4, 2> A2;
  // clang-format off
  A2 << 1, -1,
        0, 2,
        -1, 3,
        -2, 4;
  // clang-format on
  Eigen::Vector4d b2 = Eigen::Vector4d(1, 2, 1, 1) - A2 * x2;
  TestRotatedLorentzConeEval(A2, b2, x2, true);

  // [1;2;2;2] is outside of the rotated Lorentz cone.
  Eigen::Vector4d x3(1, 3, -1, 2);
  Eigen::Matrix4d A3;
  // clang-format off
  A3 << 1, 2, 3, 4,
        -1, 2, 4, 2,
        -3, 2, 1, 4,
        2, 1, 3, 2;
  // clang-format on
  Eigen::Vector4d b3 = Eigen::Vector4d(1, 2, 2, 2) - A3 * x3;
  TestRotatedLorentzConeEval(A3, b3, x3, false);

  // [-1; -2; 1] is outside of the rotated Lorentz cone.
  Vector1d x4 = Vector1d::Constant(10);
  Eigen::Vector3d A4(1, 3, 2);
  Eigen::Vector3d b4 = Eigen::Vector3d(-1, -2, 1) - A4 * x4;
  TestRotatedLorentzConeEval(A4, b4, x4, false);
}

GTEST_TEST(testConstraint, testPositiveSemidefiniteConstraint) {
  PositiveSemidefiniteConstraint cnstr(3);

  Eigen::Matrix<double, 9, 1> X1;
  // clang-format off
  X1 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
  // clang-format on
  Eigen::VectorXd y;
  cnstr.Eval(X1, y);
  EXPECT_TRUE((y.array() >= cnstr.lower_bound().array()).all());
  EXPECT_TRUE((y.array() <= cnstr.upper_bound().array()).all());

  Eigen::Matrix<double, 9, 1> X2;
  // clang-format off
  X2 << 1, 2, 0,
        2, -2, -1,
        0, -1, -2;
  // clang-format on
  cnstr.Eval(X2, y);
  EXPECT_TRUE((y.array() < cnstr.lower_bound().array()).any() ||
              (y.array() > cnstr.upper_bound().array()).any());
  EXPECT_EQ(cnstr.matrix_rows(), 3);
}

GTEST_TEST(testConstraint, testLinearMatrixInequalityConstraint) {
  Eigen::Matrix2d F0 = 2 * Eigen::Matrix2d::Identity();
  Eigen::Matrix2d F1;
  F1 << 1, 1, 1, 1;
  Eigen::Matrix2d F2;
  F2 << 1, 2, 2, 1;
  LinearMatrixInequalityConstraint cnstr({F0, F1, F2});

  // [4, 3]
  // [3, 4] is positive semidefinite
  Eigen::VectorXd y;
  cnstr.Eval(Eigen::Vector2d(1, 1), y);
  EXPECT_TRUE((y.array() >= cnstr.lower_bound().array()).all());
  EXPECT_TRUE((y.array() <= cnstr.upper_bound().array()).all());

  // [1 -2]
  // [-2 1] is not p.s.d
  cnstr.Eval(Eigen::Vector2d(0, -1), y);
  EXPECT_TRUE((y.array() < cnstr.lower_bound().array()).any() ||
              (y.array() > cnstr.upper_bound().array()).any());
}
// Test that the Eval() method of LinearComplementarityConstraint correctly
// returns the slack.
GTEST_TEST(testConstraint, testSimpleLCPConstraintEval) {
  Eigen::Matrix2d M = Eigen::Matrix2d::Identity();
  Eigen::Vector2d q(-1, -1);

  LinearComplementarityConstraint c(M, q);
  Eigen::VectorXd x;
  c.Eval(Eigen::Vector2d(1, 1), x);

  EXPECT_TRUE(
      CompareMatrices(x, Vector2d(0, 0), 1e-4, MatrixCompareType::absolute));
  c.Eval(Eigen::Vector2d(1, 2), x);

  EXPECT_TRUE(
      CompareMatrices(x, Vector2d(0, 1), 1e-4, MatrixCompareType::absolute));
}
}  // namespace
}  // namespace solvers
}  // namespace drake
