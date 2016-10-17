#include "drake/solvers/constraint.h"

#include "gtest/gtest.h"

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
void TestLorentzConeEval(const VectorXd& x_test, bool is_in_cone) {
  auto cnstr = LorentzConeConstraint();
  VectorXd y;
  // Test Eval with VectorXd.
  cnstr.Eval(x_test, y);
  Vector2d y_expected;
  y_expected(0) = x_test(0);
  y_expected(1) =
      x_test(0) * x_test(0) - x_test.tail(x_test.size() - 1).squaredNorm();
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected = (y(0) >= 0) & (y(1) >= 0);
  EXPECT_TRUE(is_in_cone == is_in_cone_expected);

  auto tx = drake::math::initializeAutoDiff(x_test);
  TaylorVecXd x_taylor = tx;
  TaylorVecXd y_taylor;
  // Test Eval with TaylorVar.
  cnstr.Eval(x_taylor, y_taylor);

  // Check if the gradient is correct.
  Eigen::Matrix<double, 2, Eigen::Dynamic> y_grad_expected(2, x_test.size());
  y_grad_expected.setZero();
  y_grad_expected(0, 0) = 1.0;
  y_grad_expected(1, 0) = 2 * x_test(0);
  for (int i = 1; i < x_test.size(); i++) {
    y_grad_expected(1, i) = -2 * x_test(i);
  }
  EXPECT_TRUE(CompareMatrices(y_grad_expected,
                              drake::math::autoDiffToGradientMatrix(y_taylor),
                              1E-10, MatrixCompareType::absolute));
}

void TestRotatedLorentzConeEval(const VectorXd& x_test, bool is_in_cone) {
  auto cnstr = RotatedLorentzConeConstraint();
  VectorXd y;
  cnstr.Eval(x_test, y);
  Vector3d y_expected;
  y_expected(0) = x_test(0);
  y_expected(1) = x_test(1);
  y_expected(2) = x_test(0) * x_test(1) -
                  x_test.tail(x_test.size() - 2).squaredNorm();
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected =
      (x_test(0) >= 0) & (x_test(1) >= 0) &
      (x_test(0) * x_test(1) >= x_test.tail(x_test.size() - 2).norm());
  EXPECT_TRUE(is_in_cone == is_in_cone_expected);

  // Eval with taylor var.
  auto tx = drake::math::initializeAutoDiff(x_test);
  TaylorVecXd x_taylor = tx;
  TaylorVecXd y_taylor;
  cnstr.Eval(x_taylor, y_taylor);

  // Check if the gradient is correct.
  Eigen::Matrix<double, 3, Eigen::Dynamic> y_grad_expected(3, x_test.size());
  y_grad_expected.setZero();
  y_grad_expected(0, 0) = 1.0;
  y_grad_expected(1, 1) = 1.0;
  y_grad_expected(2, 0) = x_test(1);
  y_grad_expected(2, 1) = x_test(0);
  for (int i = 2; i < x_test.size(); ++i) {
    y_grad_expected(2, i) = -2 * x_test(i);
  }
  EXPECT_TRUE(CompareMatrices(y_grad_expected,
                              drake::math::autoDiffToGradientMatrix(y_taylor),
                              1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(testConstraint, testLorentzConeConstraint) {
  auto cnstr = LorentzConeConstraint();
  auto lb = cnstr.lower_bound();
  auto ub = cnstr.upper_bound();
  EXPECT_TRUE(CompareMatrices(Eigen::Vector2d(0.0, 0.0), lb, 1E-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()), ub,
      1e-10, MatrixCompareType::absolute));

  // [3;1;1] is in the interior of the Lorentz cone.
  Eigen::Vector3d x1(3.0, 1.0, 1.0);
  TestLorentzConeEval(x1, true);

  // [3;2;2;1] is on the boundary of the Lorentz cone.
  Eigen::Vector4d x2(3.0, 2.0, 2.0, 1.0);
  TestLorentzConeEval(x2, true);

  // [3; 3; 1] is outside of the Lorentz cone.
  Eigen::Vector3d x3(3.0, 3.0, 1.0);
  TestLorentzConeEval(x3, false);

  // [-3; 1; 1] is outside of the Lorentz cone.
  Eigen::Vector3d x4(-3.0, 1.0, 1.0);
  TestLorentzConeEval(x4, false);
}

GTEST_TEST(testConstraint, testRotatedLorentzConeConstraint) {
  auto cnstr = RotatedLorentzConeConstraint();
  auto lb = cnstr.lower_bound();
  auto ub = cnstr.upper_bound();
  EXPECT_TRUE(CompareMatrices(Eigen::Vector3d::Zero(), lb, 1E-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()), ub,
      1e-10, MatrixCompareType::absolute));

  // [1;2;1] is in the interior of the rotated lorentz cone.
  TestRotatedLorentzConeEval(Vector3d(1, 2, 1), true);

  // [1;2;1;1] is on the boundary of the rotated Lorentz cone.
  TestRotatedLorentzConeEval(Eigen::Vector4d(1, 2, 1, 1), true);

  // [1;2;2;2] is outside of the rotated Lorentz cone.
  TestRotatedLorentzConeEval(Eigen::Vector4d(1, 2, 2, 2), false);

  // [-1; -2; 1] is outside of the rotated Lorentz cone.
  TestRotatedLorentzConeEval(Vector3d(-1, -2, 1), false);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
