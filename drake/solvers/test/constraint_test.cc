#include "drake/solvers/constraint.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;

namespace drake {
namespace solvers {
namespace {

// Tests if the Lorentz Cone constraint is imposed correctly
void TestLorentzConeEval(const VectorXd &x_test) {
  auto cnstr = LorentzConeConstraint();
  VectorXd y;
  cnstr.Eval(x_test, y);
  Vector2d y_expected;
  y_expected(0) = x_test(0);
  y_expected(1) = x_test(0)*x_test(0) - x_test.tail(x_test.size()-1).squaredNorm();
  EXPECT_TRUE(CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(testConstraint,testLorentaConeConstraint) {
// Imposes the Lorentz Cone constraint for a 3 x 1 vector x, that
// x_1 >= sqrt(x_2^2 + x_3^2)
  auto cnstr = LorentzConeConstraint();
  auto lb = cnstr.lower_bound();
  auto ub = cnstr.upper_bound();
  EXPECT_TRUE(CompareMatrices(Eigen::Vector2d(0.0, 0.0), lb, 1E-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()), ub, 1e-10, MatrixCompareType::absolute));

  // [3;1;1] satisfy the lorentz cone constraint
  Eigen::Vector3d x1(3.0, 1.0, 1.0);
  TestLorentzConeEval(x1);
}
} // namespace
} // namespace solvers
} // namespace drake
