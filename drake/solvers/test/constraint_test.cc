#include "drake/solvers/constraint.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix2d;

namespace drake {
namespace solvers {
namespace {

// Tests that MatrixInnerProductConstraint evaluates correctly
GTEST_TEST(testConstraint,testMatrixInnerProductConstraint) {
  // impose the constraint that for a matrix X ,
  // 1 <= X(0,0) + 2X(0,1) + 3X(1,0) + 4X(1,1) <= 2
  Matrix2d A;
  A << 1.0, 2.0, 3.0, 4.0;
  double lb, ub;
  lb = 1;
  ub = 2;
  MatrixInnerProductConstraint cnstr(A, lb, ub);
  Matrix2d X;
  X << 4, 5, 2, 1;
  drake::Vector1d y;
  cnstr.Eval(X,y);
  EXPECT_TRUE(CompareMatrices(y,Vector1d::Constant(24), 1e-10, MatrixCompareType::absolute));
}
} // namespace
} // namespace solvers
} // namespace drake
