#include "drake/solvers/constraint.h"

#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"

using Eigen::Dynamic;
using Eigen::Ref;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::TaylorVecXd;
using drake::Vector1d;
using drake::MatrixCompareType;

namespace drake {
namespace solvers {
namespace {
/**
 * Test appending a linear constraint to an existing linear constraint, check if
 * the bounds and the linear matrix are correct after appending.
 */
GTEST_TEST(TestConstraint, AppendLinearConstraint) {
  // For a given linear constraint 0.0 <= x(0) <= 1.0, we append a linear
  // constraint 1.0 <= x(0) + x(1) + x(2) + x(3) <= 1.1
  // to the existing constraints. We then check if the resulting overall
  // constraint is in the correct form, i.e., its linear matrix should be
  // [1 0 0 0;1 1 1 1], and the lower and upper bounds should
  // be [0.0;1.0] and [1.0;1.1] respectively. Such that the resulting overall
  // constraint is
  // [0.0;1.0] <= [1 0 0 0; 1 1 1 1]*[x(0);x(1);x(2);x(3)] <= [1.0;1.1]
  Matrix<double, 1, 4> A;
  A.setZero();
  A(0) = 1.0;
  Vector1d lb, ub;
  lb << 0.0;
  ub << 1.0;
  LinearConstraint lin_con(A, lb, ub);
  auto A_ret = lin_con.A();
  EXPECT_TRUE(CompareMatrices(A, A_ret, 1e-10, MatrixCompareType::absolute));
  auto lb_ret = lin_con.lower_bound();
  auto ub_ret = lin_con.upper_bound();
  EXPECT_TRUE(CompareMatrices(lb, lb_ret, 1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(ub, ub_ret, 1e-10, MatrixCompareType::absolute));
  Matrix<double, 1, 4> A_append = Matrix<double, 1, 4>::Ones();
  Vector1d lb_append = Vector1d::Ones();
  Vector1d ub_append = lb_append + 0.1 * Vector1d::Ones().cwiseAbs();
  lin_con.AppendConstraint(A_append, lb_append, ub_append);
  Matrix<double, 2, 4> A_new;
  A_new << A, A_append;
  Vector2d lb_new;
  lb_new << lb, lb_append;
  Vector2d ub_new;
  ub_new << ub, ub_append;
  EXPECT_TRUE(
      CompareMatrices(A_new, lin_con.A(), 1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lb_new, lin_con.lower_bound(), 1e-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(ub_new, lin_con.upper_bound(), 1e-10,
                              MatrixCompareType::absolute));
}
}  // namespace
}  // namespace solvers
}  // namespace drake
