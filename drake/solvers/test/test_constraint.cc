#include <typeinfo>

#include "drake/common/drake_assert.h"
#include "drake/solvers/constraint.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"
#include "gtest/gtest.h"

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
using drake::util::MatrixCompareType;

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(TestConstraint, AppendLinearConstraint) {
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
  Matrix<double, 3, 4> A_append = Matrix<double, 3, 4>::Random();
  Vector3d lb_append = Vector3d::Random();
  Vector3d ub_append = lb_append + Vector3d::Random().cwiseAbs();
  lin_con.appendConstraint(A_append, lb_append, ub_append);
  Matrix<double, 4, 4> A_new;
  A_new << A, A_append;
  Vector4d lb_new;
  lb_new << lb, lb_append;
  Vector4d ub_new;
  ub_new << ub, ub_append;
  EXPECT_TRUE(
      CompareMatrices(A_new, lin_con.A(), 1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lb_new, lin_con.lower_bound(), 1e-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(ub_new, lin_con.upper_bound(), 1e-10,
                              MatrixCompareType::absolute));
}
}
}
}
