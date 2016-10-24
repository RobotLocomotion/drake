#include "drake/util/drakeGradientUtil.h"

#include <array>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::Matrix;
using Eigen::MatrixXd;

namespace drake {
namespace {

GTEST_TEST(DrakeGradientUtilTest, MatGradMult) {
  const int nq = 34;
  const int A_rows = 8;
  const int A_cols = 6;
  auto dA = MatrixXd::Random(A_rows * A_cols, nq).eval();
  auto b = Matrix<double, A_cols, 1>::Random().eval();
  auto dAb = matGradMult(dA, b).eval();
  auto A = Matrix<double, A_rows, A_cols>::Random().eval();
  auto db = MatrixXd::Zero(b.rows(), nq).eval();
  auto dAb_check = matGradMultMat(A, b, dA, db);

  EXPECT_TRUE(CompareMatrices(dAb, dAb_check, 1e-10,
                              MatrixCompareType::absolute));
}

GTEST_TEST(DrakeGradientUtilTest, SetSubMatrixGradient) {
  const int A_rows = 4;
  const int A_cols = 4;
  const int nq = 34;

  std::array<int, 3> rows{{0, 1, 2}};
  std::array<int, 3> cols{{0, 1, 2}};

  int q_start = 2;
  const int q_subvector_size = 3;
  MatrixXd dA_submatrix =
      MatrixXd::Random(rows.size() * cols.size(), q_subvector_size);

  auto dA = Matrix<double, A_rows * A_cols, Eigen::Dynamic>::Random(
                A_rows * A_cols, nq).eval();
  setSubMatrixGradient<Eigen::Dynamic>(dA, dA_submatrix, rows, cols, A_rows,
                                       q_start, q_subvector_size);

  auto dA_submatrix_back = getSubMatrixGradient<Eigen::Dynamic>(
      dA, rows, cols, A_rows, q_start, q_subvector_size);

  EXPECT_TRUE(CompareMatrices(dA_submatrix_back, dA_submatrix, 1e-10,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace drake
