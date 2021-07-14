#include "drake/multibody/fixed_fem/dev/permute_tangent_matrix.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::MatrixXd;
constexpr int kNumVertices = 2;
constexpr int kNumDofs = kNumVertices * 3;
/* Returns an arbitrary tangent matrix for testing purpose. */
MatrixXd MakeTangentMatrix() {
  const int rows = kNumDofs;
  const int cols = kNumDofs;
  MatrixXd A(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      A(i, j) = cols * i + j;
    }
  }
  return A;
}

/* Verify that PermuteTangentMatrix provides the expected answer as
hand-calculated solution on a small problem. The problem consists of 2 vertices
permuted such that 0->1, 1->0. */
GTEST_TEST(PermuteTangentMatrix, AnalyticTest) {
  const MatrixXd A = MakeTangentMatrix();
  const Eigen::SparseMatrix<double> A_sparse = A.sparseView();
  const std::vector<int> vertex_permutation = {1, 0};
  const Eigen::SparseMatrix<double> permuted_A =
      PermuteTangentMatrix(A_sparse, vertex_permutation);
  const MatrixXd permuted_A_dense = permuted_A;

  MatrixXd expected_result(kNumDofs, kNumDofs);
  /* We use "i-j block" to denote the submatrix in the tangent matrix whose rows
   correspond to the i-th vertex and whose cols correspond to the j-th vertex.
  */
  /* The new 0-0 block is the same as the old 1-1 block. */
  expected_result.topLeftCorner<3, 3>() = A.bottomRightCorner<3, 3>();
  /* The new 0-1 block is the same as the old 1-0 block. */
  expected_result.topRightCorner<3, 3>() = A.bottomLeftCorner<3, 3>();
  /* The new 1-0 block is the same as the old 0-1 block. */
  expected_result.bottomLeftCorner<3, 3>() = A.topRightCorner<3, 3>();
  /* The new 1-1 block is the same as the old 0-0 block. */
  expected_result.bottomRightCorner<3, 3>() = A.topLeftCorner<3, 3>();
  EXPECT_TRUE(CompareMatrices(expected_result, permuted_A_dense));
}
}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
