#include "drake/math/eigen_sparse_triplet.h"

#include "gtest/gtest.h"

namespace drake {
namespace math {
namespace {

/**
 * Given a sparse matrix, call SparseMatrixToTriplets or
 * SparseMatrixToRowColumnValueVectors to get the triplets
 * (row_index, column_index, value) of the non-zero entries. Then reconstruct
 * the sparse matrix using setFromTriplets, anch check if the reconstructed
 * sparse matrix is the same as the original one
 */
template <typename T, int options>
void checkTriplet(const Eigen::SparseMatrix<T, options>& sp_mat) {
  auto triplets1 = SparseMatrixToTriplets(sp_mat);
  Eigen::SparseMatrix<T, options> sp_mat_expected1(sp_mat.rows(),
                                                   sp_mat.cols());
  sp_mat_expected1.setFromTriplets(triplets1.begin(), triplets1.end());
  EXPECT_TRUE(sp_mat.isApprox(sp_mat_expected1));

  std::vector<Eigen::Index> row_indices;
  std::vector<Eigen::Index> col_indices;
  std::vector<T> val;
  SparseMatrixToRowColumnValueVectors(sp_mat, row_indices, col_indices, val);
  EXPECT_TRUE(static_cast<int>(row_indices.size()) == sp_mat.nonZeros());
  EXPECT_TRUE(row_indices.size() == col_indices.size());
  EXPECT_TRUE(row_indices.size() == val.size());
  std::vector<Eigen::Triplet<T>> triplets2;
  for (int i = 0; i < static_cast<int>(row_indices.size()); i++) {
    triplets2.push_back(
        Eigen::Triplet<T>(row_indices[i], col_indices[i], val[i]));
  }
  Eigen::SparseMatrix<T, options> sp_mat_expected2(sp_mat.rows(),
                                                   sp_mat.cols());
  sp_mat_expected2.setFromTriplets(triplets2.begin(), triplets2.end());
  EXPECT_TRUE(sp_mat.isApprox(sp_mat_expected2));
}

/**
 * Test if the SparseMatrixToTriplets and SparseMatrixToRowColumnValueVectors
 * get the correct triplets (row_index, column_index, value) of the non-zero
 * entries.
 */
GTEST_TEST(testEigenSparseTriplet, sparseToTriplet) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.push_back(Eigen::Triplet<double>(0, 0, 1.0));
  triplets.push_back(Eigen::Triplet<double>(0, 1, 2.0));
  triplets.push_back(Eigen::Triplet<double>(1, 1, 3.0));

  // Test a row major sparse matrix.
  Eigen::SparseMatrix<double, Eigen::RowMajor> sp_mat1(2, 3);
  sp_mat1.setFromTriplets(triplets.begin(), triplets.end());
  checkTriplet(sp_mat1);

  // Insert a value to the sparse matrix, it should be uncompressed now.
  Eigen::SparseMatrix<double, Eigen::RowMajor> sp_mat2 = sp_mat1;
  sp_mat2.insert(1, 2) = 2.5;
  EXPECT_FALSE(sp_mat2.isCompressed());
  checkTriplet(sp_mat2);

  // Check column major sparse matrix.
  Eigen::SparseMatrix<double, Eigen::ColMajor> sp_mat3(2, 3);
  sp_mat3.setFromTriplets(triplets.begin(), triplets.end());
  checkTriplet(sp_mat3);

  // Check a dense matrix
  triplets.push_back(Eigen::Triplet<double>(1, 0, 4.0));
  Eigen::SparseMatrix<double, Eigen::RowMajor> sp_mat4(2, 2);
  sp_mat4.setFromTriplets(triplets.begin(), triplets.end());
  checkTriplet(sp_mat4);
}
}  // namespace
}  // namespace math
}  // namespace drake
