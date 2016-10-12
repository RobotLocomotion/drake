#include "drake/math/eigen_sparse_triplet.h"

#include "gtest/gtest.h"

namespace drake {
namespace math {
namespace {

template <typename T, int options>
void checkTriplet(const Eigen::SparseMatrix<T, options>& sp_mat) {
  auto triplet1 = SparseMatrixToTriplets(sp_mat);
  Eigen::SparseMatrix<T, options> sp_mat_expected1(sp_mat.rows(),
                                                   sp_mat.cols());
  sp_mat_expected1.setFromTriplets(triplet1.begin(), triplet1.end());
  EXPECT_TRUE(sp_mat.isApprox(sp_mat_expected1));

  std::vector<Eigen::Index> row_indices;
  std::vector<Eigen::Index> col_indices;
  std::vector<T> val;
  SparseMatrixToRowColumnValueVectors(sp_mat, row_indices, col_indices, val);
  EXPECT_TRUE(static_cast<int>(row_indices.size()) == sp_mat.nonZeros());
  EXPECT_TRUE(row_indices.size() == col_indices.size());
  EXPECT_TRUE(row_indices.size() == val.size());
  std::vector<Eigen::Triplet<T>> triplet2;
  for (int i = 0; i < static_cast<int>(row_indices.size()); i++) {
    triplet2.push_back(
        Eigen::Triplet<T>(row_indices[i], col_indices[i], val[i]));
  }
  Eigen::SparseMatrix<T, options> sp_mat_expected2(sp_mat.rows(),
                                                   sp_mat.cols());
  sp_mat_expected2.setFromTriplets(triplet2.begin(), triplet2.end());
  EXPECT_TRUE(sp_mat.isApprox(sp_mat_expected2));
}

GTEST_TEST(testEigenSparseTriplet, sparseToTriplet) {
  std::vector<Eigen::Triplet<double>> triplet;
  triplet.push_back(Eigen::Triplet<double>(0, 0, 1.0));
  triplet.push_back(Eigen::Triplet<double>(0, 1, 2.0));
  triplet.push_back(Eigen::Triplet<double>(1, 1, 3.0));

  Eigen::SparseMatrix<double, Eigen::RowMajor> sp_mat1(2, 3);
  sp_mat1.setFromTriplets(triplet.begin(), triplet.end());
  checkTriplet(sp_mat1);

  // Insert a value to the sparse matrix, it should be uncompressed now.
  Eigen::SparseMatrix<double, Eigen::RowMajor> sp_mat2 = sp_mat1;
  sp_mat2.insert(1, 2) = 2.5;
  EXPECT_FALSE(sp_mat2.isCompressed());
  checkTriplet(sp_mat2);

  // Check column major sparse matrix.
  Eigen::SparseMatrix<double, Eigen::ColMajor> sp_mat3(2, 3);
  sp_mat3.setFromTriplets(triplet.begin(), triplet.end());
  checkTriplet(sp_mat3);
}
}  // namespace
}  // namespace math
}  // namespace drake
