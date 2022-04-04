#include "drake/solvers/sparse_and_dense_matrix.h"

#include <thread>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace solvers {
namespace internal {
namespace {
void CheckGetDense(const SparseAndDenseMatrix* dut,
                   const Eigen::Ref<const Eigen::MatrixXd>& dense_expected) {
  EXPECT_TRUE(CompareMatrices(dut->GetAsDense(), dense_expected));
}

void CheckGetDenseThread(
    const SparseAndDenseMatrix& dut,
    const Eigen::Ref<const Eigen::MatrixXd>& dense_expected) {
  std::vector<std::thread> threads;
  for (int i = 0; i < 100; ++i) {
    // SparseAndDenseMatrix is not movable, hence I cannot pass dut but have to
    // pass its pointer
    threads.emplace_back(CheckGetDense, &dut, dense_expected);
  }
  for (auto& thread : threads) {
    thread.join();
  }
}

GTEST_TEST(SparseAndDenseMatrix, constructor_sparse) {
  // Constructed from a sparse matrix.
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.emplace_back(0, 1, 2.0);
  triplets.emplace_back(1, 0, 1.0);
  Eigen::SparseMatrix<double> sparse(3, 2);
  sparse.setFromTriplets(triplets.begin(), triplets.end());
  const SparseAndDenseMatrix dut(sparse);
  EXPECT_EQ(dut.get_as_sparse().nonZeros(), sparse.nonZeros());
  EXPECT_TRUE(CompareMatrices(dut.get_as_sparse().toDense(), sparse.toDense()));
  EXPECT_TRUE(CompareMatrices(dut.GetAsDense(), sparse.toDense()));
  CheckGetDenseThread(dut, sparse.toDense());
}

GTEST_TEST(SparseAndDenseMatrix, constructor_dense) {
  Eigen::Matrix<double, 2, 3> dense;
  dense(0, 1) = 2;
  dense(1, 0) = 1;
  const SparseAndDenseMatrix dut(dense);
  EXPECT_TRUE(CompareMatrices(dut.get_as_sparse().toDense(), dense));
  EXPECT_TRUE(CompareMatrices(dut.GetAsDense(), dense));
  CheckGetDenseThread(dut, dense);
}
}  // namespace
}  // namespace internal
}  // namespace solvers
}  // namespace drake
