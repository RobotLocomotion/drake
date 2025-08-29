#include "drake/solvers/sparse_and_dense_matrix.h"

#include <limits>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace solvers {
namespace internal {
namespace {

const double kInf = std::numeric_limits<double>::infinity();

void CheckGetDense(const SparseAndDenseMatrix* dut,
                   const Eigen::Ref<const Eigen::MatrixXd>& dense_expected) {
  EXPECT_TRUE(CompareMatrices(dut->GetAsDense(), dense_expected));
}

void CheckGetDenseThread(
    const SparseAndDenseMatrix& dut,
    const Eigen::Ref<const Eigen::MatrixXd>& dense_expected) {
  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i) {
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
  EXPECT_FALSE(dut.is_dense_constructed());
  EXPECT_TRUE(CompareMatrices(dut.get_as_sparse().toDense(), sparse.toDense()));
  EXPECT_TRUE(CompareMatrices(dut.GetAsDense(), sparse.toDense()));
  EXPECT_TRUE(dut.is_dense_constructed());
  CheckGetDenseThread(dut, sparse.toDense());
}

GTEST_TEST(SparseAndDenseMatrix, constructor_dense) {
  Eigen::Matrix<double, 2, 3> dense;
  dense.setZero();
  dense(0, 1) = 2;
  dense(1, 0) = 1;
  const SparseAndDenseMatrix dut(dense);
  EXPECT_EQ(dut.get_as_sparse().nonZeros(), 2);
  EXPECT_TRUE(dut.is_dense_constructed());
  EXPECT_TRUE(CompareMatrices(dut.get_as_sparse().toDense(), dense));
  EXPECT_TRUE(CompareMatrices(dut.GetAsDense(), dense));
  CheckGetDenseThread(dut, dense);
}

GTEST_TEST(SparseAndDenseMatrix, assign) {
  Eigen::Matrix<double, 2, 3> dense;
  dense.setZero();
  dense(0, 1) = 2;
  SparseAndDenseMatrix dut(dense);

  // Now assign from another dense matrix.
  Eigen::Matrix<double, 3, 3> dense_new;
  dense_new.setZero();
  dense_new(1, 0) = 3;
  dut = dense_new;
  EXPECT_EQ(dut.get_as_sparse().nonZeros(), 1);
  EXPECT_TRUE(CompareMatrices(dut.get_as_sparse().toDense(), dense_new));
  EXPECT_TRUE(CompareMatrices(dut.GetAsDense(), dense_new));

  // Now assign from a sparse matrix.
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.emplace_back(0, 1, 2.0);
  triplets.emplace_back(1, 0, 1.0);
  Eigen::SparseMatrix<double> sparse(3, 2);
  sparse.setFromTriplets(triplets.begin(), triplets.end());
  dut = sparse;
  EXPECT_EQ(dut.get_as_sparse().nonZeros(), 2);
  EXPECT_TRUE(CompareMatrices(dut.get_as_sparse().toDense(), sparse.toDense()));
  EXPECT_TRUE(CompareMatrices(dut.GetAsDense(), sparse.toDense()));
}

GTEST_TEST(SparseAndDenseMatrix, IsFinite) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.emplace_back(0, 1, 2.0);
  triplets.emplace_back(1, 0, 1.0);
  Eigen::SparseMatrix<double> sparse(3, 2);
  sparse.setFromTriplets(triplets.begin(), triplets.end());
  SparseAndDenseMatrix dut(sparse);
  EXPECT_TRUE(dut.IsFinite());

  sparse.coeffRef(0, 1) = kInf;
  dut = sparse;
  EXPECT_FALSE(dut.IsFinite());

  sparse.coeffRef(0, 1) = NAN;
  dut = sparse;
  EXPECT_FALSE(dut.IsFinite());
}
}  // namespace
}  // namespace internal
}  // namespace solvers
}  // namespace drake
