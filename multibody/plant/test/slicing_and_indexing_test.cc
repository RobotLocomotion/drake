#include "drake/multibody/plant/slicing_and_indexing.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace internal {
namespace {

class SlicingAndIndexingTest : public ::testing::Test {
 protected:
  static MatrixXd MakeMatrixWithLinSpacedValues(int rows, int cols) {
    const int size = rows * cols;
    const VectorXd values = VectorXd::LinSpaced(size, 1, size);
    return Eigen::Map<const MatrixXd>(values.data(), rows, cols);
  }

  const std::vector<int> kIndices = {1, 3, 4};
};

TEST_F(SlicingAndIndexingTest, SelectRows) {
  const VectorXd M = MakeMatrixWithLinSpacedValues(6, 1);
  const MatrixXd S = SelectRows(M, kIndices);
  const VectorXd S_expected = Vector3d(2, 4, 5);
  EXPECT_EQ(S, S_expected);
}

TEST_F(SlicingAndIndexingTest, ExcludeRows) {
  const VectorXd M = MakeMatrixWithLinSpacedValues(6, 1);
  const MatrixXd S = ExcludeRows(M, kIndices);
  const VectorXd S_expected = Vector3d(1, 3, 6);
  EXPECT_EQ(S, S_expected);
}

TEST_F(SlicingAndIndexingTest, SelectCols) {
  const MatrixXd M = MakeMatrixWithLinSpacedValues(6, 5);
  const MatrixXd S = SelectCols(M, kIndices);
  // clang-format off
  const MatrixXd S_expected = (MatrixXd(6, 3) <<
     7, 19, 25,
     8, 20, 26,
     9, 21, 27,
    10, 22, 28,
    11, 23, 29,
    12, 24, 30).finished();
  // clang-format on
  EXPECT_EQ(S, S_expected);
}

TEST_F(SlicingAndIndexingTest, ExcludeCols) {
  const MatrixXd M = MakeMatrixWithLinSpacedValues(6, 5);
  // Test MatrixX variant.
  {
    const MatrixXd S = ExcludeCols(M, kIndices);
    // clang-format off
    const MatrixXd S_expected = (MatrixXd(6, 2) <<
      1, 13,
      2, 14,
      3, 15,
      4, 16,
      5, 17,
      6, 18).finished();
    // clang-format on
    EXPECT_EQ(S, S_expected);
  }

  // Test dense MatrixBlock variant.
  {
    const contact_solvers::internal::MatrixBlock<double> M_block(M);
    const contact_solvers::internal::MatrixBlock<double> S =
        ExcludeCols(M_block, kIndices);
    // clang-format off
    const MatrixXd S_expected = (MatrixXd(6, 2) <<
      1, 13,
      2, 14,
      3, 15,
      4, 16,
      5, 17,
      6, 18).finished();
    // clang-format on
    EXPECT_EQ(S.MakeDenseMatrix(), S_expected);
  }

  // Test sparse MatrixBlock variant.
  {
    const contact_solvers::internal::MatrixBlock<double> M_block(
        contact_solvers::internal::Block3x3SparseMatrix<double>(0, 0));
    EXPECT_THROW(ExcludeCols(M_block, kIndices), std::runtime_error);
  }
}

TEST_F(SlicingAndIndexingTest, SelectRowsCols) {
  const MatrixXd M = MakeMatrixWithLinSpacedValues(6, 6);
  const MatrixXd S = SelectRowsCols(M, kIndices);
  // clang-format off
  const MatrixXd S_expected = (MatrixXd(3, 3) <<
     8, 20, 26,
    10, 22, 28,
    11, 23, 29).finished();
  // clang-format on
  EXPECT_EQ(S, S_expected);
}

TEST_F(SlicingAndIndexingTest, ExcludeRowsCols) {
  const MatrixXd M = MakeMatrixWithLinSpacedValues(6, 6);
  const MatrixXd S = ExcludeRowsCols(M, kIndices);
  // clang-format off
  const MatrixXd S_expected = (MatrixXd(3, 3) <<
     1, 13, 31,
     3, 15, 33,
     6, 18, 36).finished();
  // clang-format on
  EXPECT_EQ(S, S_expected);
}

TEST_F(SlicingAndIndexingTest, ExpandRows) {
  const VectorXd M = MakeMatrixWithLinSpacedValues(3, 1);
  const int expanded_size = 8;
  const VectorXd S = ExpandRows(M, expanded_size, kIndices);
  const VectorXd S_expected =
      (VectorXd(expanded_size) << 0, 1, 0, 2, 3, 0, 0, 0).finished();
  EXPECT_EQ(S, S_expected);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
