#include "drake/multibody/plant/slicing_and_indexing.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace internal {
namespace {

const std::vector<int> indices = {1, 3, 4};

MatrixXd MakeMatrixWithLinSpacedValues(int rows, int cols) {
  const int size = rows * cols;
  const VectorXd values = VectorXd::LinSpaced(size, 1, size);
  return Eigen::Map<const MatrixXd>(values.data(), rows, cols);
}

GTEST_TEST(SlicingAndIndexing, SelectRows) {
  const VectorXd M = MakeMatrixWithLinSpacedValues(6, 1);
  const MatrixXd S = SelectRows(M, indices);
  const VectorXd S_expected = (VectorXd(3, 1) << 2, 4, 5).finished();
  EXPECT_EQ(S, S_expected);
}

GTEST_TEST(SlicingAndIndexing, SelectCols) {
  const MatrixXd M = MakeMatrixWithLinSpacedValues(6, 5);
  const MatrixXd S = SelectCols(M, indices);
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

GTEST_TEST(SlicingAndIndexing, SelectRowsCols) {
  const MatrixXd M = MakeMatrixWithLinSpacedValues(6, 6);
  const MatrixXd S = SelectRowsCols(M, indices);
  // clang-format off
  const MatrixXd S_expected = (MatrixXd(3, 3) <<
     8, 20, 26,
    10, 22, 28,
    11, 23, 29).finished();
  // clang-format on
  EXPECT_EQ(S, S_expected);
}

GTEST_TEST(SlicingAndIndexing, ExpandRows) {
  const VectorXd M = MakeMatrixWithLinSpacedValues(3, 1);
  const int expanded_size = 8;
  const VectorXd S = ExpandRows(M, expanded_size, indices);
  const VectorXd S_expected =
      (VectorXd(expanded_size) << 0, 1, 0, 2, 3, 0, 0, 0).finished();
  EXPECT_EQ(S, S_expected);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
