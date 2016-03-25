#include "gtest/gtest.h"
#include "drake/util/eigen_matrix_compare.h"

using drake::util::MatrixCompareType;

namespace Drake {
namespace {

// Tests the ability for two identical matrices to be compared.
TEST(MatrixCompareTest, CompareIdenticalMatrices) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 0, 1, 2, 3;

  Eigen::MatrixXd m2(2, 2);
  m2 << 0, 1, 2, 3;

  double tolerance = 1e-8;
  std::string error_msg;

  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::absolute,
    &error_msg));

  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::relative,
    &error_msg));
}

}  // namespace
}  // namespace Drake
