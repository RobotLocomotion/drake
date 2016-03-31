#include "drake/util/eigen_matrix_compare.h"
#include "gtest/gtest.h"

using drake::util::MatrixCompareType;

namespace drake {
namespace util {
namespace {

// Tests the ability for two identical matrices to be compared.
TEST(MatrixCompareTest, CompareIdentical) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 0, 1, 2, 3;

  Eigen::MatrixXd m2(2, 2);
  m2 << 0, 1, 2, 3;

  Eigen::MatrixXd m3(2, 2);
  m3 << 100, 200, 300, 400;

  const double tolerance = 1e-8;
  std::string explanation;

  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::absolute,
                              &explanation));

  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::relative,
                              &explanation));

  EXPECT_FALSE(CompareMatrices(m1, m3, tolerance, MatrixCompareType::absolute,
                               &explanation));

  EXPECT_FALSE(CompareMatrices(m1, m3, tolerance, MatrixCompareType::relative,
                               &explanation));
}

// Tests absolute tolerance with real numbers.
TEST(MatrixCompareTest, AbsoluteCompare) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 0, 1, 2, 3;

  Eigen::MatrixXd m2(2, 2);
  m2 << 0, 1 - 1e-10, 2, 3;

  Eigen::MatrixXd m3(2, 2);
  m3 << 0, 1, 2 - 1e-8, 3;

  Eigen::MatrixXd m4(2, 2);
  m4 << 0, 1, 2, 3 - 1e-6;

  const double tolerance = 1e-8;
  std::string error_msg;

  // The difference between m1 and m2 is less than the tolerance.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::absolute,
                              &error_msg));

  // The difference between m1 and m3 is exactly equal to the tolerance.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m3, tolerance, MatrixCompareType::absolute,
                              &error_msg));

  // The difference between m1 and m4 is greater than the tolerance.
  // They should be considered different.
  EXPECT_FALSE(CompareMatrices(m1, m4, tolerance, MatrixCompareType::absolute,
                               &error_msg));
}

// Tests absolute tolerance with NaN values
TEST(MatrixCompareTest, AbsoluteNaNCompare) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 0, 1, std::numeric_limits<double>::quiet_NaN(), 3;

  Eigen::MatrixXd m2(2, 2);
  m2 << 0, 1, std::numeric_limits<double>::quiet_NaN(), 3;

  Eigen::MatrixXd m3(2, 2);
  m3 << 0, 1 - 1e-10, std::numeric_limits<double>::quiet_NaN(), 3;

  Eigen::MatrixXd m4(2, 2);
  m4 << 0, 1, 2, 3;

  const double tolerance = 1e-8;
  std::string error_msg;

  // The difference between m1 and m2 is less than the tolerance.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m2, tolerance, MatrixCompareType::absolute,
                              &error_msg));

  // The difference between m1 and m3 is exactly equal to the tolerance.
  // They should be considered equal.
  EXPECT_TRUE(CompareMatrices(m1, m3, tolerance, MatrixCompareType::absolute,
                              &error_msg));

  // The difference between m1 and m4 is greater than the tolerance.
  // They should be considered different.
  EXPECT_FALSE(CompareMatrices(m1, m4, tolerance, MatrixCompareType::absolute,
                               &error_msg));
}

// Tests absolute tolerance with real numbers.
TEST(MatrixCompareTest, RelativeCompare) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 100, 100, 100, 100;

  Eigen::MatrixXd m2(2, 2);
  m2 << 100, 100 * 0.9, 100, 100;

  std::string error_msg;

  // The difference between m1 and m2 is more than 1%.
  // They should be considered not equal.
  EXPECT_FALSE(
      CompareMatrices(m1, m2, 0.01, MatrixCompareType::relative, &error_msg));

  // The difference between m1 and m2 is equal to 10%.
  // They should be considered equal.
  EXPECT_TRUE(
      CompareMatrices(m1, m2, 0.1, MatrixCompareType::relative, &error_msg));

  // The difference between m1 and m4 is less than 20%.
  // They should be considered equal.
  EXPECT_TRUE(
      CompareMatrices(m1, m2, 0.2, MatrixCompareType::relative, &error_msg));
}

// Tests ability to not specify an error message parameter
TEST(MatrixCompareTest, NoMessageParam) {
  Eigen::MatrixXd m1(2, 2);
  m1 << 1, 2, 3, 4;

  Eigen::MatrixXd m2(2, 2);
  m2 << 1, 2, 3, 4;

  const double one_pct = 0.01;

  // The difference between m1 and m2 is less than 1%.
  // They should be considered equal.
  // Note that we do not specify an error message parameter.
  EXPECT_TRUE(CompareMatrices(m1, m2, one_pct, MatrixCompareType::relative));
}

}  // namespace
}  // namespace util
}  // namespace drake
