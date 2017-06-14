#include "drake/solvers/mixed_integer_optimization_util.h"

#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(TestMixedIntegerUtil, TestCeilLog2) {
  const int kMaxExponent = 15;
  std::array<int, kMaxExponent + 1> two_to_exponent;
  two_to_exponent[0] = 1;
  for (int i = 1; i <= kMaxExponent; ++i) {
    two_to_exponent[i] = 2 * two_to_exponent[i - 1];
  }
  EXPECT_EQ(0, ceil_log2(1));
  for (int i = 0; i < kMaxExponent; ++i) {
    for (int j = two_to_exponent[i] + 1; j <= two_to_exponent[i]; ++j) {
      EXPECT_EQ(i + 1, ceil_log2(j));
    }
  }
}

GTEST_TEST(TestGrayCode, TestCalculateGrayCodes) {
  for (int i = 0; i < 4; i++) {
    auto test_code = drake::solvers::internal::CalculateReflectedGrayCodes(i);
    // Asking for codes for 0 bits should generate 0 for 0 bits.
    // Asking for codes for i bits should generate 2^(i) codes for i bits.
    EXPECT_EQ(test_code.cols(), i);
    EXPECT_EQ(test_code.rows(), i == 0 ? 0 : 1 << i);
    // Each code should differ by only one bit from the previous code.
    for (int j = 1; j < test_code.rows(); j ++) {
      EXPECT_EQ((test_code.row(j) - test_code.row(j - 1)).cwiseAbs().sum(), 1);
    }
  }
}

GTEST_TEST(TestGrayCode, TestGrayCodeToInteger) {
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector2i(0, 0)), 0);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector2i(0, 1)), 1);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector2i(1, 1)), 2);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector2i(1, 0)), 3);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(0, 0, 0)), 0);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(0, 0, 1)), 1);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(0, 1, 1)), 2);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(0, 1, 0)), 3);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(1, 1, 0)), 4);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(1, 1, 1)), 5);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(1, 0, 1)), 6);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(1, 0, 0)), 7);
}
}  // namespace
}  // namespace solvers
}  // namespace drake