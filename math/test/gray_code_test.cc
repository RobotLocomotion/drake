#include "drake/math/gray_code.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {

GTEST_TEST(TestGrayCode, TestDegenerateCase) {
  auto zero_code = CalculateReflectedGrayCodes(0);
  EXPECT_EQ(zero_code.cols(), 0);
  EXPECT_EQ(zero_code.rows(), 0);
}

GTEST_TEST(TestGrayCode, TestCalculateGrayCodes) {
  for (int i = 1; i < 4; i++) {
    auto test_code = CalculateReflectedGrayCodes(i);
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

template <int NumDigits, int NumCodes>
void TestGrayCode(const Eigen::Ref<const Eigen::MatrixXi>& gray_codes) {
  auto gray_codes_dynamic = CalculateReflectedGrayCodes(NumDigits);
  auto gray_codes_static = CalculateReflectedGrayCodes<NumDigits>();
  static_assert(
      std::is_same_v<decltype(gray_codes_dynamic), Eigen::MatrixXi>,
      "Should be a dynamic sized matrix");
  static_assert(std::is_same_v<decltype(gray_codes_static),
                               Eigen::Matrix<int, NumCodes, NumDigits>>,
                "Should be a static sized matrix");
  EXPECT_TRUE(CompareMatrices(gray_codes, gray_codes_dynamic, 1E-5,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(gray_codes, gray_codes_static, 1E-5,
                              MatrixCompareType::absolute));
}

GTEST_TEST(TestGrayCode, TestCalculateGrayCodes0) {
  Eigen::Matrix<int, 0, 0> gray_codes;
  TestGrayCode<0, 0>(gray_codes);
}

GTEST_TEST(TestGrayCode, TestCalculateGrayCodes1) {
  Eigen::Matrix<int, 2, 1> gray_codes(0, 1);
  TestGrayCode<1, 2>(gray_codes);
}

GTEST_TEST(TestGrayCode, TestCalculateGrayCodes2) {
  Eigen::Matrix<int, 4, 2> gray_codes;
  // clang-format off
  gray_codes << 0, 0,
                0, 1,
                1, 1,
                1, 0;
  // clang-format on
  TestGrayCode<2, 4>(gray_codes);
}

GTEST_TEST(TestGrayCode, TestCalculateGrayCodes3) {
  Eigen::Matrix<int, 8, 3> gray_codes;
  // clang-format off
  gray_codes << 0, 0, 0,
                0, 0, 1,
                0, 1, 1,
                0, 1, 0,
                1, 1, 0,
                1, 1, 1,
                1, 0, 1,
                1, 0, 0;
  // clang-format on
  TestGrayCode<3, 8>(gray_codes);
}

GTEST_TEST(TestGrayCode, TestGrayCodeToInteger) {
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector2i(0, 0)), 0);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector2i(0, 1)), 1);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector2i(1, 1)), 2);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector2i(1, 0)), 3);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector3i(0, 0, 0)), 0);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector3i(0, 0, 1)), 1);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector3i(0, 1, 1)), 2);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector3i(0, 1, 0)), 3);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector3i(1, 1, 0)), 4);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector3i(1, 1, 1)), 5);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector3i(1, 0, 1)), 6);
  EXPECT_EQ(GrayCodeToInteger(Eigen::Vector3i(1, 0, 0)), 7);
}
}  // namespace
}  // namespace math
}  // namespace drake
