#include "drake/math/gray_code.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {
GTEST_TEST(TestGrayCode, TestCalculateGrayCodes) {
for (int i = 0; i < 4; i++) {
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
}  // namespace
}  // namespace math
}  // namespace drake
