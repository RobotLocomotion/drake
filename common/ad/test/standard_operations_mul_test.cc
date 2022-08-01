#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Multiplication) {
  CHECK_BINARY_OP(*, x, y, 1.0);
  CHECK_BINARY_OP(*, x, y, -1.0);
  CHECK_BINARY_OP(*, y, x, 1.0);
  CHECK_BINARY_OP(*, y, x, -1.0);
}

// The CHECK_BINARY_OP typically copies the first argument, so does not test
// aliasing between the two arguments. We'll do that here specifically.
TEST_F(StandardOperationsTest, MultiplicationInPlace) {
  AutoDiffDut x{0.75, 3, 0};
  const AutoDiffDut& y = x;
  x *= y;
  EXPECT_EQ(x.value(), 0.5625);
  // We have ∂/∂x x² = 2x x'.
  EXPECT_EQ(x.derivatives(), 2 * 0.75 * Eigen::Vector3d::Unit(0));
}

}  // namespace
}  // namespace test
}  // namespace drake
