#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Division) {
  CHECK_BINARY_OP(/, x, y, 1.0);
  CHECK_BINARY_OP(/, x, y, -1.0);
  CHECK_BINARY_OP(/, y, x, 1.0);
  CHECK_BINARY_OP(/, y, x, -1.0);
}

// The CHECK_BINARY_OP typically copies the first argument, so does not test
// aliasing between the two arguments. We'll do that here specifically.
TEST_F(StandardOperationsTest, DivisionInPlace) {
  AutoDiffDut x{0.5, 3, 0};
  const AutoDiffDut& y = x;
  x /= y;
  EXPECT_EQ(x.value(), 1.0);
  EXPECT_EQ(x.derivatives(), Eigen::Vector3d::Zero());
}

}  // namespace
}  // namespace test
}  // namespace drake
