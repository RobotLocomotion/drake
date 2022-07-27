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

}  // namespace
}  // namespace test
}  // namespace drake
