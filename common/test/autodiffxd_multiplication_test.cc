/* clang-format off */
#include "drake/common/autodiff.h"
#include "drake/common/ad/test/standard_operations_test.h"
/* clang-format on */

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Multiplication) {
  CHECK_BINARY_OP(*, x, y, 1.0);
  CHECK_BINARY_OP(*, x, y, -1.0);
  CHECK_BINARY_OP(*, y, x, 1.0);
  CHECK_BINARY_OP(*, y, x, -1.0);
}

}  // namespace
}  // namespace test
}  // namespace drake
