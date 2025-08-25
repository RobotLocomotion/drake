/* clang-format off */
#include "drake/common/autodiff.h"
#include "drake/common/ad/test/standard_operations_test.h"
/* clang-format on */

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Abs) {
  CHECK_UNARY_FUNCTION(abs, x, y, 0.1);
  CHECK_UNARY_FUNCTION(abs, x, y, -0.1);
  CHECK_UNARY_FUNCTION(abs, y, x, 0.1);
  CHECK_UNARY_FUNCTION(abs, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
