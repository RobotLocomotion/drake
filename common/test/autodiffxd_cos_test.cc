/* clang-format off to disable clang-format-includes */
#include "drake/common/autodiff.h"
#include "drake/common/ad/test/standard_operations_test.h"
/* clang-format on */

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Cos) {
  CHECK_UNARY_FUNCTION(cos, x, y, 0.1);
  CHECK_UNARY_FUNCTION(cos, x, y, -0.1);
  CHECK_UNARY_FUNCTION(cos, y, x, 0.1);
  CHECK_UNARY_FUNCTION(cos, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
