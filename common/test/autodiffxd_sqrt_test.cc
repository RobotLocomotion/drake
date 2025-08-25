/* clang-format off */
#include "drake/common/autodiff.h"
#include "drake/common/ad/test/standard_operations_test.h"
/* clang-format on */

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Sqrt) {
  CHECK_UNARY_FUNCTION(sqrt, x, y, 0.1);
  CHECK_UNARY_FUNCTION(sqrt, x, y, -0.1);
  CHECK_UNARY_FUNCTION(sqrt, y, x, 0.1);
  CHECK_UNARY_FUNCTION(sqrt, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
