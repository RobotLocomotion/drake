#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Sin) {
  CHECK_UNARY_FUNCTION(sin, x, y, 0.1);
  CHECK_UNARY_FUNCTION(sin, x, y, -0.1);
  CHECK_UNARY_FUNCTION(sin, y, x, 0.1);
  CHECK_UNARY_FUNCTION(sin, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
