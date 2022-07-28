#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Abs) {
  CHECK_UNARY_FUNCTION(abs, x, y, 1.1);
  CHECK_UNARY_FUNCTION(abs, x, y, -1.1);
  CHECK_UNARY_FUNCTION(abs, y, x, 1.1);
  CHECK_UNARY_FUNCTION(abs, y, x, -1.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
