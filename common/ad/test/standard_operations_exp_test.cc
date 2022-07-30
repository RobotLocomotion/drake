#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Exp) {
  CHECK_UNARY_FUNCTION(exp, x, y, 0.1);
  CHECK_UNARY_FUNCTION(exp, x, y, -0.1);
  CHECK_UNARY_FUNCTION(exp, y, x, 0.1);
  CHECK_UNARY_FUNCTION(exp, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
