#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Asin) {
  CHECK_UNARY_FUNCTION(asin, x, y, 0.1);
  CHECK_UNARY_FUNCTION(asin, x, y, -0.1);
  CHECK_UNARY_FUNCTION(asin, y, x, 0.1);
  CHECK_UNARY_FUNCTION(asin, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
