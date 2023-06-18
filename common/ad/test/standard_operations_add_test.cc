#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Addition) {
  CHECK_BINARY_OP(+, x, y, 1.0);
  CHECK_BINARY_OP(+, x, y, -1.0);
  CHECK_BINARY_OP(+, y, x, 1.0);
  CHECK_BINARY_OP(+, y, x, -1.0);
}

namespace {

// We need to wrap the operator under test, to give it a name.
// Eigen doesn't provide unary operator+, so we'll no-op instead.
AutoDiffDut unary_add(const AutoDiffDut& x) {
  return +x;
}
AutoDiff3 unary_add(const AutoDiff3& x) {
  return x;
}

}  // namespace

TEST_F(StandardOperationsTest, UnaryAddition) {
  CHECK_UNARY_FUNCTION(unary_add, x, y, 1.0);
  CHECK_UNARY_FUNCTION(unary_add, x, y, -1.0);
  CHECK_UNARY_FUNCTION(unary_add, y, x, 1.0);
  CHECK_UNARY_FUNCTION(unary_add, y, x, -1.0);
}

}  // namespace
}  // namespace test
}  // namespace drake
