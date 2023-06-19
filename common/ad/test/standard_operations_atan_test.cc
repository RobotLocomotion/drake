#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

// Eigen doesn't provide an atan() overload, so we need to do it.
// This assumes the standard_operations_test.h is implemented using AutoDiff3.
AutoDiff3 atan(const AutoDiff3& x) {
  // ∂/∂x atan(x) = 1 / (1 + x²)
  return AutoDiff3{std::atan(x.value()),
                   x.derivatives() / (1 + x.value() * x.value())};
}

TEST_F(StandardOperationsTest, Atan) {
  CHECK_UNARY_FUNCTION(atan, x, y, 0.1);
  CHECK_UNARY_FUNCTION(atan, x, y, -0.1);
  CHECK_UNARY_FUNCTION(atan, y, x, 0.1);
  CHECK_UNARY_FUNCTION(atan, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
