#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Multiplication) {
  LimitMalloc guard({.max_num_allocations = 1538, .min_num_allocations = 1538});
  CHECK_BINARY_OP(*, x, y, 1.0);
  CHECK_BINARY_OP(*, x, y, -1.0);
  CHECK_BINARY_OP(*, y, x, 1.0);
  CHECK_BINARY_OP(*, y, x, -1.0);
}

}  // namespace
}  // namespace test
}  // namespace drake
