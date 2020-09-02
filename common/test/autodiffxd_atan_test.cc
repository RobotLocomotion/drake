#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Atan) {
  // Apparently clang avoids the heap better than gcc in this case.
  LimitMalloc guard({.max_num_allocations = 2996, .min_num_allocations = 2860});
  CHECK_UNARY_FUNCTION(atan, x, y, 0.1);
  CHECK_UNARY_FUNCTION(atan, x, y, -0.1);
  CHECK_UNARY_FUNCTION(atan, y, x, 0.1);
  CHECK_UNARY_FUNCTION(atan, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
