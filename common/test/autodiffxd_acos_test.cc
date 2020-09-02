#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Acos) {
  LimitMalloc guard({.max_num_allocations = 2236, .min_num_allocations = 2236});
  CHECK_UNARY_FUNCTION(acos, x, y, 0.1);
  CHECK_UNARY_FUNCTION(acos, x, y, -0.1);
  CHECK_UNARY_FUNCTION(acos, y, x, 0.1);
  CHECK_UNARY_FUNCTION(acos, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
