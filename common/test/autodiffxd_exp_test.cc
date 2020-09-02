#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Exp) {
  LimitMalloc guard({.max_num_allocations = 2856, .min_num_allocations = 2856});
  CHECK_UNARY_FUNCTION(exp, x, y, 0.1);
  CHECK_UNARY_FUNCTION(exp, x, y, -0.1);
  CHECK_UNARY_FUNCTION(exp, y, x, 0.1);
  CHECK_UNARY_FUNCTION(exp, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
