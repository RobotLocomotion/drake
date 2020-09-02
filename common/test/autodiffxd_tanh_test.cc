#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Tanh) {
  LimitMalloc guard({.max_num_allocations = 2860, .min_num_allocations = 2860});
  CHECK_UNARY_FUNCTION(tanh, x, y, 0.1);
  CHECK_UNARY_FUNCTION(tanh, x, y, -0.1);
  CHECK_UNARY_FUNCTION(tanh, y, x, 0.1);
  CHECK_UNARY_FUNCTION(tanh, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
