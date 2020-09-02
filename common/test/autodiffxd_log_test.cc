#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Log) {
  LimitMalloc guard({.max_num_allocations = 2836, .min_num_allocations = 2836});
  CHECK_UNARY_FUNCTION(log, x, y, 0.1);
  CHECK_UNARY_FUNCTION(log, x, y, -0.1);
  CHECK_UNARY_FUNCTION(log, y, x, 0.1);
  CHECK_UNARY_FUNCTION(log, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
