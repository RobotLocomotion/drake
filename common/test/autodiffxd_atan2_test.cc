#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Atan2) {
  // Apparently clang avoids the heap better than gcc in this case.
  LimitMalloc guard({.max_num_allocations = 3836, .min_num_allocations = 3752});
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, x, y, 0.1);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, x, y, -0.1);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, y, x, -0.4);
}

}  // namespace
}  // namespace test
}  // namespace drake
