#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Max) {
  CHECK_BINARY_FUNCTION_ADS_ADS(max, x, y, 0.5);
  CHECK_BINARY_FUNCTION_ADS_ADS(max, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(max, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(max, y, x, -0.4);
}

}  // namespace
}  // namespace test
}  // namespace drake
