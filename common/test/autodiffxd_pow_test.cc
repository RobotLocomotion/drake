#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Pow1) {
  CHECK_BINARY_FUNCTION_ADS_ADS(pow, x, y, 0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(pow, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(pow, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(pow, y, x, -0.4);
}

TEST_F(AutoDiffXdTest, Pow2) {
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, x, y, 0.3);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, y, x, -0.4);
  // Note that Eigen's AutoDiffScalar does not provide an implementation for
  // pow(double, ADS). Therefore, we do not provide tests for that here.
}

}  // namespace
}  // namespace test
}  // namespace drake
