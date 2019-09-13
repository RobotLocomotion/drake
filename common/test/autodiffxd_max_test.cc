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

TEST_F(AutoDiffXdTest, TieBreakingCheck) {
  // Given `max(v1, v2)`, Eigen autodiff's implementation of the max function
  // and our overload return the first argument `v1` when `v1 == v2` holds. In
  // Drake, we rely on this implementation-detail. This test checks if the
  // property holds so that we can detect a possible change in future.
  const AutoDiffXd v1{1.0, Vector1<double>(1.)};
  const AutoDiffXd v2{1.0, Vector1<double>(2.)};
  EXPECT_EQ(max(v1, v2).derivatives()[0], 1.0);  // Returns v1, not v2.
}

}  // namespace
}  // namespace test
}  // namespace drake
