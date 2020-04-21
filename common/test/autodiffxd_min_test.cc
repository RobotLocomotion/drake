#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Min) {
  CHECK_BINARY_FUNCTION_ADS_ADS(min, x, y, 0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(min, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(min, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(min, y, x, -0.4);
}

TEST_F(AutoDiffXdTest, TieBreakingCheckMinBothNonEmpty) {
  // Given `min(v1, v2)`, our overload returns the first argument `v1` when
  // `v1 == v2` holds if both `v1` and `v2` have non-empty derivatives. In
  // Drake, we rely on this implementation-detail. This test checks if the
  // property holds so that we can detect a possible change in future.
  const AutoDiffXd v1{1.0, Vector1<double>(3.)};
  const AutoDiffXd v2{1.0, Vector1<double>(2.)};
  EXPECT_EQ(min(v1, v2).derivatives()[0], 3.0);  // Returns v1, not v2.
}

TEST_F(AutoDiffXdTest, TieBreakingCheckMinOneNonEmpty) {
  // Given `min(v1, v2)`, our overload returns whichever argument has non-empty
  // derivatives in the case where only one has non-empty derivatives. In
  // Drake, we rely on this implementation-detail. This test checks if the
  // property holds so that we can detect a possible change in future.
  const AutoDiffXd v1{1.0};
  const AutoDiffXd v2{1.0, Vector1<double>(2.)};
  EXPECT_TRUE(CompareMatrices(min(v1, v2).derivatives(),
                              Vector1<double>(2.)));  // Returns v2, not v1.
  EXPECT_TRUE(CompareMatrices(min(v2, v1).derivatives(),
                              Vector1<double>(2.)));  // Returns v2, not v1.
}

}  // namespace
}  // namespace test
}  // namespace drake
