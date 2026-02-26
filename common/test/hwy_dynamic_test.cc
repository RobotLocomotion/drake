#include "drake/common/hwy_dynamic.h"

#include <string>
#include <utility>

#include "hwy/tests/hwy_gtest.h"
#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test/hwy_dynamic_test_array_mul.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace internal {
namespace {

using Eigen::ArrayXd;
using Eigen::VectorXd;

// TODO(jwnimmer-tri) This similar of fixture (using hwy_gtest.h) is duplicated
// in two other places (//math and //geometry/proximity). We should probably
// seek a more elegant way to do this, instead of copying it to more places.

/* This hwy-infused test fixture replicates every test case to be run against
every target architecture variant (e.g., SSE4, AVX2, AVX512VL, etc). When run,
it filters the suite to only run tests that the current CPU can handle. */
class Fixture : public hwy::TestWithParamTarget {
 protected:
  void SetUp() override {
    // Reset Drake's target selection.
    drake::internal::HwyDynamicReset();
    // Reset Highway's target selection.
    hwy::TestWithParamTarget::SetUp();
    // Clear old statistics from prior test cases.
    GetMutableArrayMulCounters().clear();
  }
};

// Instatiate the suite for all CPU targets (using the HWY macro).
HWY_TARGET_INSTANTIATE_TEST_SUITE_P(Fixture);

TEST_P(Fixture, ArrayMulTest) {
  const ArrayXd x0 = ArrayXd::LinSpaced(16, 1.0, 16.0);
  const ArrayXd x1 = ArrayXd::LinSpaced(16, 0.0625, 1.0);
  ArrayXd y1;

  // Run the test logic twice, so that we'll be able to confirm that things
  // which should happen once vs twice were correct.
  for (int i = 0; i < 2; ++i) {
    // Clear the prior output.
    y1 = {};

    // Run the calculation.
    ArrayMul(x0, x1, &y1);

    // Check the computed answer.
    const ArrayXd expected = x0 * x1;
    ASSERT_TRUE(CompareMatrices(VectorXd{y1}, VectorXd{expected}));
  }

  // Check that Highway chose something.
  ASSERT_TRUE(hwy::GetChosenTarget().IsInitialized());

  // We should see exactly 1 call to the target selector, and 2 calls to the
  // target-specific impl function.
  const std::string expected_impl =
      fmt::format("ArrayMulImpl.{}", hwy::TargetName(GetParam()));
  using Pair = std::pair<std::string, int>;
  string_map<int>& counters = GetMutableArrayMulCounters();
  EXPECT_THAT(counters,
              testing::UnorderedElementsAre(Pair("ChooseBestArrayMul", 1),
                                            Pair(expected_impl, 2)));
}

}  // namespace
}  // namespace internal
}  // namespace drake
