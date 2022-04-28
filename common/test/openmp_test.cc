#include <numeric>
#include <unordered_set>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#if defined(_OPENMP)
#include <omp.h>
#endif

#include "drake/common/text_logging.h"

namespace drake {
namespace {

#if defined(_OPENMP)
constexpr bool kHasOpenmp = true;
#else
constexpr bool kHasOpenmp = false;
// TODO(jwnimmer-tri) This should be a Drake helper function wrapper that
// abstracts away this difference.  The openmp_helpers.hpp wrapper from
// common_robotics_utilities is a likely candidate to delegate to, or at
// least take as inspiration.
int omp_get_thread_num() { return 0; }
#endif

// Mostly, this just checks for compilation failures.
GTEST_TEST(OpenmpTest, ParallelFor) {
  drake::log()->info("Using kHasOpenmp = {}", kHasOpenmp);

  // Allocate storage for one integer result per loop.
  constexpr int num_outputs = 100;
  std::vector<int> outputs(static_cast<size_t>(num_outputs), 0);

  // Populate the storage, in parallel.
  constexpr int num_threads = 4;
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int i = 0; i < num_outputs; ++i) {
    outputs[i] = omp_get_thread_num();
  }

  // Confirm how many threads were used, and that their thread numbers were
  // the expected set of either {0} or {0, 1, 2, 3}.
  std::vector<int> expected_thread_nums(kHasOpenmp ? num_threads : 1);
  std::iota(expected_thread_nums.begin(), expected_thread_nums.end(), 0);
  EXPECT_THAT(std::unordered_set<int>(outputs.begin(), outputs.end()),
              testing::UnorderedElementsAreArray(expected_thread_nums));
}

}  // namespace
}  // namespace drake
