#include <numeric>
#include <unordered_set>
#include <vector>

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
int omp_get_thread_num() {
  return 0;
}
#endif

// Mostly, this just checks for compilation failures.
GTEST_TEST(OpenmpTest, ParallelFor) {
  drake::log()->info("Using kHasOpenmp = {}", kHasOpenmp);

  // Allocate storage for one integer result per loop.
  constexpr int num_outputs = 100;
  std::vector<int> outputs(static_cast<size_t>(num_outputs), 0);

  // Populate the storage, in parallel.
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (int i = 0; i < num_outputs; ++i) {
    outputs[i] = omp_get_thread_num();
  }

  // Our BUILD rule will run this program with a maximum of two threads.
  // Confirm how many threads were used and that their thread numbers were the
  // expected set of either {0, 1} (with OpenMP) or {0} (without OpenMP).
  std::unordered_set<int> thread_nums(outputs.begin(), outputs.end());
  if (kHasOpenmp) {
    EXPECT_THAT(thread_nums, testing::UnorderedElementsAre(0, 1));
  } else {
    EXPECT_THAT(thread_nums, testing::UnorderedElementsAre(0));
  }
}

}  // namespace
}  // namespace drake
