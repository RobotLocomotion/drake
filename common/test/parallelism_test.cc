#include "planning/parallelism.h"

#include <gtest/gtest.h>

namespace anzu {
namespace planning {
namespace {
constexpr int kNoneThreads = 1;
// This is configured by the test rule, which sets the environment variable.
constexpr int kMaxThreads = 2;

// Tests that different spellings of no parallelism are equivalent.
GTEST_TEST(ParallelismTest, NoneTest) {
  const Parallelism default_parallelism;
  EXPECT_EQ(default_parallelism.num_threads(), kNoneThreads);

  const Parallelism none_parallelism = Parallelism::None();
  EXPECT_EQ(none_parallelism.num_threads(), kNoneThreads);

  const Parallelism false_parallelism(false);
  EXPECT_EQ(false_parallelism.num_threads(), kNoneThreads);
}

// Tests that different spellings of max parallelism are equivalent.
GTEST_TEST(ParallelismTest, MaxTest) {
  const Parallelism max_parallelism = Parallelism::Max();
  EXPECT_EQ(max_parallelism.num_threads(), kMaxThreads);

  const Parallelism true_parallelism(true);
  EXPECT_EQ(true_parallelism.num_threads(), kMaxThreads);
}

// Tests construction with a specific number of threads.
GTEST_TEST(ParallelismTest, NumThreadsTest) {
  // Valid values for number of threads.
  for (const int num_threads : {1, 2, 10, 1000}) {
    EXPECT_EQ(Parallelism(num_threads).num_threads(), num_threads);
  }

  // Specified number of threads must be >= 1.
  EXPECT_THROW(Parallelism(0), std::exception);
  EXPECT_THROW(Parallelism(-1), std::exception);
  EXPECT_THROW(Parallelism(-1000), std::exception);
}

// Tests that conversion and assignment from bool work.
GTEST_TEST(ParallelismTest, BoolConversionAssignmentTest) {
  const Parallelism converted_false = false;
  EXPECT_EQ(converted_false.num_threads(), kNoneThreads);

  const Parallelism converted_true = true;
  EXPECT_EQ(converted_true.num_threads(), kMaxThreads);

  Parallelism assigned_parallelism;
  EXPECT_EQ(assigned_parallelism.num_threads(), kNoneThreads);
  assigned_parallelism = true;
  EXPECT_EQ(assigned_parallelism.num_threads(), kMaxThreads);
  assigned_parallelism = false;
  EXPECT_EQ(assigned_parallelism.num_threads(), kNoneThreads);
}

}  // namespace
}  // namespace planning
}  // namespace anzu
