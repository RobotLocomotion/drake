#include "drake/common/parallelism.h"

#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace internal {
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

// Tests all variety of environment variable string parsing.
GTEST_TEST(ParallelismTest, ParsingLogicTest) {
  // When string parsing fails, this is what we get back.
  const int fallback = std::thread::hardware_concurrency();
  EXPECT_EQ(ConfigureMaxNumThreads(nullptr, nullptr), fallback);

  // A table of test cases for string value => configured max.
  std::vector<std::pair<const char*, int>> tests = {{
      // Happy values.
      {"1", 1},
      {"2", 2},
      // Empty values.
      {"", fallback},
      {" ", fallback},
      // Garbage values.
      {"4,4,4", fallback},
      {"1.0", fallback},
      {"a", fallback},
      {"a1", fallback},
      {"0x01", fallback},
      {"☃", fallback},
      {"1☃", fallback},
      // Out-of-bounds values.
      {"-100", fallback},
      {"-1", fallback},
      {"0", fallback},
      {"999999999", fallback},
  }};
  for (const auto& [value, expected_max] : tests) {
    EXPECT_EQ(ConfigureMaxNumThreads(value, nullptr), expected_max) << value;
    EXPECT_EQ(ConfigureMaxNumThreads(nullptr, value), expected_max) << value;
  }

  // When both environment variables are set, the DRAKE one always wins even if
  // it's unparseable.
  EXPECT_EQ(ConfigureMaxNumThreads("2", "1"), 2);
  EXPECT_EQ(ConfigureMaxNumThreads("", "1"), fallback);
  EXPECT_EQ(ConfigureMaxNumThreads("-1", "1"), fallback);
}

}  // namespace
}  // namespace internal
}  // namespace drake
