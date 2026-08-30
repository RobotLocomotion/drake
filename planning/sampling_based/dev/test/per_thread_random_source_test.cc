#include "drake/planning/sampling_based/dev/per_thread_random_source.h"

#include <unordered_set>

#include <common_robotics_utilities/parallelism.hpp>
#include <gtest/gtest.h>

#include "drake/common/drake_throw.h"
#include "drake/common/parallelism.h"

namespace drake {
namespace planning {
namespace {
using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;

constexpr uint64_t kPrngSeed = 42;
constexpr int kNumDraws = 10;

void Discard(PerThreadRandomSource* source, int num_discards,
             int thread_number) {
  DRAKE_THROW_UNLESS(source != nullptr);
  DRAKE_THROW_UNLESS(num_discards > 0);

  for (int discard = 0; discard < num_discards; ++discard) {
    source->DrawRaw(thread_number);
  }
}

GTEST_TEST(PerThreadRandomSourceTest, MultipleGeneratorsTest) {
  PerThreadRandomSource source(kPrngSeed, Parallelism::Max());

  const int num_threads = Parallelism::Max().num_threads();

  std::vector<std::vector<uint64_t>> thread_draws(num_threads);

  const auto draw_per_thread = [&](const int thread_num, const int64_t) {
    for (int draw = 0; draw < kNumDraws; ++draw) {
      thread_draws.at(thread_num).emplace_back(source.DrawRaw(thread_num));
    }
  };

  StaticParallelForIndexLoop(DegreeOfParallelism(num_threads), 0, num_threads,
                             draw_per_thread,
                             ParallelForBackend::BEST_AVAILABLE);

  std::unordered_set<uint64_t> all_draws;
  for (const auto& draws : thread_draws) {
    all_draws.insert(draws.begin(), draws.end());
  }

  // We expect draws from different threads to be different.
  EXPECT_EQ(all_draws.size(), static_cast<size_t>(num_threads * kNumDraws));
}

GTEST_TEST(PerThreadRandomSourceTest, ReseedTest) {
  constexpr int thread_number = 1;
  PerThreadRandomSource initial_source(kPrngSeed, Parallelism::Max());

  Discard(&initial_source, kNumDraws, thread_number);

  const auto seeds = initial_source.SnapshotGeneratorSeeds();
  const uint64_t draw1 = initial_source.DrawRaw(thread_number);

  Discard(&initial_source, kNumDraws, thread_number);

  const uint64_t draw2 = initial_source.DrawRaw(thread_number);
  EXPECT_NE(draw1, draw2);

  initial_source.ReseedGeneratorsIndividually(seeds);
  const uint64_t draw3 = initial_source.DrawRaw(thread_number);
  EXPECT_EQ(draw1, draw3);

  PerThreadRandomSource other_source(kPrngSeed * 2, Parallelism::Max());
  const uint64_t other_draw1 = other_source.DrawRaw(thread_number);
  EXPECT_NE(draw1, other_draw1);

  other_source.ReseedGeneratorsIndividually(seeds);
  const uint64_t other_draw2 = other_source.DrawRaw(thread_number);
  EXPECT_EQ(draw1, other_draw2);

  auto malformed_seeds = seeds;
  malformed_seeds.push_back(1);
  EXPECT_THROW(other_source.ReseedGeneratorsIndividually(malformed_seeds),
               std::exception);
}

GTEST_TEST(PerThreadRandomSourceTest, CopyTest) {
  constexpr int thread_number = 1;
  PerThreadRandomSource initial_source(kPrngSeed, Parallelism::Max());
  PerThreadRandomSource copy1 = initial_source;
  PerThreadRandomSource copy2 = initial_source;

  {
    const uint64_t initial_draw = initial_source.DrawRaw(thread_number);
    const uint64_t copy1_draw = copy1.DrawRaw(thread_number);
    const uint64_t copy2_draw = copy2.DrawRaw(thread_number);
    EXPECT_EQ(initial_draw, copy1_draw);
    EXPECT_EQ(initial_draw, copy2_draw);
  }

  Discard(&copy1, kNumDraws, thread_number);

  {
    const uint64_t initial_draw = initial_source.DrawRaw(thread_number);
    const uint64_t copy1_draw = copy1.DrawRaw(thread_number);
    const uint64_t copy2_draw = copy2.DrawRaw(thread_number);
    EXPECT_NE(initial_draw, copy1_draw);
    EXPECT_EQ(initial_draw, copy2_draw);
  }

  Discard(&copy2, kNumDraws, thread_number);

  {
    const uint64_t initial_draw = initial_source.DrawRaw(thread_number);
    const uint64_t copy1_draw = copy1.DrawRaw(thread_number);
    const uint64_t copy2_draw = copy2.DrawRaw(thread_number);
    EXPECT_NE(initial_draw, copy1_draw);
    EXPECT_NE(initial_draw, copy2_draw);
    EXPECT_EQ(copy1_draw, copy2_draw);
  }
}
}  // namespace
}  // namespace planning
}  // namespace drake
