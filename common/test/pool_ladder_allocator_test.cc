#include "drake/common/pool_ladder_allocator.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

namespace {

GTEST_TEST(PoolLadderAllocatorTest, Pool) {
  using Pool3 = drake::internal::pool_ladder_allocator::Pool<3>;
  Pool3 pool;
  EXPECT_EQ(pool.size(), 0);
  double* p = pool.get();
  EXPECT_EQ(pool.size(), Pool3::kPrePopulation - 1);

  // Probe the edges of the object for valgrind's benefit.
  p[0] = 7;
  EXPECT_EQ(p[0], 7);
  p[Pool3::kSize - 1] = 10;
  EXPECT_EQ(p[Pool3::kSize - 1], 10);

  pool.put(p);
  EXPECT_EQ(pool.size(), Pool3::kPrePopulation);
  // Since gets and puts were balanced above, this test should not leak memory.
}

GTEST_TEST(PoolLadderAllocatorTest, CeilLog2) {
  using drake::internal::pool_ladder_allocator::ceil_log2;

  EXPECT_EQ(ceil_log2(0), 1);
  EXPECT_EQ(ceil_log2(1), 1);
  EXPECT_EQ(ceil_log2(2), 1);
  EXPECT_EQ(ceil_log2(3), 2);
  EXPECT_EQ(ceil_log2(4), 2);
  EXPECT_EQ(ceil_log2(5), 3);

  EXPECT_EQ(ceil_log2(31), 5);
  EXPECT_EQ(ceil_log2(32), 5);
  EXPECT_EQ(ceil_log2(33), 6);
}

GTEST_TEST(PoolLadderAllocatorTest, Ladder) {
  using drake::internal::pool_ladder_allocator::Ladder;
  Ladder ladder;

  // This loop is limited based on knowledge of the implementation.
  for (int power = 1; power <= 15; power++) {
    SCOPED_TRACE(fmt::format("power = {}", power));
    // Use knowledge of the implementation to pick the largest usable size from
    // each internal pool.
    int usable_size = (1 << power) - 1;

    double* p = ladder.get(usable_size);

    // Probe the edges of the object for valgrind's benefit.
    p[0] = 7;
    EXPECT_EQ(p[0], 7);
    p[usable_size - 1] = 10;
    EXPECT_EQ(p[usable_size - 1], 10);

    ladder.put(p);
  }
  // Since gets and puts were balanced above, this test should not leak memory.
}

GTEST_TEST(PoolLadderAllocatorTest, LadderInstanceCounts) {
  using drake::internal::pool_ladder_allocator::Ladder;
  EXPECT_EQ(Ladder::instance_count(), 0);

  {
    EXPECT_EQ(Ladder::instance_count(), 0);
    Ladder ladder;
    EXPECT_EQ(Ladder::instance_count(), 1);
  }
  EXPECT_EQ(Ladder::instance_count(), 0);

  {
    EXPECT_EQ(Ladder::instance_count(), 0);
    std::vector<Ladder> bunch(10);
    EXPECT_EQ(Ladder::instance_count(), 10);
  }
  EXPECT_EQ(Ladder::instance_count(), 0);
}

}  // namespace
