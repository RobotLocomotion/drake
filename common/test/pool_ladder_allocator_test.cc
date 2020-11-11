#include "drake/common/pool_ladder_allocator.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

namespace {

GTEST_TEST(PoolLadderAllocatorTest, Pool) {
  using Pool3 = drake::internal::pool_ladder_allocator::Pool<3>;
  Pool3 pool;
  EXPECT_EQ(pool.size(), 0);
  double* p = pool.Get();
  EXPECT_EQ(pool.size(), Pool3::kPrePopulation - 1);

  // Probe the edges of the object for valgrind's benefit.
  p[0] = 7;
  EXPECT_EQ(p[0], 7);
  p[Pool3::kSize - 1] = 10;
  EXPECT_EQ(p[Pool3::kSize - 1], 10);

  pool.Put(p);
  EXPECT_EQ(pool.size(), Pool3::kPrePopulation);
  // Since gets and puts were balanced above, this test should not leak memory.
}

GTEST_TEST(PoolLadderAllocatorTest, CeilLog2) {
  using drake::internal::pool_ladder_allocator::ceil_log2;

  EXPECT_EQ(ceil_log2(2), 1);
  EXPECT_EQ(ceil_log2(3), 2);
  EXPECT_EQ(ceil_log2(4), 2);
  EXPECT_EQ(ceil_log2(5), 3);

  EXPECT_EQ(ceil_log2(31), 5);
  EXPECT_EQ(ceil_log2(32), 5);
  EXPECT_EQ(ceil_log2(33), 6);

  EXPECT_EQ(ceil_log2(1 << 31), 31);
}


GTEST_TEST(PoolLadderAllocatorTest, Ladder) {
  using drake::internal::pool_ladder_allocator::Ladder;
  Ladder ladder;

  for (int power = 1; power <= Ladder::kMaxPower; power++) {
    SCOPED_TRACE(fmt::format("power = {}", power));
    // Use implementation parameters to pick the largest usable size from
    // each internal pool.
    const int usable_size = (1 << power) - Ladder::kOverhead;

    double* p = ladder.Get(usable_size);

    // Probe the edges of the object for valgrind's benefit. Note that these
    // probes overlap for a usable-size=1 object.
    p[0] = 7;
    EXPECT_EQ(p[0], 7);
    p[usable_size - 1] = 10;
    EXPECT_EQ(p[usable_size - 1], 10);

    ladder.Put(p);
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
