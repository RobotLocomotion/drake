// Disable the include checking intended for clients; this module is internal
// to the autodiff system.
#define DRAKE_COMMON_AUTODIFF_HEADER
#include "drake/common/eigen_dense_storage_pools.h"
#undef DRAKE_COMMON_AUTODIFF_HEADER

#include <thread>

#include <fmt/format.h>
#include <gtest/gtest.h>

namespace Eigen {

using DenseStoragePools = DenseStorage<
  /* T = */ double,
  /* Size = */ drake::internal::kMaxRowsAtCompileTimeThatTriggersPools,
  /* _Rows = */ Dynamic,
  /* _Cols = */ 1,
  /* _Options = */ 0>;


namespace {

GTEST_TEST(EigenDenseStoragePoolsTest, Basics) {
  DenseStoragePools empty;
  EXPECT_EQ(empty.cols(), 1);
  EXPECT_EQ(empty.rows(), 0);
  EXPECT_EQ(empty.data(), nullptr);

  const DenseStoragePools emptyc;
  EXPECT_EQ(emptyc.rows(), 0);
  EXPECT_EQ(emptyc.data(), nullptr);

  DenseStoragePools some(10, 10, 1);
  EXPECT_EQ(some.rows(), 10);
  EXPECT_NE(some.data(), nullptr);

  const DenseStoragePools empty_mirror(empty);
  EXPECT_EQ(empty_mirror.rows(), 0);
  EXPECT_EQ(empty_mirror.data(), nullptr);

  const DenseStoragePools some_mirror(some);
  EXPECT_EQ(some_mirror.rows(), 10);
  EXPECT_NE(some_mirror.data(), nullptr);

  DenseStoragePools empty_assigned(10, 10, 1);
  empty_assigned = empty;
  EXPECT_EQ(empty_assigned.rows(), 0);
  EXPECT_EQ(empty_assigned.data(), nullptr);

  DenseStoragePools some_assigned;
  some_assigned = some;
  EXPECT_EQ(some_assigned.rows(), 10);
  EXPECT_NE(some_assigned.data(), nullptr);

  // If this were const, the construction would be by copy rather than move.
  DenseStoragePools empty_movee0;
  const DenseStoragePools empty_moved(std::move(empty_movee0));
  EXPECT_EQ(empty_moved.rows(), 0);
  EXPECT_EQ(empty_moved.data(), nullptr);

  // If this were const, the construction would be by copy rather than move.
  DenseStoragePools some_movee0(10, 10, 1);
  const DenseStoragePools some_moved(std::move(some_movee0));
  EXPECT_EQ(some_moved.rows(), 10);
  EXPECT_NE(some_moved.data(), nullptr);

  // If this were const, the assignment would be by copy rather than move.
  DenseStoragePools empty_movee1;
  EXPECT_EQ(empty_movee1.rows(), 0);
  EXPECT_EQ(empty_movee1.data(), nullptr);
  DenseStoragePools empty_move_assigned1(10, 10, 1);
  empty_move_assigned1 = std::move(empty_movee1);
  EXPECT_EQ(empty_move_assigned1.rows(), 0);
  EXPECT_EQ(empty_move_assigned1.data(), nullptr);

  // If this were const, the construction would be by copy rather than move.
  DenseStoragePools some_movee1(10, 10, 1);
  DenseStoragePools some_move_assigned;
  some_move_assigned = std::move(some_movee1);;
  EXPECT_EQ(some_move_assigned.rows(), 10);
  EXPECT_NE(some_move_assigned.data(), nullptr);

  empty.swap(some);
  EXPECT_EQ(empty.rows(), 10);
  EXPECT_NE(empty.data(), nullptr);
  EXPECT_EQ(some.rows(), 0);
  EXPECT_EQ(some.data(), nullptr);
  empty.swap(some);
  EXPECT_EQ(empty.rows(), 0);
  EXPECT_EQ(empty.data(), nullptr);
  EXPECT_EQ(some.rows(), 10);
  EXPECT_NE(some.data(), nullptr);
}

GTEST_TEST(EigenDenseStoragePoolsTest, ConservativeResize) {
  DenseStoragePools victim;
  victim.conservativeResize(10, 10, 1);
  EXPECT_EQ(victim.rows(), 10);
  EXPECT_NE(victim.data(), nullptr);

  // Set a pattern into the data.
  for (int k = 0; k < victim.rows(); k++) {
    victim.data()[k] = k + 10;
  }
  // A no-op resize.
  victim.conservativeResize(10, 10, 1);
  for (int k = 0; k < victim.rows(); k++) {
    SCOPED_TRACE(fmt::format("k = {}", k));
    EXPECT_EQ(victim.data()[k], k + 10);
  }
  // A shrinking resize.
  victim.conservativeResize(7, 7, 1);
  for (int k = 0; k < victim.rows(); k++) {
    SCOPED_TRACE(fmt::format("k = {}", k));
    EXPECT_EQ(victim.data()[k], k + 10);
  }
  // An emptying resize.
  victim.conservativeResize(0, 0, 1);
  EXPECT_EQ(victim.data(), nullptr);
}

GTEST_TEST(EigenDenseStoragePoolsTest, ThreadCleanup) {
  using drake::internal::pool_ladder_allocator::Ladder;
  int count_before = Ladder::instance_count();

  int count_in_thread{};
  std::thread thread1([&count_in_thread]() {
      // Force pool storage to exist in the thread by creating a
      // non-empty DenseStoragePools instance.
      DenseStoragePools(10, 10, 1);

      count_in_thread = Ladder::instance_count();
      // Scope end releases the claimed storage back to pool.
      // Thread end destroys the per-thread pool.
    });
  thread1.join();

  int count_after = Ladder::instance_count();

  EXPECT_EQ(count_after, count_before);
  EXPECT_TRUE(count_in_thread > count_after);
}

}  // namespace
}  // namespace Eigen
