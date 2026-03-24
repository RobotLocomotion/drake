#include "drake/geometry/lazy_shared.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(LazySharedTest, LifecycleNull) {
  LazyShared<int> empty;
  LazyShared<int> copied(empty);
  LazyShared<int> copy_assigned;
  copy_assigned = empty;
  LazyShared<int> moved(LazyShared<int>{});
  LazyShared<int> move_assigned;
  move_assigned = LazyShared<int>{};

  // None of the objects are related to each other.
  int counter = 0;
  auto factory = [&]() {
    return std::make_unique<int>(++counter);
  };
  EXPECT_EQ(empty.GetOrMake(factory), 1);
  EXPECT_EQ(empty.GetOrMake(factory), 1);
  EXPECT_EQ(copied.GetOrMake(factory), 2);
  EXPECT_EQ(copied.GetOrMake(factory), 2);
  EXPECT_EQ(copy_assigned.GetOrMake(factory), 3);
  EXPECT_EQ(copy_assigned.GetOrMake(factory), 3);
  EXPECT_EQ(moved.GetOrMake(factory), 4);
  EXPECT_EQ(moved.GetOrMake(factory), 4);
  EXPECT_EQ(move_assigned.GetOrMake(factory), 5);
  EXPECT_EQ(move_assigned.GetOrMake(factory), 5);
}

GTEST_TEST(LazySharedTest, LifecycleNonNull) {
  int counter = 0;
  auto factory = [&]() {
    return std::make_unique<int>(++counter);
  };

  LazyShared<int> dut;
  EXPECT_EQ(dut.GetOrMake(factory), 1);
  EXPECT_EQ(dut.GetOrMake(factory), 1);

  // Copying retains the shared object.
  LazyShared<int> copied(dut);
  EXPECT_EQ(copied.GetOrMake(factory), 1);
  EXPECT_EQ(copied.GetOrMake(factory), 1);
  EXPECT_EQ(dut.GetOrMake(factory), 1);

  // Copy-assignment retains the shared object.
  LazyShared<int> copy_assigned;
  copy_assigned = dut;
  EXPECT_EQ(copy_assigned.GetOrMake(factory), 1);
  EXPECT_EQ(copy_assigned.GetOrMake(factory), 1);
  EXPECT_EQ(copied.GetOrMake(factory), 1);
  EXPECT_EQ(dut.GetOrMake(factory), 1);

  // Moving transfers the shared object.
  LazyShared<int> moved(std::move(copied));
  EXPECT_EQ(moved.GetOrMake(factory), 1);
  EXPECT_EQ(moved.GetOrMake(factory), 1);
  EXPECT_EQ(copied.GetOrMake(factory), 2);
  EXPECT_EQ(copied.GetOrMake(factory), 2);
  EXPECT_EQ(dut.GetOrMake(factory), 1);

  // Move-assignment transfers the shared object.
  LazyShared<int> move_assigned;
  move_assigned = std::move(copy_assigned);
  EXPECT_EQ(move_assigned.GetOrMake(factory), 1);
  EXPECT_EQ(move_assigned.GetOrMake(factory), 1);
  EXPECT_EQ(copy_assigned.GetOrMake(factory), 3);
  EXPECT_EQ(copy_assigned.GetOrMake(factory), 3);
  EXPECT_EQ(dut.GetOrMake(factory), 1);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
