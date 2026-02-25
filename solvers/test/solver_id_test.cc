#include "drake/solvers/solver_id.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace solvers {
namespace {

using test::LimitMalloc;

GTEST_TEST(SolverId, Equality) {
  // An ID is equal to itself.
  // Use the EQ/NE form here to make sure gtest error reporting compiles okay.
  SolverId foo{"foo"};
  EXPECT_EQ(foo, foo);
  EXPECT_NE(foo, SolverId{"bar"});

  // Named objects are equal per their assigned IDs; same name is not enough.
  // Use the op==/op!= form here to be clear which operators are being tested.
  EXPECT_FALSE(SolverId{"x"} == SolverId{"x"});
  EXPECT_TRUE(SolverId{"x"} != SolverId{"x"});
  EXPECT_FALSE(SolverId{"a"} == SolverId{"b"});
  EXPECT_TRUE(SolverId{"a"} != SolverId{"b"});
}

GTEST_TEST(SolverId, Copy) {
  // Copies of objects are equal to each other.
  SolverId foo{"foo"};
  EXPECT_TRUE(foo == foo);
  EXPECT_FALSE(foo != foo);
  SolverId foo2{foo};
  EXPECT_TRUE(foo == foo2);
  EXPECT_FALSE(foo != foo2);
}

GTEST_TEST(SolverId, Move) {
  // Moved-from IDs become empty.
  SolverId old_bar{"bar"};
  SolverId new_bar{std::move(old_bar)};
  EXPECT_EQ(old_bar.name(), "");
  EXPECT_EQ(new_bar.name(), "bar");
  EXPECT_NE(new_bar, old_bar);
}

GTEST_TEST(SolverId, NoHeap) {
  // Solver names that are <= 15 characters do not allocate.
  LimitMalloc guard;
  SolverId foo{"123456789012345"};
  SolverId bar(foo);
}

GTEST_TEST(SolverId, WarningForVeryLongName) {
  // Solver names that are > 15 characters will warn.
  // We'll just make sure nothing crashes.
  SolverId foo{"this_solver_name_is_way_too_long"};
}

GTEST_TEST(SolverId, ToStringFmtFormatter) {
  EXPECT_EQ(fmt::to_string(SolverId{"bar"}), "bar");
}

}  // namespace
}  // namespace solvers
}  // namespace drake
