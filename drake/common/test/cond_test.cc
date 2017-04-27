#include "drake/common/cond.h"

#include <gtest/gtest.h>

#include "drake/common/double_overloads.h"

namespace drake {
namespace {

// Mostly, this just checks for compilation failures.
GTEST_TEST(CondTest, BasicTest) {
  const double x = cond(true, 1, 2);
  const double y = cond(false, 3, 4);
  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 4);
}

}  // namespace
}  // namespace drake
