#include "drake/common/double_overloads.h"

#include <gtest/gtest.h>

#include "drake/common/cond.h"

namespace drake {
namespace common {
namespace {

GTEST_TEST(Cond, DoubleIfThenElse) {
  const double x = 10.0;
  const double y = 5.0;
  EXPECT_DOUBLE_EQ(if_then_else(x > y, x, y), x);
  EXPECT_DOUBLE_EQ(if_then_else(x < y, x, y), y);
}

GTEST_TEST(Cond, Cond0) {
  const double x = 10.0;
  // clang-format off
  const double cond_result{cond(true, x + 20.0,
                                false, x + 5.0,
                                true, x + 3.0,
                                0.0)};
  // clang-format on
  EXPECT_DOUBLE_EQ(cond_result, x + 20.0);
}

GTEST_TEST(Cond, Cond1) {
  const double x = 10.0;
  // clang-format off
  const double cond_result{cond(x >= 10.0, x + 20.0,
                                x >= 5.0, x + 5.0,
                                x >= 3.0, x + 3.0,
                                0.0)};
  // clang-format on
  EXPECT_DOUBLE_EQ(cond_result, x + 20.0);
}

GTEST_TEST(Cond, Cond2) {
  const double x = 8.0;
  // clang-format off
  const double cond_result{cond(x >= 10.0, x + 20.0,
                                x >= 5.0, x + 5.0,
                                x >= 3.0, x + 3.0,
                                0.0)};
  // clang-format on
  EXPECT_DOUBLE_EQ(cond_result, x + 5.0);
}

GTEST_TEST(Cond, Cond3) {
  const double x = 3.0;
  // clang-format off
  const double cond_result{cond(x >= 10.0, x + 20.0,
                                x >= 5.0, x + 5.0,
                                x >= 3.0, x + 3.0,
                                0.0)};
  // clang-format on
  EXPECT_DOUBLE_EQ(cond_result, x + 3.0);
}

GTEST_TEST(Cond, Cond4) {
  const double x = -10.0;
  // clang-format off
  const double cond_result{cond(x >= 10.0, x + 20.0,
                                x >= 5.0, x + 5.0,
                                x >= 3.0, x + 3.0,
                                0.0)};
  // clang-format on
  EXPECT_DOUBLE_EQ(cond_result, 0.0);
}
}  // namespace
}  // namespace common
}  // namespace drake
