#include "drake/common/default_value.h"

#include "gtest/gtest.h"

namespace drake {
namespace {

int ForceInt(int value) {
  return value;
}

GTEST_TEST(DefaultValueTest, Constructor) {
  EXPECT_EQ(default_value<int>(), 0);
  EXPECT_EQ(default_value<int>(1), 1);
}

GTEST_TEST(DefaultValueTest, Access) {
  default_value<int> x;
  x = 1;
  EXPECT_EQ(x, 1);
  EXPECT_EQ(ForceInt(x), 1);

  const default_value<int>& x_cref = x;
  EXPECT_EQ(x_cref, 1);
  EXPECT_EQ(ForceInt(x_cref), 1);

  int& x_value_ref = x;
  EXPECT_EQ(x_value_ref, 1);
  EXPECT_EQ(ForceInt(x_value_ref), 1);

  x = 2;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(ForceInt(x), 2);
  EXPECT_EQ(x_cref, 2);
  EXPECT_EQ(ForceInt(x_cref), 2);
  EXPECT_EQ(x_value_ref, 2);
  EXPECT_EQ(ForceInt(x_value_ref), 2);
}

GTEST_TEST(DefaultValueTest, Copy) {
  default_value<int> x{1};
  EXPECT_EQ(x, 1);

  default_value<int> y{x};
  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 1);

  x = 2;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 1);

  const default_value<int>& x_cref = x;
  default_value<int> z{x_cref};
  x = 3;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);

  default_value<int> w{22};
  w = x;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(w, 3);
  w = 4;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(w, 4);
}

// We need to indirect self-move-assign through this function; doing it
// directly in the test code generates a compiler warning.
void MoveAssign(default_value<int>* target, default_value<int>* donor) {
  *target = std::move(*donor);
}

GTEST_TEST(DefaultValueTest, Move) {
  default_value<int> x{1};
  EXPECT_EQ(x, 1);

  default_value<int> y{std::move(x)};
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  x = 2;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 1);

  default_value<int> z{std::move(x)};
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);

  default_value<int> w{22};
  x = 3;
  w = std::move(x);
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);
  EXPECT_EQ(w, 3);

  MoveAssign(&w, &w);
  EXPECT_EQ(w, 3);
}

}  // namespace
}  // namespace drake
