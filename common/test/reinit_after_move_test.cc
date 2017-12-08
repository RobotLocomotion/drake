#include "drake/common/reinit_after_move.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

// A function that helps force an implicit conversion operator to int; without
// it, EXPECT_EQ is underspecified whether we are unwrapping the first argument
// or converting the second.
int ForceInt(int value) {
  return value;
}

GTEST_TEST(DefaultValueTest, Constructor) {
  EXPECT_EQ(reinit_after_move<int>(), 0);
  EXPECT_EQ(reinit_after_move<int>(1), 1);
}

GTEST_TEST(DefaultValueTest, Access) {
  // Assignment from a RHS of int (versus reinit_after_move<int>).
  reinit_after_move<int> x;
  x = 1;

  // Conversion operator, non-const value.
  EXPECT_EQ(x, 1);
  EXPECT_EQ(ForceInt(x), 1);

  // Conversion operator, const.
  const reinit_after_move<int>& x_cref = x;
  EXPECT_EQ(x_cref, 1);
  EXPECT_EQ(ForceInt(x_cref), 1);

  // Conversion operator, non-const reference.
  int& x_value_ref = x;
  EXPECT_EQ(x_value_ref, 1);
  EXPECT_EQ(ForceInt(x_value_ref), 1);

  // All values work okay.
  x = 2;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(ForceInt(x), 2);
  EXPECT_EQ(x_cref, 2);
  EXPECT_EQ(ForceInt(x_cref), 2);
  EXPECT_EQ(x_value_ref, 2);
  EXPECT_EQ(ForceInt(x_value_ref), 2);
}

GTEST_TEST(DefaultValueTest, Copy) {
  reinit_after_move<int> x{1};
  EXPECT_EQ(x, 1);

  // Copy-construction (from non-const reference) and lack of aliasing.
  reinit_after_move<int> y{x};
  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 1);
  x = 2;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 1);

  // Copy-construction (from const reference) and lack of aliasing.
  const reinit_after_move<int>& x_cref = x;
  reinit_after_move<int> z{x_cref};
  x = 3;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);

  // Copy-assignment and lack of aliasing.
  reinit_after_move<int> w{22};
  w = x;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(w, 3);
  w = 4;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(w, 4);
}

// We need to indirect self-move-assign through this function; doing it
// directly in the test code generates a compiler warning.
void MoveAssign(reinit_after_move<int>* target, reinit_after_move<int>* donor) {
  *target = std::move(*donor);
}

GTEST_TEST(DefaultValueTest, Move) {
  reinit_after_move<int> x{1};
  EXPECT_EQ(x, 1);

  // Move-construction and lack of aliasing.
  reinit_after_move<int> y{std::move(x)};
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  x = 2;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 1);

  // Second move-construction and lack of aliasing.
  reinit_after_move<int> z{std::move(x)};
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);

  // Move-assignment and lack of aliasing.
  reinit_after_move<int> w{22};
  x = 3;
  w = std::move(x);
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);
  EXPECT_EQ(w, 3);
  x = 4;
  EXPECT_EQ(x, 4);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);
  EXPECT_EQ(w, 3);

  // Self-assignment during move.
  MoveAssign(&w, &w);
  EXPECT_EQ(w, 3);
}

}  // namespace
}  // namespace drake
