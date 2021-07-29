#include "drake/common/reset_after_move.h"

#include <type_traits>

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
  EXPECT_EQ(reset_after_move<int>(), 0);
  EXPECT_EQ(reset_after_move<int>(1), 1);
}

GTEST_TEST(DefaultValueTest, Access) {
  // Assignment from a RHS of int (versus reset_after_move<int>).
  reset_after_move<int> x;
  x = 1;

  // Conversion operator, non-const value.
  EXPECT_EQ(x, 1);
  EXPECT_EQ(ForceInt(x), 1);

  // Conversion operator, const.
  const reset_after_move<int>& x_cref = x;
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

// For the next test.
struct Thing {
  int i{};
};

// Check that the dereferencing operators *ptr and ptr-> work for pointer types.
GTEST_TEST(DefaultValueTest, Pointers) {
  int i = 5;
  reset_after_move<int*> i_ptr{&i};
  reset_after_move<const int*> i_cptr{&i};

  EXPECT_EQ(*i_ptr, 5);
  EXPECT_EQ(*i_cptr, 5);
  *i_ptr = 6;
  EXPECT_EQ(*i_ptr, 6);
  EXPECT_EQ(*i_cptr, 6);

  reset_after_move<int*> i_ptr2(std::move(i_ptr));
  reset_after_move<const int*> i_cptr2(std::move(i_cptr));
  EXPECT_EQ(i_ptr2, &i);
  EXPECT_EQ(i_cptr2, &i);
  EXPECT_EQ(i_ptr, nullptr);
  EXPECT_EQ(i_cptr, nullptr);

  Thing thing;
  reset_after_move<Thing*> thing_ptr{&thing};
  reset_after_move<const Thing*> thing_cptr{&thing};

  // Make sure there's no cloning happening.
  EXPECT_EQ(&*thing_ptr, &thing);

  thing_ptr->i = 10;
  EXPECT_EQ(thing_ptr->i, 10);
  EXPECT_EQ((*thing_ptr).i, 10);
}

GTEST_TEST(DefaultValueTest, Copy) {
  reset_after_move<int> x{1};
  EXPECT_EQ(x, 1);

  // Copy-construction (from non-const reference) and lack of aliasing.
  reset_after_move<int> y{x};
  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 1);
  x = 2;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 1);

  // Copy-construction (from const reference) and lack of aliasing.
  const reset_after_move<int>& x_cref = x;
  reset_after_move<int> z{x_cref};
  x = 3;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);

  // Copy-assignment and lack of aliasing.
  reset_after_move<int> w{22};
  w = x;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(w, 3);
  w = 4;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(w, 4);
}

// We need to indirect self-move-assign through this function; doing it
// directly in the test code generates a compiler warning.
void MoveAssign(reset_after_move<int>* target, reset_after_move<int>* donor) {
  *target = std::move(*donor);
}

GTEST_TEST(DefaultValueTest, Move) {
  reset_after_move<int> x{1};
  EXPECT_EQ(x, 1);

  // Move-construction and lack of aliasing.
  reset_after_move<int> y{std::move(x)};
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  x = 2;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 1);

  // Second move-construction and lack of aliasing.
  reset_after_move<int> z{std::move(x)};
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);

  // Move-assignment and lack of aliasing.
  reset_after_move<int> w{22};
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

// Make sure that our wrapper is sufficiently nothrow, e.g., so that it can
// be moved (not copied) when an STL container is resized.
GTEST_TEST(DefaultValueTest, Nothrow) {
  // Default constructor.
  EXPECT_TRUE(noexcept(reset_after_move<int>()));

  // Initialize-from-lvalue constructor.
  int i{};
  EXPECT_TRUE(noexcept(reset_after_move<int>{i}));

  // Initialize-from-rvalue constructor.
  EXPECT_TRUE(noexcept(reset_after_move<int>{5}));

  // Copy constructor & assignment (source must be lvalue).
  reset_after_move<int> r_int, r_int2;
  EXPECT_TRUE(noexcept(reset_after_move<int>{r_int}));
  EXPECT_TRUE(noexcept(r_int = r_int2));

  // Move constructor & assignment.
  EXPECT_TRUE(noexcept(reset_after_move<int>{std::move(r_int)}));
  EXPECT_TRUE(noexcept(r_int = std::move(r_int2)));
}

}  // namespace
}  // namespace drake
