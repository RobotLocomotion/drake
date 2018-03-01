#include "drake/common/reset_on_copy.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace {

// A function that helps force an implicit conversion operator to int; without
// it, EXPECT_EQ is underspecified whether we are unwrapping the first argument
// or converting the second.
int ForceInt(int value) {
  return value;
}

// The example from the documentation, fleshed out a little.
class Foo {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Foo)
  Foo() : items_{1, 2, 3}, use_count_{25} {}
  const std::vector<int>& items() const { return items_; }
  int use_count() const { return use_count_; }

 private:
  std::vector<int> items_;
  reset_on_copy<int> use_count_;
};

GTEST_TEST(ResetOnCopyTest, Constructor) {
  EXPECT_EQ(reset_on_copy<int>(), 0);
  EXPECT_EQ(reset_on_copy<int>(1), 1);
}

GTEST_TEST(ResetOnCopyTest, Nothrow) {
  // Default constructor.
  EXPECT_TRUE(noexcept(reset_on_copy<int>()));
  EXPECT_FALSE(noexcept(reset_on_copy<Foo>()));
  EXPECT_TRUE(noexcept(reset_on_copy<Foo*>()));

  // Initialize-from-lvalue constructor.
  int i{};
  Foo foo{};
  Foo* foo_ptr{};
  EXPECT_TRUE(noexcept(reset_on_copy<int>{i}));
  EXPECT_FALSE(noexcept(reset_on_copy<Foo>{foo}));
  EXPECT_TRUE(noexcept(reset_on_copy<Foo*>{foo_ptr}));

  // Initialize-from-rvalue constructor.
  EXPECT_TRUE(noexcept(reset_on_copy<int>{5}));
  EXPECT_FALSE(noexcept(reset_on_copy<Foo>{Foo{}}));
  EXPECT_TRUE(noexcept(reset_on_copy<Foo*>{&foo}));

  // Copy constructor & assignment (source must be lvalue).
  reset_on_copy<int> r_int, r_int2;
  reset_on_copy<Foo> r_foo, r_foo2;
  reset_on_copy<Foo*> r_foo_ptr, r_foo_ptr2;
  EXPECT_TRUE(noexcept(reset_on_copy<int>{r_int}));
  EXPECT_FALSE(noexcept(reset_on_copy<Foo>{r_foo}));
  EXPECT_TRUE(noexcept(reset_on_copy<Foo*>{r_foo_ptr}));
  EXPECT_TRUE(noexcept(r_int = r_int2));
  EXPECT_FALSE(noexcept(r_foo = r_foo2));
  EXPECT_TRUE(noexcept(r_foo_ptr = r_foo_ptr2));

  // Move constructor & assignment.
  EXPECT_TRUE(noexcept(reset_on_copy<int>{std::move(r_int)}));
  EXPECT_FALSE(noexcept(reset_on_copy<Foo>{std::move(r_foo)}));
  EXPECT_TRUE(noexcept(reset_on_copy<Foo*>{std::move(r_foo_ptr)}));
  EXPECT_TRUE(noexcept(r_int = std::move(r_int2)));
  EXPECT_FALSE(noexcept(r_foo = std::move(r_foo2)));
  EXPECT_TRUE(noexcept(r_foo_ptr = std::move(r_foo_ptr2)));
}

GTEST_TEST(ResetOnCopyTest, Access) {
  // Assignment from a RHS of int (versus reset_on_copy<int>).
  reset_on_copy<int> x;
  x = 1;

  // Conversion operator, non-const value.
  EXPECT_EQ(x, 1);
  EXPECT_EQ(ForceInt(x), 1);

  // Conversion operator, const.
  const reset_on_copy<int>& x_cref = x;
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

GTEST_TEST(ResetOnCopyTest, Pointers) {
  int i = 5;
  reset_on_copy<int*> i_ptr{&i};
  reset_on_copy<const int*> i_cptr{&i};

  EXPECT_EQ(*i_ptr, 5);
  EXPECT_EQ(*i_cptr, 5);
  *i_ptr = 6;
  EXPECT_EQ(*i_ptr, 6);
  EXPECT_EQ(*i_cptr, 6);

  reset_on_copy<int*> i_ptr2(i_ptr);
  reset_on_copy<const int*> i_cptr2(i_cptr);
  EXPECT_EQ(i_ptr2, nullptr);
  EXPECT_EQ(i_cptr2, nullptr);

  Foo my_foo;
  reset_on_copy<Foo*> my_foo_ptr{&my_foo};
  EXPECT_EQ(my_foo_ptr, &my_foo);

  // Make sure there's no cloning happening.
  EXPECT_EQ(&*my_foo_ptr, &my_foo);

  EXPECT_EQ(my_foo_ptr->use_count(), 25);
  EXPECT_EQ((*my_foo_ptr).use_count(), 25);

  // Moving should preserve the pointer and nuke the old one.
  reset_on_copy<Foo*> moved{std::move(my_foo_ptr)};
  EXPECT_EQ(moved, &my_foo);
  EXPECT_EQ(my_foo_ptr, nullptr);
}

// Copy construction and assignment should ignore the source and
// value-initialize the destinatation, except for self-assign which should
// do nothing.
GTEST_TEST(ResetOnCopyTest, Copy) {
  reset_on_copy<int> x{1};
  EXPECT_EQ(x, 1);

  // Copy-construction (from non-const reference) and lack of aliasing.
  reset_on_copy<int> y{x};
  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 0);

  // Copy-construction (from const reference) and lack of aliasing.
  const reset_on_copy<int>& x_cref = x;
  reset_on_copy<int> z{x_cref};
  x = 3;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(z, 0);

  // Copy-assignment and lack of aliasing.
  reset_on_copy<int> w{22};
  EXPECT_EQ(w, 22);
  w = x;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(w, 0);  // reset, not reinitialized to 22!
  w = 4;
  EXPECT_EQ(x, 3);
  EXPECT_EQ(w, 4);

  // Self copy assignment preserves value.
  w = w;
  EXPECT_EQ(w, 4);

  // Make sure the example works.
  Foo source;
  Foo copy(source);

  EXPECT_EQ(source.items().size(), 3);
  EXPECT_EQ(copy.items().size(), 3);
  for (size_t i=0; i < 3; ++i) {
    EXPECT_EQ(source.items()[i], i + 1);
    EXPECT_EQ(copy.items()[i], i + 1);
  }
  EXPECT_EQ(source.use_count(), 25);
  EXPECT_EQ(copy.use_count(), 0);
}

// We need to indirect self-move-assign through this function; doing it
// directly in the test code generates a compiler warning.
void MoveAssign(reset_on_copy<int>* target, reset_on_copy<int>* donor) {
  *target = std::move(*donor);
}

// Move construction and assignment should preserve the current value in the
// destination, and reset the source to be value-initialized. However,
// self-move assign should do nothing.
GTEST_TEST(ResetOnCopyTest, Move) {
  reset_on_copy<int> x{1};
  EXPECT_EQ(x, 1);

  // Move-construction and lack of aliasing.
  reset_on_copy<int> y{std::move(x)};
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  x = 2;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 1);

  // Second move-construction and lack of aliasing.
  reset_on_copy<int> z{std::move(x)};
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 2);

  // Move-assignment and lack of aliasing.
  reset_on_copy<int> w{22};
  EXPECT_EQ(w, 22);
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


// This test shows that use of reset_on_copy with non-scalar classes may
// have surprising behavior. Here, while Bar{} gets value-initialized, that
// doesn't mean its members get reset; they are at the mercy of Bar's default
// constructor, which here initializes i to 10.
struct Bar {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bar)
  Bar() = default;
  int i{10};
};

GTEST_TEST(ResetOnCopyTest, NonPodData) {
  reset_on_copy<Bar> reset_b;
  Bar& b = static_cast<Bar&>(reset_b);
  // Make sure that was a conversion, not a copy.
  EXPECT_EQ(static_cast<void*>(&b), static_cast<void*>(&reset_b));
  EXPECT_EQ(b.i, 10);

  b.i = 15;
  EXPECT_EQ(b.i, 15);

  reset_on_copy<Bar> reset_b_cpy(reset_b);
  Bar& b_cpy = static_cast<Bar&>(reset_b_cpy);
  EXPECT_EQ(static_cast<void*>(&b_cpy), static_cast<void*>(&reset_b_cpy));
  EXPECT_NE(&b_cpy, &b);

  // Note: the *contained* numerical values are value-initialized to the
  // declared default.
  EXPECT_EQ(b_cpy.i, 10);
}

// std::vector when resizing has to move elements from the old memory to
// the new memory. If there is a nothrow move constructor available, it will
// use that. Otherwise it uses the copy constructor. That behavior should be
// preserved through reset_on_copy.

// This struct has a nothrow move constructor.
struct CanMove {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CanMove)
  explicit CanMove(int val) : i(val) {}
  CanMove() = default;
  reset_on_copy<int> i;
};

// This struct uses its copy constructor to perform move construction.
struct NoMove {
  // Declaring just the copy constructor suppresses the default move
  // constructor so that copy will be used instead.
  NoMove(const NoMove&) = default;
  explicit NoMove(int val) : i(val) {}
  NoMove() = default;
  reset_on_copy<int> i;
};

GTEST_TEST(ResetOnCopyTest, StdVectorWorks) {
  std::vector<reset_on_copy<CanMove>> v;
  v.emplace_back(CanMove(1)); v.emplace_back(CanMove(2));
  EXPECT_EQ(v[0]->i, 1);
  EXPECT_EQ(v[1]->i, 2);
  const size_t old_v_capacity = v.capacity();
  v.resize(100);
  EXPECT_GT(v.capacity(), old_v_capacity);  // Make sure it grew.
  EXPECT_EQ(v[0]->i, 1);  // Moved, so preserved.
  EXPECT_EQ(v[1]->i, 2);
  EXPECT_EQ(v[99]->i, 0);  // Default constructed.

  std::vector<reset_on_copy<NoMove>> vn;
  vn.emplace_back(NoMove(1)); vn.emplace_back(NoMove(2));
  EXPECT_EQ(vn[0]->i, 0);  // Used copy constructor.
  EXPECT_EQ(vn[1]->i, 0);
  vn[0]->i = 1;
  vn[1]->i = 2;
  EXPECT_EQ(vn[0]->i, 1);
  EXPECT_EQ(vn[1]->i, 2);
  const size_t old_vn_capacity = vn.capacity();
  vn.resize(100);
  EXPECT_GT(vn.capacity(), old_vn_capacity);  // Make sure it grew.
  EXPECT_EQ(vn[0]->i, 0);  // Copied, so reset.
  EXPECT_EQ(vn[1]->i, 0);
  EXPECT_EQ(vn[99]->i, 0);  // Default constructed.
}

}  // namespace
}  // namespace drake
