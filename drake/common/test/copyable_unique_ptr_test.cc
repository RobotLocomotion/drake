#include "drake/common/copyable_unique_ptr.h"

#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace  {

using std::make_unique;
using std::unique_ptr;

//--------- A Collection of Dummy Classes for test

template <typename T>
using cup = copyable_unique_ptr<T>;

// Indicator of where a particular instance came from.
enum class Origin {
  CONSTRUCT,
  COPY,
  CLONE
};

// A fully copyable class (has both copy constructor and Clone). Also used to
// test inheritance of Clone methods.
// Copyable
struct Base {
  Base(int v, Origin org=Origin::CONSTRUCT) : origin(org), value(v) {}

  Base(const Base& b) : origin(Origin::COPY), value(b.value) {}
  unique_ptr<Base> Clone() const {
    return make_unique<Base>(value, Origin::CLONE);
  }
  Origin origin;
  int value;
};

//--------- Tests --------------------------

// Confirms that the expected trivial case behaves as expected.
GTEST_TEST(CopyableUniquePtrTest, SimpleSuccess) {
  cup<Base> ptr( new Base(1));
  EXPECT_EQ(ptr->origin, Origin::CONSTRUCT);
  cup<Base> copy(ptr);
  EXPECT_EQ(copy->origin, Origin::CLONE);
  EXPECT_EQ(copy->value, ptr->value);
  copy->value++;
  EXPECT_NE(copy->value, ptr->value);
}

}  // namespace
}  // namespace drake
