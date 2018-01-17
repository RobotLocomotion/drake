#include "drake/common/is_less_than_comparable.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

// Dummy operator< for checking is_less_than_comparable().
struct Z {};
bool operator<(const Z&, const Z&) { return true; }

// Verifies that this class is comparable using the less than operator.
GTEST_TEST(LessThanComparable, RunTime) {
  // Necessary to work around a Clang error.
  const Z z;
  operator<(z, z);

  struct X {};
  struct Y {
    bool operator<(const Y&) const { return true; }
  };
  EXPECT_TRUE(drake::is_less_than_comparable<int>::value);
  EXPECT_TRUE(drake::is_less_than_comparable<Y>::value);
  EXPECT_FALSE(drake::is_less_than_comparable<X>::value);
  EXPECT_TRUE(drake::is_less_than_comparable<Z>::value);
}

}  // namespace
}  // namespace drake
