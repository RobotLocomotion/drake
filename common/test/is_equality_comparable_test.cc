#include "drake/common/is_equality_comparable.h"

#include "gtest/gtest.h"

namespace drake {
namespace {

// Dummy operator== for checking is_equality_comparable().
struct Z {};
bool operator==(const Z&, const Z&) { return true; }

// Verifies that this class is equality comparable.
GTEST_TEST(EqualityComparable, RunTime) {
  struct X {};
  struct Y {
    bool operator==(const Y&) const { return true; }
  };
  EXPECT_TRUE(drake::is_equality_comparable<int>::value);
  EXPECT_FALSE(drake::is_equality_comparable<X>::value);
  EXPECT_TRUE(drake::is_equality_comparable<Y>::value);
  EXPECT_TRUE(drake::is_equality_comparable<Z>::value);
}

}  // namespace
}  // namespace drake
