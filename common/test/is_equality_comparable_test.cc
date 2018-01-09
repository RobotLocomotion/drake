#include "drake/common/is_equality_comparable.h"

#include "gtest/gtest.h"

namespace drake {
namespace {

// Verifies that this class is equality comparable.
GTEST_TEST(EqualityComparable, RunTime) {
  class X {};
  EXPECT_TRUE(drake::is_equality_comparable<int>::value);
  EXPECT_FALSE(drake::is_equality_comparable<X>::value);
}

}  // namespace
}  // namespace drake
