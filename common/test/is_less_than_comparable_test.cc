#include "drake/common/is_less_than_comparable.h"

#include "gtest/gtest.h"

namespace drake {
namespace {

// Verifies that this class is comparable using the less than operator.
GTEST_TEST(LessThanComparable, RunTime) {
  class X {};
  EXPECT_TRUE(drake::is_less_than_comparable<int>::value);
  EXPECT_FALSE(drake::is_less_than_comparable<X>::value);
}

}  // namespace
}  // namespace drake
