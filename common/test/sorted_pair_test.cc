#include "drake/common/sorted_pair.h"

#include "gtest/gtest.h"

namespace drake {
namespace {

// Verifies behavior of the default constructor.
GTEST_TEST(SortedPair, Default) {
  SortedPair<int> x;
  EXPECT_EQ(x.first, 0);
  EXPECT_EQ(x.second, 0);

  SortedPair<double> y;
  EXPECT_EQ(y.first, 0.0);
  EXPECT_EQ(y.second, 0.0);
}

// Verifies sorting occurs.
GTEST_TEST(SortedPair, Values) {
  SortedPair<int> x(3, 2);
  EXPECT_EQ(x.first, 2);
  EXPECT_EQ(x.second, 3);
}

// Verifies that the templated copy constructor work.
GTEST_TEST(SortedPair, Casting) {
  SortedPair<double> x = SortedPair<int>(3, 2);
  EXPECT_EQ(x.first, 2.0);
  EXPECT_EQ(x.second, 3.0);
}

// Checks the assignment operator. 
GTEST_TEST(SortedPair, Assignment) {
  SortedPair<int> x;
  SortedPair<int> y(3, 2);
  EXPECT_EQ(&(x = y), &x);
  EXPECT_EQ(x.first, 2.0);
  EXPECT_EQ(x.second, 3.0);
}

// Checks the equality operator.
GTEST_TEST(SortedPair, Equality) {
  SortedPair<int> x(1,2), y(2, 1);
  EXPECT_EQ(x, y);
}

// Checks the comparison operators.
GTEST_TEST(SortedPair, Comparison) {
  SortedPair<int> x(1, 2), y(2, 2);
  EXPECT_FALSE(x < x);
  EXPECT_FALSE(x > x);
  EXPECT_TRUE(x <= x);
  EXPECT_TRUE(x >= x);
  EXPECT_TRUE(x < y);
}

// Tests the MakeSortedPair operator.
GTEST_TEST(SortedPair, MakeSortedPair) {
  EXPECT_EQ(SortedPair<int>(1, 2), MakeSortedPair(1, 2)); 
}

}  // namespace
}  // namespace drake
