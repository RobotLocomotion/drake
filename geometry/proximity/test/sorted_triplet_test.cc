#include "drake/geometry/proximity/sorted_triplet.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Verifies behavior of the default constructor.
GTEST_TEST(SortedTriplet, Default) {
  SortedTriplet<int> x;
  EXPECT_EQ(x.first(), 0);
  EXPECT_EQ(x.second(), 0);
  EXPECT_EQ(x.third(), 0);

  SortedTriplet<double> y;
  EXPECT_EQ(y.first(), 0.0);
  EXPECT_EQ(y.second(), 0.0);
  EXPECT_EQ(y.third(), 0.0);
}

// Verifies sorting occurs.
GTEST_TEST(SortedTriplet, Values) {
  SortedTriplet<int> x(3, 4, 2);
  EXPECT_EQ(x.first(), 2);
  EXPECT_EQ(x.second(), 3);
  EXPECT_EQ(x.third(), 4);
}

// Checks the equality operator.
GTEST_TEST(SortedTriplet, Equality) {
  SortedTriplet<int> x(1, 2, 3), y(3, 2, 1);
  EXPECT_EQ(x, y);
}

// Checks the operator<.
GTEST_TEST(SortedTriplet, Comparison) {
  SortedTriplet<int> x(1, 2, 3), y(2, 2, 2);
  EXPECT_FALSE(x < x);
  EXPECT_TRUE(x < y);
}

// Uses as a key in a map.
GTEST_TEST(SortedTriplet, Map) {
  std::map<SortedTriplet<int>, char> my_map;
  SortedTriplet<int> triplet(1, 2, 3);
  SortedTriplet<int> same_triplet(2, 1, 3);
  my_map.emplace(triplet, 'A');
  my_map.emplace(same_triplet, 'B');
  EXPECT_EQ(1, my_map.size());
  EXPECT_EQ('A', my_map.at(same_triplet));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
