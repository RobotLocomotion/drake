#include "drake/common/have_intersection.h"

#include <vector>
#include <set>
#include "gtest/gtest.h"

using std::vector;
using std::set;

namespace drake {

namespace {

// Test with std::vector specific SortedVectorsHaveIntersection.
GTEST_TEST(TestHaveIntersection, SortedVectorsHaveIntersection) {
  // In order for drake::SortedVectorsHaveIntersection to work vectors must be
  // sorted.
  // An std::vector can have non sorted repeated elements. Therefore the user
  // must ensure that the previous conditions are met.
  vector<int> set1 = vector<int>{2, 9, 11, 15, 23};
  vector<int> set2 = vector<int>{9, 11, 13};
  vector<int> set3 = vector<int>{1, 8, 13};

  // set1 intersects set2 (elements 9 and 11 in common).
  EXPECT_TRUE(SortedVectorsHaveIntersection(set1, set2));

  // set2 intersects set1 (commutative).
  EXPECT_TRUE(SortedVectorsHaveIntersection(set2, set1));

  // set1 does not intersect set3 (no elements in common).
  EXPECT_FALSE(SortedVectorsHaveIntersection(set1, set3));

  // set3 does not intersect set1 (commutative).
  EXPECT_FALSE(SortedVectorsHaveIntersection(set3, set1));

  // set2 intersects set3 (element 13 in common).
  EXPECT_TRUE(SortedVectorsHaveIntersection(set2, set3));

  // set3 intersects set2 (commutative).
  EXPECT_TRUE(SortedVectorsHaveIntersection(set3, set2));

  // Empty sets.
  vector<int> set4{};
  vector<int> set5{};
  // Two empty sets.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set4, set5));

  // First non-empty against second empty.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set1, set4));

  // First empty against second non-empty.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set4, set1));

  // Disjoint sets.
  vector<int> set6 = vector<int>{30, 35, 50};
  // set1 entries are all smaller than set6 first entry.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set1, set6));

  // Tests if the above is commutative.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set6, set1));

  // Sets with repeated entries
  vector<int> set7 = vector<int>{2, 9, 9, 11, 15, 15, 23};
  vector<int> set8 = vector<int>{9, 11, 11, 13};
  vector<int> set9 = vector<int>{1, 1, 8, 13};

  // set7 intersects set8 (elements 9 and 11 in common).
  EXPECT_TRUE(SortedVectorsHaveIntersection(set7, set8));
  // set7 does not intersect set9 (no elements in common).
  EXPECT_FALSE(SortedVectorsHaveIntersection(set7, set9));
  // set8 intersects set9 (element 13 in common).
  EXPECT_TRUE(SortedVectorsHaveIntersection(set8, set9));
}

}  // namespace
}  // namespace drake
