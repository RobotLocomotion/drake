#include "drake/common/have_intersection.h"

#include <vector>
#include <set>
#include "gtest/gtest.h"

using std::vector;
using std::set;

namespace drake {

namespace {

// Test using std::vector
GTEST_TEST(TestHaveIntersection, StdVectors) {
  // In order for drake::HaveIntersection to work sets must:
  // 1. Not have repeated elements.
  // 2. Be sorted.
  // An std::vector can have non sorted repeated elements. Therefore the user
  // must ensure that the previous conditions are met.
  vector<int> set1 = vector<int>({2, 9, 11, 15, 23});
  vector<int> set2 = vector<int>({9, 11, 13});
  vector<int> set3 = vector<int>({1, 8, 13});

  // set1 intersects set2 (elements 9 and 11 in common).
  EXPECT_TRUE(HaveIntersection(set1.begin(), set1.end(),
                               set2.begin(), set2.end()));

  // set2 intersects set1 (commutative).
  EXPECT_TRUE(HaveIntersection(set2.begin(), set2.end(),
                               set1.begin(), set1.end()));

  // set1 does not intersect set3 (no elements in common).
  EXPECT_FALSE(HaveIntersection(set1.begin(), set1.end(),
                                set3.begin(), set3.end()));

  // set3 does not intersect set1 (commutative).
  EXPECT_FALSE(HaveIntersection(set3.begin(), set3.end(),
                                set1.begin(), set1.end()));

  // set2 intersects set3 (element 13 in common).
  EXPECT_TRUE(HaveIntersection(set2.begin(), set2.end(),
                               set3.begin(), set3.end()));

  // set3 intersects set2 (commutative).
  EXPECT_TRUE(HaveIntersection(set3.begin(), set3.end(),
                               set2.begin(), set2.end()));

  // Empty sets.
  vector<int> set4;
  vector<int> set5;
  // Two empty sets.
  EXPECT_FALSE(HaveIntersection(set4.begin(), set4.end(),
                                set5.begin(), set5.end()));

  // First non-empty against second empty.
  EXPECT_FALSE(HaveIntersection(set1.begin(), set1.end(),
                                set4.begin(), set4.end()));

  // First empty against second non-empty.
  EXPECT_FALSE(HaveIntersection(set4.begin(), set4.end(),
                                set1.begin(), set1.end()));

  // Disjoint sets.
  vector<int> set6 = vector<int>({30, 35, 50});
  // set1 entries are all smaller than set6 first entry.
  EXPECT_FALSE(HaveIntersection(set1.begin(), set1.end(),
                                set6.begin(), set6.end()));

  // Tests if the above is commutative.
  EXPECT_FALSE(HaveIntersection(set6.begin(), set6.end(),
                                set1.begin(), set1.end()));
}

// Test with std::vector specific SortedVectorsHaveIntersection.
GTEST_TEST(TestHaveIntersection, SortedVectorsHaveIntersection) {
  // In order for drake::SortedVectorsHaveIntersection to work vectors must:
  // 1. Not have repeated elements.
  // 2. Be sorted.
  // An std::vector can have non sorted repeated elements. Therefore the user
  // must ensure that the previous conditions are met.
  vector<int> set1 = vector<int>({2, 9, 11, 15, 23});
  vector<int> set2 = vector<int>({9, 11, 13});
  vector<int> set3 = vector<int>({1, 8, 13});

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
  vector<int> set4;
  vector<int> set5;
  // Two empty sets.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set4, set5));

  // First non-empty against second empty.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set1, set4));

  // First empty against second non-empty.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set4, set1));

  // Disjoint sets.
  vector<int> set6 = vector<int>({30, 35, 50});
  // set1 entries are all smaller than set6 first entry.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set1, set6));

  // Tests if the above is commutative.
  EXPECT_FALSE(SortedVectorsHaveIntersection(set6, set1));
}

// Test using std::set
GTEST_TEST(TestHaveIntersection, StdSets) {
  // In order for drake::HaveIntersection to work sets must:
  // 1. Not have repeated elements.
  // 2. Be sorted.
  // std::set offers this functionality.
  set<int> set1 = set<int>({2, 9, 11, 15, 23});
  set<int> set2 = set<int>({9, 11, 13});
  set<int> set3 = set<int>({1, 8, 13});

  // Adding a previously added element to a set has no effect.
  set1.insert(11);
  set1.insert(23);

  // Elements cannot be repeated. Therefore expect 5 groups instead of 7.
  EXPECT_EQ(5, set1.size());

  // set1 intersects set2 (elements 9 and 11 in common).
  EXPECT_TRUE(HaveIntersection(set1.begin(), set1.end(),
                               set2.begin(), set2.end()));

  // set2 intersects set1 (commutative).
  EXPECT_TRUE(HaveIntersection(set2.begin(), set2.end(),
                               set1.begin(), set1.end()));

  // set1 does not intersect set3 (no elements in common).
  EXPECT_FALSE(HaveIntersection(set1.begin(), set1.end(),
                                set3.begin(), set3.end()));

  // set3 does not intersect set1 (commutative).
  EXPECT_FALSE(HaveIntersection(set3.begin(), set3.end(),
                                set1.begin(), set1.end()));

  // set2 intersects set3 (element 13 in common).
  EXPECT_TRUE(HaveIntersection(set2.begin(), set2.end(),
                               set3.begin(), set3.end()));

  // set3 intersects set2 (commutative).
  EXPECT_TRUE(HaveIntersection(set3.begin(), set3.end(),
                               set2.begin(), set2.end()));

  // Empty sets.
  set<int> set4;
  set<int> set5;
  // Two empty sets.
  EXPECT_FALSE(HaveIntersection(set4.begin(), set4.end(),
                                set5.begin(), set5.end()));

  // First non-empty against second empty.
  EXPECT_FALSE(HaveIntersection(set1.begin(), set1.end(),
                                set4.begin(), set4.end()));

  // First empty against second non-empty.
  EXPECT_FALSE(HaveIntersection(set4.begin(), set4.end(),
                                set1.begin(), set1.end()));

  // Disjoint sets.
  set<int> set6 = set<int>({30, 35, 50});
  // set1 entries are all smaller than set6 first entry.
  EXPECT_FALSE(HaveIntersection(set1.begin(), set1.end(),
                                set6.begin(), set6.end()));

  // Tests if the above is commutative.
  EXPECT_FALSE(HaveIntersection(set6.begin(), set6.end(),
                                set1.begin(), set1.end()));
}

}  // namespace
}  // namespace drake
