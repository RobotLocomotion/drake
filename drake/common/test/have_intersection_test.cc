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
}

}  // namespace
}  // namespace drake
