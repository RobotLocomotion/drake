#include <gtest/gtest.h>

#include "drake/geometry/optimization/hpolyhedron.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Vector2d;

// This file contains tests for ConvexSet functions that call Solve(). We cannot
// test those functions in convex_set_test.cc because LimitMalloc and Solve are
// incompatible when MOSEK might be used.

GTEST_TEST(ConvexSetTest, IntersectsWithTest) {
  /* Test that IntersectsWith() yields correct results for the following
  arrangement of boxes:
     5                ┏━━━━━━━━━┓
                      ┃      C  ┃
     4      ┏━━━━━━━━━┃━━━━┓    ┃
            ┃         ┃    ┃    ┃
     3      ┃         ┗━━━━━━━━━┛
            ┃      B       ┃
     2 ┏━━━━┃━━━━┓         ┃
       ┃    ┃    ┃         ┃
     1 ┃    ┗━━━━━━━━━━━━━━┛
       ┃  A      ┃
     0 ┗━━━━━━━━━┛
       0    1    2    3    4    5
  */
  HPolyhedron set_A = HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(2, 2));
  HPolyhedron set_B = HPolyhedron::MakeBox(Vector2d(1, 1), Vector2d(4, 4));
  HPolyhedron set_C = HPolyhedron::MakeBox(Vector2d(3, 3), Vector2d(5, 5));
  EXPECT_TRUE(set_A.IntersectsWith(set_B));
  EXPECT_TRUE(set_B.IntersectsWith(set_A));
  EXPECT_TRUE(set_B.IntersectsWith(set_C));
  EXPECT_TRUE(set_C.IntersectsWith(set_B));
  EXPECT_FALSE(set_A.IntersectsWith(set_C));
  EXPECT_FALSE(set_C.IntersectsWith(set_A));
}

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
