#include "drake/geometry/optimization/convex_set.h"

#include <gtest/gtest.h>

#include "drake/geometry/optimization/point.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Vector2d;
using Eigen::Vector3d;

GTEST_TEST(ConvexSetsTest, BasicTest) {
  ConvexSets sets;

  const ConvexSet& a = sets.emplace_back(Point(Vector2d{1., 2.}));
  const ConvexSet& b =
      sets.emplace_back(std::make_unique<Point>(Vector3d{3., 4., 5.}));

  EXPECT_EQ(a.ambient_dimension(), 2);
  EXPECT_EQ(b.ambient_dimension(), 3);

  EXPECT_EQ(sets.size(), 2);
  EXPECT_EQ(sets[0].ambient_dimension(), 2);
  EXPECT_EQ(sets[1].ambient_dimension(), 3);

  ConvexSets copy = sets;
  EXPECT_EQ(copy.size(), 2);
  EXPECT_EQ(copy[0].ambient_dimension(), 2);
  EXPECT_EQ(copy[1].ambient_dimension(), 3);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
