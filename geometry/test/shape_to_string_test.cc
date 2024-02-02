#include "drake/geometry/shape_to_string.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(DeprecatedShapeToStringTest, Box) {
  ShapeToString reifier;
  Box b(1.5, 2.5, 3.5);
  b.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "Box(width=1.5, depth=2.5, height=3.5)");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
