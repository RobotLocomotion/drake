#include "drake/geometry/shape_to_string.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(ShapeToStringTest, Box) {
  ShapeToString reifier;
  Box b(1.5, 2.5, 3.5);
  b.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "Box(w: 1.5, d: 2.5, h: 3.5)");
}

GTEST_TEST(ShapeToStringTest, Capsule) {
  ShapeToString reifier;
  Capsule c(1.25, 2.5);
  c.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "Capsule(r: 1.25, l: 2.5)");
}

GTEST_TEST(ShapeToStringTest, Convex) {
  ShapeToString reifier;
  Convex m("/path/to/file", 1.5);
  m.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "Convex(s: 1.5, path: /path/to/file)");
}

GTEST_TEST(ShapeToStringTest, Cylinder) {
  ShapeToString reifier;
  Cylinder c(1.25, 2.5);
  c.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "Cylinder(r: 1.25, l: 2.5)");
}

GTEST_TEST(ShapeToStringTest, Ellipsoid) {
  ShapeToString reifier;
  Ellipsoid e(1.25, 2.5, 0.5);
  e.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "Ellipsoid(a: 1.25, b: 2.5, c: 0.5)");
}

GTEST_TEST(ShapeToStringTest, Halfspace) {
  ShapeToString reifier;
  HalfSpace h;
  h.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "Halfspace");
}

GTEST_TEST(ShapeToStringTest, Mesh) {
  ShapeToString reifier;
  Mesh m("/path/to/file", 1.5);
  m.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "Mesh(s: 1.5, path: /path/to/file)");
}

GTEST_TEST(ShapeToStringTest, MeshcatCone) {
  ShapeToString reifier;
  MeshcatCone c(1.5, 0.25, 0.5);
  c.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "MeshcatCone(height: 1.5, a: 0.25, b: 0.5)");
}

GTEST_TEST(ShapeToStringTest, Sphere) {
  ShapeToString reifier;
  Sphere s(1.25);
  s.Reify(&reifier);
  EXPECT_EQ(reifier.string(), "Sphere(r: 1.25)");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
