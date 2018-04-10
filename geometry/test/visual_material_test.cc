#include "drake/geometry/visual_material.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(VisualMaterial, DefaultConstructor) {
  // Documented as "light grey".
  VisualMaterial material;
  const Eigen::Vector4d& diffuse = material.diffuse();
  // Confirm grey -- r = g = b (by transitivity).
  EXPECT_EQ(diffuse(0), diffuse(1));
  EXPECT_EQ(diffuse(1), diffuse(2));
  // Confirm "light"
  EXPECT_GT(diffuse(0), 0.5);
}

GTEST_TEST(VisualMaterial, FullConstructor) {
  Eigen::Vector4d diffuse_in{0.1, 0.2, 0.3, 0.4};
  VisualMaterial material{diffuse_in};
  const Eigen::Vector4d& diffuse = material.diffuse();

  for (int i = 0; i < 4; ++i)
    EXPECT_EQ(diffuse(i), diffuse_in(i));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
