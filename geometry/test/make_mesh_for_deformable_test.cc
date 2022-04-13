#include "drake/geometry/make_mesh_for_deformable.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

constexpr double kRezHint = 0.5;

GTEST_TEST(DeformableMeshBuilderTest, Sphere) {
  const Sphere sphere(1.0);
  const VolumeMesh<double> mesh = MakeMeshForDeformable(sphere, kRezHint);
  const VolumeMesh<double> expected_mesh = MakeSphereVolumeMesh<double>(
      sphere, kRezHint, TessellationStrategy::kDenseInteriorVertices);
  EXPECT_TRUE(mesh.Equal(expected_mesh));
}

GTEST_TEST(DeformableMeshBuilderTest, Cylinder) {
  const Cylinder c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(c, kRezHint),
                              "Cylinder.*not supported.*");
}

GTEST_TEST(DeformableMeshBuilderTest, Halfspace) {
  const HalfSpace h;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(h, kRezHint),
                              "Half space.*not supported.*");
}

GTEST_TEST(DeformableMeshBuilderTest, Box) {
  const Box b(1.5, 2.5, 3.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(b, kRezHint),
                              "Box.*not supported.*");
}

GTEST_TEST(DeformableMeshBuilderTest, Capsule) {
  const Capsule c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(c, kRezHint),
                              "Capsule.*not supported.*");
}

GTEST_TEST(DeformableMeshBuilderTest, Mesh) {
  const Mesh m("path/to/file", 1.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(m, kRezHint),
                              "Mesh.*not supported.*");
}

GTEST_TEST(DeformableMeshBuilderTest, Convex) {
  const Convex m("path/to/file", 1.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(m, kRezHint),
                              "Convex.*not supported.*");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
