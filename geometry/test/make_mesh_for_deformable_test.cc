#include "drake/geometry/make_mesh_for_deformable.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

constexpr double kRezHint = 0.5;

class MeshBuilderForDeformableTest : public ::testing::Test {
 protected:
  MeshBuilderForDeformable builder_;
};

TEST_F(MeshBuilderForDeformableTest, Sphere) {
  const Sphere sphere(1.0);
  std::unique_ptr<VolumeMesh<double>> mesh = builder_.Build(sphere, kRezHint);
  const VolumeMesh<double> expected_mesh =
      MakeMeshForDeformable(sphere, kRezHint);
  EXPECT_TRUE(mesh->Equal(expected_mesh));
}

TEST_F(MeshBuilderForDeformableTest, Cylinder) {
  const Cylinder c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(builder_.Build(c, kRezHint),
                              "Cylinder.*not supported.*");
}

TEST_F(MeshBuilderForDeformableTest, Halfspace) {
  const HalfSpace h;
  DRAKE_EXPECT_THROWS_MESSAGE(builder_.Build(h, kRezHint),
                              "Half space.*not supported.*");
}

TEST_F(MeshBuilderForDeformableTest, Box) {
  const Box b(1.5, 2.5, 3.5);
  DRAKE_EXPECT_THROWS_MESSAGE(builder_.Build(b, kRezHint),
                              "Box.*not supported.*");
}

TEST_F(MeshBuilderForDeformableTest, Capsule) {
  const Capsule c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(builder_.Build(c, kRezHint),
                              "Capsule.*not supported.*");
}

TEST_F(MeshBuilderForDeformableTest, Mesh) {
  const Mesh m("path/to/file", 1.5);
  DRAKE_EXPECT_THROWS_MESSAGE(builder_.Build(m, kRezHint),
                              "Mesh.*not supported.*");
}

TEST_F(MeshBuilderForDeformableTest, Convex) {
  const Convex m("path/to/file", 1.5);
  DRAKE_EXPECT_THROWS_MESSAGE(builder_.Build(m, kRezHint),
                              "Convex.*not supported.*");
}

GTEST_TEST(MakeMeshForDeformable, Sphere) {
  const Sphere sphere(1.0);
  const VolumeMesh<double> mesh = MakeMeshForDeformable(sphere, kRezHint);
  const VolumeMesh<double> expected_mesh = MakeSphereVolumeMesh<double>(
      sphere, kRezHint, TessellationStrategy::kDenseInteriorVertices);
  EXPECT_TRUE(mesh.Equal(expected_mesh));
}

GTEST_TEST(MakeMeshForDeformable, Cylinder) {
  const Cylinder c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(c, kRezHint),
                              "Cylinder.*not supported.*");
}

GTEST_TEST(MakeMeshForDeformable, Halfspace) {
  const HalfSpace h;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(h, kRezHint),
                              "Half space.*not supported.*");
}

GTEST_TEST(MakeMeshForDeformable, Box) {
  const Box b(1.5, 2.5, 3.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(b, kRezHint),
                              "Box.*not supported.*");
}

GTEST_TEST(MakeMeshForDeformable, Capsule) {
  const Capsule c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(c, kRezHint),
                              "Capsule.*not supported.*");
}

GTEST_TEST(MakeMeshForDeformable, Mesh) {
  const Mesh m("path/to/file", 1.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(m, kRezHint),
                              "Mesh.*not supported.*");
}

GTEST_TEST(MakeMeshForDeformable, Convex) {
  const Convex m("path/to/file", 1.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(m, kRezHint),
                              "Convex.*not supported.*");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
