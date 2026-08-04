#include "drake/geometry/make_mesh_for_deformable.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

constexpr double kRezHint = 0.5;

GTEST_TEST(MakeMeshForDeformableTest, Sphere) {
  const Sphere sphere(1.0);
  std::unique_ptr<VolumeMesh<double>> mesh =
      MakeMeshForDeformable(sphere, kRezHint);
  const VolumeMesh<double> expected_mesh = MakeSphereVolumeMesh<double>(
      sphere, kRezHint, TessellationStrategy::kDenseInteriorVertices);
  EXPECT_TRUE(mesh->Equal(expected_mesh));
}

GTEST_TEST(MakeMeshForDeformableTest, Cylinder) {
  const Cylinder c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeMeshForDeformable(c, kRezHint),
      ".*don't yet generate deformable meshes.+ Cylinder.*");
}

GTEST_TEST(MakeMeshForDeformableTest, Halfspace) {
  const HalfSpace h;
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeMeshForDeformable(h, kRezHint),
      ".*don't yet generate deformable meshes.+ HalfSpace.*");
}

GTEST_TEST(MakeMeshForDeformableTest, Box) {
  const Box b(1.5, 2.5, 3.5);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeMeshForDeformable(b, kRezHint),
                              ".*don't yet generate deformable meshes.+ Box.*");
}

GTEST_TEST(MakeMeshForDeformableTest, Capsule) {
  const Capsule c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeMeshForDeformable(c, kRezHint),
      ".*don't yet generate deformable meshes.+ Capsule.*");
}

GTEST_TEST(MakeMeshForDeformableTest, Mesh) {
  const Mesh m(FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk"),
               1.5);
  std::unique_ptr<VolumeMesh<double>> mesh = MakeMeshForDeformable(m, kRezHint);
  const VolumeMesh<double> expected_mesh = MakeVolumeMeshFromVtk<double>(m);
  EXPECT_TRUE(mesh->Equal(expected_mesh));
}

GTEST_TEST(MakeMeshForDeformableTest, Convex) {
  const Convex m("path/to/file", 1.5);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeMeshForDeformable(m, kRezHint),
      ".*don't yet generate deformable meshes.+ Convex.*");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
