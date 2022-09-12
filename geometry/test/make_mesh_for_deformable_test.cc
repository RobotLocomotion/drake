#include "drake/geometry/make_mesh_for_deformable.h"

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

class MeshBuilderForDeformableTest : public ::testing::Test {
 protected:
  MeshBuilderForDeformable builder_;
};

TEST_F(MeshBuilderForDeformableTest, Sphere) {
  const Sphere sphere(1.0);
  std::unique_ptr<VolumeMesh<double>> mesh = builder_.Build(sphere, kRezHint);
  const VolumeMesh<double> expected_mesh = MakeSphereVolumeMesh<double>(
      sphere, kRezHint, TessellationStrategy::kDenseInteriorVertices);
  EXPECT_TRUE(mesh->Equal(expected_mesh));
}

TEST_F(MeshBuilderForDeformableTest, Cylinder) {
  const Cylinder c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder_.Build(c, kRezHint),
      ".*don't yet generate deformable meshes.+ Cylinder.");
}

TEST_F(MeshBuilderForDeformableTest, Halfspace) {
  const HalfSpace h;
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder_.Build(h, kRezHint),
      ".*don't yet generate deformable meshes.+ HalfSpace.");
}

TEST_F(MeshBuilderForDeformableTest, Box) {
  const Box b(1.5, 2.5, 3.5);
  DRAKE_EXPECT_THROWS_MESSAGE(builder_.Build(b, kRezHint),
                              ".*don't yet generate deformable meshes.+ Box.");
}

TEST_F(MeshBuilderForDeformableTest, Capsule) {
  const Capsule c(1.25, 2.5);
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder_.Build(c, kRezHint),
      ".*don't yet generate deformable meshes.+ Capsule.");
}

TEST_F(MeshBuilderForDeformableTest, Mesh) {
  const Mesh m(FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk"),
               1.5);
  std::unique_ptr<VolumeMesh<double>> mesh = builder_.Build(m, kRezHint);
  const VolumeMesh<double> expected_mesh = MakeVolumeMeshFromVtk<double>(m);
  EXPECT_TRUE(mesh->Equal(expected_mesh));
}

TEST_F(MeshBuilderForDeformableTest, Convex) {
  const Convex m("path/to/file", 1.5);
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder_.Build(m, kRezHint),
      ".*don't yet generate deformable meshes.+ Convex.");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
