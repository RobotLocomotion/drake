#include "drake/geometry/proximity/make_mesh_from_vtk.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(MakeVolumeMeshFromVtkTest, ScalingWithDouble) {
  Mesh mesh_specification(
      FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk"), 0.5);

  VolumeMesh<double> volume_mesh =
      MakeVolumeMeshFromVtk<double>(mesh_specification);

  const VolumeMesh<double> expected_mesh{
      {{0, 1, 2, 3}},
      {0.5 * Vector3d::Zero(), 0.5 * Vector3d::UnitX(), 0.5 * Vector3d::UnitY(),
       0.5 * Vector3d::UnitZ()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

GTEST_TEST(MakeVolumeMeshFromVtkTest, AutoDiff) {
  Mesh mesh_specification(
      FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk"));

  VolumeMesh<AutoDiffXd> volume_mesh =
      MakeVolumeMeshFromVtk<AutoDiffXd>(mesh_specification);

  const VolumeMesh<AutoDiffXd> expected_mesh{
      {{0, 1, 2, 3}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

GTEST_TEST(MakeVolumeMeshFromVtkTest, NonConvexMesh) {
  const Mesh mesh_specification(
      FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk"));

  VolumeMesh<double> volume_mesh =
      MakeVolumeMeshFromVtk<double>(mesh_specification);

  const VolumeMesh<double> expected_non_convex_mesh{
      {
          {0, 5, 2, 3},
          {0, 5, 3, 4},
          {0, 5, 4, 2},
          {0, 1, 4, 3},
          {0, 1, 3, 2},
          {0, 1, 2, 4},
      },
      {0.1 * Vector3d::Ones(), Vector3d::Zero(), Vector3d::UnitX(),
       Vector3d::UnitY(), Vector3d::UnitZ(), 0.2 * Vector3d::Ones()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_non_convex_mesh));
}

GTEST_TEST(MakeVolumeMeshFromVtkTest, NegativeVolumeThrow) {
  Mesh negative_mesh_specification(
      FindResourceOrThrow("drake/geometry/test/one_negative_tetrahedron.vtk"),
      0.5);

  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeVolumeMeshFromVtk<double>(negative_mesh_specification),
      "MakeVolumeMeshFromVtk.* tetrahedron.*"
      "with vertices .* has non-positive volume, "
      "so you might want to switch two consecutive vertices.");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
