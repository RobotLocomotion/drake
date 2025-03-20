#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

#include <filesystem>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace {

namespace fs = std::filesystem;

using Eigen::Vector3d;

GTEST_TEST(VtkToVolumeMeshTest, OneTetrahedronFile) {
  const fs::path test_file =
      FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk");
  VolumeMesh<double> volume_mesh = internal::ReadVtkToVolumeMesh(test_file);

  const VolumeMesh<double> expected_mesh{
      {{0, 1, 2, 3}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

// With an additional field variable, we can still read the mesh.
GTEST_TEST(VtkToVolumeMeshTest, KeepMeshIgnoreFieldVariables) {
  const fs::path test_file = FindResourceOrThrow(
      "drake/geometry/test/two_tetrahedra_with_field_variable.vtk");
  VolumeMesh<double> volume_mesh = internal::ReadVtkToVolumeMesh(test_file);

  const VolumeMesh<double> expected_mesh{
      {{0, 1, 2, 3}, {0, 2, 1, 4}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ(), -Vector3d::UnitZ()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

GTEST_TEST(VtkToVolumeMeshTest, Scale) {
  const fs::path test_file =
      FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk");
  // Scale from a one-meter object to a one-centimeter-ish object.
  const Vector3d kScale(0.01, 0.02, 0.03);
  VolumeMesh<double> volume_mesh =
      internal::ReadVtkToVolumeMesh(test_file, kScale);

  const VolumeMesh<double> expected_mesh{
      {{0, 1, 2, 3}},
      {kScale.cwiseProduct(Vector3d::Zero()),
       kScale.cwiseProduct(Vector3d::UnitX()),
       kScale.cwiseProduct(Vector3d::UnitY()),
       kScale.cwiseProduct(Vector3d::UnitZ())}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

GTEST_TEST(VtkToVolumeMeshTest, FromMemory) {
  // Scale from a one-meter object to a one-centimeter object.
  const double kScale = 0.01;
  const fs::path test_file =
      FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk");
  VolumeMesh<double> volume_mesh = internal::ReadVtkToVolumeMesh(
      InMemoryMesh{MemoryFile::Make(test_file)}, Vector3d::Constant(kScale));

  const VolumeMesh<double> expected_mesh{
      {{0, 1, 2, 3}},
      {kScale * Vector3d::Zero(), kScale * Vector3d::UnitX(),
       kScale * Vector3d::UnitY(), kScale * Vector3d::UnitZ()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

// When the non-uniform scale allows flipping, we want to make sure the tets
// get rewound so that inside is still inside. The volume mesh is a 2x2x2 cube.
// It's volume should be 8. But if the winding got messed up, the volume will
// be wrong (specifically, -8).
GTEST_TEST(VtkToVolumeMeshTest, InverseScaleWinding) {
  const fs::path vtk_file =
      FindResourceOrThrow("drake/geometry/test/cube_as_volume.vtk");
  for (double sx : {-1.0, 1.0}) {
    for (double sy : {-1.0, 1.0}) {
      for (double sz : {-1.0, 1.0}) {
        const Eigen::Vector3d scale(sx, sy, sz);
        SCOPED_TRACE(fmt::format("Scale [{}]", fmt_eigen(scale.transpose())));
        const VolumeMesh<double> mesh =
            internal::ReadVtkToVolumeMesh(vtk_file, scale);
        ASSERT_EQ(mesh.num_elements(), 12);
        EXPECT_EQ(mesh.CalcVolume(), 8);
      }
    }
  }
}

GTEST_TEST(VtkToVolumeMeshTest, BogusFileName) {
  const fs::path bogus_filename = "bogus_filename";
  DRAKE_EXPECT_THROWS_MESSAGE(internal::ReadVtkToVolumeMesh(bogus_filename),
                              ".*at least one tetra.*");
}

GTEST_TEST(VtkToVolumeMeshTest, WrongFileType) {
  const fs::path require_vtk_but_this_is_obj =
      FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj");
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ReadVtkToVolumeMesh(require_vtk_but_this_is_obj),
      ".*at least one tetra.*");
}

GTEST_TEST(VtkToVolumeMeshTest, WrongFileContentsCube) {
  const fs::path cube_vtk =
      FindResourceOrThrow("drake/geometry/test/cube_as_6_squares.vtk");
  DRAKE_EXPECT_THROWS_MESSAGE(internal::ReadVtkToVolumeMesh(cube_vtk),
                              ".*at least one tetra.*");
}

GTEST_TEST(VtkToVolumeMeshTest, WrongFileContentsVolume) {
  const fs::path volume_vtk =
      FindResourceOrThrow("drake/geometry/test/some_volume.vtk");
  DRAKE_EXPECT_THROWS_MESSAGE(internal::ReadVtkToVolumeMesh(volume_vtk),
                              ".*at least one tetra.*");
}

GTEST_TEST(VtkToVolumeMeshTest, WrongFileContentsUnstructured) {
  const fs::path unstructured_vtk =
      FindResourceOrThrow("drake/geometry/test/unstructured.vtk");
  DRAKE_EXPECT_THROWS_MESSAGE(internal::ReadVtkToVolumeMesh(unstructured_vtk),
                              ".*should only contain tetrahedra.*");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
