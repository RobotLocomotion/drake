#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;

GTEST_TEST(VtkToVolumeMeshTest, OneTetrahedronStream) {
  std::istringstream test_stream{
      "# vtk DataFile Version 2.0\n"
      "Example of one-tetrahedron mesh\n"
      "ASCII\n"
      "DATASET UNSTRUCTURED_GRID\n"
      "POINTS 4 float\n"
      "0.0 0.0 0.0\n"
      "1.0 0.0 0.0\n"
      "0.0 1.0 0.0\n"
      "0.0 0.0 1.0\n"
      "\n"
      "CELLS 1 5\n"
      "4 0 1 2 3\n"
      "\n"
      "CELL_TYPES 1\n"
      "10"};

  VolumeMesh<double> volume_mesh = internal::ReadVtkToVolumeMesh(&test_stream);

  const VolumeMesh<double> expected_mesh{
      {{0, 1, 2, 3}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

GTEST_TEST(VtkToVolumeMeshTest, OneTetrahedronFile) {
  const std::string test_file =
      FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk");

  VolumeMesh<double> volume_mesh = internal::ReadVtkToVolumeMesh(test_file);

  const VolumeMesh<double> expected_mesh{
      {{0, 1, 2, 3}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
