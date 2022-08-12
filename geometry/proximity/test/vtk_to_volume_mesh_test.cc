#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;

GTEST_TEST(VtkToVolumeMeshTest, OneTetrahedronStream) {
  std::istringstream test_stream{
      "POINTS 4\n"
      "0.0 0.0 0.0\n"
      "1.0 0.0 0.0\n"
      "0.0 1.0 0.0\n"
      "0.0 0.0 1.0\n"
      "CELLS 1\n"
      "4 0 1 2 3\n"};

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
