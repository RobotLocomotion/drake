#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;

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

// With an additional field variable, we can still read the mesh.
GTEST_TEST(VtkToVolumeMeshTest, KeepMeshIgnoreFieldVariables) {
  const std::string test_file = FindResourceOrThrow(
      "drake/geometry/test/two_tetrahedra_with_field_variable.vtk");
  VolumeMesh<double> volume_mesh =
      internal::ReadVtkToVolumeMesh(test_file);

  const VolumeMesh<double> expected_mesh{
      {{0, 1, 2, 3}, {0, 2, 1, 4}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ(), -Vector3d::UnitZ()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

// TODO(DamrongGuoy) Add more tests like bogus file names or non-vtk files.

}  // namespace
}  // namespace geometry
}  // namespace drake
