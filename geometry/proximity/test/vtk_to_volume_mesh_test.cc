#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"

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

GTEST_TEST(VtkToVolumeMeshTest, Scale) {
  const std::string test_file =
      FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk");
  // Scale from a one-meter object to a one-centimeter object.
  const double kScale = 0.01;
  VolumeMesh<double> volume_mesh = internal::ReadVtkToVolumeMesh(test_file,
                                                                 kScale);

  const VolumeMesh<double> expected_mesh{
      {{0, 1, 2, 3}},
      {kScale * Vector3d::Zero(), kScale * Vector3d::UnitX(),
       kScale * Vector3d::UnitY(), kScale * Vector3d::UnitZ()}};
  EXPECT_TRUE(volume_mesh.Equal(expected_mesh));
}

GTEST_TEST(VtkToVolumeMeshTest, BadScale) {
  const std::string test_file =
      FindResourceOrThrow("drake/geometry/test/one_tetrahedron.vtk");

  const double kNegativeScale = -0.01;
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ReadVtkToVolumeMesh(test_file, kNegativeScale),
      "ReadVtkToVolumeMesh: scale=.* is not a positive number.*");

  const double kZeroScale = 0.0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ReadVtkToVolumeMesh(test_file, kZeroScale),
      "ReadVtkToVolumeMesh: scale=.* is not a positive number.*");
}

GTEST_TEST(VtkToVolumeMeshTest, BogusFileName) {
  const std::string bogus_filename = "bogus_filename";
  EXPECT_THROW(internal::ReadVtkToVolumeMesh(bogus_filename),
               std::exception);
}

GTEST_TEST(VtkToVolumeMeshTest, WrongFileType) {
  const std::string require_vtk_but_this_is_obj =
      FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj");
  EXPECT_THROW(internal::ReadVtkToVolumeMesh(require_vtk_but_this_is_obj),
               std::exception);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
