#include <unordered_set>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity/create_submesh.h"
#include "drake/geometry/proximity/detect_null_simplex.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_mesh_field.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
#include "drake/geometry/proximity/volume_mesh_refiner.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;

// cd drake/geometry/proximity/test/MeshImprovement
// bazel build //geometry/proximity:teddy_mesh_refiner_test
// ../../../../bazel-bin/geometry/proximity/teddy_mesh_refiner_test
GTEST_TEST(TeddyMeshTest, VolumeMeshRefiner) {
  const std::string test_file =
      FindResourceOrThrow("drake/geometry/proximity/test/MeshImprovement"
                          "/teddy.vtk");
  VolumeMesh<double> test_mesh = internal::ReadVtkToVolumeMesh(test_file);

  ASSERT_EQ(test_mesh.num_vertices(), 335);
  EXPECT_EQ(test_mesh.num_elements(), 859);
  EXPECT_EQ(internal::DetectNullTetrahedron(test_mesh).size(), 110);
  EXPECT_EQ(internal::DetectNullInteriorTriangle(test_mesh).size(), 185);
  EXPECT_EQ(internal::DetectNullInteriorEdge(test_mesh).size(), 75);
  {
    internal::WriteVolumeMeshToVtk(
        "teddy_in_null_tetrahedron.vtk",
        internal::CreateSubMesh(test_mesh,
                                internal::DetectNullTetrahedron(test_mesh)),
        "Null tetrahedron in teddy mesh");
    internal::WriteVolumeMeshFieldLinearToVtk(
        "teddy_in_pressure.vtk", "pressure",
        internal::MakeVolumeMeshPressureField(&test_mesh, 1e7),
        "Test pressure on teddy with hydroelastic modulus 1e7 Pascals");
  }

  internal::VolumeMeshRefiner refiner(test_mesh);
  VolumeMesh<double> refined_mesh = refiner.refine();
  EXPECT_EQ(refined_mesh.num_vertices(), 410);
  EXPECT_EQ(refined_mesh.num_elements(), 1207);
  EXPECT_EQ(internal::DetectNullTetrahedron(refined_mesh).size(), 0);
  EXPECT_EQ(internal::DetectNullInteriorTriangle(refined_mesh).size(), 0);
  EXPECT_EQ(internal::DetectNullInteriorEdge(refined_mesh).size(), 0);
  {
    internal::WriteVolumeMeshToVtk("teddy_refine.vtk", refined_mesh,
                                   "Refined teddy mesh");
    internal::WriteVolumeMeshFieldLinearToVtk(
        "teddy_refine_pressure.vtk", "pressure",
        internal::MakeVolumeMeshPressureField(&refined_mesh, 1e7),
        "Pressure on refined teddy mesh with hydroelastic modulus 1e7 Pascals");
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake

