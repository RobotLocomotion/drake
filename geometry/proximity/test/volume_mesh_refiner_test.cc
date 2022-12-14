#include "drake/geometry/proximity/volume_mesh_refiner.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/detect_null_simplex.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(VolumeMeshRefinerTest, TestRefineTetrahedron) {
  //      +Z
  //       |
  //       v3
  //       |
  //       |
  //     v0+------v2---+Y
  //      /
  //     /
  //   v1
  //   /
  // +X
  //
  const VolumeMesh<double> test_mesh(
      std::vector<VolumeElement>{{0, 1, 2, 3}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ()});
  ASSERT_NE(DetectNullTetrahedron(test_mesh).size(), 0);
  ASSERT_EQ(DetectNullInteriorTriangle(test_mesh).size(), 0);
  ASSERT_EQ(DetectNullInteriorEdge(test_mesh).size(), 0);
  {
    WriteVolumeMeshToVtk("RefineTet_input.vtk", test_mesh,
                         "TestRefineTetrahedron_test_mesh");
  }

  VolumeMeshRefiner refiner(test_mesh);
  VolumeMesh<double> refined_mesh = refiner.refine();

  EXPECT_EQ(DetectNullTetrahedron(refined_mesh).size(), 0);
  EXPECT_EQ(DetectNullInteriorTriangle(refined_mesh).size(), 0);
  EXPECT_EQ(DetectNullInteriorEdge(refined_mesh).size(), 0);
  EXPECT_EQ(refined_mesh.num_vertices(), 5);
  EXPECT_EQ(refined_mesh.num_elements(), 4);
  {
    WriteVolumeMeshToVtk("RefineTet_output.vtk", refined_mesh,
                         "TestRefineTetrahedron_refined_mesh");
  }
}

GTEST_TEST(VolumeMeshRefinerTest, TestRefineTriangle) {
  // The interior triangle v0v1v2 is shared by two tetrahedra comprising the
  // mesh.
  //
  //      +Z
  //       |
  //       v3
  //       |
  //       |
  //     v0+------v2---+Y
  //      /|
  //     / |
  //   v1  v4
  //   /   |
  // +X    |
  //      -Z
  //
  const VolumeMesh<double> test_mesh(
      std::vector<VolumeElement>{{0, 1, 2, 3}, {2, 1, 0, 4}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            -Vector3d::UnitZ()});
  ASSERT_NE(DetectNullTetrahedron(test_mesh).size(), 0);
  ASSERT_NE(DetectNullInteriorTriangle(test_mesh).size(), 0);
  ASSERT_EQ(DetectNullInteriorEdge(test_mesh).size(), 0);
  {
    WriteVolumeMeshToVtk("RefineTri_input.vtk", test_mesh,
                         "TestRefineTriangle_test_mesh");
  }

  VolumeMeshRefiner refiner(test_mesh);
  VolumeMesh<double> refined_mesh = refiner.refine();

  EXPECT_EQ(DetectNullTetrahedron(refined_mesh).size(), 0);
  EXPECT_EQ(DetectNullInteriorTriangle(refined_mesh).size(), 0);
  EXPECT_EQ(DetectNullInteriorEdge(refined_mesh).size(), 0);
  EXPECT_EQ(refined_mesh.num_vertices(), 6);
  EXPECT_EQ(refined_mesh.num_elements(), 6);
  {
    WriteVolumeMeshToVtk("RefineTri_output.vtk", refined_mesh,
                         "TestRefineTriangle_refined_mesh");
  }
}

GTEST_TEST(VolumeMeshRefinerTest, TestRefineEdge) {
  // The interior edge v0v2 is shared by three tetrahedra comprising the mesh.
  //
  //      +Z
  //       |
  //       | v3
  //       |
  //       |
  //     v0+------v2---+Y
  //      /|
  //     / |
  //   v1  | v4
  //   /   |
  // +X    |
  //      -Z
  //
  const VolumeMesh<double> test_mesh(
      std::vector<VolumeElement>{{0, 1, 2, 3},
                                 {2, 1, 0, 4},
                                 {0, 2, 4, 3}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            2 * Vector3d::UnitY(),
                            Vector3d(-1, 1, 1),
                            Vector3d(-1, 1, -1)});
  ASSERT_NE(DetectNullTetrahedron(test_mesh).size(), 0);
  ASSERT_NE(DetectNullInteriorTriangle(test_mesh).size(), 0);
  ASSERT_NE(DetectNullInteriorEdge(test_mesh).size(), 0);
  {
    WriteVolumeMeshToVtk("RefineEdge_input.vtk", test_mesh,
                         "TestRefineEdge_test_mesh");
  }

  VolumeMeshRefiner refiner(test_mesh);
  VolumeMesh<double> refined_mesh = refiner.refine();

  EXPECT_EQ(DetectNullTetrahedron(refined_mesh).size(), 0);
  EXPECT_EQ(DetectNullInteriorTriangle(refined_mesh).size(), 0);
  EXPECT_EQ(DetectNullInteriorEdge(refined_mesh).size(), 0);
  EXPECT_EQ(refined_mesh.num_vertices(), 6);
  EXPECT_EQ(refined_mesh.num_elements(), 6);
  {
    WriteVolumeMeshToVtk("RefineEdge_output.vtk", refined_mesh,
                         "TestRefineEdge_refined_mesh");
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
