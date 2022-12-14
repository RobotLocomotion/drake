#include "drake/geometry/proximity/volume_mesh_refiner.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/detect_zero_simplex.h"

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
  ASSERT_EQ(DetectTetrahedronWithAllBoundaryVertices(test_mesh).size(), 1);
  ASSERT_EQ(DetectInteriorTriangleWithAllBoundaryVertices(test_mesh).size(), 0);
  ASSERT_EQ(DetectInteriorEdgeWithAllBoundaryVertices(test_mesh).size(), 0);
  ASSERT_EQ(test_mesh.num_vertices(), 4);
  ASSERT_EQ(test_mesh.num_elements(), 1);

  VolumeMeshRefiner refiner(test_mesh);
  VolumeMesh<double> refined_mesh = refiner.Refine();

  EXPECT_EQ(DetectTetrahedronWithAllBoundaryVertices(refined_mesh).size(), 0);
  EXPECT_EQ(DetectInteriorTriangleWithAllBoundaryVertices(refined_mesh).size(),
            0);
  EXPECT_EQ(DetectInteriorEdgeWithAllBoundaryVertices(refined_mesh).size(), 0);
  EXPECT_EQ(refined_mesh.num_vertices(), 5);
  EXPECT_EQ(refined_mesh.num_elements(), 4);
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
  ASSERT_EQ(DetectTetrahedronWithAllBoundaryVertices(test_mesh).size(), 2);
  ASSERT_EQ(DetectInteriorTriangleWithAllBoundaryVertices(test_mesh).size(), 1);
  ASSERT_EQ(DetectInteriorEdgeWithAllBoundaryVertices(test_mesh).size(), 0);
  ASSERT_EQ(test_mesh.num_vertices(), 5);
  ASSERT_EQ(test_mesh.num_elements(), 2);

  VolumeMeshRefiner refiner(test_mesh);
  VolumeMesh<double> refined_mesh = refiner.Refine();

  EXPECT_EQ(DetectTetrahedronWithAllBoundaryVertices(refined_mesh).size(), 0);
  EXPECT_EQ(DetectInteriorTriangleWithAllBoundaryVertices(refined_mesh).size(),
            0);
  EXPECT_EQ(DetectInteriorEdgeWithAllBoundaryVertices(refined_mesh).size(), 0);
  EXPECT_EQ(refined_mesh.num_vertices(), 6);
  EXPECT_EQ(refined_mesh.num_elements(), 6);
}

GTEST_TEST(VolumeMeshRefinerTest, TestRefineEdge) {
  // The interior edge v0v2 is shared by three tetrahedra comprising the mesh.
  // Vertex v3(-1,1,1) is in the -X+Y+Z octant, and vertex v4(-1,1,-1) is in
  // the -X+Y-Z octant.
  // quadrant.
  //
  //      +Z
  //       |   -X
  //       |   /
  //       |  /   v3(-1,1,1)
  //       | /
  //       |/
  //     v0+-----+------v2---+Y
  //      /|
  //     / |      v4(-1,1,-1)
  //   v1  |
  //   /   |
  // +X    |
  //      -Z
  //
  const VolumeMesh<double> test_mesh(
      std::vector<VolumeElement>{{0, 1, 2, 3}, {2, 1, 0, 4}, {0, 2, 4, 3}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            2 * Vector3d::UnitY(), Vector3d(-1, 1, 1),
                            Vector3d(-1, 1, -1)});
  ASSERT_EQ(DetectTetrahedronWithAllBoundaryVertices(test_mesh).size(), 3);
  ASSERT_EQ(DetectInteriorTriangleWithAllBoundaryVertices(test_mesh).size(), 3);
  ASSERT_EQ(DetectInteriorEdgeWithAllBoundaryVertices(test_mesh).size(), 1);
  ASSERT_EQ(test_mesh.num_vertices(), 5);
  ASSERT_EQ(test_mesh.num_elements(), 3);

  VolumeMeshRefiner refiner(test_mesh);
  VolumeMesh<double> refined_mesh = refiner.Refine();

  EXPECT_EQ(DetectTetrahedronWithAllBoundaryVertices(refined_mesh).size(), 0);
  EXPECT_EQ(DetectInteriorTriangleWithAllBoundaryVertices(refined_mesh).size(),
            0);
  EXPECT_EQ(DetectInteriorEdgeWithAllBoundaryVertices(refined_mesh).size(), 0);
  EXPECT_EQ(refined_mesh.num_vertices(), 6);
  EXPECT_EQ(refined_mesh.num_elements(), 6);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
