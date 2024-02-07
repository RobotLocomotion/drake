#include "drake/geometry/proximity/volume_mesh_refiner.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/detect_zero_simplex.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// TODO(DamrongGuoy) Consider testing RefineTetrahedron(), RefineTriangle(),
//  and RefineEdge() directly instead of testing them through Refine().

// Test RefineTetrahedron() with a tetrahedral mesh consisting of only one
// tetrahedron. It should become four tetrahedra that share a new vertex.
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

  // Expected properties of the result mesh
  constexpr int kExpectedNumVerts = 5;
  constexpr int kExpectedNumTets = 4;
  constexpr int kNewVertexIndex = kExpectedNumVerts - 1;
  EXPECT_EQ(refined_mesh.num_vertices(), kExpectedNumVerts);
  EXPECT_EQ(refined_mesh.num_elements(), kExpectedNumTets);
  EXPECT_EQ(refined_mesh.vertex(kNewVertexIndex), Vector3d(0.25, 0.25, 0.25));
  for (const VolumeElement& tetrahedron : refined_mesh.tetrahedra()) {
    ASSERT_TRUE(tetrahedron.vertex(0) == kNewVertexIndex ||
                tetrahedron.vertex(1) == kNewVertexIndex ||
                tetrahedron.vertex(2) == kNewVertexIndex ||
                tetrahedron.vertex(3) == kNewVertexIndex);
  }
}

// Test RefineTriangle() with a tetrahedral mesh consisting of two tetrahedra
// sharing a triangle. They should become six tetrahedra sharing a new vertex.
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
      // Two tetrahedra sharing the triangle with vertex indices 0,1,2.
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

  // Expected properties of the result mesh
  constexpr int kExpectedNumVerts = 6;
  constexpr int kExpectedNumTets = 6;
  constexpr int kNewVertexIndex = kExpectedNumVerts - 1;
  EXPECT_EQ(refined_mesh.num_vertices(), kExpectedNumVerts);
  EXPECT_EQ(refined_mesh.num_elements(), kExpectedNumTets);
  EXPECT_EQ(refined_mesh.vertex(kNewVertexIndex),
            Vector3d(1.0 / 3, 1.0 / 3, 0));
  for (const VolumeElement& tetrahedron : refined_mesh.tetrahedra()) {
    ASSERT_TRUE(tetrahedron.vertex(0) == kNewVertexIndex ||
                tetrahedron.vertex(1) == kNewVertexIndex ||
                tetrahedron.vertex(2) == kNewVertexIndex ||
                tetrahedron.vertex(3) == kNewVertexIndex);
  }
}

// Test RefineEdge() with a tetrahedral mesh consisting of four tetrahedra
// sharing an edge. They should become eight tetrahedra sharing a new vertex.
GTEST_TEST(VolumeMeshRefinerTest, TestRefineEdge) {
  // The interior edge v0v2 is shared by four tetrahedra comprising the mesh.
  //
  //      +Z
  //       |   -X
  //       |   /
  //       v3 v5
  //       | /
  //       |/
  //     v0+-----v2---+Y
  //      /|
  //     / |
  //   v1  v4
  //   /   |
  // +X    |
  //      -Z
  //
  const VolumeMesh<double> test_mesh(
      // The four tetrahedra share the edge with vertex indices 0,2.
      std::vector<VolumeElement>{
          {0, 1, 2, 3}, {1, 0, 2, 4}, {0, 2, 5, 3}, {2, 0, 5, 4}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            -Vector3d::UnitZ(), -Vector3d::UnitX()});
  ASSERT_EQ(DetectTetrahedronWithAllBoundaryVertices(test_mesh).size(), 4);
  ASSERT_EQ(DetectInteriorTriangleWithAllBoundaryVertices(test_mesh).size(), 4);
  ASSERT_EQ(DetectInteriorEdgeWithAllBoundaryVertices(test_mesh).size(), 1);
  ASSERT_EQ(test_mesh.num_vertices(), 6);
  ASSERT_EQ(test_mesh.num_elements(), 4);

  VolumeMeshRefiner refiner(test_mesh);
  VolumeMesh<double> refined_mesh = refiner.Refine();

  EXPECT_EQ(DetectTetrahedronWithAllBoundaryVertices(refined_mesh).size(), 0);
  EXPECT_EQ(DetectInteriorTriangleWithAllBoundaryVertices(refined_mesh).size(),
            0);
  EXPECT_EQ(DetectInteriorEdgeWithAllBoundaryVertices(refined_mesh).size(), 0);

  // Expected properties of the result mesh
  constexpr int kExpectedNumVerts = 7;
  constexpr int kExpectedNumTets = 8;
  constexpr int kNewVertexIndex = kExpectedNumVerts - 1;
  EXPECT_EQ(refined_mesh.num_vertices(), kExpectedNumVerts);
  EXPECT_EQ(refined_mesh.num_elements(), kExpectedNumTets);
  EXPECT_EQ(refined_mesh.vertex(kNewVertexIndex), Vector3d(0, 0.5, 0));
  for (const VolumeElement& tetrahedron : refined_mesh.tetrahedra()) {
    ASSERT_TRUE(tetrahedron.vertex(0) == kNewVertexIndex ||
                tetrahedron.vertex(1) == kNewVertexIndex ||
                tetrahedron.vertex(2) == kNewVertexIndex ||
                tetrahedron.vertex(3) == kNewVertexIndex);
  }
}

// Test a special case that the input mesh is good already. It should return
// an equivalent mesh.
GTEST_TEST(VolumeMeshRefinerTest, InputGoodAlready) {
  //      +Z
  //       |
  //       v3
  //       |
  //       |  v4 = (v0 + v1 + v2 + v3) / 4
  //     v0+------v2---+Y
  //      /
  //     /
  //   v1
  //   /
  // +X
  //
  const VolumeMesh<double> test_mesh(
      std::vector<VolumeElement>{
          {0, 1, 2, 4}, {0, 3, 1, 4}, {3, 2, 1, 4}, {3, 0, 2, 4}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            Vector3d{0.25, 0.25, 0.25}});
  ASSERT_EQ(DetectTetrahedronWithAllBoundaryVertices(test_mesh).size(), 0);
  ASSERT_EQ(DetectInteriorTriangleWithAllBoundaryVertices(test_mesh).size(), 0);
  ASSERT_EQ(DetectInteriorEdgeWithAllBoundaryVertices(test_mesh).size(), 0);
  ASSERT_EQ(test_mesh.num_vertices(), 5);
  ASSERT_EQ(test_mesh.num_elements(), 4);

  VolumeMeshRefiner refiner(test_mesh);
  VolumeMesh<double> refined_mesh = refiner.Refine();

  EXPECT_TRUE(refined_mesh.Equal(test_mesh));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
