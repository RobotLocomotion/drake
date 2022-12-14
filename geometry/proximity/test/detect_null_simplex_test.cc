#include "drake/geometry/proximity/detect_null_simplex.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(DetectNullTetrahedronTest, TestOneNullTetrahedron) {
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

  std::vector<int> zero_tetrahedron = DetectNullTetrahedron(test_mesh);

  ASSERT_EQ(zero_tetrahedron.size(), 1);
  EXPECT_EQ(zero_tetrahedron[0], 0);
}

GTEST_TEST(DetectNullTetrahedronTest, TestNoNullTetrahedron) {
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
      std::vector<VolumeElement>{{0, 1, 2, 4},
                                 {0, 3, 1, 4},
                                 {3, 2, 1, 4},
                                 {3, 0, 2, 4}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            Vector3d{0.25, 0.25, 0.25}});

  std::vector<int> zero_tetrahedron = DetectNullTetrahedron(test_mesh);

  EXPECT_EQ(zero_tetrahedron.size(), 0);
}

GTEST_TEST(DetectNullInteriorTriangleTest, TestOneNullInteriorTriangleTest) {
  // The interior triangle is shared by two tetrahedra comprising the mesh.
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
      std::vector<VolumeElement>{{0, 1, 2, 3}, {0, 2, 1, 4}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            -Vector3d::UnitZ()});

  std::vector<SortedTriplet<int>> zero_triangles = DetectNullInteriorTriangle
      (test_mesh);

  ASSERT_EQ(zero_triangles.size(), 1);
  EXPECT_EQ(zero_triangles[0], SortedTriplet<int>({0, 1, 2}));
}

GTEST_TEST(DetectNullInteriorTriangleTest, TestNoNullInteriorTriangleTest) {
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
      std::vector<VolumeElement>{{0, 1, 2, 4},
                                 {0, 3, 1, 4},
                                 {3, 2, 1, 4},
                                 {3, 0, 2, 4}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            Vector3d{0.25, 0.25, 0.25}});

  std::vector<SortedTriplet<int>> zero_triangles = DetectNullInteriorTriangle
      (test_mesh);
  EXPECT_EQ(zero_triangles.size(), 0);
}

GTEST_TEST(DetectNullInteriorEdgeTest, TestOneNullInteriorEdgeTest) {
  //         +Z
  //          |
  //          v3
  //          |
  //          |
  //        v0+------v2---+Y
  //         /
  //   v4   /
  //      v1
  //      /
  //    +X
  //
  const VolumeMesh<double> test_mesh(
      std::vector<VolumeElement>{{0, 1, 2, 3},
                                 {0, 1, 3, 4},
                                 {0, 1, 4, 2}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            Vector3d(0, -1, -1)});

  std::vector<SortedPair<int>> zero_edges = DetectNullInteriorEdge(test_mesh);
  ASSERT_EQ(zero_edges.size(), 1);
  EXPECT_EQ(zero_edges[0], SortedPair<int>({0, 1}));
}

GTEST_TEST(DetectNullInteriorEdgeTest, TestNoNullInteriorEdgeTest) {
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
      std::vector<VolumeElement>{{0, 1, 2, 4},
                                 {0, 3, 1, 4},
                                 {3, 2, 1, 4},
                                 {3, 0, 2, 4}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            Vector3d{0.25, 0.25, 0.25}});

  std::vector<SortedPair<int>> zero_edges = DetectNullInteriorEdge(test_mesh);
  EXPECT_EQ(zero_edges.size(), 0);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
