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

GTEST_TEST(CreateSubMeshTest, TestCorrectOrderAndUnique) {
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

  // Out of order and duplicated.
  const std::vector<int> tetrahedron_indices {3, 1, 3};

  const VolumeMesh<double> sub_mesh =
      CreateSubMesh(test_mesh, tetrahedron_indices);

  const VolumeMesh<double> expect_mesh(
      std::vector<VolumeElement>{{0, 3, 1, 4},   // 1 in test_mesh
                                 {3, 0, 2, 4}},  // 3 in test_mesh
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            Vector3d{0.25, 0.25, 0.25}});
  EXPECT_TRUE(sub_mesh.Equal(expect_mesh));
}

GTEST_TEST(RemoveUnusedVerticesTest, OneUnused) {
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
  // Vertex 2 is unused.
  const VolumeMesh<double> test_mesh(
      std::vector<VolumeElement>{{0, 3, 1, 4}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            Vector3d{0.25, 0.25, 0.25}});

  const VolumeMesh<double> output_mesh = RemoveUnusedVertices(test_mesh);

  // Input vertex      Expected output vertex       Vector3d
  //      0                     0                 Vector3d::Zero
  //      1                     1                 Vector3d::UnitX
  //      2 unused
  //      3                     2                 Vector3d::UnitZ
  //      4                     3                 Vector3d{0.25, 0.25, 0.25}
  //
  // Input tetrahedron    Expected output tetrahedron
  //   0 3 1 4              0 2 1 3
  const VolumeMesh<double> expect_mesh(
      std::vector<VolumeElement>{{0, 2, 1, 3}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitZ(), Vector3d{0.25, 0.25, 0.25}});

  EXPECT_TRUE(output_mesh.Equal(expect_mesh));
}

GTEST_TEST(RemoveUnusedVerticesTest, NoUnused) {
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
  // Two tetrahedra are enough to use all five vertices.
  const VolumeMesh<double> test_mesh(
      std::vector<VolumeElement>{{0, 1, 2, 4},
                                 {0, 3, 1, 4}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ(),
                            Vector3d{0.25, 0.25, 0.25}});

  const VolumeMesh<double> output_mesh = RemoveUnusedVertices(test_mesh);

  EXPECT_TRUE(output_mesh.Equal(test_mesh));
}

GTEST_TEST(CutZSubMeshTest, UpperHalf) {
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

  const VolumeMesh<double> upper_half = CutZSubMesh(test_mesh, 0, 1);

  const VolumeMesh<double> expect_mesh(
      std::vector<VolumeElement>{{0, 1, 2, 3}},
      std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                            Vector3d::UnitY(), Vector3d::UnitZ()});

  EXPECT_TRUE(upper_half.Equal(expect_mesh));
}

GTEST_TEST(MiscellaneousTest, CutCylinder_ResolutionHalf) {
  const VolumeMesh<double> test_mesh =
      internal::MakeCylinderVolumeMesh<double>(Cylinder(1, 1), 0.5);

  const VolumeMesh<double> cut_mesh = CutZSubMesh(test_mesh, 0, 0.25);
  internal::WriteVolumeMeshToVtk(
      "cylinder0.5.vtk", cut_mesh,
      "Thin cylinder mesh from CutZSubMesh Z from 0 to 0.25");

  std::vector<int> zero_tetrahedron = DetectNullTetrahedron(cut_mesh);
  EXPECT_EQ(zero_tetrahedron.size(), 44);
  internal::WriteVolumeMeshToVtk(
      "cylinder0.5_NullTet.vtk", CreateSubMesh(cut_mesh, zero_tetrahedron),
      "Null tetrahedron in cylinder mesh from CutZSubMesh Z from 0 to 0.25");
}

GTEST_TEST(MiscellaneousTest, CutCylinder_ResolutionQuarter) {
  const VolumeMesh<double> test_mesh =
      internal::MakeCylinderVolumeMesh<double>(Cylinder(1, 1), 0.25);

  const VolumeMesh<double> cut_mesh = CutZSubMesh(test_mesh, 0, 0.25);
  internal::WriteVolumeMeshToVtk(
      "cylinder0.25.vtk", cut_mesh,
      "Thin cylinder mesh from CutZSubMesh Z from 0 to 0.25");

  std::vector<int> zero_tetrahedron = DetectNullTetrahedron(cut_mesh);
  EXPECT_EQ(zero_tetrahedron.size(), 108);
  internal::WriteVolumeMeshToVtk(
      "cylinder0.25_NullTet.vtk", CreateSubMesh(cut_mesh, zero_tetrahedron),
      "Null tetrahedron in cylinder mesh from CutZSubMesh Z from 0 to 0.25");
}


}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
