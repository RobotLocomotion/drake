#include "drake/geometry/proximity/volume_to_surface_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/geometry/proximity/make_box_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Calculates unit normal vector to a triangular face of a surface mesh. The
// direction of the vector depends on the winding of the face.
template <typename T>
Vector3<T> CalcFaceNormal(const SurfaceMesh<T>& surface,
                          SurfaceFaceIndex face_index) {
// TODO(DamrongGuoy): Consider moving this function into SurfaceMesh by
//  adding a member variable `normal_M_` similar to `area_`. Consequently we
//  will update `normal_M_` when TransformVertices() and ReverseFaceWinding()
//  of SurfaceMesh are called.
  const SurfaceFace& face = surface.element(face_index);
  const Vector3<T>& r_MA = surface.vertex(face.vertex(0)).r_MV();
  const Vector3<T>& r_MB = surface.vertex(face.vertex(1)).r_MV();
  const Vector3<T>& r_MC = surface.vertex(face.vertex(2)).r_MV();
  const auto r_AB_M = r_MB - r_MA;
  const auto r_AC_M = r_MC - r_MA;
  const auto cross_M = r_AB_M.cross(r_AC_M);
  return cross_M.normalized();
}

GTEST_TEST(VolumeToSurfaceMeshTest, IdentifyBoundaryFaces) {
  // Two tetrahedra with their five vertices like this:
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
  const int data[2][4] = {{0, 1, 2, 3}, {0, 2, 1, 4}};
  const std::vector<VolumeElement> tetrahedra{VolumeElement(data[0]),
                                              VolumeElement(data[1])};

  const auto boundary_faces = IdentifyBoundaryFaces(tetrahedra);

  const std::vector<std::array<VolumeVertexIndex, 3>> expect_faces{
      {VolumeVertexIndex(1), VolumeVertexIndex(3), VolumeVertexIndex(0)},
      {VolumeVertexIndex(4), VolumeVertexIndex(1), VolumeVertexIndex(0)},
      {VolumeVertexIndex(3), VolumeVertexIndex(2), VolumeVertexIndex(0)},
      {VolumeVertexIndex(2), VolumeVertexIndex(4), VolumeVertexIndex(0)},
      {VolumeVertexIndex(1), VolumeVertexIndex(2), VolumeVertexIndex(3)},
      {VolumeVertexIndex(2), VolumeVertexIndex(1), VolumeVertexIndex(4)}};
  EXPECT_EQ(expect_faces, boundary_faces);
}

GTEST_TEST(VolumeToSurfaceMeshTest, CollectUniqueVertices) {
  const std::vector<std::array<VolumeVertexIndex, 3>> faces{
      {VolumeVertexIndex(1), VolumeVertexIndex(2), VolumeVertexIndex(3)},
      {VolumeVertexIndex(2), VolumeVertexIndex(3), VolumeVertexIndex(4)}};

  const auto unique_vertices = CollectUniqueVertices(faces);

  const std::vector<VolumeVertexIndex> expect_vertices{
      VolumeVertexIndex(1), VolumeVertexIndex(2), VolumeVertexIndex(3),
      VolumeVertexIndex(4)};
  EXPECT_EQ(expect_vertices, unique_vertices);
}

template <typename T>
void TestVolumeToSurfaceMesh(void) {
  // For this particular box and `target_edge_length`, the volume mesh
  // should have:
  // - 2x4x6 = 48 rectangular cells,
  // - 6x48 = 288 tetrahedra,
  // - 3x5x7 = 105 vertices.
  // Its surface mesh should have:
  // - 2x2x((2x4)+(2x6)+(4x6)) = 176 triangles,
  // - 3x5x7 - 1x3x5 = 105 - 15 = 90 vertices.
  const Box box(0.2, 0.4, 0.6);
  const double target_edge_length = 0.1;
  const auto volume = MakeBoxVolumeMesh<T>(box, target_edge_length);
  ASSERT_EQ(105, volume.num_vertices());
  ASSERT_EQ(288, volume.num_elements());

  const auto surface = ConvertVolumeToSurfaceMesh<T>(volume);

  EXPECT_EQ(90, surface.num_vertices());
  EXPECT_EQ(176, surface.num_faces());

  // Check that the position vector of each surface vertex is the same as
  // the position vector of the corresponding volume vertex.
  SurfaceVertexIndex surface_vertex_index(0);
  VolumeVertexIndex volume_vertex_index(0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 5; ++j) {
      for (int k = 0; k < 7; ++k) {
        if (i == 0 || i == 2 || j == 0 || j == 4 || k == 0 || k == 6) {
          const Vector3<T> volume_r_MV =
              volume.vertex(volume_vertex_index).r_MV();
          const Vector3<T> surface_r_MV =
              surface.vertex(surface_vertex_index).r_MV();
          EXPECT_EQ(volume_r_MV, surface_r_MV);
          ++surface_vertex_index;
        }
        ++volume_vertex_index;
      }
    }
  }
  ASSERT_EQ(volume.num_vertices(), volume_vertex_index);
  ASSERT_EQ(surface.num_vertices(), surface_vertex_index);

  // Check that the face normal vectors are in the outward direction.
  for (SurfaceFaceIndex f(0); f < surface.num_faces(); ++f) {
    const Vector3<T> normal_M = CalcFaceNormal(surface, f);
    // Position vector of the first vertex V of the face.
    const Vector3<T> r_MV =
        surface.vertex(surface.element(f).vertex(0)).r_MV();
    EXPECT_GT(normal_M.dot(r_MV), T(0.0));
  }
}

GTEST_TEST(VolumeToSurfaceMeshTest, TestDouble) {
  TestVolumeToSurfaceMesh<double>();
}

GTEST_TEST(VolumeToSurfaceMeshTest, TestAutoDiffXd) {
  TestVolumeToSurfaceMesh<AutoDiffXd>();
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
