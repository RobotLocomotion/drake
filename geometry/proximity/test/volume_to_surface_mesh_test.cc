#include "drake/geometry/proximity/volume_to_surface_mesh.h"

#include <array>
#include <cmath>
#include <set>
#include <tuple>

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
  const auto rhat_AB_M = r_AB_M.normalized();
  const auto rhat_AC_M = r_AC_M.normalized();
  const auto cross_M = rhat_AB_M.cross(rhat_AC_M);
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

  // TODO(DamrongGuoy): Make this test independent of the ordering of the
  //  triangles and the ordering of the vertices in each triangle. The strategy
  //  could be:
  //  1. For each triangle, put its three vertices into SortedTriplet.
  //  2. Put all SortedTriplet into a `set`.
  //  3. Check the `set`.
  //  4. Check the orientation of all faces.
  //  However, step 4 could be tricky since we do not have positions of
  //  vertices. We have only indices.

  // This test is valid only when the ordering of the triangles and the
  // ordering of the vertices within each triangle are exactly as expected.
  // Changes to the ordering in the code that otherwise defines the same set
  // of triangles would fail.
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

  const std::vector<VolumeVertexIndex> unique_vertices =
      CollectUniqueVertices(faces);

  // We copy the result into a `set` so that the test is independent of the
  // ordering of vertices in `unique_vertices`.
  const std::set<VolumeVertexIndex> vertex_set(unique_vertices.begin(),
                                               unique_vertices.end());
  // Check that there are no repeated vertices in `unique_vertices`.
  EXPECT_EQ(vertex_set.size(), unique_vertices.size());
  const std::set<VolumeVertexIndex> expect_vertex_set{
      VolumeVertexIndex(1), VolumeVertexIndex(2), VolumeVertexIndex(3),
      VolumeVertexIndex(4)};
  EXPECT_EQ(expect_vertex_set, vertex_set);
}

template <typename T>
void TestVolumeToSurfaceMesh() {
  const Box box(0.2, 0.4, 0.6);
  const double target_edge_length = 0.1;
  const auto volume = MakeBoxVolumeMesh<T>(box, target_edge_length);

  // Go through the vertices on the boundary of the volume mesh. Keep their
  // coordinates in a `set` for comparison with vertices of the surface
  // mesh. We define `Coords` as `tuple<T, T, T>` instead of using Vector3<T>,
  // so we can use `set`.
  using Coords = std::tuple<T, T, T>;
  std::set<Coords> boundary_vertex_coords;
  for (const VolumeVertex<T>& vertex : volume.vertices()) {
    const Vector3<T>& r_MV = vertex.r_MV();
    // A vertex is on the boundary of a box when one of its coordinates
    // matches an extremal value.
    using std::abs;
    if (abs(r_MV.x()) == T(0.1) || abs(r_MV.y()) == T(0.2) ||
        abs(r_MV.z()) == T(0.3)) {
      boundary_vertex_coords.insert(Coords(r_MV.x(), r_MV.y(), r_MV.z()));
    }
  }

  const auto surface = ConvertVolumeToSurfaceMesh<T>(volume);

  // Keep coordinates of vertices of the surface mesh in a `set` and
  // check that they are the same as vertices on the boundary of the
  // volume mesh.
  std::set<Coords> surface_vertex_coords;
  for (SurfaceVertexIndex i(0); i < surface.num_vertices(); ++i) {
    const Vector3<T>& r_MV = surface.vertex(i).r_MV();
    surface_vertex_coords.insert(Coords(r_MV.x(), r_MV.y(), r_MV.z()));
  }
  EXPECT_EQ(boundary_vertex_coords, surface_vertex_coords);

  // Check that `surface` has no duplicated vertices.
  EXPECT_EQ(surface_vertex_coords.size(), surface.num_vertices());

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
