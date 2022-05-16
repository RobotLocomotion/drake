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
  const std::vector<std::array<int, 3>> expect_faces{
      {1, 3, 0}, {4, 1, 0}, {3, 2, 0}, {2, 4, 0}, {1, 2, 3}, {2, 1, 4}};
  EXPECT_EQ(expect_faces, boundary_faces);
}

GTEST_TEST(VolumeToSurfaceMeshTest, CollectUniqueVertices) {
  const std::vector<std::array<int, 3>> faces{{1, 2, 3}, {2, 3, 4}};

  const std::vector<int> unique_vertices = CollectUniqueVertices(faces);

  // We copy the result into a `set` so that the test is independent of the
  // ordering of vertices in `unique_vertices`.
  const std::set<int> vertex_set(unique_vertices.begin(),
                                 unique_vertices.end());
  // Check that there are no repeated vertices in `unique_vertices`.
  EXPECT_EQ(vertex_set.size(), unique_vertices.size());
  const std::set<int> expect_vertex_set{1, 2, 3, 4};
  EXPECT_EQ(expect_vertex_set, vertex_set);
}

}  // namespace
}  // namespace internal

namespace {
template <typename T>
void TestVolumeToSurfaceMesh() {
  const Box box(0.2, 0.4, 0.6);
  const double target_edge_length = 0.1;
  const auto volume = internal::MakeBoxVolumeMesh<T>(box, target_edge_length);

  // Go through the vertices on the boundary of the volume mesh. Keep their
  // coordinates in a `set` for comparison with vertices of the surface
  // mesh. We define `Coords` as `tuple<T, T, T>` instead of using Vector3<T>,
  // so we can use `set`.
  using Coords = std::tuple<T, T, T>;
  std::set<Coords> boundary_vertex_coords;
  for (const Vector3<T>& r_MV : volume.vertices()) {
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
  for (const Vector3<T>& r_MV : surface.vertices()) {
    surface_vertex_coords.insert(Coords(r_MV.x(), r_MV.y(), r_MV.z()));
  }
  EXPECT_EQ(boundary_vertex_coords, surface_vertex_coords);

  // Check that `surface` has no duplicated vertices.
  EXPECT_EQ(surface_vertex_coords.size(), surface.num_vertices());

  // Check that the face normal vectors are in the outward direction.
  for (int f = 0; f < surface.num_triangles(); ++f) {
    // Position vector of the first vertex V of the face.
    const Vector3<T> r_MV =
        surface.vertex(surface.element(f).vertex(0));
    EXPECT_GT(surface.face_normal(f).dot(r_MV), T(0.0));
  }
}

GTEST_TEST(VolumeToSurfaceMeshTest, TestDouble) {
  TestVolumeToSurfaceMesh<double>();
}

GTEST_TEST(VolumeToSurfaceMeshTest, TestAutoDiffXd) {
  TestVolumeToSurfaceMesh<AutoDiffXd>();
}

}  // namespace
}  // namespace geometry
}  // namespace drake
