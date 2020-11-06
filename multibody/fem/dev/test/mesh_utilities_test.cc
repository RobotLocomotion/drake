#include "drake/multibody/fem/dev/mesh_utilities.h"

#include <set>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace {
GTEST_TEST(MeshUtilityTest, CreateRectangularBlockTetMesh) {
  using Eigen::Vector3d;
  // Lexicographic compare for Vector3d that facilitates comparison between two
  // sets of Vector3d's.
  struct Vector3dCompare {
    bool operator()(const Vector3d& lhs, const Vector3d& rhs) const {
      for (int i = 0; i < 3; ++i) {
        if (lhs(i) < rhs(i)) {
          return true;
        }
      }
      return false;
    }
  };
  const int N = 1;
  double h = 0.1;
  geometry::VolumeMesh<double> mesh = CreateRectangularBlockTetMesh(N, N, N, h);
  // There are 8 vertices in a cube and therefore 8 vertices in the tetrahedra
  // that subdivide the cube.
  EXPECT_EQ(mesh.num_vertices(), 8);
  // Verify the vertices are grid aligned and distance `h` apart.
  std::set<Vector3d, Vector3dCompare> mesh_vertices;
  for (int i = 0; i < mesh.num_vertices(); ++i) {
    mesh_vertices.insert(mesh.vertex(geometry::VolumeVertexIndex(i)).r_MV());
  }
  // The expected set of vertices.
  std::set<Vector3d, Vector3dCompare> expected_vertices;
  for (int i = 0; i < N + 1; ++i) {
    for (int j = 0; j < N + 1; ++j) {
      for (int k = 0; k < N + 1; ++k) {
        expected_vertices.insert(Vector3d(i * h, j * h, k * h));
      }
    }
  }
  EXPECT_EQ(mesh_vertices, expected_vertices);
  // There should be 5 tetrahedra subdividing the cube.
  EXPECT_EQ(mesh.num_elements(), 5);
  for (int i = 0; i < mesh.num_elements(); ++i) {
    // Verify that each tetrahedron in the mesh follows the vertex ordering
    // convention.
    EXPECT_GT(mesh.CalcTetrahedronVolume(geometry::VolumeElementIndex(i)), 0);
  }
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
