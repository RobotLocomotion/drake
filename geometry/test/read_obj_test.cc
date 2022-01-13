#include "drake/geometry/read_obj.h"

#include <unordered_set>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Check if the set of vertices match (not caring about the order).
void CheckVertices(
    const std::shared_ptr<std::vector<Eigen::Vector3d>>& vertices,
    const Eigen::Ref<const Eigen::Matrix3Xd>& vertices_expected, double tol) {
  EXPECT_EQ(vertices->size(), vertices_expected.cols());
  for (int i = 0; i < vertices_expected.cols(); ++i) {
    bool found_match = false;
    for (int j = 0; j < vertices_expected.cols(); ++j) {
      if (((*vertices)[j] - vertices_expected.col(i)).norm() <= tol) {
        found_match = true;
        break;
      }
    }
    EXPECT_TRUE(found_match);
  }
}

GTEST_TEST(ReadObjFile, QuadCube) {
  // TODO(hongkai.dai): add the test with triangulate=true.
  for (const double scale : {1., 0.5, 1.5}) {
    const auto [vertices, faces, num_faces] =
        ReadObjFile(FindResourceOrThrow("drake/geometry/test/quad_cube.obj"),
                    scale, false /*triangulate */);
    EXPECT_EQ(vertices->size(), 8);
    EXPECT_EQ(num_faces, 6);
    Eigen::Matrix<double, 8, 3> vertices_expected;
    // clang-format off
    vertices_expected << -1, -1, -1,
                         -1, -1, 1,
                         -1, 1, 1,
                         -1, 1, -1,
                         1, 1, -1,
                         1, -1, -1,
                         1, -1, 1,
                         1, 1, 1;
    // clang-format on
    vertices_expected *= scale;
    CheckVertices(vertices, vertices_expected.transpose(), 0.);
    // Each face has 4 vertices, hence the size of faces is num_faces * (1 + 4).
    // The point on the same face will have one coordinate with the same value.
    EXPECT_EQ(faces->size(), num_faces * 5);
    for (int i = 0; i < num_faces; ++i) {
      bool found_coord = false;
      for (int coord = 0; coord < 3; ++coord) {
        bool coord_same_value = true;
        for (int vert = 2; vert <= 4; ++vert) {
          if ((*vertices)[(*faces)[5 * i + vert]](coord) !=
              (*vertices)[(*faces)[5 * i + 1]](coord)) {
            coord_same_value = false;
            break;
          }
        }
        if (coord_same_value) {
          found_coord = true;
          break;
        }
      }
      EXPECT_TRUE(found_coord);
    }
  }
}

GTEST_TEST(ReadObjFile, Octahedron) {
  // TODO(hongkai.dai): add the test with triangulate=true. With triangulation,
  // the vertices and faces should be the same as triangulate=false.
  for (const double scale : {1., 0.5, 1.5}) {
    const auto [vertices, faces, num_faces] =
        ReadObjFile(FindResourceOrThrow("drake/geometry/test/octahedron.obj"),
                    scale, false /*triangulate */);
    EXPECT_EQ(vertices->size(), 6u);
    Eigen::Matrix<double, 6, 3> vertices_expected;
    // clang-format off
    vertices_expected << 1, -1, 0,
                         1, 1, 0,
                         -1, 1, 0,
                         -1, -1, 0,
                         0, 0, std::sqrt(2),
                         0, 0, -std::sqrt(2);
    // clang-format on
    vertices_expected *= scale;
    CheckVertices(vertices, vertices_expected.transpose(), 1E-5);
    EXPECT_EQ(num_faces, 8);
    // Each face has 3 vertices, so faces.size() = num_faces * (1 + 3).
    EXPECT_EQ(faces->size(), num_faces * 4);
  }
}

GTEST_TEST(ReadObjFile, NonconvexMesh) {
  for (const double scale : {1., 0.5, 1.5}) {
    const auto [vertices, faces, num_faces] = ReadObjFile(
        FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj"), scale,
        false /*triangulate */);
    Eigen::Matrix<double, 5, 3> vertices_expected;
    // clang-format off
    vertices_expected << 0.0, 0.0, 0.0,
                         1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0,
                         0.2, 0.2, 0.2;
    // clang-format on
    vertices_expected *= scale;
    CheckVertices(vertices, vertices_expected.transpose(), 1E-12);
    // Now check the faces.
    EXPECT_EQ(num_faces, 6);
    EXPECT_EQ(faces->size(), num_faces * 4);
    // The expected face include these vertices (directly copied from
    // non_convex_mesh.obj which uses 1-index).
    // (1, 2, 4)
    // (1, 4, 3)
    // (1, 3, 2)
    // (2, 3, 5)
    // (3, 4, 5)
    // (4, 2, 5)
    const std::set<std::set<int>> faces_expected{
        {{1, 2, 4}, {1, 4, 3}, {1, 3, 2}, {2, 3, 5}, {3, 4, 5}, {4, 2, 5}}};
    for (int i = 0; i < num_faces; ++i) {
      // obj file uses 1-index, so we add 1 for each index stored in `faces`.
      EXPECT_NE(faces_expected.find(std::set<int>({(*faces)[4 * i + 1] + 1,
                                                   (*faces)[4 * i + 2] + 1,
                                                   (*faces)[4 * i + 3] + 1})),
                faces_expected.end());
    }
  }
}
}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
