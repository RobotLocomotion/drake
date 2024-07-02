#include "drake/geometry/read_obj.h"

#include <unordered_set>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"

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
  // For quads, we expect to see a difference in face counts (and composition).
  for (const bool triangulate : {true, false}) {
    for (const double scale : {1., 0.5, 1.5}) {
      const auto [vertices, faces, num_faces] =
          ReadObjFile(FindResourceOrThrow("drake/geometry/test/quad_cube.obj"),
                      scale, triangulate);
      EXPECT_EQ(vertices->size(), 8);
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

      // The number and composition of the faces depend on triangulation. Each
      // face will have the same number of vertices: V. So, the size of the
      // `faces` data structure will be the expected_face_count * (1 + V).
      // The faces are all perpendicular to one of the frame's axes, so the
      // vertices of each face will have one measure all equal.
      const int expected_face_count = triangulate ? 12 : 6;
      const int face_vertex_count = triangulate ? 3 : 4;
      const int face_stride = 1 + face_vertex_count;

      EXPECT_EQ(num_faces, expected_face_count);
      EXPECT_EQ(faces->size(), num_faces * face_stride);
      for (int i = 0; i < num_faces; ++i) {
        bool found_coord = false;
        for (int coord = 0; coord < 3; ++coord) {
          bool coord_same_value = true;
          for (int vert = 2; vert <= face_vertex_count; ++vert) {
            if ((*vertices)[(*faces)[face_stride * i + vert]](coord) !=
                (*vertices)[(*faces)[face_stride * i + 1]](coord)) {
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
}

// A simple test to exercise the streaming-variant of ReadObjFile. We know that
// the file-variant simply delegates to this API, so this test merely serves
// as a regression test on the API.
GTEST_TEST(ReadObjStreamTest, Regression) {
  std::istringstream ss(R"""(
    v 0 0 0
    v 0 1 0
    v 1 0 0
    v 1 1 0
    f 1 2 3 4
  )""");
  const auto [vertices, faces, num_faces] =
      ReadObjStream(&ss, 2.0, true, "test");
  EXPECT_EQ(vertices->size(), 4);
  EXPECT_EQ(faces->size(), 8);  // Two encoded triangles, 4 indices per tri.
}

GTEST_TEST(ReadObjFile, Octahedron) {
  // The octahedron is already triangulated. We expect the same result whether
  // triangulate is true or false.
  for (const bool triangulate : {true, false}) {
    for (const double scale : {1., 0.5, 1.5}) {
      const auto [vertices, faces, num_faces] =
          ReadObjFile(FindResourceOrThrow("drake/geometry/test/octahedron.obj"),
                      scale, triangulate);
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

// When requesting only vertices, we get only vertices. In fact, we can get the
// vertices from an obj that would ordinarily throw if we asked for the face
// data (see BadObjectCount, below).
GTEST_TEST(ReadObjStream, VertexOnly) {
  std::istringstream ss(R"""(
    v 0 0 0
    v 0 1 0
    v 1 0 0
    v 1 1 0
  )""");
  const auto [vertices, faces, num_faces] =
      ReadObjStream(&ss, 2.0, false, "test", /* vertex_only= */ true);
  EXPECT_EQ(vertices->size(), 4);
  EXPECT_EQ(faces->size(), 0);
  EXPECT_EQ(num_faces, 0);
}

// Test the error conditions in which we have zero or more than one object in
// the obj file.
GTEST_TEST(ReadObjStream, BadObjectCount) {
  {
    std::istringstream ss(R"""(
    v 0 0 0
    v 0 1 0
    v 1 0 0
    v 1 1 0
  )""");
    DRAKE_EXPECT_THROWS_MESSAGE(ReadObjStream(&ss, 2.0, false, "test"),
                                ".*no objects.*");
  }
  {
    std::istringstream ss(R"""(
    v 0 0 0
    v 0 1 0
    v 1 0 0
    v 1 1 0
    o one
    f 1 2 3
    o two
    f 2 3 4
  )""");
    DRAKE_EXPECT_THROWS_MESSAGE(ReadObjStream(&ss, 2.0, false, "test"),
                                ".*multiple objects.*");
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
