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

// We want to simply confirm that triangulate is propagating through. We rely
// on tinyobj to triangulate correctly. We'll just confirm that quads become
// triangles upon request.
GTEST_TEST(ReadObjFile, Triangulate) {
  for (const bool triangulate : {true, false}) {
    const auto [vertices, faces, num_faces] =
        ReadObjFile(FindResourceOrThrow("drake/geometry/test/quad_cube.obj"),
                    /* scale= */ 1, triangulate);
    EXPECT_EQ(vertices->size(), 8);
    EXPECT_EQ(num_faces, triangulate ? 12 : 6);
  }
}

// Performs the test over multiple scales to make sure scale is included.
GTEST_TEST(ReadObjFile, QuadCube) {
  for (const double scale : {0.5, 1.5}) {
    const auto [vertices, faces, num_faces] =
        ReadObjFile(FindResourceOrThrow("drake/geometry/test/quad_cube.obj"),
                    scale, /* triangulate= */ false);
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

// The octahedron.obj is comprised only of triangles; we should get the same
// result regardless of whether triangulate is true or false.
GTEST_TEST(ReadObjFile, TriangulatingTriangles) {
  for (const bool triangulate : {true, false}) {
    const auto [vertices, faces, num_faces] =
        ReadObjFile(FindResourceOrThrow("drake/geometry/test/octahedron.obj"),
                    /* scale= */ 1.0, triangulate);
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
    CheckVertices(vertices, vertices_expected.transpose(), 1E-5);
    EXPECT_EQ(num_faces, 8);
    // Each face has 3 vertices, so faces.size() = num_faces * (1 + 3).
    EXPECT_EQ(faces->size(), num_faces * 4);
  }
}

// Simply tests that multiple objects are supported. Scale and triangulate have
// been tested elsewhere.
GTEST_TEST(ReadObjFile, MultipleObjects) {
  const auto [vertices, faces, num_faces] = ReadObjFile(
      FindResourceOrThrow("drake/geometry/test/two_cube_objects.obj"), 1.0,
      /* triangulate= */ false);
  Eigen::Matrix<double, 16, 3> vertices_expected;
  // clang-format off
    vertices_expected << 1, -1, -1,  // Cube 1 vertices.
                         1, -1, -1,
                         -1, -1, 1,
                         -1, -1, -1,
                         1, 1, -1,
                         1, 1, 1,
                         -1, 1, 1,
                         -1, 1, -1,
                         5, -1, -1,  // Cube 2 vertices.
                         5, -1, 1,
                         3, -1, 1,
                         3, -1, -1,
                         5, 1, -1,
                         5, 1, 1,
                         3, 1, 1,
                         3, 1, -1;
  // clang-format on
  CheckVertices(vertices, vertices_expected.transpose(), 1E-12);
  // Now check the faces - six quads per cube --> 12 faces.
  EXPECT_EQ(num_faces, 12);
  // A face includes size and vertex indices. 5 ints per quad face.
  EXPECT_EQ(faces->size(), num_faces * 5);
  // The expected faces, as defined by indexes to the face's vertices. This
  // uses the 1-indexed values taken directly from two_cube_objects.obj.
  const std::set<std::set<int>> faces_expected{{{1, 2, 3, 4},  // Cube 1.
                                                {5, 8, 7, 6},
                                                {1, 5, 6, 2},
                                                {2, 6, 7, 3},
                                                {3, 7, 8, 4},
                                                {5, 1, 4, 8},
                                                {9, 10, 11, 12},  // Cube 2.
                                                {13, 16, 15, 14},
                                                {9, 13, 14, 10},
                                                {10, 14, 15, 11},
                                                {11, 15, 16, 12},
                                                {13, 9, 12, 16}}};
  for (int i = 0; i < num_faces; ++i) {
    // obj file uses 1-index, so we add 1 for each index stored in `faces`.
    ASSERT_NE(faces_expected.find(std::set<int>({(*faces)[5 * i + 1] + 1,
                                                 (*faces)[5 * i + 2] + 1,
                                                 (*faces)[5 * i + 3] + 1,
                                                 (*faces)[5 * i + 4] + 1})),
              faces_expected.end());
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

// Test the error conditions in which we have no faces.
GTEST_TEST(ReadObjStream, EmpyObj) {
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
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
