#include "drake/geometry/read_obj.h"

#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <unordered_set>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

std::filesystem::path FindPathOrThrow(const std::string& resource_file) {
  return FindResourceOrThrow(resource_file);
}

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
GTEST_TEST(ReadObjTest, Triangulate) {
  const Eigen::Vector3d scale(1, 2, 3);
  for (const bool triangulate : {true, false}) {
    const auto [vertices, faces, num_faces] =
        ReadObj(FindPathOrThrow("drake/geometry/test/quad_cube.obj"), scale,
                triangulate);
    EXPECT_EQ(vertices->size(), 8);
    EXPECT_EQ(num_faces, triangulate ? 12 : 6);
  }
}

// Confirm that the scale factor is included.
GTEST_TEST(ReadObjTest, QuadCube) {
  const double sx = 2;
  const double sy = 3;
  const double sz = 4;
  const Eigen::Vector3d scale(sx, sy, sz);
  const auto [vertices, faces, num_faces] =
      ReadObj(FindPathOrThrow("drake/geometry/test/quad_cube.obj"), scale,
              /* triangulate= */ false);
  EXPECT_EQ(vertices->size(), 8);
  EXPECT_EQ(num_faces, 6);
  Eigen::Matrix<double, 8, 3> vertices_expected;
  // clang-format off
  // The cube obj has +/- 1 in all vertex coordinates.
  vertices_expected << -sx, -sy, -sz,
                       -sx, -sy,  sz,
                       -sx,  sy,  sz,
                       -sx,  sy, -sz,
                        sx,  sy, -sz,
                        sx, -sy, -sz,
                        sx, -sy,  sz,
                        sx,  sy,  sz;
  // clang-format on
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

// The octahedron.obj is comprised only of triangles; we should get the same
// result regardless of whether triangulate is true or false.
GTEST_TEST(ReadObjTest, TriangulatingNoop) {
  const Eigen::Vector3d scale(1, 1, 1);
  for (const bool triangulate : {true, false}) {
    const auto [vertices, faces, num_faces] =
        ReadObj(FindPathOrThrow("drake/geometry/test/octahedron.obj"), scale,
                triangulate);
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
GTEST_TEST(ReadObjTest, MultipleObjects) {
  const Eigen::Vector3d scale(1, 1, 1);
  const auto [vertices, faces, num_faces] = ReadObj(
      FindPathOrThrow("drake/geometry/test/two_cube_objects.obj"), scale,
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
    ASSERT_NE(faces_expected.find(std::set<int>(
                  {(*faces)[5 * i + 1] + 1, (*faces)[5 * i + 2] + 1,
                   (*faces)[5 * i + 3] + 1, (*faces)[5 * i + 4] + 1})),
              faces_expected.end());
  }
}

// A quick reality check that we can successfully invoke with a MeshSource.
// Previous tests have relied on implicit conversion from path to source.
GTEST_TEST(ReadObjTest, MeshSourceRegression) {
  const Eigen::Vector3d scale(2, 2, 2);
  // In-memory source.
  {
    const MeshSource source(InMemoryMesh{MemoryFile(R"""(v 0 0 0
                                                         v 0 1 0
                                                         v 1 0 0
                                                         v 1 1 0
                                                         f 1 2 3 4
                                                       )""",
                                                    ".obj", "test")});
    const auto [vertices, faces, num_faces] =
        ReadObj(source, scale, /* triangulate= */ true);
    EXPECT_EQ(vertices->size(), 4);
    EXPECT_EQ(faces->size(), 8);  // Two encoded triangles, 4 ints per tri.
  }

  // File path source.
  {
    const MeshSource source(
        FindPathOrThrow("drake/geometry/test/quad_cube.obj"));
    const auto [vertices, faces, num_faces] =
        ReadObj(source, scale, /* triangulate= */ false);
    EXPECT_EQ(vertices->size(), 8);
    EXPECT_EQ(faces->size(), 30);  // Six encoded quads, 5 ints per quad.
  }
}

// When requesting only vertices, we get only vertices. In fact, we can get the
// vertices from an obj that would ordinarily throw if we asked for the face
// data (see ErrorModes, below).
GTEST_TEST(ReadObjTest, VertexOnly) {
  const Eigen::Vector3d scale(2, 2, 2);
  const MeshSource source(InMemoryMesh{MemoryFile(R"""(v 0 0 0
                                                       v 0 1 0
                                                       v 1 0 0
                                                       v 1 1 0
                                                     )""",
                                                  ".obj", "no_faces")});
  const auto [vertices, faces, num_faces] = ReadObj(
      source, scale, /* triangulate= */ true, /* vertices_only= */ true);
  EXPECT_EQ(vertices->size(), 4);
  EXPECT_EQ(faces->size(), 0);
  EXPECT_EQ(num_faces, 0);
}

// When the non-uniform scale allows flipping, we want to make sure the faces
// get rewound so that inside is still inside. We know the cube is centered
// on the origin. That means the origin should be on the "inside" of every
// triangle. We'll sample one triangle as evidence that winding got handled.
GTEST_TEST(ReadObjTest, InverseScaleWinding) {
  for (double sx : {-1.0, 1.0}) {
    for (double sy : {-1.0, 1.0}) {
      for (double sz : {-1.0, 1.0}) {
        const Eigen::Vector3d scale(sx, sy, sz);
        const auto [vertices, faces, num_faces] =
            ReadObj(FindPathOrThrow("drake/geometry/test/quad_cube.obj"), scale,
                    /* triangulate= */ true);
        ASSERT_EQ(num_faces, 12);
        // Remember: faces is encoded as [n0, v0_0, v0_1, v0_2, n1, ...].
        const Eigen::Vector3d& p_GoA_G = vertices->at(faces->at(1));
        const Eigen::Vector3d& p_GoB_G = vertices->at(faces->at(2));
        const Eigen::Vector3d& p_GoC_G = vertices->at(faces->at(3));

        const Eigen::Vector3d p_AB_G = p_GoB_G - p_GoA_G;
        const Eigen::Vector3d p_AC_G = p_GoC_G - p_GoA_G;
        const Eigen::Vector3d n_G = p_AB_G.cross(p_AC_G);
        // The vector OA should point in basically the same direction as the
        // normal.
        SCOPED_TRACE(fmt::format("Scale [{}]", fmt_eigen(scale.transpose())));
        EXPECT_GT(n_G.dot(p_GoA_G), 0);
      }
    }
  }
}

class ReadObjDiagnosticsTest : public test::DiagnosticPolicyTestBase {};

// Problems parsing the requested source.
TEST_F(ReadObjDiagnosticsTest, ErrorModes) {
  const Eigen::Vector3d scale(2, 2, 2);
  // tinyobj errors broadcast as diagnostic errors. Without providing a
  // diagnostic policy, it throws.
  {
    const std::string bad_index_obj = R"""(
    v 0 0 0
    v 1 0 0
    v 0 1 0
    # Zero-indexed vertex indices.
    f 0 1 2
    )""";
    const MeshSource source(
        InMemoryMesh{MemoryFile(bad_index_obj, ".obj", "bad indices")});
    auto [verts, _1, _2] =
        ReadObj(source, scale, /* triangulate= */ false,
                /* vertices_only= */ true, diagnostic_policy_);
    EXPECT_EQ(verts, nullptr);
    EXPECT_THAT(TakeError(), testing::HasSubstr("zero value for vertex"));
    DRAKE_EXPECT_THROWS_MESSAGE(ReadObj(source, scale, /* triangulate= */ false,
                                        /* vertices_only= */ true),
                                "[^]*zero value for vertex[^]*");
  }

  // tinyobj warnings broadcast as diagnostic warnings. Without providing a
  // diagnostic policy, a mesh is returned.
  {
    const std::string bad_index_obj = R"""(
    v 0 0 0
    v 1 0 0
    v 0 1 0
    # A vertex too high causes a warning.
    f 1 2 4
    )""";
    const MeshSource source(
        InMemoryMesh{MemoryFile(bad_index_obj, ".obj", "group")});
    auto [verts, _1, _2] =
        ReadObj(source, scale, /* triangulate= */ false,
                /* vertices_only= */ true, diagnostic_policy_);
    EXPECT_EQ(verts->size(), 3);
    EXPECT_THAT(TakeWarning(),
                testing::HasSubstr("Vertex indices out of bounds"));
    // This will write a warning to the console.
    EXPECT_NO_THROW(ReadObj(source, scale, /* triangulate= */ false,
                            /* vertices_only= */ true));
  }

  // There are no faces in an otherwise valid .obj.
  const std::string no_face_obj = R"""(
  v 0 0 0
  v 0 1 0
  v 1 0 0
  v 1 1 0
  )""";
  {
    const MeshSource source(
        InMemoryMesh{MemoryFile(no_face_obj, ".obj", "no_faces")});
    auto [verts, _1, _2] =
        ReadObj(source, scale, /* triangulate= */ false,
                /* vertices_only= */ false, diagnostic_policy_);
    EXPECT_EQ(verts, nullptr);
    EXPECT_THAT(TakeError(), testing::HasSubstr("no objects"));
    DRAKE_EXPECT_THROWS_MESSAGE(ReadObj(source, scale, /* triangulate= */ false,
                                        /* vertices_only= */ false),
                                ".*no objects.*");
  }

  // Not an .obj - from memory.
  {
    const MeshSource source(
        InMemoryMesh{MemoryFile(no_face_obj, ".bad", "wrong_extension")});
    auto [verts, _1, _2] =
        ReadObj(source, scale, /* triangulate= */ false,
                /* vertices_only= */ true, diagnostic_policy_);
    EXPECT_EQ(verts, nullptr);
    EXPECT_THAT(TakeError(), testing::HasSubstr("wrong extension: '.bad'"));
    DRAKE_EXPECT_THROWS_MESSAGE(ReadObj(source, scale, /* triangulate= */ false,
                                        /* vertices_only= */ false),
                                ".*wrong extension.*");
  }

  // Not an .obj - from disk.
  {
    const MeshSource source("some_file.bad");
    auto [verts, _1, _2] =
        ReadObj(source, scale, /* triangulate= */ false,
                /* vertices_only= */ true, diagnostic_policy_);
    EXPECT_EQ(verts, nullptr);
    EXPECT_THAT(TakeError(), testing::HasSubstr("wrong extension: '.bad'"));
    DRAKE_EXPECT_THROWS_MESSAGE(ReadObj(source, scale, /* triangulate= */ false,
                                        /* vertices_only= */ false),
                                ".*wrong extension.*");
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
