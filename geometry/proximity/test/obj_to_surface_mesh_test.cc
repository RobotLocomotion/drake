#include "drake/geometry/proximity/obj_to_surface_mesh.h"

#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Tests TinyObjToSurfaceVertices through ReadObjToSurfaceMesh. We cannot
// test TinyObjToSurfaceVertices directly because we hide tinyobj from
// public API.
GTEST_TEST(ObjToSurfaceMeshTest, TinyObjToSurfaceVertices) {
  std::istringstream test_stream {
      "v  1.0 -1.0 -1.0\n"
      "v  1.0 -1.0  1.0\n"
      "v -1.0 -1.0  1.0\n"
      "f 1 2 3\n"};
  for (const double scale : {1.0, 2.0, 5.0}) {
    // Seek to the beginning of the stream in each iteration.
    test_stream.seekg(0, test_stream.beg);

    const std::vector<SurfaceVertex<double>> surface_vertices(
        ReadObjToSurfaceMesh(&test_stream, scale).vertices());

    EXPECT_EQ(3, surface_vertices.size());
    const std::vector<Vector3<double>> expect_vertices{
        scale * Vector3<double>{1.0, -1.0, -1.0},  // first vertex.
        scale * Vector3<double>{1.0, -1.0, 1.0},   // second vertex.
        scale * Vector3<double>{-1.0, -1.0, 1.0}   // third vertex.
    };

    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(expect_vertices[i], surface_vertices[i].r_MV());
    }
  }
}


// Tests TinyObjToSurfaceFaces through ReadObjToSurfaceMesh. We cannot test
// TinyObjToSurfaceFaces directly because we hide tinyobj from public API.
GTEST_TEST(ObjToSurfaceMeshTest, TinyObjToSurfaceFaces) {
  std::istringstream test_stream{
      "v  1.0 -1.0 -1.0\n"
      "v  1.0 -1.0  1.0\n"
      "v -1.0 -1.0  1.0\n"
      "v -1.0 -1.0 -1.0\n"
      "f 1 2 3\n"
      "f 1 3 4\n"};

  const std::vector<SurfaceFace> surface_faces(
      ReadObjToSurfaceMesh(&test_stream).faces());

  EXPECT_EQ(2, surface_faces.size());
  // Vertex indices in obj file start with 1, but vertex indices in our
  // SurfaceMesh start with 0.
  const int expect_faces[2][3]{{0, 1, 2}, {0, 2, 3}};
  auto face_equal = [](const SurfaceFace& f, const SurfaceFace& g) -> bool {
    return std::make_tuple(f.vertex(0), f.vertex(1), f.vertex(2)) ==
        std::make_tuple(g.vertex(0), g.vertex(1), g.vertex(2));
  };
  for (int i = 0; i < 2; ++i) {
    EXPECT_TRUE(face_equal(SurfaceFace(expect_faces[i]), surface_faces[i]));
  }
}

GTEST_TEST(ObjToSurfaceMeshTest, ReadObjToSurfaceMesh) {
  const std::string filename =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  SurfaceMesh<double> surface = ReadObjToSurfaceMesh(filename);

  ASSERT_EQ(surface.num_vertices(), 8);
  ASSERT_EQ(surface.num_faces(), 12);

  // This test relies on the specific content of the file quad_cube.obj.
  // These coordinates came from the first section of quad_cube.obj.
  // clang-format off
  std::vector<Vector3<double>> expect_vertices {
      { 1.000000, -1.000000, -1.000000},
      { 1.000000, -1.000000,  1.000000},
      {-1.000000, -1.000000,  1.000000},
      {-1.000000, -1.000000, -1.000000},
      { 1.000000,  1.000000, -1.000000},
      { 1.000000,  1.000000,  1.000001},
      {-1.000000,  1.000000,  1.000000},
      {-1.000000,  1.000000, -1.000000}
  };
  // clang-format on

  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(expect_vertices[i], surface.vertex(SurfaceVertexIndex(i)).r_MV());
  }

  // This test relies on the specific content of the file quad_cube.obj. The
  // last section of quad_cube.obj describes the six square faces of the cube.
  // We expect that each square is subdivided into two triangles. Note that
  // vertex indices in the file quad_cube.obj are from 1 to 8, but tinyobj
  // has vertex indices from 0 to 7. We assume that tinyobj subdivides a
  // polygon into a triangle fan around the first vertex; polygon ABCDE...
  // becomes triangles ABC, ACD, ADE, etc.
  int expect_faces[12][3]{
      {0, 1, 2}, {0, 2, 3},  // face 1 2 3 4 in quad_cube.obj
      {4, 7, 6}, {4, 6, 5},  // face 5 8 7 6 in quad_cube.obj
      {0, 4, 5}, {0, 5, 1},  // face 1 5 6 2 in quad_cube.obj
      {1, 5, 6}, {1, 6, 2},  // face 2 6 7 3 in quad_cube.obj
      {2, 6, 7}, {2, 7, 3},  // face 3 7 8 4 in quad_cube.obj
      {4, 0, 3}, {4, 3, 7}   // face 5 1 4 8 in quad_cube.obj
  };

  auto face_equal = [](const SurfaceFace& f, const SurfaceFace& g) -> bool {
    return std::make_tuple(f.vertex(0), f.vertex(1), f.vertex(2)) ==
           std::make_tuple(g.vertex(0), g.vertex(1), g.vertex(2));
  };
  for (int i = 0; i < 12; ++i) {
    EXPECT_TRUE(face_equal(SurfaceFace(expect_faces[i]),
                           surface.element(SurfaceFaceIndex(i))));
  }
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionInvalidFilePath) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      ReadObjToSurfaceMesh(std::string("invalid_file_path")),
      std::runtime_error, "Cannot open file 'invalid_file_path'");
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionForEmptyObj) {
  std::istringstream empty("");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ReadObjToSurfaceMesh(&empty), std::runtime_error,
      ".*must have one and only one object defined in it. Found 0 objects.");
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionForMultipleObjects) {
  std::istringstream two_objects{
      "o first_object\n"
      "v 1.0 0.0 0.0\n"
      "v 0.0 1.0 0.0\n"
      "v 0.0 0.0 1.0\n"
      "f 1 2 3\n"
      "\n"
      "o second_object\n"
      "v 2.0 0.0 0.0\n"
      "v 0.0 2.0 0.0\n"
      "v 0.0 0.0 2.0\n"
      "f 4 5 6\n"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      ReadObjToSurfaceMesh(&two_objects), std::runtime_error,
      ".*must have one and only one object defined in it. Found 2 objects.");
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionForFaceOutsideObject) {
  std::istringstream face_outside_o_statement{
      "# extra face\n"
      "v 3.0 0.0 0.0\n"
      "v 0.0 3.0 0.0\n"
      "v 0.0 0.0 3.0\n"
      "f 1 2 3\n"
      "\n"
      "o main_object\n"
      "v 1.0 0.0 0.0\n"
      "v 0.0 1.0 0.0\n"
      "v 0.0 0.0 1.0\n"
      "f 4 5 6\n"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      ReadObjToSurfaceMesh(&face_outside_o_statement),
      std::runtime_error,
      ".*must have one and only one object defined in it. Found 2 objects.");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
