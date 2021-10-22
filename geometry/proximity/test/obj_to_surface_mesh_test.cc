#include "drake/geometry/proximity/obj_to_surface_mesh.h"

#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace {

// Tests TinyObjToSurfaceVertices through ReadObjToTriangleSurfaceMesh. We
// cannot test TinyObjToSurfaceVertices directly because we hide tinyobj from
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

    const std::vector<Vector3<double>> surface_vertices(
        ReadObjToTriangleSurfaceMesh(&test_stream, scale).vertices());

    EXPECT_EQ(3, surface_vertices.size());
    const std::vector<Vector3<double>> expect_vertices{
        scale * Vector3<double>{1.0, -1.0, -1.0},  // first vertex.
        scale * Vector3<double>{1.0, -1.0, 1.0},   // second vertex.
        scale * Vector3<double>{-1.0, -1.0, 1.0}   // third vertex.
    };

    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(expect_vertices[i], surface_vertices[i]);
    }
  }
}


// Tests TinyObjToSurfaceFaces through ReadObjToTriangleSurfaceMesh. We cannot
// test TinyObjToSurfaceFaces directly because we hide tinyobj from public API.
GTEST_TEST(ObjToSurfaceMeshTest, TinyObjToSurfaceFaces) {
  std::istringstream test_stream{
      "v  1.0 -1.0 -1.0\n"
      "v  1.0 -1.0  1.0\n"
      "v -1.0 -1.0  1.0\n"
      "v -1.0 -1.0 -1.0\n"
      "f 1 2 3\n"
      "f 1 3 4\n"};

  const std::vector<SurfaceTriangle> surface_faces(
      ReadObjToTriangleSurfaceMesh(&test_stream).triangles());

  EXPECT_EQ(2, surface_faces.size());
  // Vertex indices in obj file start with 1, but vertex indices in our
  // TriangleSurfaceMesh start with 0.
  const int expect_faces[2][3]{{0, 1, 2}, {0, 2, 3}};
  auto face_equal = [](const SurfaceTriangle& f,
                       const SurfaceTriangle& g) -> bool {
    return std::make_tuple(f.vertex(0), f.vertex(1), f.vertex(2)) ==
           std::make_tuple(g.vertex(0), g.vertex(1), g.vertex(2));
  };
  for (int i = 0; i < 2; ++i) {
    EXPECT_TRUE(face_equal(SurfaceTriangle(expect_faces[i]), surface_faces[i]));
  }
}

GTEST_TEST(ObjToSurfaceMeshTest, ReadObjToTriangleSurfaceMesh) {
  const std::string filename =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  TriangleSurfaceMesh<double> surface = ReadObjToTriangleSurfaceMesh(filename);

  ASSERT_EQ(surface.num_vertices(), 8);
  ASSERT_EQ(surface.num_triangles(), 12);

  // This test relies on the specific content of the file quad_cube.obj.
  // These coordinates came from the first section of quad_cube.obj.
  // clang-format off
  std::vector<Vector3<double>> expect_vertices {
      { 1.000000, -1.000000, -1.000000},
      { 1.000000, -1.000000,  1.000000},
      {-1.000000, -1.000000,  1.000000},
      {-1.000000, -1.000000, -1.000000},
      { 1.000000,  1.000000, -1.000000},
      { 1.000000,  1.000000,  1.000000},
      {-1.000000,  1.000000,  1.000000},
      {-1.000000,  1.000000, -1.000000}
  };
  // clang-format on

  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(expect_vertices[i], surface.vertex(i));
  }

  // TODO(SeanCurtis-TRI) Devise a formulation of this that is less sensitive
  //  to the details of the triangulation algorithm.
  // This test is brittle. It depends on:
  //   - the exact construction of quad faces in quad_cube.obj.
  //   - the algorithm that tinyobj uses to triangulate n-gons into triangles.
  // While we have 100% control over the former, we have no control over the
  // latter. History has shown that the algorithm changes. In the past, it
  // would create a fan around the *first* declared vertex, this current
  // encoding creates a fan around the second. In other words, it's the
  // difference between the following two triangulations
  //
  //    ┌───┐   ┌───┐
  //    │╲  │   │  ╱│
  //    │ ╲ │   │ ╱ │
  //    │  ╲│   │╱  │
  //    └───┘   └───┘
  //
  // This test may fail with subsequent updates to the triangulation algorithm.
  int expect_faces[12][3] {
      {0, 1, 3}, {1, 2, 3},  // face 1 2 3 4 in quad_cube.obj
      {4, 7, 5}, {7, 6, 5},  // face 5 8 7 6 in quad_cube.obj
      {0, 4, 1}, {4, 5, 1},  // face 1 5 6 2 in quad_cube.obj
      {1, 5, 2}, {5, 6, 2},  // face 2 6 7 3 in quad_cube.obj
      {2, 6, 3}, {6, 7, 3},  // face 3 7 8 4 in quad_cube.obj
      {4, 0, 7}, {0, 3, 7}   // face 5 1 4 8 in quad_cube.obj
  };

  auto face_equal = [](const SurfaceTriangle& f,
                       const SurfaceTriangle& g) -> ::testing::AssertionResult {
    const auto f_indices =
        std::make_tuple(f.vertex(0), f.vertex(1), f.vertex(2));
    const auto g_indices =
        std::make_tuple(g.vertex(0), g.vertex(1), g.vertex(2));
    if (f_indices == g_indices) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure()
           << "\n  Expected: " << f.vertex(0) << ", " << f.vertex(1) << ", "
           << f.vertex(2) << "\n  Found: " << g.vertex(0) << ", " << g.vertex(1)
           << ", " << g.vertex(2);
  };

  for (int i = 0; i < 12; ++i) {
    EXPECT_TRUE(
        face_equal(SurfaceTriangle(expect_faces[i]), surface.element(i)));
  }
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionInvalidFilePath) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      ReadObjToTriangleSurfaceMesh(std::string("invalid_file_path")),
      std::runtime_error, "Cannot open file 'invalid_file_path'");
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionForEmptyFile) {
  std::istringstream empty("");
  DRAKE_EXPECT_THROWS_MESSAGE(ReadObjToTriangleSurfaceMesh(&empty),
                              std::runtime_error,
                              "The Wavefront obj file has no faces.");
}

void FailOnWarning(std::string_view message) {
  throw std::runtime_error(fmt::format("FailOnWarning: {}", message));
}

GTEST_TEST(ObjToSurfaceMeshTest, WarningCallback) {
  // This *.obj file refers to a separate *.mtl file.  In various cases below,
  // this may cause warnings from the parser.
  const std::string filename =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");

  // When loaded as a stream (such that the *.mtl file is missing) with
  // a defaulted callback, we will drake::log() but not throw.
  {
    std::ifstream input(filename);
    EXPECT_NO_THROW(ReadObjToTriangleSurfaceMesh(&input, 1.0));
  }

  // When loaded as a stream (such that the *.mtl file is missing), the user-
  // provided callback may choose to throw, and our test stub callback does so.
  {
    std::ifstream input(filename);
    DRAKE_EXPECT_THROWS_MESSAGE(
        ReadObjToTriangleSurfaceMesh(&input, 1.0, &FailOnWarning),
        "FailOnWarning: Warning parsing Wavefront obj file : "
        ".*CubeMaterial.*not found.*");
  }

  // When parsing using a filename, we are able to locate the *.mtl file with
  // no warnings.
  EXPECT_NO_THROW(ReadObjToTriangleSurfaceMesh(filename, 1.0, &FailOnWarning));
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionFileHasNoFaces) {
  std::istringstream no_faces{R"(
v 1.0 0.0 0.0
v 0.0 1.0 0.0
v 0.0 0.0 1.0
)"};
  DRAKE_EXPECT_THROWS_MESSAGE(ReadObjToTriangleSurfaceMesh(&no_faces),
                              std::runtime_error,
                              "The Wavefront obj file has no faces.");
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionObjectHasNoFaces) {
  std::istringstream no_faces{R"(
o object_without_faces
v 1.0 0.0 0.0
v 0.0 1.0 0.0
v 0.0 0.0 1.0
)"};
  DRAKE_EXPECT_THROWS_MESSAGE(ReadObjToTriangleSurfaceMesh(&no_faces),
                              std::runtime_error,
                              "The Wavefront obj file has no faces.");
}

// Confirms that we can accept an obj file with faces (f lines) without
// objects (o lines).
GTEST_TEST(ObjToSurfaceMeshTest, AcceptFacesWithoutObject) {
  std::istringstream faces_without_objects{R"(
v 1.0 0.0 0.0
v 0.0 1.0 0.0
v 0.0 0.0 1.0
f 1 2 3
)"};
  TriangleSurfaceMesh<double> surface =
      ReadObjToTriangleSurfaceMesh(&faces_without_objects);
  ASSERT_EQ(3, surface.num_vertices());
  std::vector<Vector3<double>> expect_vertices {
    {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, { 0.0, 0.0, 1.0 }
  };
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(expect_vertices[i], surface.vertex(i));
  }
  ASSERT_EQ(1, surface.num_triangles());
  int expect_face[3] = {0, 1, 2};
  for (int v = 0; v < 3; ++v) {
    EXPECT_EQ(expect_face[v], surface.element(0).vertex(v));
  }
}

// Confirms that we can accept multiple objects in one obj file.
GTEST_TEST(ObjToSurfaceMeshTest, AcceptMultipleObjects) {
  std::istringstream two_objects{R"(
o first_object
v 1.0 0.0 0.0
v 0.0 1.0 0.0
v 0.0 0.0 1.0
f 1 2 3

o second_object
v 2.0 0.0 0.0
v 0.0 2.0 0.0
v 0.0 0.0 2.0
f 4 5 6
)"};
  TriangleSurfaceMesh<double> surface =
      ReadObjToTriangleSurfaceMesh(&two_objects);
  ASSERT_EQ(6, surface.num_vertices());
  std::vector<Vector3<double>> expect_vertices{
      {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
      {2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 2.0}};
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(expect_vertices[i], surface.vertex(i));
  }
  ASSERT_EQ(2, surface.num_triangles());
  int expect_faces[2][3]{{0, 1, 2}, {3, 4, 5}};
  for (int f = 0; f < 2; ++f) {
    for (int v = 0; v < 3; ++v) {
      EXPECT_EQ(expect_faces[f][v], surface.element(f).vertex(v));
    }
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
