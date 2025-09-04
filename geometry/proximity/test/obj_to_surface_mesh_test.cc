#include "drake/geometry/proximity/obj_to_surface_mesh.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace {

namespace fs = std::filesystem;

// Creates the TriangleSurfaceMesh expected when parsing quad_cube.obj scaled
// by the given scale factor. Note: this depends on tinyobj's triangulation
// method. If that changes, this mesh definition may have to adapt. That's fine.
TriangleSurfaceMesh<double> MeshForQuadCube(
    const Vector3<double>& scale = Vector3<double>::Ones()) {
  // clang-format off
  // Vertices copied from the file and then scaled.
  std::vector<Vector3<double>> vertices {
      scale.cwiseProduct(Vector3<double>{ 1.000000, -1.000000, -1.000000}),
      scale.cwiseProduct(Vector3<double>{ 1.000000, -1.000000,  1.000000}),
      scale.cwiseProduct(Vector3<double>{-1.000000, -1.000000,  1.000000}),
      scale.cwiseProduct(Vector3<double>{-1.000000, -1.000000, -1.000000}),
      scale.cwiseProduct(Vector3<double>{ 1.000000,  1.000000, -1.000000}),
      scale.cwiseProduct(Vector3<double>{ 1.000000,  1.000000,  1.000000}),
      scale.cwiseProduct(Vector3<double>{-1.000000,  1.000000,  1.000000}),
      scale.cwiseProduct(Vector3<double>{-1.000000,  1.000000, -1.000000})
  };
  // clang-format on

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
  std::vector<SurfaceTriangle> triangles{
      {0, 1, 3}, {1, 2, 3},  // face 1 2 3 4 in quad_cube.obj
      {4, 7, 5}, {7, 6, 5},  // face 5 8 7 6 in quad_cube.obj
      {0, 4, 1}, {4, 5, 1},  // face 1 5 6 2 in quad_cube.obj
      {1, 5, 2}, {5, 6, 2},  // face 2 6 7 3 in quad_cube.obj
      {2, 6, 3}, {6, 7, 3},  // face 3 7 8 4 in quad_cube.obj
      {4, 0, 7}, {0, 3, 7}   // face 5 1 4 8 in quad_cube.obj
  };
  return TriangleSurfaceMesh<double>(std::move(triangles), std::move(vertices));
}

GTEST_TEST(ObjToSurfaceMeshTest, FromPath) {
  const fs::path filename =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");

  // Uniform scale.
  {
    const TriangleSurfaceMesh<double> surface =
        ReadObjToTriangleSurfaceMesh(filename, 2);

    EXPECT_TRUE(surface.Equal(MeshForQuadCube(Vector3<double>::Constant(2))));
  }

  // Non-uniform scale.
  {
    const Vector3<double> scale(2, 3, 4);
    const TriangleSurfaceMesh<double> surface =
        ReadObjToTriangleSurfaceMesh(filename, scale);

    EXPECT_TRUE(surface.Equal(MeshForQuadCube(scale)));
  }
}

// Tests the MeshSource-based overload. We'll use one kind of source to test
// uniform scale and the other to test non-uniform scale.
GTEST_TEST(ObjToSurfaceMeshTest, MeshSource) {
  constexpr double scale = 2.0;
  const Vector3<double> scale3(2, 3, 4);
  const std::string filename =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  MeshSource disk_source(filename);
  MeshSource memory_source(InMemoryMesh{MemoryFile::Make(filename)});
  const TriangleSurfaceMesh<double> disk_surface =
      ReadObjToTriangleSurfaceMesh(disk_source, scale);
  const TriangleSurfaceMesh<double> memory_surface =
      ReadObjToTriangleSurfaceMesh(memory_source, scale3);

  EXPECT_TRUE(
      disk_surface.Equal(MeshForQuadCube(Vector3<double>::Constant(scale))));
  EXPECT_TRUE(memory_surface.Equal(MeshForQuadCube(scale3)));
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionInvalidFilePath) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      ReadObjToTriangleSurfaceMesh(fs::path("invalid_file_path.obj")),
      ".*cannot read the file 'invalid_file_path.obj'.");
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionForEmptyFile) {
  const MeshSource empty(InMemoryMesh{MemoryFile("", ".obj", "empty")});
  DRAKE_EXPECT_THROWS_MESSAGE(ReadObjToTriangleSurfaceMesh(empty),
                              ".*OBJ data parsed contains no objects.*");
}

void FailOnWarning(std::string_view message) {
  throw std::runtime_error(fmt::format("FailOnWarning: {}", message));
}

GTEST_TEST(ObjToSurfaceMeshTest, WarningCallback) {
  // *One* source of a warning is a degenerate face (fewer than three vertices).
  // We'll show that the warning callback gets invoked for this one warning and
  // consider that as representative for all tinyobj warnings.
  const MeshSource obj(InMemoryMesh{MemoryFile(R"""(
  v 1 0 0
  v 0 1 0
  v 0 0 1
  f 1 2 3
  f 2 3
  )""",
                                               ".obj", "trigger warning")});

  // By not setting the callback, the default warning behavior won't throw.
  EXPECT_NO_THROW(ReadObjToTriangleSurfaceMesh(obj));

  // Now we'll throw on warnings.
  DRAKE_EXPECT_THROWS_MESSAGE(
      ReadObjToTriangleSurfaceMesh(obj, 1.0, &FailOnWarning),
      "[^]*Degenerated face found[^]*");
}

GTEST_TEST(ObjToSurfaceMeshTest, ThrowExceptionFileHasNoFaces) {
  const MeshSource no_faces(
      InMemoryMesh{MemoryFile(R"""(
v 1.0 0.0 0.0
v 0.0 1.0 0.0
v 0.0 0.0 1.0
)""",
                              ".obj", "Obj with no faces")});
  DRAKE_EXPECT_THROWS_MESSAGE(ReadObjToTriangleSurfaceMesh(no_faces),
                              ".*OBJ data parsed contains no objects.*");
}

// Confirms that we can accept an obj file with faces (f lines) without
// objects (o lines).
GTEST_TEST(ObjToSurfaceMeshTest, AcceptFacesWithoutObject) {
  const MeshSource faces_without_objects(
      InMemoryMesh{MemoryFile(R"""(
v 1.0 0.0 0.0
v 0.0 1.0 0.0
v 0.0 0.0 1.0
f 1 2 3
)""",
                              ".obj", "faces without objects")});
  TriangleSurfaceMesh<double> surface =
      ReadObjToTriangleSurfaceMesh(faces_without_objects);
  ASSERT_EQ(3, surface.num_vertices());
  std::vector<Vector3<double>> expect_vertices{
      {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
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
  const MeshSource two_objects(InMemoryMesh{MemoryFile(R"""(
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
)""",
                                                       ".obj", "two objects")});
  TriangleSurfaceMesh<double> surface =
      ReadObjToTriangleSurfaceMesh(two_objects);
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

class ObjToMeshDiagnosticsTest : public test::DiagnosticPolicyTestBase {};

// Calling ReadObjToTriangleSurfaceMesh() is documented as throwing on error.
// However, DoReadObjToSurfaceMesh can take a diagnostic policy that *doesn't*
// throw. It can return nullopt.
TEST_F(ObjToMeshDiagnosticsTest, ErrorModes) {
  const std::string no_face_obj = R"""(
  v 0 0 0
  v 0 1 0
  v 1 0 0
  v 1 1 0
  )""";
  const MeshSource source(
      InMemoryMesh{MemoryFile(no_face_obj, ".obj", "no_faces")});
  const Eigen::Vector3d scale(1, 2, 3);
  auto maybe_mesh =
      internal::DoReadObjToSurfaceMesh(source, scale, diagnostic_policy_);
  ASSERT_FALSE(maybe_mesh.has_value());
  EXPECT_THAT(TakeError(), testing::HasSubstr("no objects"));
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::DoReadObjToSurfaceMesh(source, scale,
                                       drake::internal::DiagnosticPolicy()),
      "[^]*no objects[^]*");
}
}  // namespace
}  // namespace geometry
}  // namespace drake
