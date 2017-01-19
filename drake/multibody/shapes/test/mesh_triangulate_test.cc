#include "drake/multibody/shapes/geometry.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"

namespace DrakeShapes {
namespace {

const char kUri[] = "";  // No specific URI is required for these tests.

// Confirms that attempts to load non-triangle meshes by default throws an
// exception.
GTEST_TEST(MeshShapeTests, ParseQuadMeshFail) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/quad_cube.obj";
  Mesh mesh(kUri, kFileName);
  DrakeShapes::PointsVector vertices;
  DrakeShapes::TrianglesVector triangles;
  try {
    mesh.LoadObjFile(&vertices, &triangles, false /*triangulate*/);
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    const std::string kExpectedMessage = "In file \"" + kFileName + "\" (line" +
            " 15). Only triangular faces are supported. However 4 indices are" +
            " provided.";
    EXPECT_EQ(e.what(), kExpectedMessage);
  }
}

// Confirms that the quad mesh is triangulated into a viable alternative
// triangle mesh upon request.
GTEST_TEST(MeshShapeTests, TriangulateQuadMesh) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/quad_cube.obj";
  Mesh mesh(kUri, kFileName);
  DrakeShapes::PointsVector vertices;
  DrakeShapes::TrianglesVector triangles;

  mesh.LoadObjFile(&vertices, &triangles, true /*triangulate*/);

  // This is a *limited* test; it only tests the size of the mesh data and not
  // values.
  // Unchanged number of vertices from the file.
  EXPECT_EQ(vertices.size(), 8u);
  // Six quads become 12 triangles.
  EXPECT_EQ(triangles.size(), 12u);
}

// Confirms that triangle meshes are successfully loaded.
GTEST_TEST(MeshShapeTests, ParseTriMesh) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/tri_cube.obj";
  Mesh mesh(kUri, kFileName);
  DrakeShapes::PointsVector vertices;
  DrakeShapes::TrianglesVector triangles;

  mesh.LoadObjFile(&vertices, &triangles, false /*triangulate*/);

  // This is a *limited* test; it only tests the size of the mesh data and not
  // values.
  EXPECT_EQ(vertices.size(), 8u);
  EXPECT_EQ(triangles.size(), 12u);
}

}  // namespace
}  // namespace DrakeShapes
