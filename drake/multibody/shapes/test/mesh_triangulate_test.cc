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

// Tests the triangulation code's ability to detect when triangulation leads
// to overlapping triangles.  Confirms that an exception is thrown.
GTEST_TEST(MeshShapeTests, DetectBadTriangulation) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/concave_face_bad.obj";
  Mesh mesh(kUri, kFileName);
  DrakeShapes::PointsVector vertices;
  DrakeShapes::TrianglesVector triangles;

  try {
    mesh.LoadObjFile(&vertices, &triangles, true /*triangulate*/);
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    const std::string kExpectedMessage = "Unable to triangulate obj in file '" +
        kFileName + ". See log for details.";
    EXPECT_EQ(e.what(), kExpectedMessage);
  }
}

// Tests the triangulation code's ability to detect when triangulation leads
// to overlapping triangles.  Confirms that an exception is thrown.
GTEST_TEST(MeshShapeTests, TriangulateConcaveFaceSuccess) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/concave_face_good.obj";
  Mesh mesh(kUri, kFileName);
  DrakeShapes::PointsVector vertices;
  DrakeShapes::TrianglesVector triangles;

  mesh.LoadObjFile(&vertices, &triangles, true /*triangulate*/);

  // This is a *limited* test; it only tests the size of the mesh data and not
  // values.
  // Unchanged number of vertices from the file.
  EXPECT_EQ(vertices.size(), 5u);
  // four triangles and 1 quad become 6 triangles.
  EXPECT_EQ(triangles.size(), 6u);
}

// Tests the triangulation code's ability to respond appropriately when a face
// references a vertex that hasn't been parsed yet.  (Unlikely, but possible
// within the obj file specification.)
GTEST_TEST(MeshShapeTests, DetectTriangulateParseOrderError) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/out_of_order_vertex.obj";
  Mesh mesh(kUri, kFileName);
  DrakeShapes::PointsVector vertices;
  DrakeShapes::TrianglesVector triangles;

  try {
    mesh.LoadObjFile(&vertices, &triangles, true /*triangulate*/);
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    const std::string kExpectedMessage = "Unable to triangulate obj in file '" +
        kFileName + ". See log for details.";
    EXPECT_EQ(e.what(), kExpectedMessage);
  }
}

// Tests the triangulation code's ability to detect a degenerate triangle and
// respond with an exception.
GTEST_TEST(MeshShapeTests, DetectTriangulateDegenerateTriangle) {
  // Tests for collinear vertices.
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/collinear_vertex.obj";
  Mesh mesh(kUri, kFileName);
  DrakeShapes::PointsVector vertices;
  DrakeShapes::TrianglesVector triangles;

  try {
    mesh.LoadObjFile(&vertices, &triangles, true /*triangulate*/);
    // An exception *should* be thrown; no exception implies test failure.
    EXPECT_TRUE(false);
  } catch (std::runtime_error& e) {
    const std::string kExpectedMessage = "Unable to triangulate obj in file '" +
        kFileName + ". See log for details.";
    EXPECT_EQ(e.what(), kExpectedMessage);
  }

  // Tests for "tiny" triangles.
  const std::string kFileName2 = drake::GetDrakePath() +
      "/multibody/shapes/test/tiny_triangle.obj";
  Mesh mesh2(kUri, kFileName2);

  try {
    mesh2.LoadObjFile(&vertices, &triangles, true /*triangulate*/);
    // An exception *should* be thrown; no exception implies test failure.
    EXPECT_TRUE(false);
  } catch (std::runtime_error& e) {
    const std::string kExpectedMessage = "Unable to triangulate obj in file '" +
        kFileName2 + ". See log for details.";
    EXPECT_EQ(e.what(), kExpectedMessage);
  }
}

}  // namespace
}  // namespace DrakeShapes
