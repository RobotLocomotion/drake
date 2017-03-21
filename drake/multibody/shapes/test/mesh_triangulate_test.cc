/* clang-format off */
#include "drake/multibody/shapes/geometry.h"
/* clang-format on */

#include <gtest/gtest.h>

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
  PointsVector vertices;
  TrianglesVector triangles;
  try {
    mesh.LoadObjFile(&vertices, &triangles,
                     Mesh::TriangulatePolicy::kFailOnNonTri);
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
  PointsVector vertices;
  TrianglesVector triangles;

  mesh.LoadObjFile(&vertices, &triangles, Mesh::TriangulatePolicy::kTry);

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
  PointsVector vertices;
  TrianglesVector triangles;

  mesh.LoadObjFile(&vertices, &triangles,
                   Mesh::TriangulatePolicy::kFailOnNonTri);

  // This is a *limited* test; it only tests the size of the mesh data and not
  // values.
  EXPECT_EQ(vertices.size(), 8u);
  EXPECT_EQ(triangles.size(), 12u);
}

// Confirms that triangle meshes are unchanged, even when triangulation is
// explicitly called for.
GTEST_TEST(MeshShapeTests, TriangulateTriMesh) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/tri_cube.obj";
  Mesh mesh(kUri, kFileName);
  PointsVector vertices;
  TrianglesVector triangles;

  mesh.LoadObjFile(&vertices, &triangles,
                   Mesh::TriangulatePolicy::kTry);

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
  PointsVector vertices;
  TrianglesVector triangles;

  try {
    mesh.LoadObjFile(&vertices, &triangles,
                     Mesh::TriangulatePolicy::kTry);
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    const std::string kExpectedMessage = "Trying to triangulate the face in '" +
        kFileName + "' on line 38 led to bad triangles. The triangle based on "
        "vertices 4, 1, and 2 (1-indexed) is wound in the opposite direction "
        "from the previous triangle. Consider triangulating by hand.";
    EXPECT_EQ(e.what(), kExpectedMessage);
  }
}

// Tests the triangulation code's ability to detect when a face is non-planar
// beyond the allowable threshold.
GTEST_TEST(MeshShapeTests, DetectNonPlanarTriangulation) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/non_planar.obj";
  Mesh mesh(kUri, kFileName);
  PointsVector vertices;
  TrianglesVector triangles;

  try {
  mesh.LoadObjFile(&vertices, &triangles,
      Mesh::TriangulatePolicy::kTry);
  GTEST_FAIL();
  } catch (std::runtime_error& e) {
  const std::string kExpectedMessage = "Trying to triangulate the face in '" +
      kFileName + "' on line 25.  The face is not sufficiently planar. " +
      "Consider triangulating by hand.";
  EXPECT_EQ(e.what(), kExpectedMessage);
  }
}

// Tests the triangulation code's ability to triangulate concave
// faces successfully.
GTEST_TEST(MeshShapeTests, TriangulateConcaveFaceSuccess) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/concave_face_good.obj";
  Mesh mesh(kUri, kFileName);
  PointsVector vertices;
  TrianglesVector triangles;

  mesh.LoadObjFile(&vertices, &triangles, Mesh::TriangulatePolicy::kTry);

  // This is a *limited* test; it only tests the size of the mesh data and not
  // values.
  // Unchanged number of vertices from the file.
  EXPECT_EQ(vertices.size(), 5u);
  // Four triangles and 1 quad become 6 triangles.
  EXPECT_EQ(triangles.size(), 6u);
}

// Tests the triangulation code's response to when a face references a vertex
// that hasn't been parsed yet: throwing a std::runtime_error.  (Unlikely, but
// possible within the OBJ file specification.)
GTEST_TEST(MeshShapeTests, DetectTriangulateParseOrderError) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/out_of_order_vertex.obj";
  Mesh mesh(kUri, kFileName);
  PointsVector vertices;
  TrianglesVector triangles;

  try {
    mesh.LoadObjFile(&vertices, &triangles, Mesh::TriangulatePolicy::kTry);
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    const std::string kExpectedMessage = "Unable to triangulate face in file "
        "'" + kFileName + " on line 7. See log for details.";
    EXPECT_EQ(e.what(), kExpectedMessage);
  }
}

// Tests the triangulation code's ability to detect a degenerate triangle and
// respond with an exception.
GTEST_TEST(MeshShapeTests, DetectTriangulateDegenerateTriangle) {
  // Tests for collinear vertices.
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/colinear_vertex.obj";
  Mesh mesh(kUri, kFileName);
  PointsVector vertices;
  TrianglesVector triangles;

  try {
    mesh.LoadObjFile(&vertices, &triangles, Mesh::TriangulatePolicy::kTry);
    // An exception *should* be thrown; no exception implies test failure.
    // Using this instead of GTEST_FAIL(), so the second half of this test
    // can run, even if this one "fails".
    EXPECT_TRUE(false);
  } catch (std::runtime_error& e) {
    const std::string kExpectedMessage = "Unable to triangulate face in file "
        "'" + kFileName + " on line 8. See log for details.";
    EXPECT_EQ(e.what(), kExpectedMessage);
  }

  // Tests for "tiny" triangles.
  const std::string kFileName2 = drake::GetDrakePath() +
      "/multibody/shapes/test/tiny_triangle.obj";
  Mesh mesh2(kUri, kFileName2);

  try {
    mesh2.LoadObjFile(&vertices, &triangles, Mesh::TriangulatePolicy::kTry);
    // An exception *should* be thrown; no exception implies test failure.
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    const std::string kExpectedMessage = "Unable to triangulate face in file "
        "'" + kFileName2 + " on line 25. See log for details.";
    EXPECT_EQ(e.what(), kExpectedMessage);
  }
}

}  // namespace
}  // namespace DrakeShapes
