#include "drake/multibody/shapes/geometry.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"

namespace DrakeShapes {
namespace {

const char kUri[] = "";  // No specific URI is required for these tests.

void CheckAllNormalsFaceOutwards(Eigen::Matrix3Xd verts,
                                 TrianglesVector faces) {
  // This checks that all faces have outward-facing normals when
  // the points are considered to be oriented clockwise.
  for (size_t i = 0; i < faces.size(); i++) {
    Eigen::Vector3d pt_a = verts.col(faces[i][0]);
    Eigen::Vector3d pt_b = verts.col(faces[i][1]);
    Eigen::Vector3d pt_c = verts.col(faces[i][2]);
    // Since points are clockwise, pt_c - pt_a should be
    // "rightward" of pt_b - pt_a, so this cross product
    // should face "outward" from the face, as detected via
    // dot product with a point on the face.
    double norm_sign = pt_a.transpose() *
      ((pt_c - pt_a).cross(pt_b - pt_a));

    EXPECT_GT(norm_sign, 0);
  }
}
// Tests that we can query faces from a box.
GTEST_TEST(FaceQueryTests, FaceQueryFromBox) {
  // Tests that we can extract faces from a cube.
  Box box(Eigen::Vector3d(1.0, 2.0, 3.0));

  // Do vertex extraction.
  Eigen::Matrix3Xd verts;
  box.getPoints(verts);

  // Do the actual face extraction.
  EXPECT_TRUE(box.hasFaces());
  TrianglesVector faces;
  box.getFaces(faces);

  // We should have (6 faces * 2 tris per face = 12) faces.
  EXPECT_EQ(faces.size(),  12);

  CheckAllNormalsFaceOutwards(verts, faces);
}

// Tests that we can query faces from a mesh.
GTEST_TEST(FaceQueryTests, FaceQueryFromMesh) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/tri_cube.obj";
  Mesh mesh(kUri, kFileName);

  // Do vertex extraction.
  Eigen::Matrix3Xd verts;
  mesh.getPoints(verts);

  // Do the actual face extraction.
  EXPECT_TRUE(mesh.hasFaces());
  TrianglesVector faces;
  mesh.getFaces(faces);

  // We should have (6 faces * 2 tris per face = 12) faces.
  EXPECT_EQ(faces.size(), 12);
}

}  // namespace
}  // namespace DrakeShapes
