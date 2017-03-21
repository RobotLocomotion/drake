/* clang-format off */
#include "drake/multibody/shapes/geometry.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"

namespace DrakeShapes {
namespace {

const char * kUri = "";  // No specific URI is required for these tests.

// Loads an origin-centered 2x2x2 cube from an .obj file,
// applies different scaling in the x, y, and z axes, and
// checks that the resulting vertex coordinates are correctly
// scaled.
GTEST_TEST(MeshScalingTests, ScaleCubeMesh) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/tri_cube.obj";
  Mesh mesh(kUri, kFileName);

  // Scale each axis differently, and apply no scaling to
  // the y-axis.
  // TODO(gizatt) `scale_` should not be public and should
  // be modified either in the Mesh constructor or by a
  // setter.
  mesh.scale_ = Eigen::Vector3d(2.0, 1.0, 0.5);

  // Calling getPoints() causes the mesh file to be parsed to populate
  // the vertex data. So, vertices extracted by this call will
  // have our new scaling applied.
  Eigen::Matrix3Xd verts;
  mesh.getPoints(verts);

  // Because this cube is centered on the origin,
  // and has side length 2, the vertices will be
  // symmetrically arranged around the origin at
  // distances along each axis equal to the scale factor
  // along that axis.
  for (int i = 0; i < verts.cols(); i++) {
    EXPECT_EQ(fabs(verts(0, i)), 2.0);
    EXPECT_EQ(fabs(verts(1, i)), 1.0);
    EXPECT_EQ(fabs(verts(2, i)), 0.5);
  }

  // Do the same check as above, but using the more complete
  // LoadObjFile Mesh method.
  PointsVector vertices;
  TrianglesVector triangles;
  mesh.LoadObjFile(&vertices, &triangles,
                   Mesh::TriangulatePolicy::kFailOnNonTri);
  // As above, given our scaling, we confirm that vertices
  // have been scaled in the appropriate directions.
  for (size_t i = 0; i < vertices.size(); i++) {
    EXPECT_EQ(fabs(vertices[i][0]), 2.0);
    EXPECT_EQ(fabs(vertices[i][1]), 1.0);
    EXPECT_EQ(fabs(vertices[i][2]), 0.5);
  }
}
}  // namespace
}  // namespace DrakeShapes
