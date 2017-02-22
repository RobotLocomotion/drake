#include "drake/multibody/shapes/geometry.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"

namespace DrakeShapes {
namespace {

const char kUri[] = "";  // No specific URI is required for these tests.
const double kVertexSameThreshold = 1E-12;  // Threshold for vertices in the
                                            // test cases being the same.

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
      ((pt_b - pt_a).cross(pt_c - pt_a));

    EXPECT_GT(norm_sign, 0);
  }
}

void CheckThatTriMeshIsClosedNonoverlapping(Eigen::Matrix3Xd verts,
                                            TrianglesVector faces) {
  // This checks that a given set of triangles represents a closed shape
  // (of nonzero volume) by confirming that each edge that appears,
  // appears exactly twice. If this is true across all
  // faces, then the shape will have a well-defined inside and outside
  // (without holes or duplicated faces).

  struct Edge {
    Eigen::Vector3d a;
    Eigen::Vector3d b;
  };
  std::vector<Edge> edges;
  std::vector<int> edge_counts;

  // For each face...
  for (size_t i = 0; i < faces.size(); i++) {
    // and each edge in that face...
    for (int k_a = 0; k_a < 3; k_a ++) {
      for (int k_b = 0; k_b < 3; k_b ++) {
        if (k_a != k_b) {
          Edge new_edge;
          new_edge.a = verts.col(faces[i][k_a]);
          new_edge.b = verts.col(faces[i][k_b]);

          // Check distance against all known edges.
          bool is_new_edge = true;
          for (size_t edge_ind = 0; edge_ind < edges.size(); edge_ind++) {
            // Edge comparison is based on distance between
            // the constituent vertices of the pair of edges.
            // We check both vertex orders in case the edges
            // go in opposite directions.
            double dist_1 = (new_edge.a - edges[edge_ind].a).norm() +
                            (new_edge.b - edges[edge_ind].b).norm();
            double dist_2 = (new_edge.b - edges[edge_ind].a).norm() +
                            (new_edge.a - edges[edge_ind].b).norm();
            if (dist_1 < kVertexSameThreshold ||
                dist_2 < kVertexSameThreshold) {
              is_new_edge = false;
              edge_counts[edge_ind]++;
              break;
            }
          }

          // If this edge doesn't exist already, start keeping
          // track of it.
          if (!is_new_edge) {
            edges.push_back(new_edge);
            edge_counts.push_back(1);
          }
        }
      }
    }
  }

  for (auto iter = edge_counts.begin();
            iter != edge_counts.end();
            iter++) {
    EXPECT_EQ(*iter, 2);
  }
}

// Tests that we can query faces from a box.
GTEST_TEST(FaceQueryTests, FaceQueryFromBox) {
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

  // Since our box is centered on the origin, all normals
  // should face outwards from the origin.
  CheckAllNormalsFaceOutwards(verts, faces);

  CheckThatTriMeshIsClosedNonoverlapping(verts, faces);
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

  // Since our box is centered on the origin, all normals
  // should face outwards from the origin.
  CheckAllNormalsFaceOutwards(verts, faces);

  CheckThatTriMeshIsClosedNonoverlapping(verts, faces);
}

}  // namespace
}  // namespace DrakeShapes
