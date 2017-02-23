#include "drake/multibody/shapes/geometry.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"

namespace DrakeShapes {
namespace {

const char * kUri = "";  // No specific URI is required for these tests.
const double kVertexSameThreshold = 1E-10;  // Threshold for vertices in the
                                            // test cases being the same.

// This function checks that the specified vertex is present in the
// vertex set.
void CheckPointInVertexSet(Eigen::Matrix3Xd verts,
                           Eigen::Vector3d target_vert) {
  // This expression returns the distance to the nearest neighbor,
  // in terms of Euclidean distance (squaredNorm), to target_vert in verts.
  double min_dist =
    (verts.colwise() - target_vert).colwise().squaredNorm().minCoeff();
  EXPECT_LT(min_dist, kVertexSameThreshold);
}

// This function checks that all faces in the supplied mesh have normals
// that face outward from the origin when the points in the faces are
// considered to be oriented clockwise.
// You can use this function to check if all normals are oriented outwards
// for a CONVEX MESH CONTAINING ON THE ORIGIN. You can't use it to perform
// that check for non-convex meshes, or for ones that don't contain the origin.
void CheckAllNormalsFaceOutwards(Eigen::Matrix3Xd verts,
                                 TrianglesVector faces) {
  for (const auto& face : faces) {
    Eigen::Vector3d pt_a = verts.col(face[0]);
    Eigen::Vector3d pt_b = verts.col(face[1]);
    Eigen::Vector3d pt_c = verts.col(face[2]);
    // Since points are clockwise, pt_c - pt_a should be
    // "rightward" of pt_b - pt_a, so this cross product
    // should face "outward" from the face, as detected via
    // dot product with a point on the face.
    double norm_sign = pt_a.transpose() *
      ((pt_b - pt_a).cross(pt_c - pt_a));

    EXPECT_GT(norm_sign, 0);
  }
}

// This checks that a given set of triangles represents a closed shape
// (of nonzero volume) by confirming that each edge that appears,
// appears exactly twice. If this is true across all
// faces, then the shape is a closed manifold (i.e. watertight).
// This check does NOT test that neighboring faces have normals facing
// the same direction.
//
// Pass the expected number of unique edges as expected_unique_edges,
// or -1 if unknown.
void CheckThatTriMeshIsClosed(Eigen::Matrix3Xd verts,
                              TrianglesVector faces,
                              int expected_unique_edges) {
  // Create an edge structure which stores the indices of the two vertices
  // in ascending order (a always <= b).
  struct Edge {
    int a;
    int b;
    int count = 1;
    Edge(int a_in, int b_in) {
      a = std::min(a_in, b_in);
      b = std::max(a_in, b_in);
    }
    bool operator==(const Edge e) const {
      return (e.a == a && e.b == b);
    }
  };

  std::vector<Edge> edges;
  std::vector<int> edge_counts;
  // For each face...
  for (const auto& face : faces) {
    // and each edge in that face as we go around the face...
    for (int k = 0; k < 3; k ++) {
      Edge new_edge(face[k], face[(k+1)%3]);

      // Check distance against all known edges.
      bool is_new_edge = true;
      for (auto& edge : edges) {
        if (edge == new_edge) {
          is_new_edge = false;
          edge.count++;
          break;
        }
      }

      // If this edge doesn't exist already, start keeping
      // track of it.
      if (is_new_edge) {
        edges.push_back(new_edge);
      }
    }
  }

  for (const auto& edge : edges) {
    EXPECT_EQ(edge.count, 2);
  }

  if (expected_unique_edges >= 0) {
    EXPECT_EQ(edges.size(), expected_unique_edges);
  }
}

// Tests that we can query faces from a box, and that those faces
// satisfy a few conditions:
//   - 12 faces are returned (2 triangles per rectangular box face).
//   - The faces form a closed, non-overlapping shape. See
//     CheckThatTriMeshIsClosed for details of this.
//   - Assuming face vertices are returned in ccw order when viewed
//     from the outside, all face normals should face outwards from
//     the inside of the box.
GTEST_TEST(FaceQueryTests, FaceQueryFromBox) {
  Box box(Eigen::Vector3d(1.0, 2.0, 3.0));

  // Do vertex extraction.
  Eigen::Matrix3Xd verts;
  box.getPoints(verts);

  // Given our cube dimensions above and that the cube is centered on the
  // origin, these vertices should be present:
  const double x = 1.0 / 2.0;
  const double y = 2.0 / 2.0;
  const double z = 3.0 / 2.0;
  CheckPointInVertexSet(verts, Eigen::Vector3d(x,  y,  z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(x,  y, -z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(x, -y,  z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(x, -y, -z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(-x,  y,  z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(-x,  y, -z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(-x, -y,  z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(-x, -y, -z));

  // Do the actual face extraction.
  EXPECT_TRUE(box.hasFaces());
  TrianglesVector faces;
  box.getFaces(faces);

  // We should have (6 faces * 2 tris per face = 12) faces.
  EXPECT_EQ(faces.size(),  12);

  // Since our box mesh is centered on the origin, all normals
  // should face outwards from the origin if they are correctly
  // oriented outwards.
  CheckAllNormalsFaceOutwards(verts, faces);

  // We expect 18 unique edges on a triangulated cube.
  CheckThatTriMeshIsClosed(verts, faces, 18);
}

// Tests that we can query faces from a mesh. This test uses a box
// mesh, so the same conditions that are checked in FaceQueryFromBox
// are employed:
//   - 12 faces are returned (2 triangles per rectangular box face).
//   - The faces form a closed, non-overlapping shape. See
//     CheckThatTriMeshIsClosed for details of this.
//   - Assuming face vertices are returned in ccw order when viewed
//     from the outside, all face normals should face outwards from
//     the inside of the box.
GTEST_TEST(FaceQueryTests, FaceQueryFromMesh) {
  const std::string kFileName = drake::GetDrakePath() +
      "/multibody/shapes/test/tri_cube.obj";
  Mesh mesh(kUri, kFileName);

  // Do vertex extraction.
  Eigen::Matrix3Xd verts;
  mesh.getPoints(verts);

  // This OBJ file contains a cube of side length 2, centered
  // at the origin. So these vertices should be present:
  const double x = 2.0 / 2.0;
  const double y = 2.0 / 2.0;
  const double z = 2.0 / 2.0;
  CheckPointInVertexSet(verts, Eigen::Vector3d(x,  y,  z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(x,  y, -z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(x, -y,  z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(x, -y, -z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(-x,  y,  z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(-x,  y, -z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(-x, -y,  z));
  CheckPointInVertexSet(verts, Eigen::Vector3d(-x, -y, -z));

  // Do the actual face extraction.
  EXPECT_TRUE(mesh.hasFaces());
  TrianglesVector faces;
  mesh.getFaces(faces);

  // We should have (6 faces * 2 tris per face = 12) faces.
  EXPECT_EQ(faces.size(), 12);

  // Since our box mesh is centered on the origin, all normals
  // should face outwards from the origin if they are correctly
  // oriented outwards.
  CheckAllNormalsFaceOutwards(verts, faces);

  // We expect 18 unique edges on a triangulated cube.
  CheckThatTriMeshIsClosed(verts, faces, 18);
}

}  // namespace
}  // namespace DrakeShapes
