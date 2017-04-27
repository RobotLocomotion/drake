/* clang-format off */
#include "drake/multibody/shapes/geometry.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"

namespace DrakeShapes {
namespace {

const char * kUri = "";  // No specific URI is required for these tests.
const double kVertexSameThreshold = 1E-10;  // Threshold for vertices in the
                                            // test cases being the same.

// This function checks that the specified vertex is present in the
// vertex set, to threshold kVertexSameThreshold.
void CheckPointInVertexSet(const Eigen::Matrix3Xd& verts,
                           const Eigen::Vector3d& target_vert) {
  // This expression returns the distance to the nearest neighbor,
  // in terms of Euclidean distance (squaredNorm), to target_vert in verts.
  double min_dist =
    (verts.colwise() - target_vert).colwise().squaredNorm().minCoeff();
  EXPECT_LT(min_dist, kVertexSameThreshold);
}

// This function checks that all faces in the supplied mesh have normals
// that face outward from the origin.
// It assumes that:
//    - The points in the faces are listed in counter-clockwise order
//      when viewed from the outside.
//    - The supplied mesh is convex and strictly contains the origin.
void CheckAllNormalsFaceOutwards(const Eigen::Matrix3Xd& verts,
                                 const TrianglesVector& faces) {
  for (const auto& face : faces) {
    Eigen::Vector3d pt_a = verts.col(face[0]);
    Eigen::Vector3d pt_b = verts.col(face[1]);
    Eigen::Vector3d pt_c = verts.col(face[2]);
    // Since points are counterclockwise, pt_b - pt_a should be
    // "rightward" of pt_c - pt_a, so this cross product
    // should face "outward" from the face, as detected via
    // dot product with a point on the face.
    double norm_sign = pt_a.transpose() *
      ((pt_b - pt_a).cross(pt_c - pt_a));

    EXPECT_GT(norm_sign, 0);
  }
}

// This checks that a given set of triangles is a closed manifold
// (of nonzero volume) by confirming that each edge appears exactly twice --
// once with one directionality, and once with the opposite directionality.
// - We recover edge directionality (and face normal direction) by assuming that
//   vertices for a face are listed in CCW order when viewed from the outside.
// - If a pair of faces share a directed edge, and the direction of the edge on
//   each face is in opposition, then the faces face the same direction.
// - If all edges are shared by exactly two faces, then the shape is a closed
//   manifold (i.e. watertight).
// - If all shared edges are of opposing directions, then all faces must also
//   face the same direction.
// Returns the number of unique edges found in the shape.
int CheckThatTriMeshIsClosedAndConsistent(const Eigen::Matrix3Xd& verts,
                                           const TrianglesVector& faces) {
  // Create an edge structure which stores the indices of two vertices,
  // facilitates comparison with other edges, and facilitates order-aware
  // edge counting.
  class Edge {
   public:
    Edge(int a_in, int b_in) {
      a_ = std::min(a_in, b_in);
      b_ = std::max(a_in, b_in);
      if (a_in <= b_in) {
        count_same_ = 1;
      } else {
        count_flipped_ = 1;
      }
    }
    // Compare us with the supplied edge and increment the appropriate
    // identical-edge-count if so. Returns whether the supplied edge
    // is the same as this edge.
    bool try_increment(const Edge& e) {
      if (e.get_a() == a_ && e.get_b() == b_) {
        count_same_ += e.get_count_same();
        count_flipped_ += e.get_count_flipped();
        return true;
      } else {
        return false;
      }
    }
    int get_a() const { return a_; }
    int get_b() const { return b_; }
    int get_count_same() const { return count_same_; }
    int get_count_flipped() const { return count_flipped_; }

   private:
    int a_ = -1;
    int b_ = -1;
    int count_same_ = 0;
    int count_flipped_ = 0;
  };

  std::vector<Edge> edges;
  std::vector<int> edge_counts;
  // For each face...
  for (const auto& face : faces) {
    // and each edge in that face as we go around the face...
    for (int k = 0; k < 3; k ++) {
      Edge new_edge(face[k], face[(k+1)%3]);

      // Check distance against all known edges.
      // This is O(N^2) and inefficient, but OK since our
      // test meshes are all small.
      bool is_new_edge = true;
      for (auto& edge : edges) {
        if (edge.try_increment(new_edge)) {
          is_new_edge = false;
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
    EXPECT_EQ(edge.get_count_same(), 1);
    EXPECT_EQ(edge.get_count_flipped(), 1);
  }

  return edges.size();
}

// Tests that verts and faces queried from a Box of known size satisfy:
//   - 8 vertices are returned at the expected 8 corners of a box of that size.
//   - 12 faces are returned (2 triangles per rectangular box face).
//   - The faces form a closed manifold with consistently outward-oriented
//     normals.
// For these items to be tested, our implementation assumes that:
//   - The box faces list vertices in CCW order when viewed from the outside,
//     so that we can recover normal direction from the faces.
//   - The box is convex and centered on the origin.
GTEST_TEST(FaceQueryTests, FaceQueryFromBox) {
  // These are half-side lengths -- the box extends this far in the
  // corresponding positive and negative axis:
  const double x = 1.0 / 2.0;
  const double y = 2.0 / 2.0;
  const double z = 3.0 / 2.0;
  Box box(Eigen::Vector3d(2.0*x, 2.0*y, 2.0*z));

  // Do vertex extraction.
  Eigen::Matrix3Xd verts;
  box.getPoints(verts);

  // Given our cube dimensions above and that the cube is centered on the
  // origin, these vertices should be present:
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
  box.getFaces(&faces);

  // We should have (6 faces * 2 tris per face = 12) faces.
  EXPECT_EQ(faces.size(),  12);

  // Since our box mesh is centered on the origin, all normals
  // should face outwards from the origin if the triangle
  // definitions have the correct counter-clockwise winding.
  CheckAllNormalsFaceOutwards(verts, faces);

  // We expect 18 unique edges on a triangulated cube.
  int num_edges = CheckThatTriMeshIsClosedAndConsistent(verts, faces);
  EXPECT_EQ(num_edges, 18);
}

// Tests that verts and faces queried from a Box of known size satisfy:
//   - 8 vertices are returned at the expected 8 corners of a box of that size.
//   - 12 faces are returned (2 triangles per rectangular box face).
//   - The faces form a closed manifold with consistently outward-oriented
//     normals.
// For these items to be tested, our implementation assumes that:
//   - The box faces list vertices in CCW order when viewed from the outside,
//     so that we can recover normal direction from the faces.
//   - The box is convex and centered on the origin.
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
  mesh.getFaces(&faces);

  // We should have (6 faces * 2 tris per face = 12) faces.
  EXPECT_EQ(faces.size(), 12);

  // Since our box mesh is centered on the origin, all normals
  // should face outwards from the origin if the triangle
  // definitions have the correct counter-clockwise winding.
  CheckAllNormalsFaceOutwards(verts, faces);

  // We expect 18 unique edges on a triangulated cube.
  int num_edges = CheckThatTriMeshIsClosedAndConsistent(verts, faces);
  EXPECT_EQ(num_edges, 18);
}

}  // namespace
}  // namespace DrakeShapes
