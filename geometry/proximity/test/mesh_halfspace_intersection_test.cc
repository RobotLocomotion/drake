#include "drake/geometry/proximity/mesh_halfspace_intersection.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace {

typedef double T;

template <typename T>
class MeshHalfspaceIntersectionTest : public ::testing::Test {
 public:
  void SetUp() {
    // The tests all use halfspace normal [0 0 1], with point [0 0 0]
    // lying on the halfspace.
    normal_H_ = Vector3<T>(0, 0, 1);
    const Vector3<T> point_H(0, 0, 0);
    d_ = normal_H_.dot(point_H);
  }

  /// Returns the normal to the halfspace.
  const Vector3<T>& normal_H() const { return normal_H_; }

  /// Returns the constant d to the halfspace such that normal_H' * x = d,
  /// where x is an arbitrary point on the surface of the halfspace.
  const T& halfspace_constant() const { return d_; }

  /// Returns the default triangle.
  std::array<int, 3> triangle() const { return {0, 1, 2}; }

  private:
    Vector3<T> normal_H_;
    T d_;
};
TYPED_TEST_CASE_P(MeshHalfspaceIntersectionTest);

// Verifies that a triangle that lies fully outside of the halfspace yields an
// empty intersection. This covers Case (10) in the code.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, NoIntersection) {
  // Construct the vertices of the triangle to lie outside the halfspace.
  std::vector<Vector3<T>> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1)
  };

  // Verify no intersection.
  std::unordered_map<SortedPair<int>, int> edges_to_newly_created_vertices;
  std::vector<int> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_H,
      &intersection_indices);
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());
  EXPECT_EQ(triangle_vertices_H.size(), 3);
  EXPECT_TRUE(intersection_indices.empty());
}

// Verifies that a triangle that lies inside or on the halfspace yields
// that same triangle. This covers Cases (1) and (4) in the code.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, InsideOrOnIntersection) {
  // Construct the vertices of the triangle to lie inside the halfspace.
  std::vector<Vector3<T>> triangle_vertices_inside_H = {
    Vector3<T>(0, 0, -1), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection.
  std::unordered_map<SortedPair<int>, int> edges_to_newly_created_vertices;
  std::vector<int> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_inside_H,
      &intersection_indices);
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());
  ASSERT_EQ(intersection_indices.size(), 3);
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(intersection_indices[i], i);

  // Construct the vertices of the triangle to lie on the halfspace, using a
  // distance very close to zero.
  const T near_zero = std::numeric_limits<double>::epsilon();
  std::vector<Vector3<T>> triangle_vertices_on_H = {
    Vector3<T>(0, 0, near_zero), Vector3<T>(1, 0, near_zero),
    Vector3<T>(0, 1, near_zero)
  };

  // Verify the intersection, loosening the tolerance only slightly so the
  // test will pass.
  intersection_indices.clear();
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_on_H,
      &intersection_indices);
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());
  ASSERT_EQ(intersection_indices.size(), 3);
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(intersection_indices[i], i);
}

// Verifies that a triangle that has exactly one vertex lying on the halfspace
// produces either (a) no intersection (if the other two vertices lie outside the
// halfspace) or (b) the original triangle (if the other two vertices lie
// within the halfspace). This covers Cases (9) and (2), respectively.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, VertexOnHalfspaceIntersection) {
  // Construct two vertices of the triangle to lie outside the halfspace and the
  // other to lie on the halfspace.
  std::vector<Vector3<T>> triangle_vertices_two_outside_H = {
    Vector3<T>(0, 0, 0), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1)
  };

  // Verify that there is no intersection.
  std::unordered_map<SortedPair<int>, int> edges_to_newly_created_vertices;
  std::vector<int> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_two_outside_H,
      &intersection_indices);
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());
  EXPECT_TRUE(intersection_indices.empty());

  // Construct two vertices of the triangle to lie inside the halfspace and the
  // other to lie on the halfspace.
  std::vector<Vector3<T>> triangle_vertices_two_inside_H = {
    Vector3<T>(0, 0, 0), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection.
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_two_inside_H,
      &intersection_indices);
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());
  ASSERT_EQ(intersection_indices.size(), 3);
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(intersection_indices[i], i);
}

// Verifies that a triangle that has exactly two vertices lying on the halfspace
// produces either (a) no intersection (if the other vertex lies outside the
// halfspace) or (b) the original triangle (if the other vertex lies
// within the halfspace). This covers Cases (7) and (3), respectively.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, EdgeOnHalfspaceIntersection) {
  // Construct one vertex of the triangle to lie outside the halfspace and the
  // other two to lie on the halfspace.
  std::vector<Vector3<T>> triangle_vertices_one_outside_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0)
  };

  // Verify that there is no intersection.
  std::unordered_map<SortedPair<int>, int> edges_to_newly_created_vertices;
  std::vector<int> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_one_outside_H,
      &intersection_indices);
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());
  EXPECT_TRUE(intersection_indices.empty());

  // Construct one vertex of the triangle to lie inside the halfspace and the
  // other two to lie on the halfspace.
  std::vector<Vector3<T>> triangle_vertices_one_inside_H = {
    Vector3<T>(0, 0, -1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0)
  };

  // Verify the intersection.
  intersection_indices.clear();
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_one_inside_H,
      &intersection_indices);
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());
  ASSERT_EQ(intersection_indices.size(), 3);
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(intersection_indices[i], i);
}

// Verifies that a triangle that has two vertices within the halfspace and
// another outside the halfspace produces a quadrilateral. This covers Case 5.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, QuadrilateralResults) {
  // Construct one vertex of the triangle to lie outside the halfspace and the
  // other two to lie inside the halfspace.
  std::vector<Vector3<T>> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection. Note that the following test is overly specific
  // to the logic of the mesh-halfspace intersection routine: it is not actually
  // required that the vertices be output in the order inside.
  std::unordered_map<SortedPair<int>, int> edges_to_newly_created_vertices;
  std::vector<int> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_H,
      &intersection_indices);
  ASSERT_EQ(triangle_vertices_H.size(), 5);
  ASSERT_EQ(intersection_indices.size(), 4);
  ASSERT_EQ(edges_to_newly_created_vertices.size(), 2);
  EXPECT_EQ(intersection_indices[0], 1);
  EXPECT_EQ(intersection_indices[1], 2);
  EXPECT_EQ(intersection_indices[2], 3);
  EXPECT_EQ(intersection_indices[3], 4);

  // Two vertices will lie halfway between the vertex outside the halfspace
  // and each vertex inside the halfspace.
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_LT((triangle_vertices_H[4] -
             (triangle_vertices_H[0] + triangle_vertices_H[2]) * 0.5).norm(),
            zero_tol);
  EXPECT_LT((triangle_vertices_H[3] -
             (triangle_vertices_H[0] + triangle_vertices_H[1]) * 0.5).norm(),
            zero_tol);

  // TODO(edrumwri): Verify that the polygon winding yields a right handed
  // normal.
}

// Verifies that a triangle that has one vertex outside the halfspace, one
// vertex inside the halfspace, and one vertex on the halfspace produces a
// triangle. This test covers Case 6.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, OutsideInsideOn) {
  std::vector<Vector3<T>> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection. Note that the following test is overly specific
  // to the logic of the mesh-halfspace intersection routine: it is not actually
  // required that the vertices be output in the order inside.
  std::unordered_map<SortedPair<int>, int> edges_to_newly_created_vertices;
  std::vector<int> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_H,
      &intersection_indices);
  ASSERT_EQ(intersection_indices.size(), 3);
  ASSERT_EQ(edges_to_newly_created_vertices.size(), 1);
  ASSERT_EQ(triangle_vertices_H.size(), 4);
  // One vertex will be drawn from inside the halfspace.
  EXPECT_EQ(intersection_indices[1], 2);
  // Two vertices will be drawn from the surface of the halfspace.
  EXPECT_EQ(intersection_indices[0], 1);
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_EQ(intersection_indices[2], 3);
  EXPECT_LT((triangle_vertices_H[3] -
             (triangle_vertices_H[0] + triangle_vertices_H[2]) * 0.5).norm(),
            zero_tol);

  // TODO(edrumwri): Verify that the polygon winding yields a right handed
  // normal.
}

// Verifies that a triangle with one vertex inside the halfspace and two
// vertices outside of the halfspace produces a triangle. This test covers
// Case 8.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, OneInsideTwoOutside) {
  std::vector<Vector3<T>> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection. Note that the following test is overly specific
  // to the logic of the mesh-halfspace intersection routine: it is not actually
  // required that the vertices be output in the order inside.
  std::unordered_map<SortedPair<int>, int> edges_to_newly_created_vertices;
  std::vector<int> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      this->triangle(), this->normal_H(), this->halfspace_constant(),
      &edges_to_newly_created_vertices, &triangle_vertices_H,
      &intersection_indices);
  ASSERT_EQ(intersection_indices.size(), 3);
  ASSERT_EQ(edges_to_newly_created_vertices.size(), 2);
  EXPECT_EQ(intersection_indices[0], 2);
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_EQ(intersection_indices[1], 3);
  EXPECT_LT((triangle_vertices_H[3] -
             (triangle_vertices_H[0] + triangle_vertices_H[2]) * 0.5).norm(),
            zero_tol);
  EXPECT_EQ(intersection_indices[2], 4);
  EXPECT_LT((triangle_vertices_H[4] -
             (triangle_vertices_H[1] + triangle_vertices_H[2]) * 0.5).norm(),
            zero_tol);

  // TODO(edrumwri): Verify that the polygon winding yields a right handed
  // normal.
}
/*

// Tests that a mesh of a box intersecting the halfspace like flotsam produces
// an open box shape.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, BoxMesh) {
  // Set the box vertices according to the following diagram. The box is
  // centered at the origin.
  //               v₁     v₃
  //               ●------●
  //              /|     /|
  //             / |  v₇/ |
  //         v₅ ●------●  |
  //            |  |   |  |
  //            |  ●---|--● v₂
  //            | /v₀  | /
  //            |/     |/
  //    +Z   v₄ ●------● v₆
  //     |
  //     |
  //     o------+Y
  //    /
  //   /
  // +X
  std::vector<Vector3<T>> vertices;
  vertices.emplace_back(-1, -1, -1);
  vertices.emplace_back(-1, -1, 1);
  vertices.emplace_back(-1, 1, -1);
  vertices.emplace_back(-1, 1, 1);
  vertices.emplace_back(1, -1, -1);
  vertices.emplace_back(1, -1, 1);
  vertices.emplace_back(1, 1, -1);
  vertices.emplace_back(1, 1, 1);

  // Set the twelve box faces using a counter-clockwise winding.
  std::vector<std::array<int, 3>> faces;
  faces.push_back({4, 6, 7});  // +X face
  faces.push_back({7, 5, 4});  // +X face
  faces.push_back({1, 3, 2});  // -X face
  faces.push_back({2, 0, 1});  // -X face
  faces.push_back({2, 3, 7});  // +Y face
  faces.push_back({7, 6, 2});  // +Y face
  faces.push_back({4, 5, 1});  // -Y face
  faces.push_back({1, 0, 4});  // -Y face
  faces.push_back({7, 3, 1});  // +Z face
  faces.push_back({1, 5, 7});  // +Z face
  faces.push_back({4, 0, 2});  // -Z face
  faces.push_back({2, 6, 4});  // -Z face

  // Compute the intersection.
  const SurfaceMesh<T> mesh = ConstructSurfaceMeshFromMeshHalfspaceIntersection(
      vertices, faces, this->normal_H(), this->halfspace_constant());
  ASSERT_EQ(mesh.num_vertices(), 8);
  ASSERT_EQ(mesh.num_faces(), 10);
}

*/
REGISTER_TYPED_TEST_CASE_P(MeshHalfspaceIntersectionTest, NoIntersection,
                           InsideOrOnIntersection,
                           VertexOnHalfspaceIntersection,
                           EdgeOnHalfspaceIntersection, QuadrilateralResults,
                           OutsideInsideOn, OneInsideTwoOutside);//, BoxMesh);

typedef ::testing::Types<double> MyTypes;
INSTANTIATE_TYPED_TEST_CASE_P(My, MeshHalfspaceIntersectionTest, MyTypes);

}  // namespace
}  // namespace geometry
}  // namespace drake
