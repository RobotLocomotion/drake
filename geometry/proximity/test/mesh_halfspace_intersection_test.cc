#include "drake/geometry/proximity/mesh_halfspace_intersection.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace {

typedef double T;

// The following tests all use halfspace normal [0 0 1], with point [0 0 0]
// lying on the halfspace.
const Vector3<T> normal_H(0, 0, 1);
const Vector3<T> point_H(0, 0, 0);
const T d = normal_H.dot(point_H);

// Verifies that a triangle that lies fully outside of the halfspace yields an
// empty intersection. This covers Case (10) in the code.
GTEST_TEST(MeshHalfspaceIntersectionTest, NoIntersection) {
  // Construct the vertices of the triangle to lie outside the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1)
  };

  // Verify no intersection.
  std::vector<Vector3<T>> newly_created_vertices;
  std::vector<MeshIndex> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(triangle_vertices_H, normal_H,
                                                d, &newly_created_vertices,
                                                &intersection_indices);
  EXPECT_TRUE(newly_created_vertices.empty());
  EXPECT_TRUE(intersection_indices.empty());
}

// Verifies that a triangle that lies inside or on the halfspace yields
// that same triangle. This covers Cases (1) and (4) in the code.
GTEST_TEST(MeshHalfspaceIntersectionTest, InsideOrOnIntersection) {
  // Construct the vertices of the triangle to lie inside the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_inside_H = {
    Vector3<T>(0, 0, -1), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection.
  std::vector<Vector3<T>> newly_created_vertices;
  std::vector<MeshIndex> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      triangle_vertices_inside_H, normal_H, d, &newly_created_vertices,
      &intersection_indices);
  EXPECT_TRUE(newly_created_vertices.empty());
  ASSERT_EQ(intersection_indices.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(intersection_indices[i].index, i);
    EXPECT_EQ(intersection_indices[i].index_into_triangle_vertices, true);
  }

  // Construct the vertices of the triangle to lie on the halfspace, using a
  // distance very close to zero.
  const T near_zero = std::numeric_limits<double>::epsilon();
  const std::array<Vector3<T>, 3> triangle_vertices_on_H = {
    Vector3<T>(0, 0, near_zero), Vector3<T>(1, 0, near_zero),
    Vector3<T>(0, 1, near_zero)
  };

  // Verify the intersection, loosening the tolerance only slightly so the
  // test will pass.
  newly_created_vertices.clear();
  intersection_indices.clear();
  ConstructTriangleHalfspaceIntersectionPolygon(
      triangle_vertices_on_H, normal_H, d, &newly_created_vertices,
      &intersection_indices);
  EXPECT_TRUE(newly_created_vertices.empty());
  ASSERT_EQ(intersection_indices.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(intersection_indices[i].index, i);
    EXPECT_EQ(intersection_indices[i].index_into_triangle_vertices, true);
  }
}

// Verifies that a triangle that has exactly one vertex lying on the halfspace
// produces either (a) no intersection (if the other two vertices lie outside the
// halfspace) or (b) the original triangle (if the other two vertices lie
// within the halfspace). This covers Cases (9) and (2), respectively.
GTEST_TEST(MeshHalfspaceIntersectionTest, VertexOnHalfspaceIntersection) {
  // Construct two vertices of the triangle to lie outside the halfspace and the
  // other to lie on the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_two_outside_H = {
    Vector3<T>(0, 0, 0), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1)
  };

  // Verify that there is no intersection.
  std::vector<Vector3<T>> newly_created_vertices;
  std::vector<MeshIndex> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      triangle_vertices_two_outside_H, normal_H, d, &newly_created_vertices,
      &intersection_indices);
  EXPECT_TRUE(newly_created_vertices.empty());
  EXPECT_TRUE(intersection_indices.empty());

  // Construct two vertices of the triangle to lie inside the halfspace and the
  // other to lie on the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_two_inside_H = {
    Vector3<T>(0, 0, 0), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection.
  newly_created_vertices.clear();
  intersection_indices.clear();
  ConstructTriangleHalfspaceIntersectionPolygon(
      triangle_vertices_two_inside_H, normal_H, d, &newly_created_vertices,
      &intersection_indices);
  EXPECT_TRUE(newly_created_vertices.empty());
  ASSERT_EQ(intersection_indices.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(intersection_indices[i].index, i);
    EXPECT_EQ(intersection_indices[i].index_into_triangle_vertices, true);
  }
}

// Verifies that a triangle that has exactly two vertices lying on the halfspace
// produces either (a) no intersection (if the other vertex lies outside the
// halfspace) or (b) the original triangle (if the other vertex lies
// within the halfspace). This covers Cases (7) and (3), respectively.
GTEST_TEST(MeshHalfspaceIntersectionTest, EdgeOnHalfspaceIntersection) {
  // Construct one vertex of the triangle to lie outside the halfspace and the
  // other two to lie on the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_one_outside_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0)
  };

  // Verify that there is no intersection.
  std::vector<Vector3<T>> newly_created_vertices;
  std::vector<MeshIndex> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      triangle_vertices_one_outside_H, normal_H, d, &newly_created_vertices,
      &intersection_indices);
  EXPECT_TRUE(newly_created_vertices.empty());
  EXPECT_TRUE(intersection_indices.empty());

  // Construct one vertex of the triangle to lie inside the halfspace and the
  // other two to lie on the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_one_inside_H = {
    Vector3<T>(0, 0, -1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0)
  };

  // Verify the intersection.
  newly_created_vertices.clear();
  intersection_indices.clear();
  ConstructTriangleHalfspaceIntersectionPolygon(
      triangle_vertices_one_inside_H, normal_H, d, &newly_created_vertices,
      &intersection_indices);
  EXPECT_TRUE(newly_created_vertices.empty());
  ASSERT_EQ(intersection_indices.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(intersection_indices[i].index, i);
    EXPECT_EQ(intersection_indices[i].index_into_triangle_vertices, true);
  }
}

// Verifies that a triangle that has two vertices within the halfspace and
// another outside the halfspace produces a quadrilateral. This covers Case 5.
GTEST_TEST(MeshHalfspaceIntersectionTest, QuadrilateralResults) {
  // Construct one vertex of the triangle to lie outside the halfspace and the
  // other two to lie inside the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection. Note that the following test is overly specific
  // to the logic of the mesh-halfspace intersection routine: it is not actually
  // required that the vertices be output in the order inside.
  std::vector<Vector3<T>> newly_created_vertices;
  std::vector<MeshIndex> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      triangle_vertices_H, normal_H, d, &newly_created_vertices,
      &intersection_indices);
  ASSERT_EQ(intersection_indices.size(), 4);
  ASSERT_EQ(newly_created_vertices.size(), 2);
  // Two vertices will be drawn from inside the halfspace.
  EXPECT_EQ(intersection_indices[0].index, 1);
  EXPECT_EQ(intersection_indices[0].index_into_triangle_vertices, true);
  EXPECT_EQ(intersection_indices[1].index, 2);
  EXPECT_EQ(intersection_indices[1].index_into_triangle_vertices, true);
  // Two vertices will lie halfway between the vertex outside the halfspace
  // and each vertex inside the halfspace.
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_LT((newly_created_vertices[0] -
             (triangle_vertices_H[0] + triangle_vertices_H[2]) * 0.5).norm(),
            zero_tol);
  EXPECT_LT((newly_created_vertices[1] -
             (triangle_vertices_H[0] + triangle_vertices_H[1]) * 0.5).norm(),
            zero_tol);
  EXPECT_EQ(intersection_indices[2].index_into_triangle_vertices, false);
  EXPECT_EQ(intersection_indices[2].index, 0);
  EXPECT_EQ(intersection_indices[3].index_into_triangle_vertices, false);
  EXPECT_EQ(intersection_indices[3].index, 1);
}

// Verifies that a triangle that has one vertex outside the halfspace, one
// vertex inside the halfspace, and one vertex on the halfspace produces a
// triangle. This test covers Case 6.
GTEST_TEST(MeshHalfspaceIntersectionTest, OutsideInsideOn) {
  const std::array<Vector3<T>, 3> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection. Note that the following test is overly specific
  // to the logic of the mesh-halfspace intersection routine: it is not actually
  // required that the vertices be output in the order inside.
  std::vector<Vector3<T>> newly_created_vertices;
  std::vector<MeshIndex> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      triangle_vertices_H, normal_H, d, &newly_created_vertices,
      &intersection_indices);
  ASSERT_EQ(intersection_indices.size(), 3);
  ASSERT_EQ(newly_created_vertices.size(), 1);
  // One vertex will be drawn from inside the halfspace.
  EXPECT_EQ(intersection_indices[1].index_into_triangle_vertices, true);
  EXPECT_EQ(intersection_indices[1].index, 2);
  // Two vertices will be drawn from the surface of the halfspace.
  EXPECT_EQ(intersection_indices[0].index_into_triangle_vertices, true);
  EXPECT_EQ(intersection_indices[0].index, 1);
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_EQ(intersection_indices[2].index_into_triangle_vertices, false);
  EXPECT_EQ(intersection_indices[2].index, 0);
  EXPECT_LT((newly_created_vertices[0] -
             (triangle_vertices_H[0] + triangle_vertices_H[2]) * 0.5).norm(),
            zero_tol);
}

// Verifies that a triangle with one vertex inside the halfspace and two
// vertices outside of the halfspace produces a triangle. This test covers
// Case 8.
GTEST_TEST(MeshHalfspaceIntersectionTest, OneInsideTwoOutside) {
  const std::array<Vector3<T>, 3> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection. Note that the following test is overly specific
  // to the logic of the mesh-halfspace intersection routine: it is not actually
  // required that the vertices be output in the order inside.
  std::vector<Vector3<T>> newly_created_vertices;
  std::vector<MeshIndex> intersection_indices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      triangle_vertices_H, normal_H, d, &newly_created_vertices,
      &intersection_indices);
  ASSERT_EQ(intersection_indices.size(), 3);
  ASSERT_EQ(newly_created_vertices.size(), 2);
  EXPECT_EQ(intersection_indices[0].index_into_triangle_vertices, true);
  EXPECT_EQ(intersection_indices[0].index, 2);
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_EQ(intersection_indices[1].index_into_triangle_vertices, false);
  EXPECT_EQ(intersection_indices[1].index, 0);
  EXPECT_LT((newly_created_vertices[0] -
             (triangle_vertices_H[0] + triangle_vertices_H[2]) * 0.5).norm(),
            zero_tol);
  EXPECT_EQ(intersection_indices[2].index_into_triangle_vertices, false);
  EXPECT_EQ(intersection_indices[2].index, 1);
  EXPECT_LT((newly_created_vertices[1] -
             (triangle_vertices_H[1] + triangle_vertices_H[2]) * 0.5).norm(),
            zero_tol);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
