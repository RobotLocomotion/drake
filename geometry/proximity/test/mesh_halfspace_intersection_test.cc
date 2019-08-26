#include "drake/geometry/proximity/mesh_halfspace_intersection.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace {

typedef double T;

// Verifies that a triangle that lies fully outside of the halfspace yields an
// empty intersection. This covers Case (10) in the code.
GTEST_TEST(MeshHalfspaceIntersectionTest, NoIntersection) {
  // Make the halfspace normal point toward [0 0 1], with point [0 0 0] lying
  // on the halfspace.
  const Vector3<T> normal_H(0, 0, 1);
  const Vector3<T> point_H(0, 0, 0);
  const T d = normal_H.dot(point_H);

  // Construct the vertices of the triangle to lie above the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1)
  };

  // Verify no intersection.
  std::vector<Vector3<T>> intersection =
      ConstructTriangleHalfspaceIntersectionPolygon(
          triangle_vertices_H, normal_H, d);
  EXPECT_TRUE(intersection.empty());
}

// Verifies that a triangle that lies inside or on the halfspace yields
// that same triangle. This covers Cases (1) and (4) in the code.
GTEST_TEST(MeshHalfspaceIntersectionTest, InsideOrOnIntersection) {
  // Make the halfspace normal point toward [0 0 1], with point [0 0 0] lying
  // on the halfspace.
  const Vector3<T> normal_H(0, 0, 1);
  const Vector3<T> point_H(0, 0, 0);
  const T d = normal_H.dot(point_H);

  // Construct the vertices of the triangle to lie below the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_inside_H = {
    Vector3<T>(0, 0, -1), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection.
  std::vector<Vector3<T>> intersection =
      ConstructTriangleHalfspaceIntersectionPolygon(
          triangle_vertices_inside_H, normal_H, d);
  ASSERT_EQ(intersection.size(), 3);
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(intersection[i], triangle_vertices_inside_H[i]);

  // Construct the vertices of the triangle to lie on the halfspace, using a
  // distance very close to zero.
  const T near_zero = std::numeric_limits<double>::epsilon();
  const std::array<Vector3<T>, 3> triangle_vertices_on_H = {
    Vector3<T>(0, 0, near_zero), Vector3<T>(1, 0, near_zero),
    Vector3<T>(0, 1, near_zero)
  };

  // Verify the intersection, loosening the tolerance only slightly so the
  // test will pass.
  intersection = ConstructTriangleHalfspaceIntersectionPolygon(
          triangle_vertices_on_H, normal_H, d, near_zero * 2);
  ASSERT_EQ(intersection.size(), 3);
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(intersection[i], triangle_vertices_on_H[i]);
}

// Verifies that a triangle that has exactly one vertex lying on the halfspace
// produces either (a) no intersection (if the other two vertices lie above the
// halfspace) or (b) the original triangle (if the other two vertices lie
// within the halfspace). This covers Cases (9) and (2), respectively.
GTEST_TEST(MeshHalfspaceIntersectionTest, VertexOnHalfspaceIntersection) {
  // Make the halfspace normal point toward [0 0 1], with point [0 0 0] lying
  // on the halfspace.
  const Vector3<T> normal_H(0, 0, 1);
  const Vector3<T> point_H(0, 0, 0);
  const T d = normal_H.dot(point_H);

  // Construct two vertices of the triangle to lie above the halfspace and the
  // other to lie on the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_above_H = {
    Vector3<T>(0, 0, 0), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1)
  };

  // Verify that there is no intersection.
  std::vector<Vector3<T>> intersection =
      ConstructTriangleHalfspaceIntersectionPolygon(
          triangle_vertices_above_H, normal_H, d);
  EXPECT_TRUE(intersection.empty());

  // Construct two vertices of the triangle to lie below the halfspace and the
  // other to lie on the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_below_H = {
    Vector3<T>(0, 0, 0), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection.
  intersection = ConstructTriangleHalfspaceIntersectionPolygon(
          triangle_vertices_below_H, normal_H, d);
  ASSERT_EQ(intersection.size(), 3);
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(intersection[i], triangle_vertices_below_H[i]);
}

// Verifies that a triangle that has exactly two vertices lying on the halfspace
// produces either (a) no intersection (if the other vertex lies above the
// halfspace) or (b) the original triangle (if the other vertex lies
// within the halfspace). This covers Cases (7) and (3), respectively.
GTEST_TEST(MeshHalfspaceIntersectionTest, EdgeOnHalfspaceIntersection) {
  // Make the halfspace normal point toward [0 0 1], with point [0 0 0] lying
  // on the halfspace.
  const Vector3<T> normal_H(0, 0, 1);
  const Vector3<T> point_H(0, 0, 0);
  const T d = normal_H.dot(point_H);

  // Construct one vertex of the triangle to lie above the halfspace and the
  // other two to lie on the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_above_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0)
  };

  // Verify that there is no intersection.
  std::vector<Vector3<T>> intersection =
      ConstructTriangleHalfspaceIntersectionPolygon(
          triangle_vertices_above_H, normal_H, d);
  EXPECT_TRUE(intersection.empty());

  // Construct one vertex of the triangle to lie below the halfspace and the
  // other two to lie on the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_below_H = {
    Vector3<T>(0, 0, -1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0)
  };

  // Verify the intersection.
  intersection = ConstructTriangleHalfspaceIntersectionPolygon(
          triangle_vertices_below_H, normal_H, d);
  ASSERT_EQ(intersection.size(), 3);
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(intersection[i], triangle_vertices_below_H[i]);
}

// Verifies that a triangle that has two vertices within the halfspace and
// another above the halfspace produces a quadrilateral.
GTEST_TEST(MeshHalfspaceIntersectionTest, QuadrilateralResults) {
  // Make the halfspace normal point toward [0 0 1], with point [0 0 0] lying
  // on the halfspace.
  const Vector3<T> normal_H(0, 0, 1);
  const Vector3<T> point_H(0, 0, 0);
  const T d = normal_H.dot(point_H);

  // Construct one vertex of the triangle to lie above the halfspace and the
  // other two to lie below the halfspace.
  const std::array<Vector3<T>, 3> triangle_vertices_H = {
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1)
  };

  // Verify the intersection.
  const std::vector<Vector3<T>> intersection =
      ConstructTriangleHalfspaceIntersectionPolygon(triangle_vertices_H,
                                                    normal_H, d);
  ASSERT_EQ(intersection.size(), 4);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
