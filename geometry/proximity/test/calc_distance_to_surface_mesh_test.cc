#include "drake/geometry/proximity/calc_distance_to_surface_mesh.h"

#include <gtest/gtest.h>

#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using std::vector;

struct Line {
  Vector3d A;
  Vector3d B;
};

/* We use a different way to calculate distance to a line from that in the .cc
 file to avoid testing with the same code in implementation. Here we assume
 everything is in the same frame. */
double CalcDistanceToLine(const Vector3d& Q, Line l) {
  /* Compute the projection point P of Q on the line and find the distance. */
  const Vector3d p_AB = l.B - l.A;
  const Vector3d dir_AB = p_AB.normalized();
  const Vector3d p_AQ = Q - l.A;
  const Vector3d P = dir_AB.dot(p_AQ) * dir_AB + l.A;
  return (Q - P).norm();
}

double CalcDistanceToPoint(const Vector3d& p_TQ, const Vector3d& p_TA) {
  return (p_TQ - p_TA).norm();
}

/* Returns the distance from the point p_TQ to the xy-plane in the T frame (see
 below). */
double CalcDistanceToTxyPlane(const Vector3d& p_TQ) {
  return std::abs(p_TQ(2));
}

/* We test the distance calculation from various points to a triangle ABC that
 has its three vertices at (0, 0, 0), (1, 0, 0), and (0, 1, 0) when measured and
 expressed in the triangle's frame T.

                     4        /
                            /
                      C   /
              ----------•
                        | \
                y-axis  |   \
                        |     \
                        |       \
                        |         \               /
                3       |    0      \     5     /
                        |             \       /
                        |               \   /
        --------------- • --------------- •    x-axis
                      A |                 | B
                2       |    1            |      6
                        |                 |
 We sample points from regions 0 through 6 (with negative, positive, and zero z
 values) and verify that the calculated distances match analytic results. */
GTEST_TEST(CalcDistanceToSurfaceMeshTest, SingleTriangle) {
  const Vector3d p_TA(0, 0, 0);
  const Vector3d p_TB(1, 0, 0);
  const Vector3d p_TC(0, 1, 0);
  const math::RollPitchYaw<double> rpy_WT(1, 2, 3);
  const Vector3d p_WTo(4, 5, 6);
  const math::RigidTransform<double> X_WT(rpy_WT, p_WTo);

  vector<Vector3d> vertices = {X_WT * p_TA, X_WT * p_TB, X_WT * p_TC};
  vector<SurfaceTriangle> triangles = {{0, 1, 2}};
  TriangleSurfaceMesh<double> mesh_W(std::move(triangles), std::move(vertices));

  constexpr double kEps = 1e-14;
  constexpr double kZScale = 0.5;

  /* For each voronoi region, we arbitrarily pick a point below, above, or on
   the plane the triangle is in, to sample representative relative positions of
   the point from the triangle. */
  // Region 0.
  {
    for (int sign_z : {-1, 0, 1}) {
      const Vector3d p_TQ(0.25, 0.25, sign_z * kZScale);
      const Vector3d p_WQ = X_WT * p_TQ;
      EXPECT_NEAR(CalcDistanceToTxyPlane(p_TQ),
                  CalcDistanceToSurfaceMesh(p_WQ, mesh_W), kEps);
    }
  }
  // Region 1.
  {
    for (int sign_z : {-1, 0, 1}) {
      const Vector3d p_TQ(0.25, -0.25, sign_z * kZScale);
      const Vector3d p_WQ = X_WT * p_TQ;
      EXPECT_NEAR(CalcDistanceToLine(p_TQ, {p_TA, p_TB}),
                  CalcDistanceToSurfaceMesh(p_WQ, mesh_W), kEps);
    }
  }
  // Region 2.
  {
    for (int sign_z : {-1, 0, 1}) {
      const Vector3d p_TQ(-0.25, -0.25, sign_z * kZScale);
      const Vector3d p_WQ = X_WT * p_TQ;
      EXPECT_NEAR(CalcDistanceToPoint(p_TQ, p_TA),
                  CalcDistanceToSurfaceMesh(p_WQ, mesh_W), kEps);
    }
  }
  // Region 3.
  {
    for (int sign_z : {-1, 0, 1}) {
      const Vector3d p_TQ(-0.25, 0.0, sign_z * kZScale);
      const Vector3d p_WQ = X_WT * p_TQ;
      EXPECT_NEAR(CalcDistanceToLine(p_TQ, {p_TA, p_TC}),
                  CalcDistanceToSurfaceMesh(p_WQ, mesh_W), kEps);
    }
  }
  // Region 4.
  {
    for (int sign_z : {-1, 0, 1}) {
      const Vector3d p_TQ(-0.25, 1.5, sign_z * kZScale);
      const Vector3d p_WQ = X_WT * p_TQ;
      EXPECT_NEAR(CalcDistanceToPoint(p_TQ, p_TC),
                  CalcDistanceToSurfaceMesh(p_WQ, mesh_W), kEps);
    }
  }
  // Region 5.
  {
    for (int sign_z : {-1, 0, 1}) {
      const Vector3d p_TQ(1.0, 1.5, sign_z * kZScale);
      const Vector3d p_WQ = X_WT * p_TQ;
      EXPECT_NEAR(CalcDistanceToLine(p_TQ, {p_TB, p_TC}),
                  CalcDistanceToSurfaceMesh(p_WQ, mesh_W), kEps);
    }
  }
  // Region 6.
  {
    for (int sign_z : {-1, 0, 1}) {
      const Vector3d p_TQ(1.2, -0.3, sign_z * kZScale);
      const Vector3d p_WQ = X_WT * p_TQ;
      EXPECT_NEAR(CalcDistanceToPoint(p_TQ, p_TB),
                  CalcDistanceToSurfaceMesh(p_WQ, mesh_W), kEps);
    }
  }
}

/* Confirm that CalcDistanceToSurfaceMesh() is the minimum across all triangles.

 We create a simple mesh with two triangles and query distance at a point Q
 that lies equidistant to the two triangles (in fact, it lies on the boundary
 of the two triangles voronoi regions). If we perturb the query point in either
 direction, the reported distance must decrease.

 The mesh is formed by two triangles forming a tent-like shape. Below, shown
 in two orthogonal views.

                          z                   z
                          ┆             B     ┆     C
                          ○ B, C        ○───────────○
                         ╱┆╲             ╲░░░░┆░░░░╱
                        ╱ ┆ ╲             ╲░░░┆░░░╱
                       ╱  •Q ╲             ╲░░•Q░╱
                      ╱   ┆   ╲             ╲░┆░╱
                    ┄○┄┄┄┄┼┄┄┄┄○┄ x     ┄┄┄┄┄┄○┄┄┄┄┄┄┄ y
                       D  ┆  A                ┆ A, D
                          ┆                   ┆
                        front                side
 */
GTEST_TEST(CalcDistanceToSurfaceMeshTest, MultipleTriangles2) {
  const Vector3d p_TA(1, 0, 0);
  const Vector3d p_TB(0, -1, 1);
  const Vector3d p_TC(0, 1, 1);
  const Vector3d p_TD(-1, 0, 0);

  const math::RollPitchYaw<double> rpy_WT(1, 2, 3);
  const Vector3d p_WTo(4, 5, 6);
  const math::RigidTransform<double> X_WT(rpy_WT, p_WTo);

  vector<Vector3d> vertices = {X_WT * p_TA, X_WT * p_TB, X_WT * p_TC,
                               X_WT * p_TD};
  vector<SurfaceTriangle> triangles = {{0, 1, 2}, {1, 2, 3}};
  TriangleSurfaceMesh<double> mesh_W(std::move(triangles), std::move(vertices));

  constexpr double kEps = 1e-14;
  /* Q lines on the z-axis half way up. Its distance to the mesh should be equal
   to its distance to the line passing through <1,0,0> and <0,0,1> (measured
   and expressed in frame T). */
  const Vector3d p_TQ(0, 0, 0.5);
  const double d_Q_expected =
      CalcDistanceToLine(p_TQ, {Vector3d::UnitX(), Vector3d::UnitZ()});
  const Vector3d p_WQ = X_WT * p_TQ;
  const double d_Q = CalcDistanceToSurfaceMesh(p_WQ, mesh_W);
  EXPECT_NEAR(d_Q, d_Q_expected, kEps);

  // Q1 is slightly to the left of Q.
  const Vector3d p_TQ1 = p_TQ + Vector3d{-1e-3, 0, 0};
  const Vector3d p_WQ1 = X_WT * p_TQ1;
  EXPECT_LT(CalcDistanceToSurfaceMesh(p_WQ1, mesh_W), d_Q);

  // Q2 is slightly to the right of Q.
  const Vector3d p_TQ2 = p_TQ + Vector3d{1e-3, 0, 0};
  const Vector3d p_WQ2 = X_WT * p_TQ2;
  EXPECT_LT(CalcDistanceToSurfaceMesh(p_WQ2, mesh_W), d_Q);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
