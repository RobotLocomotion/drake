/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mixed_integer_rotation_constraint_internal.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/random_rotation.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

using Eigen::Vector3d;

namespace drake {
namespace solvers {
namespace {
void CompareIntersectionResults(const std::vector<Vector3d>& desired,
                                const std::vector<Vector3d>& actual) {
  EXPECT_EQ(desired.size(), actual.size());
  Eigen::Matrix<bool, Eigen::Dynamic, 1> used =
      Eigen::Matrix<bool, Eigen::Dynamic, 1>::Constant(desired.size(), false);
  double tol = 1e-8;
  for (int i = 0; i < static_cast<int>(desired.size()); i++) {
    // need not be in the same order.
    bool found_match = false;
    for (int j = 0; j < static_cast<int>(desired.size()); j++) {
      if (used(j)) continue;
      if ((desired[i] - actual[j]).lpNorm<2>() < tol) {
        used(j) = true;
        found_match = true;
        break;
      }
    }
    EXPECT_TRUE(found_match);
  }
}

// For 2 binary variable per half axis, we know it cuts the first orthant into
// 7 regions. 3 of these regions have 4 co-planar vertices; 3 of these regions
// have 4 non-coplanar vertices, and one region has 3 vertices.
GTEST_TEST(RotationTest, TestAreAllVerticesCoPlanar) {
  Eigen::Vector3d n;
  double d;

  // 4 co-planar vertices.
  std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 3> bmin_bmax_coplanar{
      {{Eigen::Vector3d(0.5, 0.5, 0), Eigen::Vector3d(1, 1, 0.5)},
       {Eigen::Vector3d(0.5, 0, 0.5), Eigen::Vector3d(1, 0.5, 1)},
       {Eigen::Vector3d(0, 0.5, 0.5), Eigen::Vector3d(0.5, 1, 1)}}};
  for (const auto& bmin_bmax : bmin_bmax_coplanar) {
    auto pts = internal::ComputeBoxEdgesAndSphereIntersection(bmin_bmax.first,
                                                              bmin_bmax.second);
    EXPECT_TRUE(internal::AreAllVerticesCoPlanar(pts, &n, &d));
    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(n.norm(), 1, 1E-10);
      EXPECT_NEAR(n.dot(pts[i]), d, 1E-10);
      EXPECT_TRUE((n.array() > 0).all());
    }
  }

  // 4 non co-planar vertices.
  std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 3>
      bmin_bmax_non_coplanar{
          {{Eigen::Vector3d(0.5, 0, 0), Eigen::Vector3d(1, 0.5, 0.5)},
           {Eigen::Vector3d(0, 0.5, 0), Eigen::Vector3d(0.5, 1, 0.5)},
           {Eigen::Vector3d(0, 0, 0.5), Eigen::Vector3d(0.5, 0.5, 1)}}};
  for (const auto& bmin_bmax : bmin_bmax_non_coplanar) {
    auto pts = internal::ComputeBoxEdgesAndSphereIntersection(bmin_bmax.first,
                                                              bmin_bmax.second);
    EXPECT_FALSE(internal::AreAllVerticesCoPlanar(pts, &n, &d));
    EXPECT_TRUE(CompareMatrices(n, Eigen::Vector3d::Zero()));
    EXPECT_EQ(d, 0);
  }

  // 3 vertices
  Eigen::Vector3d bmin(0.5, 0.5, 0.5);
  Eigen::Vector3d bmax(1, 1, 1);
  auto pts = internal::ComputeBoxEdgesAndSphereIntersection(bmin, bmax);
  EXPECT_TRUE(internal::AreAllVerticesCoPlanar(pts, &n, &d));
  EXPECT_TRUE(CompareMatrices(n, Eigen::Vector3d::Constant(1.0 / std::sqrt(3)),
                              1E-10, MatrixCompareType::absolute));
  EXPECT_NEAR(pts[0].dot(n), d, 1E-10);
}

void CheckInnerFacets(const std::vector<Vector3d>& pts) {
  // Compute the inner facets of the convex hull of pts. Make sure for each
  // facet, there are three points on the facet, and the facet points
  // outward from the origin.
  Eigen::Matrix<double, Eigen::Dynamic, 3> A;
  Eigen::VectorXd b;
  internal::ComputeInnerFacetsForBoxSphereIntersection(pts, &A, &b);
  for (int i = 0; i < A.rows(); ++i) {
    for (const auto& pt : pts) {
      EXPECT_LE((A.row(i) * pt)(0), b(i) + 1E-10);
    }
    EXPECT_NEAR(A.row(i).norm(), 1, 1E-10);
    // A.row(i) is the inverse of the facet normal, that points outward from the
    // origin.
    EXPECT_TRUE((A.row(i).array() <= 0).all());
    int num_pts_on_plane = 0;
    for (const auto& pt : pts) {
      if (std::abs((A.row(i) * pt)(0) - b(i)) < 1E-10) {
        ++num_pts_on_plane;
      }
    }
    EXPECT_GE(num_pts_on_plane, 3);
  }
}

void CompareHalfspaceRelaxation(const std::vector<Vector3d> &pts) {
  // Computes a possibly less tight n and d analytically. For each triangle with
  // vertices pts[i], pts[j] and pts[k], determine if the halfspace coinciding
  // with the triangle is a cutting plane (namely all vertices in pts are on one
  // side of the halfspace). Compute the farthest distance from the cutting
  // planes to the origin.
  DRAKE_DEMAND(pts.size() >= 3);

  double d = -1;
  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(pts.size()); ++j) {
      for (int k = j + 1; k < static_cast<int>(pts.size()); ++k) {
        // Find the normal of the triangle.
        Eigen::Vector3d normal_tmp = (pts[k] - pts[i]).cross(pts[j] - pts[i]);
        normal_tmp.normalize();
        if (normal_tmp(0) < 0) {
          normal_tmp = -normal_tmp;
        }
        double d_tmp = normal_tmp.transpose() * pts[i];
        bool is_cutting_plane = true;
        for (const auto &pt : pts) {
          if (pt.transpose() * normal_tmp < d_tmp - 1E-10) {
            is_cutting_plane = false;
            break;
          }
        }
        if (is_cutting_plane) {
          d = std::max(d, d_tmp);
        }
      }
    }
  }

  Eigen::Vector3d n_expected;
  double d_expected;
  internal::ComputeHalfSpaceRelaxationForBoxSphereIntersection(pts, &n_expected,
                                                               &d_expected);
  if (pts.size() == 3) {
    EXPECT_NEAR(d_expected, d, 1E-6);
  }
  EXPECT_GE(d_expected, d - 1E-8);
  for (const auto &pt : pts) {
    EXPECT_GE(pt.transpose() * n_expected - d_expected, -1E-6);
  }
}

GTEST_TEST(RotationTest, TestHalfSpaceRelaxation) {
  // In some cases, the half space relaxation can be computed analytically. We
  // compare the analytical result, against
  // ComputeHalfSpaceRelaxationForBoxSphereIntersection()
  std::vector<Eigen::Vector3d> pts;
  Eigen::Vector3d n;
  double d;

  // For three points case, the half space relaxation is just the plane
  // coinciding with the three points.
  pts.emplace_back(1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0);
  pts.emplace_back(2.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0);
  pts.emplace_back(2.0 / 3.0, 2.0 / 3.0, 1.0 / 3.0);
  internal::ComputeHalfSpaceRelaxationForBoxSphereIntersection(pts, &n, &d);
  EXPECT_TRUE(CompareMatrices(n, Eigen::Vector3d::Constant(1 / std::sqrt(3)),
                              10 * std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_NEAR(d, std::sqrt(3) * 5 / 9, 1E-10);

  // Four points, symmetric about the plane x = y. The tightest half space
  // relaxation is not the plane coinciding with any three of the points, but
  // just coinciding with two of the points.
  pts.clear();
  // The first two points are on the x = y plane.
  pts.emplace_back(1.0 / 3.0, 1.0 / 3.0, std::sqrt(7) / 3.0);
  pts.emplace_back(2.0 / 3.0, 2.0 / 3.0, 1.0 / 3.0);
  // The last two points are symmetric about the x = y plane.
  pts.emplace_back(1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0);
  pts.emplace_back(2.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0);
  internal::ComputeHalfSpaceRelaxationForBoxSphereIntersection(pts, &n, &d);
  // The normal vector should be on the x = y plane.
  EXPECT_NEAR(n(0), n(1), 1E-8);
  EXPECT_NEAR(n.dot(pts[0]), d, 1E-8);
  EXPECT_NEAR(n.dot(pts[1]), d, 1E-8);
}

GTEST_TEST(RotationTest, TestInnerFacetsAndHalfSpace) {
  // We show that the inner facet is tighter than the half space for some case.
  // To this end, we show that for a box [0 0.5 0] <= x <= [0.5 1 0.5], there is
  // some point that does not satisfy the inner facets constraint A*x<=b, but
  // satisfies the half space constraint nᵀ*x>=d.
  const Eigen::Vector3d bmin(0, 0.5, 0);
  const Eigen::Vector3d bmax(0.5, 1, 0.5);
  const auto intersection_pts =
      internal::ComputeBoxEdgesAndSphereIntersection(bmin, bmax);
  DRAKE_DEMAND(intersection_pts.size() == 4);
  Eigen::Vector3d n;
  double d;
  internal::ComputeHalfSpaceRelaxationForBoxSphereIntersection(intersection_pts,
                                                               &n, &d);
  Eigen::Matrix<double, Eigen::Dynamic, 3> A;
  Eigen::VectorXd b;
  internal::ComputeInnerFacetsForBoxSphereIntersection(intersection_pts, &A,
                                                       &b);
  // Now form the optimization program
  // A.row(i) * x > b(i) + epsilon for at least one i
  // nᵀ * x >= d
  // bmin <= x <= bmax
  // We will show that there is a feasible solution to this program.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  auto z = prog.NewBinaryVariables(b.rows(), "z");
  for (int i = 0; i < b.rows(); ++i) {
    prog.AddLinearConstraint((A.row(i) * x)(0) >= b(i) + 1E-5 + (z(i) - 1) * 2);
  }
  prog.AddLinearConstraint(z.cast<symbolic::Expression>().sum() >= 1);
  prog.AddLinearConstraint(n.dot(x) >= d);
  prog.AddBoundingBoxConstraint(bmin, bmax, x);
  const MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());
}

// Test a number of closed-form solutions for the intersection of a box in the
// positive orthant with the unit circle.
GTEST_TEST(RotationTest, TestIntersectBoxWithCircle) {
  std::vector<Vector3d> desired;

  // Entire first octant.
  Vector3d box_min(0, 0, 0);
  Vector3d box_max(1, 1, 1);
  desired.push_back(Vector3d(1, 0, 0));
  desired.push_back(Vector3d(0, 1, 0));
  desired.push_back(Vector3d(0, 0, 1));
  CompareIntersectionResults(
      desired,
      internal::ComputeBoxEdgesAndSphereIntersection(box_min, box_max));
  CompareHalfspaceRelaxation(desired);
  CheckInnerFacets(desired);

  // Lifts box bottom (in z).  Still has 3 solutions.
  box_min << 0, 0, 1.0 / 3.0;
  desired[0] << std::sqrt(8) / 3.0, 0, 1.0 / 3.0;
  desired[1] << 0, std::sqrt(8) / 3.0, 1.0 / 3.0;
  CompareIntersectionResults(
      desired,
      internal::ComputeBoxEdgesAndSphereIntersection(box_min, box_max));
  CompareHalfspaceRelaxation(desired);
  CheckInnerFacets(desired);

  // Lowers box top (in z).  Now we have four solutions.
  box_max << 1, 1, 2.0 / 3.0;
  desired[2] << std::sqrt(5) / 3.0, 0, 2.0 / 3.0;
  desired.push_back(Vector3d(0, std::sqrt(5) / 3.0, 2.0 / 3.0));
  CompareIntersectionResults(
      desired,
      internal::ComputeBoxEdgesAndSphereIntersection(box_min, box_max));
  CompareHalfspaceRelaxation(desired);
  CheckInnerFacets(desired);

  // Gets a different four edges by shortening the box (in x).
  box_max(0) = .5;
  desired[0] << .5, std::sqrt(23.0) / 6.0, 1.0 / 3.0;
  desired[2] << .5, std::sqrt(11.0) / 6.0, 2.0 / 3.0;
  CompareIntersectionResults(
      desired,
      internal::ComputeBoxEdgesAndSphereIntersection(box_min, box_max));
  CompareHalfspaceRelaxation(desired);
  CheckInnerFacets(desired);

  // Now three edges again as we shorten the box (in y).
  box_max(1) = .6;
  desired.pop_back();
  desired[0] << .5, std::sqrt(11.0) / 6.0, 2.0 / 3.0;
  desired[1] << 2 * std::sqrt(11.0) / 15.0, .6, 2.0 / 3.0;
  desired[2] << .5, .6, std::sqrt(39.0) / 10.0;
  CompareIntersectionResults(
      desired,
      internal::ComputeBoxEdgesAndSphereIntersection(box_min, box_max));
  CompareHalfspaceRelaxation(desired);
  CheckInnerFacets(desired);

  // All four intersections are on the vertical edges.
  box_min << 1.0 / 3.0, 1.0 / 3.0, 0;
  box_max << 2.0 / 3.0, 2.0 / 3.0, 1;
  desired[0] << 1.0 / 3.0, 1.0 / 3.0, std::sqrt(7.0) / 3.0;
  desired[1] << 2.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0;
  desired[2] << 1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0;
  desired.push_back(Vector3d(2.0 / 3.0, 2.0 / 3.0, 1.0 / 3.0));
  CompareIntersectionResults(
      desired,
      internal::ComputeBoxEdgesAndSphereIntersection(box_min, box_max));
  CompareHalfspaceRelaxation(desired);
  CheckInnerFacets(desired);

  // box_max right on the unit sphere.
  box_max << 1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0;
  box_min << 0, 1.0 / 3.0, 0;
  // Should return just the single point.
  desired.erase(desired.begin() + 1, desired.end());
  desired[0] = box_max;
  CompareIntersectionResults(
      desired,
      internal::ComputeBoxEdgesAndSphereIntersection(box_min, box_max));
  CheckInnerFacets(desired);

  // Multiple vertices are on the sphere.
  box_min << 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0;
  box_max << 2.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0;
  desired.clear();
  desired.push_back(Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3));
  desired.push_back(Eigen::Vector3d(2.0 / 3, 1.0 / 3, 2.0 / 3));
  desired.push_back(Eigen::Vector3d(2.0 / 3, 2.0 / 3, 1.0 / 3));
  CompareIntersectionResults(
      desired,
      internal::ComputeBoxEdgesAndSphereIntersection(box_min, box_max));
  CheckInnerFacets(desired);

  // Six intersections.
  box_min = Eigen::Vector3d::Constant(1.0 / 3.0);
  box_max = Eigen::Vector3d::Constant(sqrt(6) / 3.0);
  desired.clear();
  // The intersecting points are the 6 permutations of
  // (1.0 / 3.0, sqrt(2) / 3.0, sqrt(6) / 3.0)
  desired.resize(6);
  desired[0] << 1.0 / 3.0, sqrt(2) / 3.0, sqrt(6) / 3.0;
  desired[1] << 1.0 / 3.0, sqrt(6) / 3.0, sqrt(2) / 3.0;
  desired[2] << sqrt(2) / 3.0, 1.0 / 3.0, sqrt(6) / 3.0;
  desired[3] << sqrt(2) / 3.0, sqrt(6) / 3.0, 1.0 / 3.0;
  desired[4] << sqrt(6) / 3.0, 1.0 / 3.0, sqrt(2) / 3.0;
  desired[5] << sqrt(6) / 3.0, sqrt(2) / 3.0, 1.0 / 3.0;
  CompareIntersectionResults(
      desired,
      internal::ComputeBoxEdgesAndSphereIntersection(box_min, box_max));
  CompareHalfspaceRelaxation(desired);
  CheckInnerFacets(desired);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
