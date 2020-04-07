#include "drake/solvers/mixed_integer_rotation_constraint_internal.h"

#include <algorithm>
#include <array>
#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace internal {
namespace {
// Given two coordinates, find the (positive) third coordinate on that
// intersects with the unit circle.
double Intercept(double x, double y) {
  DRAKE_ASSERT(x * x + y * y <= 1);
  return std::sqrt(1 - x * x - y * y);
}
}  // namespace

std::vector<Eigen::Vector3d> ComputeBoxEdgesAndSphereIntersection(
    const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax) {
  // Assumes the positive orthant (and bmax > bmin).
  DRAKE_ASSERT(bmin(0) >= 0 && bmin(1) >= 0 && bmin(2) >= 0);
  DRAKE_ASSERT(bmax(0) > bmin(0) && bmax(1) > bmin(1) && bmax(2) > bmin(2));

  // Assumes the unit circle intersects the box.
  DRAKE_ASSERT(bmin.lpNorm<2>() <= 1);
  DRAKE_ASSERT(bmax.lpNorm<2>() >= 1);

  std::vector<Eigen::Vector3d> intersections;

  if (bmin.lpNorm<2>() == 1) {
    // Then only the min corner intersects.
    intersections.push_back(bmin);
    return intersections;
  }

  if (bmax.lpNorm<2>() == 1) {
    // Then only the max corner intersects.
    intersections.push_back(bmax);
    return intersections;
  }

  // The box has at most 12 edges, each edge can intersect with the unit sphere
  // at most once, since the box is in the first orthant.
  intersections.reserve(12);

  // 1. Loop through each vertex of the box, add it to intersections if
  // the vertex is on the sphere.
  for (int i = 0; i < 8; ++i) {
    Eigen::Vector3d vertex{};
    for (int axis = 0; axis < 3; ++axis) {
      vertex(axis) = i & (1 << axis) ? bmin(axis) : bmax(axis);
    }
    if (vertex.norm() == 1) {
      intersections.push_back(vertex);
    }
  }

  // 2. Loop through each edge, find the intersection between each edge and the
  // unit sphere, if one exists.
  for (int axis = 0; axis < 3; ++axis) {
    // axis = 0 means edges along x axis;
    // axis = 1 means edges along y axis;
    // axis = 2 means edges along z axis;
    int fixed_axis1 = (axis + 1) % 3;
    int fixed_axis2 = (axis + 2) % 3;
    // 4 edges along each axis;

    // First finds the two end points on the edge.
    Eigen::Vector3d pt_closer, pt_farther;
    pt_closer(axis) = bmin(axis);
    pt_farther(axis) = bmax(axis);
    std::array<double, 2> fixed_axis1_val = {
        {bmin(fixed_axis1), bmax(fixed_axis1)}};
    std::array<double, 2> fixed_axis2_val = {
        {bmin(fixed_axis2), bmax(fixed_axis2)}};
    for (double val1 : fixed_axis1_val) {
      pt_closer(fixed_axis1) = val1;
      pt_farther(fixed_axis1) = pt_closer(fixed_axis1);
      for (double val2 : fixed_axis2_val) {
        pt_closer(fixed_axis2) = val2;
        pt_farther(fixed_axis2) = pt_closer(fixed_axis2);

        // Determines if there is an intersecting point between the edge and the
        // sphere. If the intersecting point is not the vertex of the box, then
        // push this intersecting point to intersections directly.
        if (pt_closer.norm() < 1 && pt_farther.norm() > 1) {
          Eigen::Vector3d pt_intersect{};
          pt_intersect(fixed_axis1) = pt_closer(fixed_axis1);
          pt_intersect(fixed_axis2) = pt_closer(fixed_axis2);
          pt_intersect(axis) =
              Intercept(pt_intersect(fixed_axis1), pt_intersect(fixed_axis2));
          intersections.push_back(pt_intersect);
        }
      }
    }
  }
  return intersections;
}

/**
 * Compute the outward unit length normal of the triangle, with the three
 * vertices being `pt0`, `pt1` and `pt2`.
 * @param pt0 A vertex of the triangle, in the first orthant (+++).
 * @param pt1 A vertex of the triangle, in the first orthant (+++).
 * @param pt2 A vertex of the triangle, in the first orthant (+++).
 * @param n The unit length normal vector of the triangle, pointing outward from
 * the origin.
 * @param d The intersecpt of the plane. Namely nᵀ * x = d for any point x on
 * the triangle.
 */
void ComputeTriangleOutwardNormal(const Eigen::Vector3d& pt0,
                                  const Eigen::Vector3d& pt1,
                                  const Eigen::Vector3d& pt2,
                                  Eigen::Vector3d* n, double* d) {
  DRAKE_DEMAND((pt0.array() >= 0).all());
  DRAKE_DEMAND((pt1.array() >= 0).all());
  DRAKE_DEMAND((pt2.array() >= 0).all());
  *n = (pt2 - pt0).cross(pt1 - pt0);
  // If the three points are almost colinear, then throw an error.
  double n_norm = n->norm();
  if (n_norm < 1E-3) {
    throw std::runtime_error("The points are almost colinear.");
  }
  *n = (*n) / n_norm;
  if (n->sum() < 0) {
    (*n) *= -1;
  }
  *d = pt0.dot(*n);
  DRAKE_DEMAND((n->array() >= 0).all());
}

bool AreAllVerticesCoPlanar(const std::vector<Eigen::Vector3d>& pts,
                            Eigen::Vector3d* n, double* d) {
  DRAKE_DEMAND(pts.size() >= 3);
  ComputeTriangleOutwardNormal(pts[0], pts[1], pts[2], n, d);
  // Determine if the other vertices are on the plane nᵀ * x = d.
  bool pts_on_plane = true;
  for (int i = 3; i < static_cast<int>(pts.size()); ++i) {
    if (std::abs(n->dot(pts[i]) - *d) > 1E-10) {
      pts_on_plane = false;
      n->setZero();
      *d = 0;
      break;
    }
  }
  return pts_on_plane;
}

void ComputeHalfSpaceRelaxationForBoxSphereIntersection(
    const std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d* n, double* d) {
  DRAKE_DEMAND(pts.size() >= 3);
  // We first prove that for a given normal vector n, and ANY unit
  // length vector v within the intersection region between the
  // surface of the unit sphere and the interior of the axis-aligned
  // box, the minimal of nᵀ * v, always occurs at one of the vertex of
  // the intersection region, if the box and the vector n are in the
  // same orthant. Namely min nᵀ * v = min(nᵀ * pts.col(i))
  // To see this, for any vector v in the intersection region, suppose
  // it is on an arc, aligned with one axis. Without loss of
  // generality we assume the aligned axis is x axis, namely
  // v(0) = t, box_min(0) <= t <= box_max(0)
  // and v(1)² + v(2)² = 1 - t², with the bounds
  // box_min(1) <= v(1) <= box_max(1)
  // box_min(2) <= v(2) <= box_max(2)
  // And the inner product nᵀ * v =
  // n(0) * t + s * (n(1) * cos(α) + n(2) * sin(α))
  // where we define s = sqrt(1 - t²)
  // Using the property of trigonometric function, we know that
  // the minimal of (n(1) * cos(α) + n(2) * sin(α)) is obtained at
  // the boundary of α. Thus we know that the minimal of nᵀ * v is
  // always obtained at one of the vertex pts.col(i).

  // To find the tightest bound d satisfying nᵀ * v >= d for all
  // vector v in the intersection region, we use the fact that for
  // a given normal vector n, the minimal of nᵀ * v is always obtained
  // at one of the vertices pts.col(i), and formulate the following
  // SOCP to find the normal vector n
  // max d
  // s.t d <= nᵀ * pts.col(i)
  //     nᵀ * n <= 1

  // First compute a plane coinciding with a triangle, formed by 3 vertices
  // in the intersection region. If all the vertices are on that plane, then the
  // normal of the plane is n, and we do not need to run the optimization.
  // If there are only 3 vertices in the intersection region, then the normal
  // vector n is the normal of the triangle, formed by these three vertices.

  bool pts_on_plane = AreAllVerticesCoPlanar(pts, n, d);
  if (pts_on_plane) {
    return;
  }

  // If there are more than 3 vertices in the intersection region, and these
  // vertices are not co-planar, then we find the normal vector `n` through an
  // optimization, whose formulation is mentioned above.
  MathematicalProgram prog_normal;
  auto n_var = prog_normal.NewContinuousVariables<3>();
  auto d_var = prog_normal.NewContinuousVariables<1>();
  prog_normal.AddLinearCost(-d_var(0));
  for (const auto& pt : pts) {
    prog_normal.AddLinearConstraint(n_var.dot(pt) >= d_var(0));
  }

  // TODO(hongkai.dai): This optimization is expensive, especially if we have
  // multiple rotation matrices, all relaxed with the same number of binary
  // variables per half axis, the result `n` and `d` are the same. Should
  // consider hard-coding the result, to avoid repeated computation.

  Vector4<symbolic::Expression> lorentz_cone_vars;
  lorentz_cone_vars << 1, n_var;
  prog_normal.AddLorentzConeConstraint(lorentz_cone_vars);
  const auto result = Solve(prog_normal);
  DRAKE_DEMAND(result.is_success());
  *n = result.GetSolution(n_var);
  *d = result.GetSolution(d_var(0));

  DRAKE_DEMAND((*n)(0) > 0 && (*n)(1) > 0 && (*n)(2) > 0);
  DRAKE_DEMAND(*d > 0 && *d < 1);
}

void ComputeInnerFacetsForBoxSphereIntersection(
    const std::vector<Eigen::Vector3d>& pts,
    Eigen::Matrix<double, Eigen::Dynamic, 3>* A, Eigen::VectorXd* b) {
  for (const auto& pt : pts) {
    DRAKE_DEMAND((pt.array() >= 0).all());
  }
  A->resize(0, 3);
  b->resize(0);
  // Loop through each triangle, formed by connecting the vertices of the
  // intersection region. We write the plane coinciding with the triangle as
  // cᵀ * x >= d. If all the vertices of the intersection region satisfies
  // cᵀ * pts[i] >= d, then we know the intersection region satisfies
  // cᵀ * x >= d for all x being a point in the intersection region. Here we
  // use the proof in the ComputeHalfSpaceRelaxationForBoxSphereIntersection(),
  // that the minimal value of cᵀ * x over all x inside the intersection
  // region, occurs at one of the vertex of the intersection region.
  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(pts.size()); ++j) {
      for (int k = j + 1; k < static_cast<int>(pts.size()); ++k) {
        // First compute the triangle formed by vertices pts[i], pts[j] and
        // pts[k].
        Eigen::Vector3d c;
        double d;
        ComputeTriangleOutwardNormal(pts[i], pts[j], pts[k], &c, &d);
        // A halfspace cᵀ * x >= d is valid, if all vertices pts[l] satisfy
        // cᵀ * pts[l] >= d.
        bool is_valid_halfspace = true;
        // Now check if the other vertices pts[l] satisfies cᵀ * pts[l] >= d.
        for (int l = 0; l < static_cast<int>(pts.size()); ++l) {
          if ((l != i) && (l != j) && (l != k)) {
            if (c.dot(pts[l]) < d - 1E-10) {
              is_valid_halfspace = false;
              break;
            }
          }
        }
        // If all vertices pts[l] satisfy cᵀ * pts[l] >= d, then add this
        // constraint to A * x <= b
        if (is_valid_halfspace) {
          A->conservativeResize(A->rows() + 1, Eigen::NoChange);
          b->conservativeResize(b->rows() + 1, Eigen::NoChange);
          A->row(A->rows() - 1) = -c.transpose();
          (*b)(b->rows() - 1) = -d;
        }
      }
    }
  }
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
