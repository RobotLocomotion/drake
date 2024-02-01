#include "drake/geometry/proximity/calc_distance_to_surface_mesh.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;

namespace {

/* Struct to hold vertex positions of a triangle. */
struct Triangle {
  Vector3<double> A;
  Vector3<double> B;
  Vector3<double> C;
};

/* Given three 2d points Q, B, and C, returns true if Q lies in the triangle
 OBC, where O is the origin. */
bool IsInTriangle(const Vector2<double>& Q, const Vector2<double>& B,
                  const Vector2<double>& C) {
  // Find the barycentric coordinate of Q in the triangle OBC,
  // (s, t, 1-s-t). If they are all non-negative, then Q lies in the
  // triangle. To find the barycentric coordinate we solve
  // Q = s * B + t * C for s and t.
  Matrix2<double> M;
  M.col(0) = B;
  M.col(1) = C;
  const Vector2<double> bary = M.colPivHouseholderQr().solve(Q);
  return bary(0) >= 0.0 && bary(1) >= 0.0 && bary(0) + bary(1) <= 1.0;
}

/* Computes the squared distance from the point Q to the non-degenerate line
 segment AB (the length of AB is positive). */
double CalcSquaredDistanceToLineSegment(const Vector3<double>& Q,
                                        const Vector3<double>& A,
                                        const Vector3<double>& B) {
  Vector3<double> AB = B - A;
  Vector3<double> AQ = Q - A;

  // If the projection of Q on the line AB is equal to A + s * BA for s > 0,
  // then the shortest distance is BQ.
  if (AQ.dot(AB) <= 0.0) return AQ.squaredNorm();

  Vector3<double> BQ = Q - B;

  // If the projection of Q on the line AB is equal to B + t * AB for t > 0,
  // then the shortest distance is BQ.
  if (BQ.dot(AB) >= 0.0) return BQ.squaredNorm();

  /* Compute the squared distance from Q to its projection P on AB.

           Q           R    |AB x AQ| = area of parallelogram ABRQ
          /-----------/     |AB| = base of parallelogram
         / |.        /      |QP| = height of parallelogram
        /  |  .     /            = distance from Q to AB.
       /   |    .  /        |AB x AQ| / |AB| = |QP|
      -----o------/         |AB x AQ|² / |AB|² = |QP|²
     A     P          B
  */
  return (AB.cross(AQ)).squaredNorm() / AB.squaredNorm();
}

/* Computes the squared distance from the point p_WQ to a non-degenerate
 triangle t_W. */
double CalcSquaredDistanceToTriangle(const Vector3<double>& p_WQ,
                                     Triangle t_W) {
  using math::RotationMatrix;
  // First transform everything into the triangle's frame. We characterize the
  // triangle's frame as the unique frame that
  // 1. has origin at vertex A of the triangle,
  // 2. the x-axis points in the direction of AB, and
  // 3. the z-axis points in the direction of AB x AC.
  const Vector3<double> p_AB_W = t_W.B - t_W.A;
  const Vector3<double> p_AC_W = t_W.C - t_W.A;
  Matrix3<double> R_TW;
  auto Tx_W = R_TW.row(0);
  auto Ty_W = R_TW.row(1);
  auto Tz_W = R_TW.row(2);
  Tx_W = p_AB_W.normalized();
  Tz_W = p_AB_W.cross(p_AC_W).normalized();
  Ty_W = Tz_W.cross(Tx_W);
  DRAKE_ASSERT(RotationMatrix<double>(R_TW).IsValid());

  const Vector3<double> p_TA = Vector3<double>::Zero();
  /* Without shortcuts this expression would be more correctly written as:
     p_AB_T = R_TW * p_AB_W
   Using the observation that A and To are coincident, we get
     p_AB_T = p_ToB_T = p_TB.
   Similar for p_TC and p_TQ. */
  const Vector3<double> p_TB = R_TW * p_AB_W;
  const Vector3<double> p_TC = R_TW * p_AC_W;
  const Vector3<double> p_TQ = R_TW * (p_WQ - t_W.A);

  /* If the projection of p_TQ to the x-y plane lies in triangle ABC, then the
   shortest distance is to the projection. */
  if (IsInTriangle(p_TQ.head(2), p_TB.head(2), p_TC.head(2))) {
    return p_TQ(2) * p_TQ(2);
  }

  /* Otherwise, the shortest distance is realized on the edge. */
  const double d_AB_squared =
      CalcSquaredDistanceToLineSegment(p_TQ, p_TA, p_TB);
  const double d_AC_squared =
      CalcSquaredDistanceToLineSegment(p_TQ, p_TA, p_TC);
  const double d_BC_squared =
      CalcSquaredDistanceToLineSegment(p_TQ, p_TB, p_TC);

  return std::min({d_AB_squared, d_AC_squared, d_BC_squared});
}

// TODO(xuchenhan-tri): Consider using the algorithm presented in chapter 5.1.5
//  of "Real Time Collision Detection" (Christer Ericson) with fewer flops if
//  performance is an issue for CalcSquaredDistanceToTriangle().

class BvhVisitor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BvhVisitor);

  static double CalcMinDistance(
      const Vector3d& p_WQ, const TriangleSurfaceMesh<double>& mesh_W,
      const Bvh<Obb, TriangleSurfaceMesh<double>>::NodeType& node_W) {
    BvhVisitor visitor{p_WQ, mesh_W};
    visitor.Visit(node_W);
    return std::sqrt(visitor.min_squared_);
  }

 private:
  BvhVisitor(const Vector3d& p_WQ, const TriangleSurfaceMesh<double>& mesh_W)
      : p_WQ_{p_WQ}, mesh_W_{mesh_W} {}

  void Visit(const Bvh<Obb, TriangleSurfaceMesh<double>>::NodeType& node_W) {
    if (node_W.is_leaf()) {
      for (int i = 0; i < node_W.num_element_indices(); ++i) {
        const SurfaceTriangle& t = mesh_W_.triangles()[node_W.element_index(i)];
        min_squared_ = std::min(min_squared_,
                                CalcSquaredDistanceToTriangle(
                                    p_WQ_, {mesh_W_.vertices()[t.vertex(0)],
                                            mesh_W_.vertices()[t.vertex(1)],
                                            mesh_W_.vertices()[t.vertex(2)]}));
      }
      return;
    }

    // The rest of this function is for the subtree of the internal node rooted
    // at node_W.

    const Vector3d box_half_width_B = node_W.bv().half_width();
    const RigidTransformd& X_WB = node_W.bv().pose();
    const Vector3d p_BQ = X_WB.inverse() * p_WQ_;

    const auto [closest_point_on_box_boundary_B, grad_signed_distance_B, _] =
        point_distance::DistanceToPoint<double>::ComputeDistanceToBox<3>(
            box_half_width_B, p_BQ);
    // Mathematically we can calculate the signed distance phi from the gradient
    // grad_phi, the query point Q and its closest point C on the box's
    // boundary as:
    //      phi = grad_phi.dot(Q - C)
    // because:
    // 1. The unsigned distance is clearly |Q-C|. We just need to set the
    //    correct sign depending on the location of Q relative to the box.
    // 2. grad_phi is a unit vector pointing outward from the box.  It is
    //    parallel to Q-C (or antiparallel if Q is inside the box).
    const double signed_distance =
        grad_signed_distance_B.dot(p_BQ - closest_point_on_box_boundary_B);

    // Check for possible pruning.
    if (signed_distance > 0) {
      // The query point is outside, so we can get the lower bound.
      const double squared_distance_from_this_node_is_at_least =
          signed_distance * signed_distance;
      // Use the lower bound to possibly prune this subtree.
      if (squared_distance_from_this_node_is_at_least > min_squared_) {
        return;
      }
    }

    // We couldn't prune the subtree, recursively search both children.
    this->Visit(node_W.left());
    this->Visit(node_W.right());
  }

 private:
  const Vector3<double>& p_WQ_;
  const TriangleSurfaceMesh<double>& mesh_W_;

  double min_squared_{std::numeric_limits<double>::infinity()};
};

}  // namespace

double CalcDistanceToSurfaceMesh(
    const Vector3<double>& p_WQ, const TriangleSurfaceMesh<double>& mesh_W,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_W) {
  return BvhVisitor::CalcMinDistance(p_WQ, mesh_W, bvh_W.root_node());
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
