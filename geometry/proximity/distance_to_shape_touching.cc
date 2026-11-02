#include "drake/geometry/proximity/distance_to_shape_touching.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

using Eigen::Vector3d;
using math::RigidTransformd;

namespace {

// Various stages in the computation need to dismiss zero-length vectors. This
// is the shared tolerance for designating a vector as "zero length".
constexpr double kZeroLengthEps = 1e-14;

// Two boxes are expected to be touching (signed distance within
// kTouchingTolerance of zero). In that case, there should be an axis along
// which the boxes can be projected such that their projections don't overlap.
// They *may* overlap as much as kTouchingTolerance (due to its role in
// classification). However, we have observed that the act of projection can
// introduce further rounding error. So, we allow for a further three bits of
// noise when evaluating the projected overlap.
constexpr double kOverlapTolerance = 8 * kTouchingTolerance;

}  // namespace

Vector3d CalcGradientWhenTouching(const fcl::CollisionObjectd& a,
                                  const math::RigidTransformd& X_WA,
                                  const fcl::CollisionObjectd& b,
                                  const math::RigidTransformd& X_WB,
                                  const Eigen::Vector3d& p_ACa,
                                  const Eigen::Vector3d& p_BCb) {
  // This function serves for all sphere-X geometry pairs. However, we don't
  // necessarily expect that it is used for all those pairs. For example, some
  // sphere-X touching pairs are resolved by DistancePairGeometry in
  // distance_to_shape_callback.h.
  if (a.collisionGeometry()->getNodeType() == fcl::GEOM_SPHERE &&
      static_cast<const fcl::Sphered*>(a.collisionGeometry().get())->radius >
          0) {
    const Vector3d nhat_AB_A = p_ACa.normalized();
    const Vector3d nhat_BA_W = X_WA.rotation() * (-nhat_AB_A);
    return nhat_BA_W;
  }
  if (b.collisionGeometry()->getNodeType() == fcl::GEOM_SPHERE &&
      static_cast<const fcl::Sphered*>(b.collisionGeometry().get())->radius >
          0) {
    const Vector3d nhat_BA_B = p_BCb.normalized();
    const Vector3d nhat_BA_W = X_WB.rotation() * nhat_BA_B;
    return nhat_BA_W;
  }
  if (a.collisionGeometry()->getNodeType() == fcl::GEOM_BOX &&
      b.collisionGeometry()->getNodeType() == fcl::GEOM_BOX) {
    const auto& box_A =
        *static_cast<const fcl::Boxd*>(a.collisionGeometry().get());
    const auto& box_B =
        *static_cast<const fcl::Boxd*>(b.collisionGeometry().get());
    return BoxBoxGradient(box_A, box_B, X_WA, X_WB, p_ACa, p_BCb);
  }
  // TODO(14789): Take care of other shapes and zero-radius spheres. For now,
  //  return NaN.
  const double kNan = std::numeric_limits<double>::quiet_NaN();
  return Vector3d(kNan, kNan, kNan);
}

Vector3d PointOnBoxSurfaceHelper(const Vector3d& p_BQ, const fcl::Boxd& box_B) {
  const Vector3d half_size = box_B.side / 2.0;
  Vector3d n{0.0, 0.0, 0.0};
  // Our classification is sensitive to the precision of p_BQ and this
  // internal tolerance. We assume that the caller always passes p_BQ on the
  // surface of the box with appropriate precisions. See the unit test
  // GTEST_TEST(PointOnBoxSurfaceHelper, CornerCases) for examples of how the
  // precision of p_BQ can affect the classification.
  constexpr double kEps = 1e-14;
  using std::abs;
  // Mark the vector `n` with a 1 at index `i` if the point lies approximately
  // on either the positive or negative face of the box on dimension `i`.
  for (int i = 0; i < 3; ++i) {
    double diff = abs(half_size(i) - abs(p_BQ(i)));
    // We found this threshold empirically.
    if (diff <= 8 * kEps) {
      n(i) = 1.0;
    }
  }
  return n;
}

std::pair<double, double> ProjectedMinMax(const fcl::Boxd& box_B,
                                          const RigidTransformd& X_AB,
                                          const Vector3d& unit_vector_A) {
  const Vector3d half_size_B = box_B.side / 2.0;
  const Vector3d unit_vector_B = X_AB.rotation().transpose() * unit_vector_A;
  double max_value = half_size_B.dot(unit_vector_B.cwiseAbs());
  double d = X_AB.translation().dot(unit_vector_A);
  return {d - max_value, d + max_value};
}

std::optional<Vector3d> MaybeMakeSeparatingVector(
    const fcl::Boxd& box_A, const fcl::Boxd& box_B, const RigidTransformd& X_AB,
    const std::vector<Vector3d>& v_As) {
  const Vector3d half_size_A = box_A.side / 2.0;
  for (const Vector3d& v_A : v_As) {
    double v_A_norm = v_A.norm();
    if (v_A_norm < kZeroLengthEps) {
      continue;
    }
    const Vector3d unit_vector_A = v_A / v_A_norm;

    const double max_A = half_size_A.dot(unit_vector_A.cwiseAbs());
    const double min_A = -max_A;
    const auto [min_B, max_B] = ProjectedMinMax(box_B, X_AB, unit_vector_A);

    // The catch-all case in BoxBoxGradientImpl() applies this same test.
    if (max_A < min_B + kOverlapTolerance) {
      return -unit_vector_A;
    }
    if (max_B < min_A + kOverlapTolerance) {
      return unit_vector_A;
    }
  }
  // None of the provided vectors serve as a separating axis within the allowed
  // tolerance; as documented, nullopt indicates that fact.
  return {};
}

namespace {

// The implementation of BoxBoxGradient(), with box_B's pose and the result
// expressed in box_A's frame.
Vector3d BoxBoxGradientImpl(const fcl::Boxd& box_A, const fcl::Boxd& box_B,
                            const math::RigidTransformd& X_AB,
                            const Eigen::Vector3d& p_ACa,
                            const Eigen::Vector3d& p_BCb) {
  const math::RotationMatrixd& R_AB = X_AB.rotation();
  const Vector3d v_A = PointOnBoxSurfaceHelper(p_ACa, box_A);
  const Vector3d v_B = PointOnBoxSurfaceHelper(p_BCb, box_B);
  double s_A = v_A.sum();
  double s_B = v_B.sum();

  // Face-to-*. Use the face normal if a witness point is strictly inside
  // a face (but neither edge nor vertex). Rounding in the witness points can
  // misclassify them, so we confirm that the face normal separates the boxes.

  // p_ACa reports as on a face of box_A, determine the sign of the face normal,
  // and confirm its validity as a separating vector.
  if (s_A == 1) {
    const Vector3d normal_A = (v_A.dot(p_ACa) > 0 ? v_A : -v_A);
    const std::optional<Vector3d> nhat_BA_A =
        MaybeMakeSeparatingVector(box_A, box_B, X_AB, {-normal_A});
    if (nhat_BA_A.has_value()) {
      return nhat_BA_A.value();
    }
  }
  // p_BCb is on a face of box_B, determine the sign of the face normal, express
  // it in frame A, and confirm its validity as a separating vector.
  if (s_B == 1) {
    // p_BCb is on a face of B
    const Vector3d normal_B = (v_B.dot(p_BCb) > 0 ? v_B : -v_B);
    const std::optional<Vector3d> nhat_BA_A =
        MaybeMakeSeparatingVector(box_A, box_B, X_AB, {R_AB * normal_B});
    if (nhat_BA_A.has_value()) {
      return nhat_BA_A.value();
    }
  }

  // The index of the non-zero entry determines the axis of the edge that either
  // Ca or Cb lies on. Note that axis_index_A is valid only when s_A == 2,
  // and axis_index_B is valid only when s_B == 2.  We will use them under
  // those conditions only.
  const int axis_index_A = (Vector3d::Ones() - v_A).dot(Vector3d(0, 1, 2));
  const int axis_index_B = (Vector3d::Ones() - v_B).dot(Vector3d(0, 1, 2));
  // Edge-to-edge.
  // Arbitrary threshold to determine if edges are parallel.
  constexpr double kParallelEps = 1e-14;
  if (s_A == 2 && s_B == 2) {
    const Vector3d cross_product_A =
        Vector3d::Unit(axis_index_A).cross(R_AB.col(axis_index_B));
    if (cross_product_A.norm() > kParallelEps) {
      // The two edges are not parallel.
      const std::optional<Vector3d> nhat_BA_A =
          MaybeMakeSeparatingVector(box_A, box_B, X_AB, {cross_product_A});
      if (nhat_BA_A.has_value()) {
        return nhat_BA_A.value();
      }
    } else {
      // Parallel edge-to-edge. A separating plane passes through at least
      // one of the two faces sharing the edge in Box A.
      const std::optional<Vector3d> nhat_BA_A =
          MaybeMakeSeparatingVector(box_A, box_B, X_AB,
                                    {Vector3d::Unit((axis_index_A + 1) % 3),
                                     Vector3d::Unit((axis_index_A + 2) % 3)});
      if (nhat_BA_A.has_value()) {
        return nhat_BA_A.value();
      }
    }
  }

  // Edge-vertex. An edge in Box A touches a vertex in Box B. A separating
  // plane passes through the edge in Box A and one of the edges incident
  // to the vertex in Box B.
  // Ca is strictly on an edge and Cb is strictly on a vertex.
  if (s_A == 2 && s_B == 3) {
    const Vector3d edge_vector_A_A = Vector3d::Unit(axis_index_A);
    const std::optional<Vector3d> nhat_BA_A = MaybeMakeSeparatingVector(
        box_A, box_B, X_AB,
        {edge_vector_A_A.cross(R_AB.col(0)), edge_vector_A_A.cross(R_AB.col(1)),
         edge_vector_A_A.cross(R_AB.col(2))});
    if (nhat_BA_A.has_value()) {
      return nhat_BA_A.value();
    }
  }
  // Vertex-edge. This is the symmetric case of the above case.
  // Ca is strictly on a vertex and Cb is strictly on an edge.
  if (s_A == 3 && s_B == 2) {
    const Vector3d edge_vector_B_A = R_AB.col(axis_index_B);
    const std::optional<Vector3d> nhat_BA_A =
        MaybeMakeSeparatingVector(box_A, box_B, X_AB,
                                  {edge_vector_B_A.cross(Vector3d::UnitX()),
                                   edge_vector_B_A.cross(Vector3d::UnitY()),
                                   edge_vector_B_A.cross(Vector3d::UnitZ())});
    if (nhat_BA_A.has_value()) {
      return nhat_BA_A.value();
    }
  }

  // Vertex-vertex and catch-all cases.
  // Unlike other cases above, first we search for a separating vector from
  // the three axes of each box. If the search fail, we search from their
  // cross products.
  std::vector<Vector3d> candidates_A = {Vector3d::UnitX(), Vector3d::UnitY(),
                                        Vector3d::UnitZ(), R_AB.col(0),
                                        R_AB.col(1),       R_AB.col(2)};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      candidates_A.push_back(Vector3d::Unit(i).cross(R_AB.col(j)));
    }
  }
  // If no candidate separates the boxes within tolerance, they penetrate more
  // than expected. Among the fifteen candidate directions, the one with the
  // minimum overlap is the direction of the minimum translation that separates
  // the boxes, i.e., the signed-distance gradient.
  const Vector3d half_size_A = box_A.side / 2.0;
  double min_overlap = std::numeric_limits<double>::infinity();
  Vector3d best_nhat_BA_A = Vector3d::Zero();
  for (const Vector3d& candidate_A : candidates_A) {
    const double candidate_norm = candidate_A.norm();
    if (candidate_norm < kZeroLengthEps) {
      continue;
    }
    const Vector3d unit_vector_A = candidate_A / candidate_norm;
    const double max_A = half_size_A.dot(unit_vector_A.cwiseAbs());
    const double min_A = -max_A;
    const auto [min_B, max_B] = ProjectedMinMax(box_B, X_AB, unit_vector_A);
    // Same test as MaybeMakeSeparatingVector().
    if (max_A < min_B + kOverlapTolerance) {
      return -unit_vector_A;
    }
    if (max_B < min_A + kOverlapTolerance) {
      return unit_vector_A;
    }
    // Moving A along -unit_vector_A by (max_A - min_B) separates the boxes;
    // likewise along +unit_vector_A by (max_B - min_A).
    if (max_A - min_B < min_overlap) {
      min_overlap = max_A - min_B;
      best_nhat_BA_A = -unit_vector_A;
    }
    if (max_B - min_A < min_overlap) {
      min_overlap = max_B - min_A;
      best_nhat_BA_A = unit_vector_A;
    }
  }
  return best_nhat_BA_A;
}

}  // namespace

Vector3d BoxBoxGradient(const fcl::Boxd& box_A, const fcl::Boxd& box_B,
                        const math::RigidTransformd& X_WA,
                        const math::RigidTransformd& X_WB,
                        const Eigen::Vector3d& p_ACa,
                        const Eigen::Vector3d& p_BCb) {
  // Far from the World origin, quantities measured in World frame lose
  // precision. Measuring box_B in box_A's frame subtracts the World positions
  // first, so the computations are only as large as the boxes and their
  // separation.
  const RigidTransformd X_AB = X_WA.InvertAndCompose(X_WB);
  const Vector3d nhat_BA_A =
      BoxBoxGradientImpl(box_A, box_B, X_AB, p_ACa, p_BCb);
  return X_WA.rotation() * nhat_BA_A;
}

}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
