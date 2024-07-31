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

std::pair<double, double> ProjectedMinMax(const fcl::Boxd& box_A,
                                          const RigidTransformd& X_WA,
                                          const Vector3d& unit_vector_W) {
  const Vector3d half_size_A = box_A.side / 2.0;
  const Vector3d unit_vector_A = X_WA.rotation().transpose() * unit_vector_W;
  double max_value = half_size_A.dot(unit_vector_A.cwiseAbs());
  double d = X_WA.translation().dot(unit_vector_W);
  return {d - max_value, d + max_value};
}

std::optional<Vector3d> MaybeMakeSeparatingVector(
    const fcl::Boxd& box_A, const fcl::Boxd& box_B, const RigidTransformd& X_WA,
    const RigidTransformd& X_WB, const std::vector<Vector3d>& v_Ws) {
  constexpr double kEps = 1e-14;
  for (const Vector3d& v_W : v_Ws) {
    double v_W_norm = v_W.norm();
    if (v_W_norm < kEps) {
      continue;
    }
    const Vector3d unit_vector_W = v_W / v_W_norm;

    const auto [min_A, max_A] = ProjectedMinMax(box_A, X_WA, unit_vector_W);
    const auto [min_B, max_B] = ProjectedMinMax(box_B, X_WB, unit_vector_W);

    // The caller (BoxBoxGradient()) already tested that the two witness points
    // are closer than kEps, so the two touching boxes should numerically
    // overlap within that order of precision along the separating vector.
    // Empirically we found that we lost three more bits of precisions from
    // the projection procedures, so we use the 8 * kEps threshold.
    if (max_A < min_B + 8 * kEps) {
      return -unit_vector_W;
    }
    if (max_B < min_A + 8 * kEps) {
      return unit_vector_W;
    }
  }
  return {};
}

Vector3d BoxBoxGradient(const fcl::Boxd& box_A, const fcl::Boxd& box_B,
                        const math::RigidTransformd& X_WA,
                        const math::RigidTransformd& X_WB,
                        const Eigen::Vector3d& p_ACa,
                        const Eigen::Vector3d& p_BCb) {
  // Verify that the witness points co-locate.
  const double kEps = 1e-14;
  DRAKE_DEMAND((X_WA * p_ACa - X_WB * p_BCb).norm() < kEps);

  const Vector3d v_A = PointOnBoxSurfaceHelper(p_ACa, box_A);
  const Vector3d v_B = PointOnBoxSurfaceHelper(p_BCb, box_B);
  double s_A = v_A.sum();
  double s_B = v_B.sum();

  // Face-to-*. Use the face normal if a witness point is strictly inside
  // a face (but neither edge nor vertex).

  // p_ACa is on a face of box_A, determine the sign of the face normal, and
  // transform to world coordinates.
  if (s_A == 1) {
    const Vector3d normal_A = (v_A.dot(p_ACa) > 0 ? v_A : -v_A);
    return -(X_WA.rotation() * normal_A);
  }
  // p_BCb is on a face of box_B, determine the sign of the face normal, and
  // transform to world coordinates.
  if (s_B == 1) {
    // p_BCb is on a face of B
    const Vector3d normal_B = (v_B.dot(p_BCb) > 0 ? v_B : -v_B);
    return X_WB.rotation() * normal_B;
  }

  // The index of the non-zero entry determines the axis of the edge that either
  // Ca or Cb lies on. Note that axis_index_A is valid only when s_A == 2,
  // and axis_index_B is valid only when s_B == 2.  We will use them under
  // those conditions only.
  const int axis_index_A = (Vector3d::Ones() - v_A).dot(Vector3d(0, 1, 2));
  const int axis_index_B = (Vector3d::Ones() - v_B).dot(Vector3d(0, 1, 2));
  // Edge-to-edge.
  if (s_A == 2 && s_B == 2) {
    const Vector3d cross_product_W =
        X_WA.rotation()
            .col(axis_index_A)
            .cross(X_WB.rotation().col(axis_index_B));
    if (cross_product_W.norm() > kEps) {
      // The two edges are not parallel.
      const std::optional<Vector3d> nhat_BA_W = MaybeMakeSeparatingVector(
          box_A, box_B, X_WA, X_WB, {cross_product_W});
      if (nhat_BA_W.has_value()) {
        return nhat_BA_W.value();
      }
    } else {
      // Parallel edge-to-edge. A separating plane passes through at least
      // one of the two faces sharing the edge in Box A.
      const std::optional<Vector3d> nhat_BA_W = MaybeMakeSeparatingVector(
          box_A, box_B, X_WA, X_WB,
          {X_WA.rotation().col((axis_index_A + 1) % 3),
           X_WA.rotation().col((axis_index_A + 2) % 3)});
      if (nhat_BA_W.has_value()) {
        return nhat_BA_W.value();
      }
    }
  }

  // Edge-vertex. An edge in Box A touches a vertex in Box B. A separating
  // plane passes through the edge in Box A and one of the edges incident
  // to the vertex in Box B.
  // Ca is strictly on an edge and Cb is strictly on a vertex.
  if (s_A == 2 && s_B == 3) {
    const Vector3d edge_vector_A_W = X_WA.rotation().col(axis_index_A);
    const std::optional<Vector3d> nhat_BA_W = MaybeMakeSeparatingVector(
        box_A, box_B, X_WA, X_WB,
        {edge_vector_A_W.cross(X_WB.rotation().col(0)),
         edge_vector_A_W.cross(X_WB.rotation().col(1)),
         edge_vector_A_W.cross(X_WB.rotation().col(2))});
    if (nhat_BA_W.has_value()) {
      return nhat_BA_W.value();
    }
  }
  // Vertex-edge. This is the symmetric case of the above case.
  // Ca is strictly on a vertex and Cb is strictly on an edge.
  if (s_A == 3 && s_B == 2) {
    const Vector3d edge_vector_B_W = X_WB.rotation().col(axis_index_B);
    const std::optional<Vector3d> nhat_BA_W = MaybeMakeSeparatingVector(
        box_A, box_B, X_WA, X_WB,
        {edge_vector_B_W.cross(X_WA.rotation().col(0)),
         edge_vector_B_W.cross(X_WA.rotation().col(1)),
         edge_vector_B_W.cross(X_WA.rotation().col(2))});
    if (nhat_BA_W.has_value()) {
      return nhat_BA_W.value();
    }
  }

  // Vertex-vertex and catch-all cases.
  // Unlike other cases above, first we search for a separating vector from
  // the three axes of each box. If the search fail, we search from their
  // cross products.
  std::optional<Vector3d> nhat_BA_W = MaybeMakeSeparatingVector(
      box_A, box_B, X_WA, X_WB,
      {X_WA.rotation().col(0), X_WA.rotation().col(1), X_WA.rotation().col(2),
       X_WB.rotation().col(0), X_WB.rotation().col(1), X_WB.rotation().col(2)});
  if (nhat_BA_W.has_value()) {
    return nhat_BA_W.value();
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      nhat_BA_W = MaybeMakeSeparatingVector(
          box_A, box_B, X_WA, X_WB,
          {X_WA.rotation().col(i).cross(X_WB.rotation().col(j))});
      if (nhat_BA_W.has_value()) {
        return nhat_BA_W.value();
      }
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
