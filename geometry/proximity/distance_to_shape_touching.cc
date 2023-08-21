#include "drake/geometry/proximity/distance_to_shape_touching.h"

#include <algorithm>
#include <limits>
#include <vector>

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

using Eigen::Vector3d;
using math::RigidTransformd;

Vector3d FaceNormalIfStrictlyOnFace(const Vector3d& p_BQ,
                                    const fcl::Boxd& box_B) {
  const Vector3d half_size = box_B.side / 2.0;
  const double kEps = 1e-14;
  int count = 0;
  using std::abs;
  Vector3d candidate_direction(0, 0, 0);
  for (int i = 0; i < 3; ++i) {
    double diff = abs(half_size(i) - abs(p_BQ(i)));
    if (diff <= kEps) {
      ++count;
      if (p_BQ(i) > 0) {
        candidate_direction(i) = 1.0;
      } else {
        candidate_direction(i) = -1.0;
      }
    }
  }
  if (count == 1) {
    return candidate_direction;
  }
  const double kNan = std::numeric_limits<double>::quiet_NaN();
  return Vector3d(kNan, kNan, kNan);
}

int AxisIndexIfStrictlyOnEdge(const Eigen::Vector3d& p_BQ,
                              const fcl::Boxd& box_B) {
  const Vector3d half_size = box_B.side / 2.0;
  const double kEps = 1e-14;
  int count = 0;
  using std::abs;
  int candidate_axis = -2;
  for (int i = 0; i < 3; ++i) {
    double diff = abs(half_size(i) - abs(p_BQ(i)));
    if (diff <= kEps) {
      ++count;
    } else {
      candidate_axis = i;
    }
  }
  if (count == 2) {
    DRAKE_THROW_UNLESS(0 <= candidate_axis && candidate_axis < 3);
    return candidate_axis;
  }
  return -1;
}

bool IsAtVertex(const Vector3d& p_BQ, const fcl::Boxd& box_B) {
  const Vector3d half_size = box_B.side / 2.0;
  const double kEps = 1e-14;
  int count = 0;
  using std::abs;
  for (int i = 0; i < 3; ++i) {
    double diff = abs(half_size(i) - abs(p_BQ(i)));
    if (diff <= kEps) {
      ++count;
    }
  }
  if (count == 3) {
    return true;
  }
  return false;
}

std::pair<double, double> ProjectedMinMax(const fcl::Boxd& box_A,
                                          const RigidTransformd& X_WA,
                                          const Vector3d& unit_vector_W) {
  const Vector3d half_size_A = box_A.side / 2.0;
  double min_value = std::numeric_limits<double>::infinity();
  double max_value = -std::numeric_limits<double>::infinity();
  for (const double x : {half_size_A.x(), -half_size_A.x()}) {
    for (const double y : {half_size_A.y(), -half_size_A.y()}) {
      for (const double z : {half_size_A.z(), -half_size_A.z()}) {
        const Vector3d p_AV(x, y, z);
        const Vector3d p_WV = X_WA * p_AV;
        const double projected = unit_vector_W.dot(p_WV);
        min_value = std::min(projected, min_value);
        max_value = std::max(projected, max_value);
      }
    }
  }
  return {min_value, max_value};
}

Vector3d MakeSeparatingVector(const fcl::Boxd& box_A, const fcl::Boxd& box_B,
                              const RigidTransformd& X_WA,
                              const RigidTransformd& X_WB,
                              const std::vector<Vector3d>& v_Ws) {
  const double kEps = 1e-14;
  for (const Vector3d& v_W : v_Ws) {
    if (v_W.norm() < kEps) {
      continue;
    }
    const Vector3d unit_vector_W = v_W.normalized();

    double min_A, max_A;
    std::tie(min_A, max_A) = ProjectedMinMax(box_A, X_WA, unit_vector_W);
    double min_B, max_B;
    std::tie(min_B, max_B) = ProjectedMinMax(box_B, X_WB, unit_vector_W);

    if (max_A < min_B + kEps) {
      return -unit_vector_W;
    }
    if (max_B < min_A + kEps) {
      return unit_vector_W;
    }
  }
  const double kNan = std::numeric_limits<double>::quiet_NaN();
  return Vector3d(kNan, kNan, kNan);
}

Vector3d BoxBoxGradient(const fcl::Boxd& box_A,
                        const fcl::Boxd& box_B,
                        const math::RigidTransformd& X_WA,
                        const math::RigidTransformd& X_WB,
                        const Eigen::Vector3d& p_ACa,
                        const Eigen::Vector3d& p_BCb) {
  // Verify that the two boxes touch by checking that their witness points
  // co-locate.
  DRAKE_THROW_UNLESS((X_WA * p_ACa - X_WB * p_BCb).norm() < 1e-14);

  // Face-to-*. Use the face normal if a witness point is strictly inside
  // a face (but neither edge nor vertex).
  const Vector3d normal_A = FaceNormalIfStrictlyOnFace(p_ACa, box_A);
  if (!normal_A.array().isNaN().any()) {
    return -(X_WA.rotation() * normal_A);
  }
  const Vector3d normal_B = FaceNormalIfStrictlyOnFace(p_BCb, box_B);
  if (!normal_B.array().isNaN().any()) {
    return X_WB.rotation() * normal_B;
  }

  // Edge-to-edge.
  const int axis_index_A = AxisIndexIfStrictlyOnEdge(p_ACa, box_A);
  const int axis_index_B = AxisIndexIfStrictlyOnEdge(p_BCb, box_B);
  const bool Ca_is_strictly_on_edge = (axis_index_A != -1);
  const bool Cb_is_strictly_on_edge = (axis_index_B != -1);
  if (Ca_is_strictly_on_edge && Cb_is_strictly_on_edge) {
    // Non-parallel edge-to-edge. Edge Ea in Box A and edge Eb in Box B crosses
    // each other.  A separation plane passes through Ea and Eb. Its separation
    // axis is along the crosss product of the edge directions of Ea and Eb.
    const Vector3d nhat_BA_W =
        MakeSeparatingVector(box_A, box_B, X_WA, X_WB,
                             std::vector<Vector3d>{
                                 X_WA.rotation().col(axis_index_A).cross(
                                     X_WB.rotation().col(axis_index_B))});
    if (!nhat_BA_W.array().isNaN().any()) {
      return nhat_BA_W;
    } else {
      // Parallel edge-to-edge. An edge in Box A and an edge in Box B
      // overlaps. A separation plane passes through at least one of the
      // two faces sharing the edge. For simplicity, we check all three
      // candidates instead of just two candidates.
      return MakeSeparatingVector(
          box_A, box_B, X_WA, X_WB,
          std::vector<Vector3d>{X_WA.rotation().col(0),
                                X_WA.rotation().col(1),
                                X_WA.rotation().col(2)});
    }
  }

  // Edge-vertex. An edge in Box A touches a vertex in Box B. A separation
  // plane passes through that edge in Box A and one of the edges of that
  // vertex in Box B.
  const bool Cb_is_at_a_vertex = IsAtVertex(p_BCb, box_B);
  if (Ca_is_strictly_on_edge && Cb_is_at_a_vertex) {
    const Vector3d edge_vector_A_W = X_WA.rotation().col(axis_index_A);
    const Vector3d nhat_BA_W = MakeSeparatingVector(
        box_A, box_B, X_WA, X_WB,
        std::vector<Vector3d>{
            edge_vector_A_W.cross(X_WB.rotation().col(0)),
            edge_vector_A_W.cross(X_WB.rotation().col(1)),
            edge_vector_A_W.cross(X_WB.rotation().col(2))});
    if (!nhat_BA_W.array().isNaN().any()) {
      return nhat_BA_W;
    }
  }
  // Vertex-edge. This is the symmetric case of the above case.
  const bool Ca_is_at_a_vertex = IsAtVertex(p_ACa, box_A);
  if (Cb_is_strictly_on_edge && Ca_is_at_a_vertex) {
    const Vector3d edge_vector_B_W = X_WB.rotation().col(axis_index_B);
    const Vector3d nhat_BA_W = MakeSeparatingVector(
        box_A, box_B, X_WA, X_WB,
        std::vector<Vector3d>{
            edge_vector_B_W.cross(X_WA.rotation().col(0)),
            edge_vector_B_W.cross(X_WA.rotation().col(1)),
            edge_vector_B_W.cross(X_WA.rotation().col(2))});
    if (!nhat_BA_W.array().isNaN().any()) {
      return nhat_BA_W;
    }
  }

  // Vertex-vertex.
  DRAKE_ASSERT(Ca_is_at_a_vertex && Cb_is_at_a_vertex);
  Vector3d nhat_BA_W = MakeSeparatingVector(
      box_A, box_B, X_WA, X_WB,
      std::vector<Vector3d>{X_WA.rotation().col(0),
                            X_WA.rotation().col(1),
                            X_WA.rotation().col(2),
                            X_WB.rotation().col(0),
                            X_WB.rotation().col(1),
                            X_WB.rotation().col(2)});
  if (!nhat_BA_W.array().isNaN().any()) {
    return nhat_BA_W;
  }
  nhat_BA_W = MakeSeparatingVector(
      box_A, box_B, X_WA, X_WB,
      std::vector<Vector3d>{
          X_WA.rotation().col(0).cross(X_WB.rotation().col(0)),
          X_WA.rotation().col(0).cross(X_WB.rotation().col(1)),
          X_WA.rotation().col(0).cross(X_WB.rotation().col(2)),
          X_WA.rotation().col(1).cross(X_WB.rotation().col(0)),
          X_WA.rotation().col(1).cross(X_WB.rotation().col(1)),
          X_WA.rotation().col(1).cross(X_WB.rotation().col(2)),
          X_WA.rotation().col(2).cross(X_WB.rotation().col(0)),
          X_WA.rotation().col(2).cross(X_WB.rotation().col(1)),
          X_WA.rotation().col(2).cross(X_WB.rotation().col(2))});
  if (!nhat_BA_W.array().isNaN().any()) {
    return nhat_BA_W;
  }

  DRAKE_UNREACHABLE();
}

}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
