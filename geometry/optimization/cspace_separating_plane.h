/** @file
 This file is used in our C-IRIS algorithm and will be used in our C-Space path
 certifier, which certifies collision-free region in the configuration space, by
 finding separating planes for each pair of geometry over all configurations in
 a C-space region. For the detailed algorithm please refer to the paper <pre>
 Certified Polyhedral Decompositions of Collision-Free Configuration Space
 by Hongkai Dai*, Alexandre Amice*, Peter Werner, Annan Zhang and Russ Tedrake.
 </pre>
 */
#pragma once

#include <utility>

#include "drake/common/symbolic/polynomial.h"
#include "drake/geometry/optimization/c_iris_collision_geometry.h"

namespace drake {
namespace geometry {
namespace optimization {
/**
 Wraps the information that a pair of collision geometries are separated by a
 plane.
 One collision geometry is on the "positive" side of the separating
 plane, namely {x| aᵀx + b ≥ δ} (with δ ≥ 0}, and the other collision geometry
 is on the "negative" side of the separating plane, namely {x|aᵀx+b ≤ −δ}.
 @tparam T The type of decision_variables. T= symbolic::Variable or double.
 */
template <typename T>
struct CSpaceSeparatingPlane {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CSpaceSeparatingPlane)

  CSpaceSeparatingPlane(
      Vector3<symbolic::Polynomial> m_a, symbolic::Polynomial m_b,
      const CIrisCollisionGeometry* m_positive_side_geometry,
      const CIrisCollisionGeometry* m_negative_side_geometry,
      multibody::BodyIndex m_expressed_body,
      const Eigen::Ref<const VectorX<T>>& m_decision_variables)
      : a{std::move(m_a)},
        b{std::move(m_b)},
        positive_side_geometry{m_positive_side_geometry},
        negative_side_geometry{m_negative_side_geometry},
        expressed_body{m_expressed_body},
        decision_variables{m_decision_variables} {}

  /// Return the geometry on the specified side.
  [[nodiscard]] const CIrisCollisionGeometry* geometry(
      PlaneSide plane_side) const {
    return plane_side == PlaneSide::kPositive ? positive_side_geometry
                                              : negative_side_geometry;
  }

  Vector3<symbolic::Polynomial> a;
  symbolic::Polynomial b;
  const CIrisCollisionGeometry* positive_side_geometry;
  const CIrisCollisionGeometry* negative_side_geometry;
  multibody::BodyIndex expressed_body;
  VectorX<T> decision_variables;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
