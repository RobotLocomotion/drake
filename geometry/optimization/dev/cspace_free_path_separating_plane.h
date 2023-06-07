/** @file
 This file is used in our C-Space Free Path algorithm, which certifies
 collision-free path in the tangent configuration space, by finding separating
 planes for each pair of geometry over all configurations in a path.
 */
#pragma once

#include "drake/common/symbolic/monomial_util.h"
#include "drake/geometry/optimization/cspace_separating_plane.h"

namespace drake {
namespace geometry {
namespace optimization {

template <typename T>
struct CSpacePathSeparatingPlane : public CSpaceSeparatingPlane<T> {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CSpacePathSeparatingPlane)

  CSpacePathSeparatingPlane(
      Vector3<symbolic::Polynomial> m_a, symbolic::Polynomial m_b,
      const CIrisCollisionGeometry* m_positive_side_geometry,
      const CIrisCollisionGeometry* m_negative_side_geometry,
      multibody::BodyIndex m_expressed_body, int m_plane_order,
      const Eigen::Ref<const VectorX<T>>& m_decision_variables)
      : CSpaceSeparatingPlane<T>(m_a, m_b, m_positive_side_geometry,
                                 m_negative_side_geometry, m_expressed_body,
                                 m_plane_order,
                                 m_decision_variables) {}
};

/**
 Computes the parameters a, b in the plane { x | aᵀx+b=0 }.
 a and b are both univariate polynomials. The coefficients of these
 polynomials are in `decision_variables` in graded lexicographic order.
 The possible combination of D, S, V are
 1. D=symbolic::Variable, S=symbolic::Variable, V=symbolic::Polynomial.
 2. D=double,             S=symbolic::Variable, V=symbolic::Polynomial
 3. D=double,             S=double,             V=double
 */
template <typename D, typename S, typename V>
typename std::enable_if<std::is_same_v<S, symbolic::Variable> ||
                        std::is_same_v<S, double>>::type
CalcPlane(const VectorX<D>& decision_variables, const S& mu_for_plane,
          int plane_degree, Vector3<V>* a_val, V* b_val) {
  CalcPlane(decision_variables, Vector1<S>(mu_for_plane), plane_degree,
                     a_val, b_val);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
