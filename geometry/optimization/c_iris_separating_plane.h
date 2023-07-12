/** @file
 This file is used in our C-IRIS algorithm, which certifies collision-free
 region in the configuration space, by finding separating planes for each pair
 of geometry over all configurations in a C-space region. For the detailed
 algorithm please refer to the paper
 <pre>
 Certified Polyhedral Decompositions of Collision-Free Configuration Space
 by Hongkai Dai*, Alexandre Amice*, Peter Werner, Annan Zhang and Russ Tedrake.
 </pre>
 */
#pragma once
#include <utility>

#include "drake/common/drake_deprecated.h"
#include "drake/geometry/optimization/cspace_separating_plane.h"

namespace drake {
namespace geometry {
namespace optimization {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template <typename T>
struct DRAKE_DEPRECATED("2023-12-01", "Use CSpaceSeparatingPlane instead.")
    CIrisSeparatingPlane : public CSpaceSeparatingPlane<T> {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CIrisSeparatingPlane)

  CIrisSeparatingPlane(Vector3<symbolic::Polynomial> m_a,
                       symbolic::Polynomial m_b,
                       const CIrisCollisionGeometry* m_positive_side_geometry,
                       const CIrisCollisionGeometry* m_negative_side_geometry,
                       multibody::BodyIndex m_expressed_body,
                       SeparatingPlaneOrder m_plane_order,
                       const Eigen::Ref<const VectorX<T>>& m_decision_variables)
      : CSpaceSeparatingPlane<T>(m_a, m_b, m_positive_side_geometry,
                                 m_negative_side_geometry, m_expressed_body,
                                 1 /* plane_degree */, m_decision_variables) {
    // Cross-check the hard-coded plane_degree for CSpaceSeparatingPlane.
    DRAKE_DEMAND(m_plane_order == SeparatingPlaneOrder::kAffine);
  }
};

/**
 Computes the parameters a, b in the plane { x | aᵀx+b=0 }.
 a and b are both polynomials of `s_for_plane`. The coefficients of these
 polynomials are in `decision_variables`.
 The possible combination of D, S, V are
 1. D=symbolic::Variable, S=symbolic::Variable, V=symbolic::Polynomial.
 2. D=double,             S=symbolic::Variable, V=symbolic::Polynomial
 3. D=double,             S=double,             V=double
 */
template <typename D, typename S, typename V>
DRAKE_DEPRECATED("2023-12-01",
                 "Use CalcPlane with plane degree being an integer instead of "
                 "SeparatingPlaneOrder enum.")
void CalcPlane(const VectorX<D>& decision_variables,
               const VectorX<S>& s_for_plane, SeparatingPlaneOrder order,
               Vector3<V>* a_val, V* b_val) {
  // Cross-check the hard-coded plane_degree immediately below.
  DRAKE_DEMAND(order == SeparatingPlaneOrder::kAffine);
  const int plane_degree = 1;
  CalcPlane(decision_variables, s_for_plane, plane_degree, a_val, b_val);
}
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
