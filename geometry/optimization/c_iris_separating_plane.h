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

#include "drake/geometry/optimization/cspace_separating_plane.h"

namespace drake {
namespace geometry {
namespace optimization {
/** The separating plane aᵀx + b ≥ δ, aᵀx+b ≤ −δ has parameters a and b. These
 parameters a polynomial function of `s_for_plane` with the specified degree.
 `s_for_plane` is a sub
 set of the configuration-space variable `s`, please refer
 to RationalForwardKinematics class or the paper above on the meaning of s.
 */
// TODO(hongkai.dai): Deprecate this class.
enum class SeparatingPlaneOrder {
  kAffine = 1,  ///< a and b are affine function of s.
  // N.B. Never add new enum values here!
};

template <typename T>
struct CIrisSeparatingPlane : public CSpaceSeparatingPlane<T> {
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
void CalcPlane(const VectorX<D>& decision_variables,
               const VectorX<S>& s_for_plane, SeparatingPlaneOrder order,
               Vector3<V>* a_val, V* b_val) {
  // Cross-check the hard-coded plane_degree immediately below.
  DRAKE_DEMAND(order == SeparatingPlaneOrder::kAffine);
  const int plane_degree = 1;
  CalcPlane(decision_variables, s_for_plane, plane_degree, a_val, b_val);
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
