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
/** The separating plane aᵀx + b ≥ δ, aᵀx + b ≤ −δ has parameters a and b.
 These parameterize a polynomial function of `s_for_plane` with the
 specified order.  `s_for_plane` is a sub set of the configuration-space
 variable `s`, please refer to the RationalForwardKinematics class or
 the paper above on the meaning of s.
 */
enum class SeparatingPlaneOrder {
  kAffine = 1,  ///< a and b are affine functions of s.
};

/** Convert an integer degree to the SeparatingPlaneOrder */
[[nodiscard]] SeparatingPlaneOrder ToPlaneOrder(int plane_degree);

/** Convert SeparatingPlaneOrder to an integer degree. */
[[nodiscard]] int ToPlaneDegree(SeparatingPlaneOrder plane_order);

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CSpaceSeparatingPlane);

  CSpaceSeparatingPlane(
      Vector3<symbolic::Polynomial> m_a, symbolic::Polynomial m_b,
      const CIrisCollisionGeometry* m_positive_side_geometry,
      const CIrisCollisionGeometry* m_negative_side_geometry,
      multibody::BodyIndex m_expressed_body, int m_plane_degree,
      const Eigen::Ref<const VectorX<T>>& m_decision_variables)
      : a{std::move(m_a)},
        b{std::move(m_b)},
        positive_side_geometry{m_positive_side_geometry},
        negative_side_geometry{m_negative_side_geometry},
        expressed_body{m_expressed_body},
        plane_degree{m_plane_degree},
        decision_variables{m_decision_variables} {}

  /// Return the geometry on the specified side.
  [[nodiscard]] const CIrisCollisionGeometry* geometry(
      PlaneSide plane_side) const {
    return plane_side == PlaneSide::kPositive ? positive_side_geometry
                                              : negative_side_geometry;
  }

  [[nodiscard]] SortedPair<geometry::GeometryId> geometry_pair() const {
    return SortedPair<geometry::GeometryId>(positive_side_geometry->id(),
                                            negative_side_geometry->id());
  }

  Vector3<symbolic::Polynomial> a;
  symbolic::Polynomial b;
  const CIrisCollisionGeometry* positive_side_geometry;
  const CIrisCollisionGeometry* negative_side_geometry;
  multibody::BodyIndex expressed_body;
  int plane_degree{1};
  VectorX<T> decision_variables;
};

namespace internal {

void CalcPlane(const VectorX<symbolic::Variable>& decision_variables,
               const VectorX<symbolic::Variable>& vars_for_plane,
               int plane_degree, Vector3<symbolic::Polynomial>* a_val,
               symbolic::Polynomial* b_val);

void CalcPlane(const VectorX<double>& decision_variables,
               const VectorX<symbolic::Variable>& vars_for_plane,
               int plane_degree, Vector3<symbolic::Polynomial>* a_val,
               symbolic::Polynomial* b_val);

void CalcPlane(const VectorX<double>& decision_variables,
               const VectorX<double>& vars_for_plane, int plane_degree,
               Vector3<double>* a_val, double* b_val);

}  // namespace internal

/**
 Computes the parameters a, b in the plane { x | aᵀx+b=0 }.
 a and b are both polynomials of `vars_for_plane`. The coefficients of these
 polynomials are in `decision_variables` in graded reverse lexicographic order.

 @tparam D, S, V The valid combination of D, S, V are
 1. D=symbolic::Variable, S=symbolic::Variable, V=symbolic::Polynomial.
 2. D=double,             S=symbolic::Variable, V=symbolic::Polynomial
 3. D=double,             S=double,             V=double
 */
template <typename D, typename S, typename V>
void CalcPlane(const VectorX<D>& decision_variables,
               const VectorX<S>& s_for_plane, int plane_degree,
               Vector3<V>* a_val, V* b_val) {
  internal::CalcPlane(decision_variables, s_for_plane, plane_degree, a_val,
                      b_val);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
