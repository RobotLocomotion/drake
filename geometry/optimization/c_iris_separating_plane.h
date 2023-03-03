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

#include "drake/common/symbolic/polynomial.h"
#include "drake/geometry/optimization/c_iris_collision_geometry.h"

namespace drake {
namespace geometry {
namespace optimization {
/** The separating plane aᵀx + b ≥ δ, aᵀx+b ≤ −δ has parameters a and b. These
 parameters a polynomial function of `s_for_plane` with the specified degree.
 `s_for_plane` is a subset of the configuration-space variable `s`, please refer
 to RationalForwardKinematics class or the paper above on the meaning of s.
 */
// TODO(hongkai.dai): I might support kConstant in the future.
enum class SeparatingPlaneOrder {
  kAffine,  ///< a and b are affine function of s.
};

/**
 Wraps the information that a pair of collision geometries are separated by a
 plane.
 One collision geometry is on the "positive" side of the separating
 plane, namely {x| aᵀx + b ≥ δ} (with δ ≥ 0}, and the other collision geometry
 is on the "negative" side of the separating plane, namely {x|aᵀx+b ≤ −δ}.
 @tparam T The type of decision_variables. T= symbolic::Variable or double.
 */
template <typename T>
struct CIrisSeparatingPlane {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CIrisSeparatingPlane)

  CIrisSeparatingPlane(Vector3<symbolic::Polynomial> m_a,
                       symbolic::Polynomial m_b,
                       const CIrisCollisionGeometry* m_positive_side_geometry,
                       const CIrisCollisionGeometry* m_negative_side_geometry,
                       multibody::BodyIndex m_expressed_body,
                       SeparatingPlaneOrder m_plane_order,
                       const Eigen::Ref<const VectorX<T>>& m_decision_variables)
      : a{std::move(m_a)},
        b{std::move(m_b)},
        positive_side_geometry{m_positive_side_geometry},
        negative_side_geometry{m_negative_side_geometry},
        expressed_body{m_expressed_body},
        plane_order{m_plane_order},
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
  SeparatingPlaneOrder plane_order;
  VectorX<T> decision_variables;
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
  static_assert(
      (std::is_same_v<D, symbolic::Variable> &&
       std::is_same_v<S, symbolic::Variable> &&
       std::is_same_v<V, symbolic::Polynomial>) ||
          (std::is_same_v<D, double> && std::is_same_v<S, symbolic::Variable> &&
           std::is_same_v<V, symbolic::Polynomial>) ||
          (std::is_same_v<D, double> && std::is_same_v<S, double> &&
           std::is_same_v<V, double>),
      "CalcPlane: unsupported scalar type."
      "expected one of (symbolic::Variable, symbolic::Variable, "
      "symbolic::Polynomial), (double, symbolic::Variable, "
      "symbolic::Polynomial), or (double, double double)");

  switch (order) {
    case SeparatingPlaneOrder::kAffine: {
      DRAKE_DEMAND(decision_variables.rows() == 4 * s_for_plane.rows() + 4);
      Eigen::Matrix<D, 3, Eigen::Dynamic> a_coeff(3, s_for_plane.rows());
      int var_count = 0;
      for (int i = 0; i < 3; ++i) {
        a_coeff.row(i) =
            decision_variables.segment(var_count, s_for_plane.rows());
        var_count += s_for_plane.rows();
      }
      const Vector3<D> a_constant =
          decision_variables.template segment<3>(var_count);
      var_count += 3;
      const VectorX<D> b_coeff =
          decision_variables.segment(var_count, s_for_plane.rows());
      var_count += s_for_plane.rows();
      const D& b_constant = decision_variables(var_count);
      var_count++;
      DRAKE_DEMAND(var_count == decision_variables.rows());
      if constexpr (std::is_same_v<D, double> && std::is_same_v<S, double> &&
                    std::is_same_v<V, double>) {
        *a_val = a_coeff * s_for_plane + a_constant;
        *b_val = b_coeff.dot(s_for_plane) + b_constant;
        return;
      }
      if constexpr (std::is_same_v<S, symbolic::Variable> &&
                    std::is_same_v<V, symbolic::Polynomial>) {
        const symbolic::Monomial monomial_one{};
        for (int i = 0; i < 3; ++i) {
          symbolic::Polynomial::MapType monomial_to_coeff_map;
          for (int j = 0; j < s_for_plane.rows(); ++j) {
            monomial_to_coeff_map.emplace(symbolic::Monomial(s_for_plane(j)),
                                          a_coeff(i, j));
          }
          monomial_to_coeff_map.emplace(monomial_one, a_constant(i));
          (*a_val)(i) = symbolic::Polynomial(monomial_to_coeff_map);
        }
        symbolic::Polynomial::MapType monomial_to_coeff_map;
        for (int j = 0; j < s_for_plane.rows(); ++j) {
          monomial_to_coeff_map.emplace(symbolic::Monomial(s_for_plane(j)),
                                        b_coeff(j));
        }
        monomial_to_coeff_map.emplace(monomial_one, b_constant);
        *b_val = symbolic::Polynomial(monomial_to_coeff_map);
        return;
      }
    }
  }
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
