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
      multibody::BodyIndex m_expressed_body, int m_plane_degree,
      const Eigen::Ref<const VectorX<T>>& m_decision_variables)
      : CSpaceSeparatingPlane<T>(m_a, m_b, m_positive_side_geometry,
                                 m_negative_side_geometry, m_expressed_body,
                                 m_decision_variables),
        plane_degree{m_plane_degree} {}

  int plane_degree;
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
void CalcPathPlane(const VectorX<D>& decision_variables, const S& mu_for_plane,
                   int plane_degree, Vector3<V>* a_val, V* b_val) {
  static_assert(
      (std::is_same_v<D, symbolic::Variable> &&
       std::is_same_v<S, symbolic::Variable> &&
       std::is_same_v<V, symbolic::Polynomial>) ||
          (std::is_same_v<D, double> && std::is_same_v<S, symbolic::Variable> &&
           std::is_same_v<V, symbolic::Polynomial>) ||
          (std::is_same_v<D, double> && std::is_same_v<S, double> &&
           std::is_same_v<V, double>),
      "CalcPathPlane: unsupported scalar type."
      "expected one of (symbolic::Variable, symbolic::Variable, "
      "symbolic::Polynomial), (double, symbolic::Variable, "
      "symbolic::Polynomial), or (double, double double)");

  const int num_coeffs_per_poly = plane_degree + 1;
  DRAKE_DEMAND(decision_variables.size() == 4 * num_coeffs_per_poly);
  Eigen::Matrix<D, 3, Eigen::Dynamic> a_coeff(3, num_coeffs_per_poly);
  int var_count = 0;
  for (int i = 0; i < 3; ++i) {
    a_coeff.row(i) = decision_variables.segment(var_count, num_coeffs_per_poly);
    var_count += num_coeffs_per_poly;
  }
  const VectorX<D> b_coeff =
      decision_variables.segment(var_count, num_coeffs_per_poly);
  var_count += num_coeffs_per_poly;
  DRAKE_DEMAND(var_count == decision_variables.size());

  if constexpr (std::is_same_v<D, double> && std::is_same_v<S, double> &&
                std::is_same_v<V, double>) {
    for (int j = 0; j < num_coeffs_per_poly; ++j) {
      for (int i = 0; i < 3; ++i) {
        (*a_val)(i) += a_coeff(i, j) * pow(mu_for_plane, plane_degree - j);
      }
      (*b_val) += b_coeff(j) * pow(mu_for_plane, plane_degree - j);
    }
    return;
  }
  if constexpr (std::is_same_v<S, symbolic::Variable> &&
                std::is_same_v<V, symbolic::Polynomial>) {
    const Eigen::Matrix<symbolic::Monomial, Eigen::Dynamic, 1> basis =
        symbolic::MonomialBasis(symbolic::Variables{mu_for_plane},
                                plane_degree);
    for (int i = 0; i < 3; ++i) {
      symbolic::Polynomial::MapType monomial_to_coeff_map;
      for (int j = 0; j < basis.rows(); ++j) {
        monomial_to_coeff_map.emplace(basis(j), a_coeff(i, j));
      }
      (*a_val)(i) = symbolic::Polynomial(monomial_to_coeff_map);
    }
    symbolic::Polynomial::MapType monomial_to_coeff_map;
    for (int j = 0; j < basis.rows(); ++j) {
      monomial_to_coeff_map.emplace(basis(j), b_coeff(j));
    }
    *b_val = symbolic::Polynomial(monomial_to_coeff_map);
    return;
  }
  DRAKE_UNREACHABLE();
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
