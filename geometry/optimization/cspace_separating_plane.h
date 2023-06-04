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

#include "drake/common/symbolic/monomial_util.h"
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
      multibody::BodyIndex m_expressed_body, int m_plane_order,
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
  int plane_order{1};
  VectorX<T> decision_variables;
};

/**
 Computes the parameters a, b in the plane { x | aᵀx+b=0 }.
 a and b are both polynomials of `vars_for_plane`. The coefficients of these
 polynomials are in `decision_variables` in graded reverse lexicographic order.
 The possible combination of D, S, V are
 1. D=symbolic::Variable, S=symbolic::Variable, V=symbolic::Polynomial.
 2. D=double,             S=symbolic::Variable, V=symbolic::Polynomial
 3. D=double,             S=double,             V=double
 */
template <typename D, typename S, typename V>
void CalcPlane(const VectorX<D>& decision_variables,
               const VectorX<S>& vars_for_plane, int plane_degree,
               Vector3<V>* a_val, V* b_val) {
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
  const int num_vars = vars_for_plane.size();
  const int num_coeffs_per_poly =
      symbolic::NChooseK(num_vars + plane_degree, plane_degree);
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
    auto compute_grev_lex_evaluated_powers =
        [&num_vars](const Eigen::Ref<const VectorX<double>>& values,
                    const int degree) {
          symbolic::Variables vars;
          symbolic::Environment evals;
          for (int i = 0; i < num_vars; ++i) {
            const symbolic::Variable cur_var{fmt::format("x{}", i)};
            vars.insert(cur_var);
            evals.insert(cur_var, values(i));
          }
          // Make a grevlex basis that we can now evaluate with the desired
          // values.
          Eigen::VectorX<symbolic::Monomial> basis =
              symbolic::MonomialBasis(vars, degree);
          Eigen::VectorXd ret{basis.size()};
          for (int i = 0; i < basis.size(); ++i) {
            ret(i) = basis(i).Evaluate(evals);
          }
          return ret;
        };
    const Eigen::VectorXd evaluated_powers =
        compute_grev_lex_evaluated_powers(vars_for_plane, plane_degree);
    (*a_val) = (a_coeff * evaluated_powers).topRows(3);
    (*b_val) = (b_coeff.transpose() * evaluated_powers);
    return;
  }
  if constexpr (std::is_same_v<S, symbolic::Variable> &&
                std::is_same_v<V, symbolic::Polynomial>) {
    const Eigen::Matrix<symbolic::Monomial, Eigen::Dynamic, 1> basis =
        symbolic::MonomialBasis(symbolic::Variables{vars_for_plane},
                                plane_degree);
    for (int i = 0; i < 3; ++i) {
      symbolic::Polynomial::MapType monomial_to_coeff_map;
      for (int j = 0; j < basis.size(); ++j) {
        monomial_to_coeff_map.emplace(basis(j), a_coeff(i, j));
      }
      (*a_val)(i) = symbolic::Polynomial(monomial_to_coeff_map);
    }
    symbolic::Polynomial::MapType monomial_to_coeff_map;
    for (int j = 0; j < basis.size(); ++j) {
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
