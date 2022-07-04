#include "drake/multibody/dev/c_iris/rational_forward_kinematics.h"

#include <set>

namespace drake {
namespace multibody {
namespace c_iris {
namespace {
bool CheckPolynomialIndeterminatesAreCosSinDelta(
    const symbolic::Polynomial& e_poly,
    const VectorX<symbolic::Variable>& cos_vars,
    const VectorX<symbolic::Variable>& sin_vars) {
  VectorX<symbolic::Variable> cos_sin_vars(cos_vars.rows() + sin_vars.rows());
  cos_sin_vars << cos_vars, sin_vars;
  const symbolic::Variables cos_sin_vars_variables(cos_sin_vars);
  return e_poly.indeterminates().IsSubsetOf(cos_sin_vars_variables);
}

void ReplaceCosAndSinWithRationalFunction(
    const symbolic::Polynomial& e_poly,
    const VectorX<symbolic::Variable>& cos_vars,
    const VectorX<symbolic::Variable>& sin_vars,
    const VectorX<symbolic::Variable>& t, const symbolic::Variables& t_set,
    const VectorX<symbolic::Polynomial>& one_plus_t_angles_squared,
    const VectorX<symbolic::Polynomial>& two_t_angles,
    const VectorX<symbolic::Polynomial>& one_minus_t_angles_squared,
    symbolic::RationalFunction* e_rational) {
  DRAKE_DEMAND(cos_vars.rows() == sin_vars.rows());
  DRAKE_DEMAND(cos_vars.rows() == t.rows());
  DRAKE_DEMAND(
      CheckPolynomialIndeterminatesAreCosSinDelta(e_poly, cos_vars, sin_vars));
  // First find the angles whose cos or sin appear in the polynomial. This
  // will determine the denominator of the rational function.
  std::set<int> angle_indices;
  for (const auto& pair : e_poly.monomial_to_coefficient_map()) {
    // Also check that this monomial can't contain both cos_vars(i) and
    // sin_vars(i).
    for (int i = 0; i < cos_vars.rows(); ++i) {
      const int angle_degree =
          pair.first.degree(cos_vars(i)) + pair.first.degree(sin_vars(i));
      DRAKE_DEMAND(angle_degree <= 1);
      if (angle_degree == 1) {
        angle_indices.insert(i);
      }
    }
  }
  if (angle_indices.empty()) {
    *e_rational = symbolic::RationalFunction(
        symbolic::Polynomial(e_poly.ToExpression(), t_set));
    return;
  }
  const symbolic::Monomial monomial_one{};
  symbolic::Polynomial denominator{1};
  for (int angle_index : angle_indices) {
    // denominator *= (1 + t(angle_index)^2)
    denominator *= one_plus_t_angles_squared[angle_index];
  }
  symbolic::Polynomial numerator{};

  for (const auto& [monomial, coeff] : e_poly.monomial_to_coefficient_map()) {
    // If the monomial contains cos_vars(i), then replace cos_vars(i) with
    // 1 - t(i) * t(i).
    // If the monomial contains sin_vars(i), then replace sin_vars(i) with
    // 2 * t(i).
    // Otherwise, multiplies with 1 + t(i) * t(i)

    // The coefficient could contain "t_set", (the indeterminates for e are
    // cos_vars and sin_vars). Hence we first need to write the coefficient
    // as a polynomial of indeterminates interset(t_set, coeff.variables()).
    symbolic::Polynomial numerator_term(
        coeff, symbolic::intersect(t_set, coeff.GetVariables()));
    for (int angle_index : angle_indices) {
      if (monomial.degree(cos_vars(angle_index)) > 0) {
        numerator_term *= one_minus_t_angles_squared[angle_index];
      } else if (monomial.degree(sin_vars(angle_index)) > 0) {
        numerator_term *= two_t_angles[angle_index];
      } else {
        numerator_term *= one_plus_t_angles_squared[angle_index];
      }
    }
    numerator += numerator_term;
  }

  *e_rational = symbolic::RationalFunction(numerator, denominator);
}
}  // namespace

void ReplaceCosAndSinWithRationalFunction(
    const symbolic::Polynomial& e_poly,
    const VectorX<symbolic::Variable>& cos_vars,
    const VectorX<symbolic::Variable>& sin_vars,
    const VectorX<symbolic::Variable>& t,
    symbolic::RationalFunction* e_rational) {
  const symbolic::Monomial monomial_one{};
  VectorX<symbolic::Polynomial> one_minus_t_square(t.rows());
  VectorX<symbolic::Polynomial> two_t(t.rows());
  VectorX<symbolic::Polynomial> one_plus_t_square(t.rows());
  for (int i = 0; i < t.rows(); ++i) {
    one_minus_t_square[i] = symbolic::Polynomial(
        {{monomial_one, 1}, {symbolic::Monomial(t(i), 2), -1}});
    two_t[i] = symbolic::Polynomial({{symbolic::Monomial(t(i), 1), 2}});
    one_plus_t_square[i] = symbolic::Polynomial(
        {{monomial_one, 1}, {symbolic::Monomial(t(i), 2), 1}});
  }
  const symbolic::Variables t_set{t};
  ReplaceCosAndSinWithRationalFunction(e_poly, cos_vars, sin_vars, t, t_set,
                                       one_plus_t_square, two_t,
                                       one_minus_t_square, e_rational);
}
}  // namespace c_iris
}  // namespace multibody
}  // namespace drake
