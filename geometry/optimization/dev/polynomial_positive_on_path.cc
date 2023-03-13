#include "polynomial_positive_on_path.h"
#include "drake/common/symbolic/monomial_util.h"

namespace drake {
namespace geometry {
namespace optimization {

ParametrizedPolynomialPositiveOnUnitInterval::
    ParametrizedPolynomialPositiveOnUnitInterval(
        const symbolic::Polynomial& poly,
        const symbolic::Variable& interval_variable,
        const symbolic::Variables& parameters,
        const std::optional<const symbolic::Variables>& auxillary_variables)
    : p_(poly), parameters_(parameters) {
  int d = poly.Degree(interval_variable);
  const solvers::MathematicalProgram::NonnegativePolynomial type =
      solvers::MathematicalProgram::NonnegativePolynomial::kSos;

  // This basis is [μᵈ, ... μ, 1, y₁, ..., yₙ]
  VectorX<symbolic::Monomial> multiplier_basis_2d{
      d + 1 + (auxillary_variables.value_or(symbolic::Variables())).size()};
  multiplier_basis_2d << symbolic::MonomialBasis({interval_variable}, d);
  if (auxillary_variables.has_value()) {
    const VectorX<symbolic::Monomial> aux_basis{
        symbolic::MonomialBasis(auxillary_variables.value(), d)};
    // don't include the 1 basis element twice.
    multiplier_basis_2d << aux_basis.topRows(d);
  }

  // Constructs the multiplier polynomials and their associated Gram matrices as
  // well as the polynomial p_. Recall that p_ has already been initialized to
  // poly(μ,y).
  if (d > 0) {
    auto [lambda, Q_lambda] =
        psatz_variables_and_psd_constraints_.NewSosPolynomial(
            multiplier_basis_2d, type, "Sl");
    if (d % 2 == 0) {
      auto [nu, Q_nu] = psatz_variables_and_psd_constraints_.NewSosPolynomial(
          multiplier_basis_2d.tail(multiplier_basis_2d.size() -
                                   1),  // exclude μᵈ monomial
          type, "Sv");
      p_ -= lambda + nu * interval_variable *
                         (symbolic::Polynomial(1, {interval_variable}) -
                          interval_variable);
    } else {
      auto [nu, Q_nu] = psatz_variables_and_psd_constraints_.NewSosPolynomial(
          multiplier_basis_2d, type, "Sv");
      p_ -= lambda * interval_variable +
            nu * (symbolic::Polynomial(1, {interval_variable}) -
                  interval_variable);
    }
  } else {
    // If poly is of degree 0, then it is a scalar, and we just need to
    // constraint that p_ >= 0.
    const solvers::VectorXDecisionVariable lambda{
        psatz_variables_and_psd_constraints_.NewContinuousVariables(1, "Sl")};
    psatz_variables_and_psd_constraints_.AddBoundingBoxConstraint(
        0, std::numeric_limits<double>::infinity(), lambda);
    p_ -= lambda(0);
  }
}
//
//void ParametrizedPolynomialPositiveOnUnitInterval::
//    AddPositivityConstraintToProgram(const symbolic::Environment& env,
//                                     solvers::MathematicalProgram* prog) {
//  DRAKE_DEMAND(env.size() == parameters_.size());
//  for (const auto& parameter : parameters_) {
//    DRAKE_DEMAND(env.find(parameter) != env.cend());
//  }
//
//  prog->AddDecisionVariables(
//      psatz_variables_and_psd_constraints_.decision_variables());
//  for (const auto& binding :
//       psatz_variables_and_psd_constraints_.GetAllConstraints()) {
//    prog->AddConstraint(binding);
//  }
//  prog->AddEqualityConstraintBetweenPolynomials(p_.EvaluatePartial(env),
//                                                symbolic::Polynomial());
//}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
