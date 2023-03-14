#pragma once

#include <optional>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace optimization {

/**
 * Certifying a path as collision free will require certifying that many matrix
 * SOS conditions yᵀP(μ;s)y ≥ 0 for μ ∈ [0,1] where the s is a multivariate
 * parameter which must be evaluated before the optimization program is solved.
 * This class contains the information for adding the constraint that a
 * polynomial be positive on the unit interval. The polynomial is parametrized
 * as some of its indeterminates needing to be evaluated before being added to
 * the program.
 *
 * @param poly The parametrized polynomial which we will enforce positivity of.
 * This polynomial can be arbitrary degree in the interval variable and must be
 * quadratic in all other indeterminates.
 * @param interval_variable The variable μ associated to the unit interval.
 * @param parameters The parameters which must be evaluated before enforcing the
 * positivity of poly.
 * @param auxillary_variables If poly is the polynomial associated to a
 * univariate matrix SOS program, these are the auxillary variables used to
 * convert the matrix SOS to a single polynomial.
 */
class ParametrizedPolynomialPositiveOnUnitInterval {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParametrizedPolynomialPositiveOnUnitInterval)

  ParametrizedPolynomialPositiveOnUnitInterval(
      const symbolic::Polynomial& poly,
      const symbolic::Variable& interval_variable);

  // Add the constraint that this parametrized polynomial is positive on the
  // unit interval. The Environment env must contain an evaluation for all the
  // parameters in parameters_. The MathematicalProgram prog must already
  // contain the indeterminates of psatz_variables_and_psd_constraints_.
  void AddPositivityConstraintToProgram(const symbolic::Environment& env,
                                        solvers::MathematicalProgram* prog);

  const symbolic::Polynomial& get_p() const { return p_; }
  const symbolic::Polynomial& get_lambda() const { return lambda_; }
  const symbolic::Polynomial& get_nu() const { return nu_; }

  const solvers::MathematicalProgram* get_psatz_variables_and_psd_constraints()
      const {
    return &psatz_variables_and_psd_constraints_;
  }

 private:
  // TODO(Alexandre.Amice) make members const.

  // A polynomial q(μ,y) = ∑ f(μ)yᵢyⱼ (where μ is univariate and y is
  // multivariate) is positive on the interval μ ∈ [0,1] if and only if there
  // exists biforms λ(μ,y) = ∑ ϕᵢ²(μ,y) and ν(μ,y) = ∑ ψᵢ²(μ,y) such that
  // q(μ,y) = λ(μ,y) + ν(μ,y)*μ*(1-μ) if deg(q, μ) = 2d or q(μ,y) = λ(μ,y)*μ +
  // ν(μ,y)*(1-μ) if deg(q, μ) = 2d + 1. Moreover, in both cases deg(ϕᵢ, μ) ≤ d,
  // deg(ϕᵢ, y) = 1, and and deg(ψᵢ, y) = 1. In the first case
  // deg(ψᵢ, μ) ≤ d - 1, and in the second deg(ψᵢ, μ) ≤ d. If deg(poly,μ) > 0 we
  // construct the polynomial p_(μ,y) = poly(μ,y)-q(μ,y) which we will later
  // constrain to be equal to 0.
  symbolic::Polynomial p_;

  // The λ(μ,y) in the documentation of p_
  symbolic::Polynomial lambda_{0};
  // The ν(μ,y) in the documentation of p_
  symbolic::Polynomial nu_{0};

  // A program which stores the psd variables and constraints associated to λ
  // and ν. See the description of p_.
  solvers::MathematicalProgram psatz_variables_and_psd_constraints_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
