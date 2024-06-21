#pragma once

#include <optional>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace optimization {

/**
 * Certifying a path as collision free will require certifying that many matrix
 * SOS conditions poly(μ,y) = [1;y]ᵀP(μ;s)[1;y] ≥ 0 for μ ∈ [0,1] where the s is
 * a multivariate parameter which must be evaluated before the optimization
 * program is solved. This class contains the information for adding the
 * constraint that a polynomial be positive on the unit interval. The polynomial
 * is parametrized as some of its indeterminates needing to be evaluated before
 * being added to the program.
 *
 * @param poly The parametrized polynomial which we will enforce positivity of.
 * This polynomial can be arbitrary degree in the interval variable μ and must
 * be quadratic in all other indeterminates y.
 * @param interval_variable The variable μ associated to the unit interval.
 * @param parameters The parameters which must be evaluated before enforcing the
 * positivity of poly. This is a subset of the polynomial's decision variables.
 */
class ParametrizedPolynomialPositiveOnUnitInterval {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParametrizedPolynomialPositiveOnUnitInterval);

  ParametrizedPolynomialPositiveOnUnitInterval(
      const symbolic::Polynomial& poly,
      const symbolic::Variable& interval_variable,
      const symbolic::Variables& parameters);

  // Add the constraint that this parametrized polynomial is positive on the
  // unit interval. The Environment env must contain an evaluation for all the
  // parameters in parameters_. The MathematicalProgram prog must already
  // contain the indeterminates of psatz_variables_and_psd_constraints_.
  void AddPositivityConstraintToProgram(
      const symbolic::Environment& env,
      solvers::MathematicalProgram* prog) const;

  const symbolic::Variable& get_mu() const { return mu_; }
  const symbolic::Polynomial& get_p() const { return p_; }
  const symbolic::Polynomial& get_poly() const { return poly_; }
  const symbolic::Polynomial& get_lambda() const { return lambda_; }
  const symbolic::Polynomial& get_nu() const { return nu_; }
  const symbolic::Variables& get_parameters() const { return parameters_; }

  const solvers::MathematicalProgram& get_psatz_variables_and_psd_constraints()
      const {
    return psatz_variables_and_psd_constraints_;
  }

 private:
  // TODO(Alexandre.Amice) make all members const.
  // The variable parametrizing the path. We will enforce that a certain
  // polynomial is positive for μ ∈ [0,1].
  const symbolic::Variable mu_;

  // The polynomial that this structure will certify is positive.
  symbolic::Polynomial poly_;

  // Consider polynomial poly(μ,y) = [1;y]ᵀP(μ;s)[1;y] (where μ is univariate
  // and y is multivariate), biforms λ(μ,y) = ∑ ϕᵢ²(μ,y) and ν(μ,y) = ∑
  // ψᵢ²(μ,y), and define q(μ,y) = λ(μ,y) + ν(μ,y)*μ*(1-μ) if deg(poly, μ) = 2d
  // or q(μ,y) = λ(μ,y)*μ + ν(μ,y)*(1-μ) if deg(poly, μ) = 2d + 1. Then
  // poly(μ,y) is positive on the interval μ ∈ [0,1] if and only if p_(μ,y) =
  // poly(μ,y)-q(μ,y) = 0. Moreover, in both the odd and even degree case
  // deg(ϕᵢ, μ) ≤ d, deg(ϕᵢ, y) = 1, and and deg(ψᵢ, y) = 1. In the even case,
  // deg(ψᵢ, μ) ≤ d - 1, and in the odd case deg(ψᵢ, μ) ≤ d.
  //
  // If deg(poly,μ) > 0 we construct the polynomial p_(μ,y) = poly(μ,y)-q(μ,y)
  // which we will later constrain to be equal to 0 when a value of parameters.
  // If deg(poly,μ) = 0, then q is either constrained to be a positive scalar if
  // it has no other indeterminates, or it is constrained to be a sum-of-squares
  // if it is a quadratic form in y.
  symbolic::Polynomial p_;

  // The subset of the decision variables in p_ which must be evaluated before
  // we enforce poly_'s positivity in a Mathematical Program.
  const symbolic::Variables parameters_;

  // The λ(μ,y) in the documentation of p_
  symbolic::Polynomial lambda_{0};
  // The ν(μ,y) in the documentation of p_
  symbolic::Polynomial nu_{0};

  // A program which stores the psd variables and constraints associated to λ
  // and ν. See the description of p_. The use of this member is idiosyncratic,
  // so it is not recommended to provide an accessor to it.
  solvers::MathematicalProgram psatz_variables_and_psd_constraints_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
