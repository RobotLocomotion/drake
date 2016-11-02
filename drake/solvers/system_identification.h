#pragma once

#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/polynomial.h"
#include "drake/common/trig_poly.h"

#include "drake/common/drake_export.h"

namespace drake {
namespace solvers {

/// Utility functions for system identification.
/**
 * This class is a holder for templated utility methods.  It should not be
 * constructed.  It must be template-instantiated (in its cpp file) for each
 * supported variant of Polynomial (currently only Polynomial<double>).
 *
 * For the purposes of system identification we require here that the set of
 * variables in a polynomial can be divided into two groups:
 *
 *  * "Parameter variables" are unchanged through many evaluations of the
 *    Polynomial and so may be factored and renamed (lumped) at will.  In
 *    effect parameter variables are treated as constants.
 *
 *  * "Active variables" are those that may vary between evaluations of the
 *    Polynomial, for instance because they are inputs or state variables
 *    of the system.
 *
 * Note: The term "system identification" used throughout here refers to the
 * process of simplifying the equations defining a physical system to a
 * minimum number of "lumped" parameters and then estimating the values of
 * those parameters based on empirical data.
 */
template <typename CoefficientType>
class DRAKE_EXPORT SystemIdentification {
 public:
  typedef ::Polynomial<CoefficientType> PolyType;
  typedef typename PolyType::Monomial MonomialType;
  typedef typename PolyType::Term TermType;
  typedef typename PolyType::VarType VarType;
  typedef std::map<PolyType, VarType> LumpingMapType;
  typedef std::map<VarType, CoefficientType> PartialEvalType;

  /// Extract lumped parameters from a given polynomial.
  /**
   * Given a Polynomial, poly, and a set of variables of poly that should be
   * treated as parameters (that is, variables eligible to be lumped), obtain
   * all of the unique expressions by which combinations of the remaining
   * active variables are multiplied to form the monomials of the Polynomial.
   *
   * For instance, if we have the polynomial:
   *   a*x + b*x + a*c*y + a*c*y**2
   * And our parameters are a, b, and c, then our lumped parameters are:
   *   lump1 == a+b ;  lump2 == a*c
   * and we return:
   *   { (a + b) -> VarType("lump", 1);  (a * c) -> VarType("lump", 2) }
   *
   * Note however that this function provides no guarantees of the lumped
   * parameter names generated except that they are unique -- "lump1" and
   * "lump2" here are examples.
   */
  static LumpingMapType GetLumpedParametersFromPolynomial(
      const PolyType& poly,
      const std::set<VarType>& parameter_vars);

  /// Same as GetLumpedParametersFromPolynomial but for multiple Polynomials.
  /**
   * It is preferrable to use this if you have multiple Polynomials as it
   * saves you from having to union the resulting LumpingMapType results
   * together.
   */
  static LumpingMapType GetLumpedParametersFromPolynomials(
      const std::vector<PolyType>& polys,
      const std::set<VarType>& parameter_vars);

  /// Rewrite a Polynomial in terms of lumped parameters.
  /**
   * For instance, if we have the polynomial:
   *   a*x + b*x + a*c*y + a*c*y**2
   * And our lumped parameters are:
   *   lump1 == a+b ;  lump2 == a*c
   * And our polynomial is now:
   *   lump1*x + lump2*y + lump2*y**2
   */
  static PolyType RewritePolynomialWithLumpedParameters(
      const PolyType& poly,
      const LumpingMapType& lumped_parameters);

  /** Estimate some parameters of a polynomial based on empirical data.
   *
   * Given one or more polynomial equations P[i](a, b, ... x, y, ...) = 0, and
   * measured values of some its arguments (x, y, ..., referred to as the
   * "active variables"), estimate values for the remaining arguments (a, b,
   * ..., referred to as the "parameters").
   *
   * Measured x, y, ... is provided in a list of maps, active_var_values.
   *
   * The return value is a pair, {estimates, error}, where:
   *   * estimates is a map of polynomial VarTypes (a, b, ...) to their
   *     estimated values, suitable as input for Polynomial::evaluatePartial.
   *   * error is the root-mean-square error of the estimates.
   */
  static std::pair<PartialEvalType, CoefficientType> EstimateParameters(
      const VectorXPoly& polys,
      const std::vector<PartialEvalType>& active_var_values);

  /** A helper struct to hold System ID results */
  struct SystemIdentificationResult {
    /// The lumped parameters that were used in system ID.
    LumpingMapType lumped_parameters;

    /// The input polynomials, rewritten using the lumped parameters so that
    /// only active variable and first-order lumped variable terms remain.
    VectorXTrigPoly lumped_polys;

    /// The estimated value for each lumped parameter.
    PartialEvalType lumped_parameter_values;

    /// The input polynomials, with all estimates substituted in so that
    /// only active variables remain.
    VectorXTrigPoly partially_evaluated_polys;

    /// The root-mean-square error of the estimation step.
    CoefficientType rms_error;
  };

  /** Performs full lumped-parameter identification of a system of TrigPolys.
   *
   * This is a convenience method meant to capture what is expected to be the
   * most common usage pattern of system ID.
   *
   * Given one or more TrigPoly expressions to be set equal to zero, and a
   * list of observed values for a subset of the variables, lumps up the
   * remaining variables into lumped parameters and estimates a value for each
   * of those.  This is broadly equivalent to calling the following methods
   * in sequence:
   *
   *  * GetLumpedParametersFromPolynomials
   *  * RewritePolynomialWithLumpedParameters
   *  * EstimateParameters
   */
  static SystemIdentificationResult LumpedSystemIdentification(
      const VectorXTrigPoly& polys,
      const std::vector<PartialEvalType>& active_var_values);


 private:
  /// This class is not constructable.
  SystemIdentification() {}

  /// Return every combination of the given variables in the polynomials.
  /**
   * For instance, if we want combinations of x and y in the polynomial
   *   a * x + b * x*y + b * y^2 + c * y^2,
   * then return x, x*y, and y^2.
   *
   * NOTE: This will also return the empty combination iff there are terms for
   * the polynomial that do not contain any of the requested variables.  This
   * behaviour is slightly surprising but in practice matches what we want to
   * do with the combinations.
   */
  static std::set<MonomialType>
  GetAllCombinationsOfVars(
      const std::vector<PolyType>& polys,
      const std::set<VarType>& vars);

  /// Test if one monomial is a product of parameters times another monomial.
  /**
   * Return true iff monomial "haystack" consists only of the monomial "needle"
   * times variables not in "active_vars".
   *
   * For instance, with active vars x, y:
   *    haystack 3 * a * x * x * y   matches needle  x * x * y
   *    haystack x * x * y * y does not match needle x * x * y
   */
  static bool MonomialMatches(
    const MonomialType& haystack,
    const MonomialType& needle,
    const std::set<VarType>& active_vars);

  /// Factor out the smallest coefficient in a Polynomial.
  /**
   * The resulting pair is the factored coefficient and a polynomial whose
   * smallest coefficient is 1.  This allows lumped parameter polynomials to
   * be compared to determine when they differ only by coefficient.
   *
   * For instance, given the polynomial:
   *   2 * x + 3 * y
   * this will return
   *   {2, x + 1.5 * y}
   */
  static std::pair<CoefficientType, PolyType>
  CanonicalizePolynomial(const PolyType& poly);

  /// Obtain a new variable ID not already in vars_in_use.  The string part of
  /// the variable's name will be prefix.
  static VarType CreateUnusedVar(const std::string& prefix,
                                 const std::set<VarType>& vars_in_use);

  /** Classify the variables in a group of polynomials and measurements.
   *
   * Given a system of polynomials and a set of variable values, return the
   * sets of all the values, the parameter values (ie those with values not
   * given) and the active values (those with values given).
   *
   * Currently only the second return (the parameter values) is used; the
   * other returns have proven useful in previous versions of this code and
   * are retained for that reason.
   */
  static std::tuple<const std::set<VarType>,
                    const std::set<VarType>,
                    const std::set<VarType>> ClassifyVars(
                        const std::vector<Polynomiald>& polys,
                        const std::vector<PartialEvalType>& active_var_values);
};
}  // namespace solvers
}  // namespace drake
