#pragma once

#include <map>
#include <set>
#include <stdexcept>
#include <vector>

#include "drake/util/Polynomial.h"
#include "drake/drakePolynomial_export.h"

namespace drake {
namespace util {

/// Utility functions for system identification.
/**
 * This class is a holder for templated utility methods.  It should not be
 * constructed.  It must be template-instantiated (in its cpp file) for each
 * supported variant of Polynomial (currently only Polynomial<double>).
 *
 * Note: The term "system identification" used throughout here refers to the
 * process of simplifying the equations defining a physical system to a
 * minimum number of "lumped" parameters and then estimating the values of
 * those parameters based on empirical data.  For reference, see e.g.:
 *   https://dl.dropboxusercontent.com/u/3432353/parameterEstimation.pdf
 */
template <typename CoefficientType>
class DRAKEPOLYNOMIAL_EXPORT SystemIdentification {
 public:
  typedef ::Polynomial<CoefficientType> PolyType;
  typedef typename PolyType::Monomial MonomialType;
  typedef typename PolyType::Term TermType;
  typedef typename PolyType::VarType VarType;
  typedef std::map<PolyType, VarType> LumpingMapType;

  /// Extract lumped parameters from a given polynomial.
  /**
   * Given a Polynomial, poly, and a set of "variables of interest" (that is,
   * variables that are not to be considered parameters and should not be
   * lumped), obtain all of the unique expressions by which combinations of
   * the variables of interest are multiplied to form the monomials of the
   * Polynomial.
   *
   * For instance, if we have the polynomial:
   *   a*x + b*x + a*c*y + a*c*y**2
   * And our variables of interest are x and y, then our lumped parameters are:
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
      const std::set<VarType>& vars_of_interest);

  /// Same as GetLumpedParametersFromPolynomial but for multiple Polynomials.
  /**
   * It is preferrable to use this if you have multiple Polynomials as it
   * saves you from having to union the resulting LumpingMapType results
   * together.
   */
  static LumpingMapType GetLumpedParametersFromPolynomials(
      const std::vector<PolyType>& polys,
      const std::set<VarType>& vars_of_interest);

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

 private:
  /// This class is not constructable.
  SystemIdentification() {}

  /// Return every combination of the given vars_of_interest in the polynomials.
  /**
   * For instance, if x and y are of interest on the polynomial
   *   a * x + b * x*y + b * y^2 + c * y^2,
   * then return x, x*y, and y^2.
   *
   * NOTE: This will also return the empty combination iff there are terms for
   * the polynomial that do not contain any vars of interest.  This behaviour
   * is slightly surprising but in practice matches what we want to do with
   * the combinations.
   */
  static std::set<MonomialType>
  GetAllCombinationsOfVars(
      const std::vector<PolyType>& polys,
      const std::set<VarType>& vars_of_interest);

  /// Test if one monomial is a product of parameters times another monomial.
  /**
   * Return true iff monomial "haystack" consists only of the monomial "needle"
   * times variables not in "vars_of_interest".
   *
   * For instance, with vars-of-interest x, y:
   *    haystack 3 * a * x * x * y   matches needle  x * x * y
   *    haystack x * x * y * y does not match needle x * x * y
   */
  static bool MonomialMatches(
    const MonomialType& haystack,
    const MonomialType& needle,
    const std::set<VarType>& vars_of_interest);

  /// Factor out the smallest coefficient in a Polynomial.
  /**
   * The resulting pair is the factored coefficient and a polynomial whose
   * smallest coefficient is 1.
   */
  static std::pair<CoefficientType, PolyType>
  NormalizePolynomial(const PolyType& poly);
};
}  // namespace util
}  // namespace drake

