#ifndef DRAKE_UTIL_SYSTEM_IDENTIFICATION_H_
#define DRAKE_UTIL_SYSTEM_IDENTIFICATION_H_

#include <map>
#include <set>
#include <stdexcept>

#include "drake/util/Polynomial.h"
#include "drake/drakePolynomial_export.h"

namespace drake {
namespace util {

/// Utility functions for system identification.
/**
 * This class is a holder for templated utility methods.  It should not be
 * constructed.  It must be template-instantiated (in its cpp file) for each
 * supported variant of Polynomial (currently only Polynomial<double>).
 */
template <typename _CoefficientType = double>
class DRAKEPOLYNOMIAL_EXPORT SystemIdentification {
public:
  typedef _CoefficientType CoefficientType;
  typedef ::Polynomial<CoefficientType> PolyType;
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
   */
  static LumpingMapType GetLumpedParametersFromPolynomial(
      PolyType poly,
      std::set<VarType> vars_of_interest);

  /// Same as GetLumpedParametersFromPolynomial but for multiple Polynomials.
  /**
   * It is preferrable to use this if you have multiple Polynomials as it
   * saves you from having to union the resulting LumpingMapType results
   * together.
   */
  static LumpingMapType GetLumpedParametersFromPolynomials(
      std::set<PolyType> polys,
      std::set<VarType> vars_of_interest);

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
      PolyType poly,
      LumpingMapType lumped_parameters);

private:
  /// This class is not constructable.
  SystemIdentification() {}

};
};
};
#endif //  DRAKE_UTIL_SYSTEM_IDENTIFICATION_H_
