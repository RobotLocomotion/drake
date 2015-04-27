#ifndef UTIL_LCMUTIL_H_
#define UTIL_LCMUTIL_H_

#include "PiecewisePolynomial.h"
#include "drake/lcmt_polynomial.hpp"
#include "drake/lcmt_polynomial_matrix.hpp"
#include "drake/lcmt_piecewise_polynomial.hpp"

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif

DLLEXPORT void encodePolynomial(const Polynomial<double>& polynomial, drake::lcmt_polynomial& msg);

DLLEXPORT Polynomial<double> decodePolynomial(drake::lcmt_polynomial& msg);



#endif /* UTIL_LCMUTIL_H_ */
