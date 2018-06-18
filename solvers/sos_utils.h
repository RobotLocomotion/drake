#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {

/**
  * Given input polynomial p, outputs a set M of monomials with the following
  * guarantee: if p = \sum^n_{i=1} f_i * f_i for some (unknown) polynomials
  * f_1,...,f_n, then the span of M contains each f_i.  Given M, one can
  * then find the polynomials f_i using semidefinite programming; see, e.g.,
  * Chapter 3 of Semidefinite Optimization and Convex Algebraic Geometry
  * by G. Blekherman, P. Parrilo, R. Thomas.
  * @param p A polynomial
  * @return A vector whose entries are the elements of M
*/
drake::VectorX<symbolic::Monomial> ConstructMonomialBasis(
                                   const drake::symbolic::Polynomial & p);
}  // namespace solvers
}  // namespace drake

