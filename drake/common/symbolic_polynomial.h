#pragma once

#include <Eigen/Core>

#include "drake/common/monomial.h"

namespace drake {
namespace symbolic {
/**
 * Returns the total degrees of the polynomial @p e w.r.t the variables in
 * @p vars. For example, the total degree of
 * e = x^2*y + 2 * x*y*z^3 + x * z^2
 * w.r.t (x, y) is 3 (from x^2 * y)
 * w.r.t (x, z) is 4 (from x*y*z^3)
 * w.r.t (z)    is 3 (from x*y*z^3)
 * @param e A symbolic polynomial
 * @pre{e.is_polynomial() should be true)
 * @param vars A set of variables.
 * @pre{vars is a subset of the variables in @p e)
 * @return The total degree
 */
int degree(const symbolic::Expression& e, const Variables& vars);
}  // namespace symbolic
}  // namespace drake