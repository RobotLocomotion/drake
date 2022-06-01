#pragma once

#include <functional>
#include <iostream>
#include <tuple>
#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/*
  Uses a Newton-Raphson method to compute a root of `function` within the
  bracket [x_lower, x_upper]. This method stops when the difference between the
  previous iterate xᵏ and the next iteration xᵏ⁺¹ is below the absolute
  tolerance `abs_tolerance`, i.e. when |xᵏ⁺¹ - xᵏ| < abs_tolerance.

  This method iteratively shrinks the bracket containing the root. Moreover, it
  switches to bisection whenever a Newton iterate falls outside the bracket or
  when Newton's method is slow. Using this procedure, this method is guaranteed
  to find a root (which might not be unique) within [x_lower, x_upper], with
  accuracy given by `abs_tolerance`.

  This method expects that sign(function(x_lower)) != sign(function(x_upper)).
  For continuous functions, this ensures there exists a root in [x_lower,
  x_upper]. For discontinuous functions, the solver "sees" a discontinuity as a
  sharp transition within a narrow gap of size `abs_tolerance`. Therefore the
  solver will return a "root" located at where the discontinuity occurs. For
  instance, consider the function y(x) = 1/(x-c). While clearly discontinuous at
  x = c, this method will return c as the root whenever c is within the supplied
  bracket. Another example more common in practice is the function y(x) = x +
  H(x) - 1/2, with H(x) the Heaviside function. While discontinuous at x = 0,
  this method will return x = 0 as the root.

  @returns the pair (root, number_of_evaluations)

  @pre x_lower <= x_upper
  @pre x_guess is in [x_lower, x_upper]
  @pre sign(function(x_lower)) != sign(function_x_upper)
  @pre abs_tolerance > 0
  @pre max_iterations > 0
*/
std::pair<double, int> DoNewtonWithBisectionFallback(
    const std::function<std::pair<double, double>(double)>& function,
    double x_lower, double x_upper, double x_guess, double abs_tolerance,
    int max_iterations);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
