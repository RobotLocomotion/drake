#pragma once

#include <cmath>
#include <functional>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Class to represent an interval [x_lower, x_upper] within which we know there
// is a root. For a continuous function f(x) we know there is a root within the
// given interval if sign(f(x_lower)) != sign(f(x_upper)).
class Bracket {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bracket);

  // Constructs a valid bracket [x_lower, x_upper].
  // Assuming the function is continuous, validity is verified by checking that
  // sign(f_lower) != sign(f_upper).
  Bracket(double x_lower, double f_lower, double x_upper, double f_upper)
      : x_lower_(x_lower),
        f_lower_(f_lower),
        x_upper_(x_upper),
        f_upper_(f_upper) {
    DRAKE_DEMAND(x_lower < x_upper);
    DRAKE_DEMAND(has_different_sign(f_lower, f_upper));
  }

  // Returns `true` if x is inside the bracket.
  bool inside(double x) { return x_lower_ <= x && x <= x_upper_; }

  // Updates `this` bracket to [x, x_upper()] if sign(f) == sign(f_lower) or to
  // [x_lower(), x] if sign(f) == sign(f_upper)
  void Update(double x, double f) {
    if (has_different_sign(f, f_upper_)) {
      x_lower_ = x;
      f_lower_ = f;
    } else {
      x_upper_ = x;
      f_upper_ = f;
    }
  }

  double x_lower() const { return x_lower_; }
  double x_upper() const { return x_upper_; }
  double f_lower() const { return f_lower_; }
  double f_upper() const { return f_upper_; }

 private:
  // Helper that returns true if sign(a) == sign(b).
  static bool has_different_sign(double a, double b) {
    return std::signbit(a) ^ std::signbit(b);
  }
  double x_lower_{};
  double f_lower_{};
  double x_upper_{};
  double f_upper_{};
};

/*
  Uses a Newton-Raphson method to compute a root of `function` within the
  bracket [x_lower, x_upper].

  This method returns successfully when either:
    1. the difference between the previous iterate xᵏ and the next iteration
       xᵏ⁺¹ is below the absolute tolerance `x_tolerance`, i.e. when |xᵏ⁺¹ -
       xᵏ| < x_tolerance or,
    2. the value of `function` at iterate xᵏ is within absolute tolerance
       `f_tolerance`, i.e. |f(xᵏ)| < f_tolerance.

  This method iteratively shrinks the bracket containing the root. Moreover, it
  switches to bisection whenever a Newton iterate falls outside the bracket or
  when Newton's method is slow. Using this procedure, this method is guaranteed
  to find a root (which might not be unique) within [x_lower, x_upper], with
  accuracy given by `abs_tolerance`. This guarantee is only true for continuous
  functions (discontinuous derivatives are allowed).

  TODO(amcastro-tri): Consider exploring the secant method instead of bisection.

  @param function Function f(x). It returns the pair (value, derivative) at x.
  @param bracket Search bracket.
  @param x_guess Initial guess for the root.
  @param x_tolerance Absolute tolerance for the root, with the same units as the
  function argument x. The user is responsible for providing tolerances that are
  attainable in practice when we consider round-off errors.
  @param f_tolerance Absolute tolerance for the function value, with the same
  units as `function`. The user is responsible for providing tolerances that are
  attainable in practice when we consider round-off errors.
  @param max_interations Maximum number of iterations the solver is allowed to
  take.

  @returns the pair (root, number_of_evaluations)
  @throws std::exception if convergence is not attained in `max_iterations`.

  @pre `function` is a C⁰ function.
  @pre x_lower <= x_upper
  @pre x_guess is in bracket.
  @pre x_tolerance > 0
  @pre f_tolerance > 0
  @pre max_iterations > 0
*/
std::pair<double, int> DoNewtonWithBisectionFallback(
    const std::function<std::pair<double, double>(double)>& function,
    Bracket bracket, double x_guess, double x_tolerance, double f_tolerance,
    int max_iterations);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
