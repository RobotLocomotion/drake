#include "drake/multibody/contact_solvers/newton_with_bisection.h"

#include <functional>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

std::pair<double, int> DoNewtonWithBisectionFallback(
    const std::function<std::pair<double, double>(double)>& function,
    Bracket bracket, double x_guess, double x_tolerance, double f_tolerance,
    int max_iterations) {
  using std::abs;
  using std::swap;
  // Pre-conditions on the bracket.
  DRAKE_THROW_UNLESS(bracket.inside(x_guess));

  // Pre-conditions on the algorithm's parameters.
  DRAKE_THROW_UNLESS(x_tolerance > 0);
  DRAKE_THROW_UNLESS(f_tolerance > 0);
  DRAKE_THROW_UNLESS(max_iterations > 0);

  if (abs(bracket.f_lower()) < f_tolerance)
    return std::make_pair(bracket.x_lower(), 0);

  if (abs(bracket.f_upper()) < f_tolerance)
    return std::make_pair(bracket.x_upper(), 0);

  double root = x_guess;  // Initialize to user supplied guess.
  double minus_dx = bracket.x_lower() - bracket.x_upper();
  double minus_dx_previous = minus_dx;
  double f{NAN}, df{NAN};  // Initialize with garbage.

  // Helper to perform a bisection update. It returns the pair (root, -dx).
  auto do_bisection = [&bracket]() {
    const double dx_negative = 0.5 * (bracket.x_lower() - bracket.x_upper());
    // N.B. This way of updating the root will lead to root == x_lower if
    // the value of minus_dx is insignificant compared to x_lower when using
    // floating point precision.
    const double x = bracket.x_lower() - dx_negative;
    return std::make_pair(x, dx_negative);
  };

  // Helper to perform a Newton update. It returns the pair (root, -dx).
  auto do_newton = [&f, &df, &root]() {
    const double dx_negative = f / df;
    double x = root;
    // N.B. x will not change if dx_negative is negligible within machine
    // precision.
    x -= dx_negative;
    return std::make_pair(x, dx_negative);
  };

  for (int num_evaluations = 1; num_evaluations <= max_iterations;
       ++num_evaluations) {
    // The one evaluation per iteration.
    std::tie(f, df) = function(root);

    // Update the bracket around root to guarantee that there exist a root
    // within the interval [x_lower, x_upper].
    bracket.Update(root, f);

    DRAKE_LOGGER_DEBUG(
        "x = {:10.4g}. [x_lower, x_upper] = [{:10.4g}, "
        "{:10.4g}]. dx = {:10.4g}. f = {:10.4g}. dfdx = {:10.4g}.",
        root, bracket.x_lower(), bracket.x_upper(), -minus_dx, f, df);

    // Exit if f(root) is close to zero.
    if (abs(f) < f_tolerance) return std::make_pair(root, num_evaluations);

    // N.B. This check is based on the check used within method rtsafe from
    // Numerical Recipes.
    // N.B. One way to think about this: if we assume 0 ≈ |fᵏ| << |fᵏ⁻¹| and
    // f'ᵏ⁻¹ ≈ f'ᵏ (this would be the case when Newton is converging
    // quadratically), then we can estimate fᵏ⁻¹ from values at the last
    // iteration as fᵏ⁻¹ ≈ fᵏ + dxᵏ⁻¹⋅f'ᵏ⁻¹ ≈ dxᵏ⁻¹⋅f'ᵏ. Therefore the
    // inequality below is an approximation for |2⋅fᵏ| > |fᵏ⁻¹|. That is, we use
    // Newton's method when |fᵏ| < |fᵏ⁻¹|/2. Otherwise we use bisection which
    // guarantees convergence, though linearly.
    const bool newton_is_slow = 2.0 * abs(f) > abs(minus_dx_previous * df);

    minus_dx_previous = minus_dx;
    if (newton_is_slow) {
      std::tie(root, minus_dx) = do_bisection();
      DRAKE_LOGGER_DEBUG("Bisect. k = {:d}.", num_evaluations);
    } else {
      std::tie(root, minus_dx) = do_newton();
      if (bracket.inside(root)) {
        DRAKE_LOGGER_DEBUG("Newton. k = {:d}.", num_evaluations);
      } else {
        std::tie(root, minus_dx) = do_bisection();
        DRAKE_LOGGER_DEBUG("Bisect. k = {:d}.", num_evaluations);
      }
    }

    // No need for additional evaluations if the root is within tolerance.
    if (abs(minus_dx) < x_tolerance)
      return std::make_pair(root, num_evaluations);
  }

  // If here, then DoNewtonWithBisectionFallback did not converge.
  // This will happen for instance when the maximum number of iterations is too
  // small.
  throw std::runtime_error(fmt::format(
      "NewtonWithBisectionFallback did not converge.\n"
      "|dx| = {}. x_lower = {}. x_upper = {}. x = {}. f = {}.",
      abs(minus_dx), bracket.x_lower(), bracket.x_upper(), root, f));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
