#pragma once

#include <fstream>
#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

template <class T>
class LineSearch {
 public:
  struct LineSearchOutput {
    VectorX<T> g_output;
    double f_new;
    VectorX<T> x_new;
  };  

  /// Searches the line x+αΔx for a α that results in a "good" value of the
  /// objective function.
  /// @param x The current assignment of the variables.
  /// @param dx The computed descent direction for x (i.e., Δx).
  /// @param f_old The old objective function value (determined by computing the
  ///             Euclidean norm of the vector output of the algebraic constraint
  ///             equations and then scaling by 1/2).
  /// @param gradient The gradient of the objective function (i.e., the gradient
  ///                 of the Euclidean norm of the vector output of the algebraic
  ///                 constraint equations and then scaling by 1/2).
  /// @returns a structure containing the new value of x, the new value of the
  ///          objective function, and the new outputs of the algebraic constraint
  ///          equations.
  /// @note This routine is an implementation of the line search algorithm
  ///       described in "Numerical Recipes in C" (Section 9.7). The routine
  ///       does not attempt to optimize α, as common wisdom indicates that
  ///       approach to be computationally inefficient. Instead, this algorithm
  ///       seeks the α for which (1) the average decrease of the objective (f)
  ///       is at least some fraction of the initial rate of decrease Δλ⋅∇, where
  ///       ∇ is @p gradient- this ensures that f does not decrease
  ///       too slowly relative to the step lengths- and (2) the rate of decrease
  ///       in the objective at x+αΔx is greater than a fraction of the rate of
  ///       decrease in f at x (addressing the problem where the
  ///       step lengths are too small relative to the initial rate of decrease
  ///       of f). This algorithm attempts to fit a quadratic (for the first
  ///       backtracking attempt) or cubic (for subsequent backtracking attempts)
  ///       to find a good value for α.
  static LineSearchOutput search_line(
      const VectorX<T>& x, const VectorX<T>& dx,
      double f_old, const VectorX<T>& gradient,
      std::function<VectorX<T>(const VectorX<T>&)> g) {
    const double meps = std::numeric_limits<double>::epsilon();
    const double gamma = 1e-4;               // Rate of decrease constant.
    const double deltax_tol =  meps*100;     // Convergence criterion for Δx.

    // The initial step length scalar- we always try a full Newton step first.
    double alpha = 1.0;

    // Get the slope of the step.
    double slope = gradient.dot(dx);

    // Compute the new x.
    LineSearchOutput out;

    bool first_loop = true;
    double last_alpha;
    double last_f;

    // Determine minimum possible value of alpha.
    double test = 0.0;
    for (int i=0; i< dx.size(); ++i) 
      test = std::max(test, std::abs(dx(i)) / std::max(std::abs(x(i)), 1.0));
    const double alpha_min = deltax_tol / test;

    while (true) {
      // Update x.
      out.x_new = x + dx*alpha;

      // Compute the new value of the objective function.
      out.g_output = g(out.x_new);
      out.f_new = out.g_output.squaredNorm() / 2;

      // Look for minimum alpha.
      if (alpha < alpha_min) {
        break;
      } else {
        // Look for sufficient decrease.
        if (out.f_new <= f_old+gamma*alpha*slope) {
          break;
        } else {
          double tmp_beta;
          if (first_loop) {
            tmp_beta = -slope / (2 * (out.f_new - f_old - slope));
            first_loop = false; 
          } else {
            const double rhs1 = out.f_new - f_old - alpha*slope;
            const double rhs2 = last_f - f_old - last_alpha * slope;
            const double alpha2 = alpha*alpha;
            const double last_alpha2 = last_alpha*last_alpha;
            const double a = (rhs1 / alpha2 - rhs2 / last_alpha2) / 
                             (alpha - last_alpha);
            const double b = (-last_alpha * rhs1 / alpha2 + 
                              alpha * rhs2 / last_alpha2) / (alpha - last_alpha);
            if (a == 0.0) {
              tmp_beta = -slope / (2 * b);
            } else {
              const T disc = b*b - 3*a*slope;
              if (disc < 0.0) {
                tmp_beta = alpha/2;
              } else {
                if (b <= 0.0)
                  tmp_beta = (-b+std::sqrt(disc))/(3*a);
                else
                  tmp_beta = -slope/(b+std::sqrt(disc));
              }

              if (tmp_beta > alpha/2)
                tmp_beta  = alpha/2;
            }
          }

          last_alpha = alpha;
          last_f = out.f_new;
          alpha = std::max(tmp_beta, alpha/10);
        }
      }
    }

    std::ofstream fout("alpha.dat", std::ostream::app);
    fout << alpha << std::endl;
    fout.close();

    return out;
  }
};
}  // namespace systems
}  // namespace drake
