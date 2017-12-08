#pragma once

#include <cmath>

#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace multilane {

/// A cubic polynomial, f(p) = a + b*p + c*p^2 + d*p^3.
class CubicPolynomial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CubicPolynomial)

  /// Default constructor, all zero coefficients.
  CubicPolynomial() : CubicPolynomial(0., 0., 0., 0.) {}

  /// Constructs a cubic polynomial given all four coefficients.
  CubicPolynomial(double a, double b, double c, double d)
      : a_(a), b_(b), c_(c), d_(d) {
    const double df = f_p(1.) - f_p(0.);
    s_1_ = std::sqrt(1. + (df * df));
  }

  // Returns the a coefficient.
  double a() const { return a_; }

  // Returns the b coefficient.
  double b() const { return b_; }

  // Returns the c coefficient.
  double c() const { return c_; }

  // Returns the d coefficient.
  double d() const { return d_; }

  /// Evaluates the polynomial f at @p p.
  double f_p(double p) const {
    return a_ + (b_ * p) + (c_ * p * p) + (d_ * p * p * p);
  }

  /// Evaluates the derivative df/dp at @p p.
  double f_dot_p(double p) const {
    return b_ + (2. * c_ * p) + (3. * d_ * p * p);
  }

  /// Evaluates the double-derivative d^2f/dp^2 at @p p.
  double f_ddot_p(double p) const { return (2. * c_) + (6. * d_ * p); }

  // TODO(maddog@tri.global)  s_p() and p_s() need to be replaced with a
  //                          properly integrated path-length parameterization.
  //                          For the moment, we are calculating the length by
  //                          approximating the curve with a single linear
  //                          segment from (0, f(0)) to (1, f(1)), which is
  //                          not entirely awful for relatively flat curves.
  /// Returns the path-length s along the curve (p, f(p)) from p = 0 to @p p.
  double s_p(double p) const { return s_1_ * p; }

  /// Returns the inverse of the path-length parameterization s_p(p).
  double p_s(double s) const { return s / s_1_; }

  // TODO(maddog@tri.global) Until s(p) is a properly integrated path-length
  //                         parameterization, we have a need to calculate the
  //                         derivative of the actual linear function
  //                         involved in our bogus path-length approximation.
  double fake_gprime(double p) const {
    unused(p);
    // return df;  which is...
    return f_p(1.) - f_p(0.);
  }

 private:
  double a_{};
  double b_{};
  double c_{};
  double d_{};
  double s_1_{};
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
