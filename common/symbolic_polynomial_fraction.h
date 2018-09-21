#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <ostream>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * Represents symbolic polynomial fractions. A function f(x) is a polynomial
 * fraction, if f(x) = p(x) / q(x), where both p(x) and q(x) are polynomials of
 * x. Note that polynomial fraction is closed under (+, -, x, /). One
 * application of polynomial fraction is in polynomial optimization, where we
 * represent (or approximate) functions using polynomial fractions, and then
 * convert the constraint f(x) = h(x) (where h(x) is a polynomial) to a
 * polynomial constraint p(x) - q(x) * h(x) = 0, or convert the inequality
 * constraint f(x) >= h(x) as p(x) - q(x) * h(x) >= 0 if we know q(x) > 0.
 */
class PolynomialFraction {
 public:
  /** Constructs a zero polynomial fraction 0 / 1. */
  PolynomialFraction();

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PolynomialFraction)

  /**
   * Constructs a polynomial fraction numerator / denominator.
   * @param numerator The numerator of the fraction.
   * @param denominator The denominator of the fraction.
   * @pre denominator cannot be structurally equal to 0.
   * @pre None of the indeterminates in the numerator can be decision variables
   * in the denominator; similarly none of the indeterminates in the denominator
   * can be decision variables in the numerator.
   * @throw logic_error if the precondition is not satisfied.
   */
  PolynomialFraction(const Polynomial& numerator,
                     const Polynomial& denominator);

  ~PolynomialFraction() = default;

  /// Getter for the numerator.
  const Polynomial& numerator() const { return numerator_; }

  /// Getter for the denominator.
  const Polynomial& denominator() const { return denominator_; }

  PolynomialFraction& operator+=(const PolynomialFraction& f);
  PolynomialFraction& operator+=(const Polynomial& p);
  PolynomialFraction& operator+=(double c);

  PolynomialFraction& operator-=(const PolynomialFraction& f);
  PolynomialFraction& operator-=(const Polynomial& p);
  PolynomialFraction& operator-=(double c);

  PolynomialFraction& operator*=(const PolynomialFraction& f);
  PolynomialFraction& operator*=(const Polynomial& p);
  PolynomialFraction& operator*=(double c);

  PolynomialFraction& operator/=(const PolynomialFraction& f);
  PolynomialFraction& operator/=(const Polynomial& p);
  PolynomialFraction& operator/=(double c);

  /**
   * Returns true if this polynomial fraction and f are structurally equal.
   */
  bool EqualTo(const PolynomialFraction& f) const;

  friend std::ostream& operator<<(std::ostream&, const PolynomialFraction& f);

 private:
  // Throws std::logic_error if an indeterminate of the denominator (numerator,
  // respectively) is a decision variable of the numerator (denominator).
  void CheckInvariant() const;
  Polynomial numerator_;
  Polynomial denominator_;
};

/**
 * Unary minus operatrion for polynomial fraction.
 * if f(x) = p(x) / q(x), then -f(x) = (-p(x)) / q(x)
 */
PolynomialFraction operator-(PolynomialFraction f);

PolynomialFraction operator+(PolynomialFraction f1,
                             const PolynomialFraction& f2);
PolynomialFraction operator+(PolynomialFraction f, const Polynomial& p);
PolynomialFraction operator+(const Polynomial& p, PolynomialFraction f);
PolynomialFraction operator+(PolynomialFraction f, double c);
PolynomialFraction operator+(double c, PolynomialFraction f);

PolynomialFraction operator-(PolynomialFraction f1,
                             const PolynomialFraction& f2);
PolynomialFraction operator-(PolynomialFraction f, const Polynomial& p);
PolynomialFraction operator-(const Polynomial& p, PolynomialFraction f);
PolynomialFraction operator-(PolynomialFraction f, double c);
PolynomialFraction operator-(double c, PolynomialFraction f);

PolynomialFraction operator*(PolynomialFraction f1,
                             const PolynomialFraction& f2);
PolynomialFraction operator*(PolynomialFraction f, const Polynomial& p);
PolynomialFraction operator*(const Polynomial& p, PolynomialFraction f);
PolynomialFraction operator*(PolynomialFraction f, double c);
PolynomialFraction operator*(double c, PolynomialFraction f);

PolynomialFraction operator/(PolynomialFraction f1,
                             const PolynomialFraction& f2);
PolynomialFraction operator/(PolynomialFraction f, const Polynomial& p);
PolynomialFraction operator/(const Polynomial& p, const PolynomialFraction& f);
PolynomialFraction operator/(PolynomialFraction f, double c);
PolynomialFraction operator/(double c, const PolynomialFraction& f);

/// Returns the polynomial fraction @p f raised to @p n.
PolynomialFraction pow(const PolynomialFraction& f, int n);
}  // namespace symbolic
}  // namespace drake
