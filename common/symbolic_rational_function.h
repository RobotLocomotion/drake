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
 * Represents symbolic rational function. A function f(x) is a rational
 * function, if f(x) = p(x) / q(x), where both p(x) and q(x) are polynomials of
 * x. Note that rational functions are closed under (+, -, x, /). One
 * application of rational function is in polynomial optimization, where we
 * represent (or approximate) functions using rational functions, and then
 * convert the constraint f(x) = h(x) (where h(x) is a polynomial) to a
 * polynomial constraint p(x) - q(x) * h(x) = 0, or convert the inequality
 * constraint f(x) >= h(x) as p(x) - q(x) * h(x) >= 0 if we know q(x) > 0.
 *
 * This class represents a special subset of the symbolic::Expression. While a
 * symbolic::Expression can represent a rational function, extracting the
 * numerator and denominator, generally, is quite difficult; for instance, from
 * p1(x) / q1(x) + p2(x) / q2(x) + ... + pn(x) / qn(x). This class's explicit
 * structure facilitates this decomposition.
 */
class RationalFunction {
 public:
  /** Constructs a zero rational function 0 / 1. */
  RationalFunction();

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RationalFunction)

  /**
   * Constructs the rational function: numerator / denominator.
   * @param numerator The numerator of the fraction.
   * @param denominator The denominator of the fraction.
   * @pre denominator cannot be structurally equal to 0.
   * @pre None of the indeterminates in the numerator can be decision variables
   * in the denominator; similarly none of the indeterminates in the denominator
   * can be decision variables in the numerator.
   * @throw logic_error if the precondition is not satisfied.
   */
  RationalFunction(const Polynomial& numerator, const Polynomial& denominator);

  ~RationalFunction() = default;

  /// Getter for the numerator.
  const Polynomial& numerator() const { return numerator_; }

  /// Getter for the denominator.
  const Polynomial& denominator() const { return denominator_; }

  RationalFunction& operator+=(const RationalFunction& f);
  RationalFunction& operator+=(const Polynomial& p);
  RationalFunction& operator+=(double c);

  RationalFunction& operator-=(const RationalFunction& f);
  RationalFunction& operator-=(const Polynomial& p);
  RationalFunction& operator-=(double c);

  RationalFunction& operator*=(const RationalFunction& f);
  RationalFunction& operator*=(const Polynomial& p);
  RationalFunction& operator*=(double c);

  RationalFunction& operator/=(const RationalFunction& f);
  RationalFunction& operator/=(const Polynomial& p);
  RationalFunction& operator/=(double c);

  /**
   * Returns true if this rational function and f are structurally equal.
   */
  bool EqualTo(const RationalFunction& f) const;

  friend std::ostream& operator<<(std::ostream&, const RationalFunction& f);

 private:
  // Throws std::logic_error if an indeterminate of the denominator (numerator,
  // respectively) is a decision variable of the numerator (denominator).
  void CheckIndeterminates() const;
  Polynomial numerator_;
  Polynomial denominator_;
};

/**
 * Unary minus operation for rational function.
 * if f(x) = p(x) / q(x), then -f(x) = (-p(x)) / q(x)
 */
RationalFunction operator-(RationalFunction f);

RationalFunction operator+(RationalFunction f1, const RationalFunction& f2);
RationalFunction operator+(RationalFunction f, const Polynomial& p);
RationalFunction operator+(const Polynomial& p, RationalFunction f);
RationalFunction operator+(RationalFunction f, double c);
RationalFunction operator+(double c, RationalFunction f);

RationalFunction operator-(RationalFunction f1, const RationalFunction& f2);
RationalFunction operator-(RationalFunction f, const Polynomial& p);
RationalFunction operator-(const Polynomial& p, RationalFunction f);
RationalFunction operator-(RationalFunction f, double c);
RationalFunction operator-(double c, RationalFunction f);

RationalFunction operator*(RationalFunction f1, const RationalFunction& f2);
RationalFunction operator*(RationalFunction f, const Polynomial& p);
RationalFunction operator*(const Polynomial& p, RationalFunction f);
RationalFunction operator*(RationalFunction f, double c);
RationalFunction operator*(double c, RationalFunction f);

RationalFunction operator/(RationalFunction f1, const RationalFunction& f2);
RationalFunction operator/(RationalFunction f, const Polynomial& p);
RationalFunction operator/(const Polynomial& p, const RationalFunction& f);
RationalFunction operator/(RationalFunction f, double c);
RationalFunction operator/(double c, const RationalFunction& f);

/**
 * Returns the rational function @p f raised to @p n.
 * If n is positive, (f/g)ⁿ = fⁿ / gⁿ;
 * If n is negative, (f/g)ⁿ = g⁻ⁿ / f⁻ⁿ;
 * (f/g)⁰ = 1 / 1.
 */
RationalFunction pow(const RationalFunction& f, int n);
}  // namespace symbolic
}  // namespace drake
