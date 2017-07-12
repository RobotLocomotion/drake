#pragma once

#include <ostream>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/monomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {
/// Represents symbolic polynomials. A symbolic polynomial keeps a mapping from
/// a monomial of indeterminates to its coefficient in a symbolic expression.
///
/// A polynomial `p` has to satisfy an invariant such that
/// `p.decision_variables() ∩ p.indeterminates() = ∅`. We have CheckInvariant()
/// method to check the invariant.
///
/// Note that arithmetic operations (+,-,*) between a Polynomial and a Variable
/// are not provided. The problem is that Variable class has no intrinsic
/// information if a variable is a decision variable or an indeterminate while
/// we need this information to perform arithmetic operations over Polynomials.
class Polynomial {
 public:
  using MapType =
      std::unordered_map<Monomial, Expression, hash_value<Monomial>>;

  /// Constructs a zero polynomial.
  Polynomial() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Polynomial)

  /// Constructs a polynomial from a map, Monomial → Expression.
  explicit Polynomial(MapType init);

  /// Constructs a polynomial from a monomial @p m. Note that all variables
  /// in `m` are considered as indeterminates.
  explicit Polynomial(const Monomial& m);

  /// Constructs a polynomial from an expression @p e. Note that all variables
  /// in `e` are considered as indeterminates.
  explicit Polynomial(const Expression& e);

  /// Constructs a polynomial from an expression @p e by decomposing it with
  /// respect to @p indeterminates.
  Polynomial(const Expression& e, const Variables& indeterminates);

  /// Returns the indeterminates of this polynomial.
  Variables indeterminates() const;

  /// Returns the decision variables of this polynomial.
  Variables decision_variables() const;

  /// Returns the highest degree of this polynomial in a variable @p v.
  int Degree(const Variable& v) const;

  /// Returns the total degree of this polynomial.
  int TotalDegree() const;

  /// Returns the mapping from a Monomial to its corresponding coefficient of
  /// this polynomial.
  const MapType& monomial_to_coefficient_map() const;

  /// Returns an equivalent symbolic expression of this polynomial.
  Expression ToExpression() const;

  Polynomial& operator+=(const Polynomial& p);
  Polynomial& operator+=(const Monomial& m);
  Polynomial& operator+=(double c);

  Polynomial& operator-=(const Polynomial& p);
  Polynomial& operator-=(const Monomial& m);
  Polynomial& operator-=(double c);

  Polynomial& operator*=(const Polynomial& p);
  Polynomial& operator*=(const Monomial& m);
  Polynomial& operator*=(double c);

  /// Returns true if this polynomial and @p p are structurally equal.
  bool EqualTo(const Polynomial& p) const;

  /// Returns a symbolic formula representing the condition where this
  /// polynomial and @p p are the same.
  Formula operator==(Polynomial p) const;

 private:
  // Adds (coeff * m) to this polynomial.
  Polynomial& Add(const Expression& coeff, const Monomial& m);
  // Throws std::runtime_error if there is a variable appeared in both of
  // decision_variables() and indeterminates().
  void CheckInvariant() const;
  MapType monomial_to_coefficient_map_;
};

/// Unary minus operation for polynomial.
Polynomial operator-(Polynomial p);
Polynomial operator+(Polynomial p1, const Polynomial& p2);
Polynomial operator+(Polynomial p, const Monomial& m);
Polynomial operator+(const Monomial& m, Polynomial p);
Polynomial operator+(const Monomial& m1, const Monomial& m2);
Polynomial operator+(Polynomial p, double c);
Polynomial operator+(double c, Polynomial p);
Polynomial operator-(Polynomial p1, const Polynomial& p2);
Polynomial operator-(Polynomial p, const Monomial& m);
Polynomial operator-(const Monomial& m, Polynomial p);
Polynomial operator-(const Monomial& m1, const Monomial& m2);
Polynomial operator-(Polynomial p, double c);
Polynomial operator-(double c, Polynomial p);
Polynomial operator*(Polynomial p1, const Polynomial& p2);
Polynomial operator*(Polynomial p, const Monomial& m);
Polynomial operator*(const Monomial& m, Polynomial p);
Polynomial operator*(double c, Polynomial p);
Polynomial operator*(Polynomial p, double c);
Polynomial operator*(const Monomial& m, double c);
Polynomial operator*(double c, const Monomial& m);

/// Returns polynomial @p rasied to @p n.
Polynomial pow(const Polynomial& p, int n);

std::ostream& operator<<(std::ostream& os, const Polynomial& p);
}  // namespace symbolic
}  // namespace drake
