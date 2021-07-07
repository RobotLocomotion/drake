#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <map>
#include <ostream>

#include <Eigen/Core>
#include <fmt/format.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * Represents symbolic generic polynomials using a given basis (for example,
 * monomial basis, Chebyshev basis, etc). A generic symbolic polynomial keeps a
 * mapping from a basis element of indeterminates to its coefficient in a
 * symbolic expression. A generic polynomial `p` has to satisfy an invariant
 * such that `p.decision_variables() ∩ p.indeterminates() = ∅`. We have
 * CheckInvariant() method to check the invariant.
 * For polynomials using different basis, you could refer to section 3.1.5 of
 * Semidefinite Optimization and Convex Algebraic Geometry on the pros/cons of
 * each basis.
 *
 * We provide two instantiations of this template
 * - BasisElement = MonomialBasisElement
 * - BasisElement = ChebyshevBasisElement
 * @tparam BasisElement Must be a subclass of PolynomialBasisElement.
 */
template <typename BasisElement>
class GenericPolynomial {
 public:
  static_assert(
      std::is_base_of_v<PolynomialBasisElement, BasisElement>,
      "BasisElement should be a derived class of PolynomialBasisElement");
  /** Type of mapping from basis element to coefficient */
  using MapType = std::map<BasisElement, Expression>;

  /** Constructs a zero polynomial. */
  GenericPolynomial() = default;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GenericPolynomial)

  /** Constructs a default value. This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit GenericPolynomial(std::nullptr_t)
      : GenericPolynomial<BasisElement>() {}

  /** Constructs a generic polynomial from a map, basis_element → coefficient.
   * For example
   * @code{cc}
   * GenericPolynomial<MonomialBasiElement>(
   *   {{MonomialBasisElement(x, 2), a}, {MonomialBasisElement(x, 3), a+b}})
   * @endcode
   * constructs a polynomial ax²+(a+b)x³.*/
  explicit GenericPolynomial(MapType init);

  /** Constructs a generic polynomial from a single basis element @p m.
   * @note that all variables in `m` are considered as indeterminates. Namely
   * the constructed generic polynomial contains the map with a single key `m`,
   * with the coefficient being 1.
   */
  // Note that this implicit conversion is desirable to have a dot product of
  // two Eigen::Vector<BasisElement>s return a GenericPolynomial<BasisElement>.
  // NOLINTNEXTLINE(runtime/explicit)
  GenericPolynomial(const BasisElement& m);

  /** Constructs a polynomial from an expression @p e. Note that all variables
   * in `e` are considered as indeterminates.
   *
   * @throws std::exception if @p e is not a polynomial.
   */
  explicit GenericPolynomial(const Expression& e);

  /** Constructs a polynomial from an expression @p e by decomposing it with
   * respect to @p indeterminates.
   *
   * @note The indeterminates for the polynomial are @p indeterminates. Even if
   * a variable in @p indeterminates does not show up in @p e, that variable is
   * still registered as an indeterminate in this polynomial, as
   * this->indeterminates() be the same as @p indeterminates.
   *
   * @throws std::exception if @p e is not a polynomial in @p
   * indeterminates.
   */
  GenericPolynomial(const Expression& e, Variables indeterminates);

  /** Returns the indeterminates of this generic polynomial. */
  [[nodiscard]] const Variables& indeterminates() const {
    return indeterminates_;
  }

  /** Returns the decision variables of this generic polynomial. */
  [[nodiscard]] const Variables& decision_variables() const {
    return decision_variables_;
  }

  /** Sets the indeterminates to `new_indeterminates`.
   *
   * Changing the indeterminates will change
   * `basis_element_to_coefficient_map()`, and also potentially the degree of
   * the polynomial. Here is an example.
   *
   * @code
   * // p is a quadratic polynomial with x being the only indeterminate.
   * symbolic::GenericPolynomial<MonomialBasisElement> p(a * x * x + b * x + c,
   * {x});
   * // p.basis_element_to_coefficient_map() contains {1: c, x: b, x*x:a}.
   * std::cout << p.TotalDegree(); // prints 2.
   * // Now set (a, b, c) to the indeterminates. p becomes a linear
   * // polynomial of a, b, c.
   * p.SetIndeterminates({a, b, c});
   * // p.basis_element_to_coefficient_map() now is {a: x * x, b: x, c: 1}.
   * std::cout << p.TotalDegree(); // prints 1.
   * @endcode
   * This function can be expensive, as it potentially reconstructs the
   * polynomial (using the new indeterminates) from the expression.
   */
  void SetIndeterminates(const Variables& new_indeterminates);

  /** Returns the map from each basis element to its coefficient. */
  [[nodiscard]] const MapType& basis_element_to_coefficient_map() const {
    return basis_element_to_coefficient_map_;
  }

  /** Returns the highest degree of this generic polynomial in an indeterminate
   * @p v. */
  [[nodiscard]] int Degree(const Variable& v) const;

  /** Returns the total degree of this generic polynomial. */
  [[nodiscard]] int TotalDegree() const;

  /** Returns an equivalent symbolic expression of this generic polynomial.*/
  [[nodiscard]] Expression ToExpression() const;

  /**
   * Differentiates this generic polynomial with respect to the variable @p x.
   * Note that a variable @p x can be either a decision variable or an
   * indeterminate.
   */
  [[nodiscard]] GenericPolynomial<BasisElement> Differentiate(
      const Variable& x) const;

  /** Computes the Jacobian matrix J of the generic polynomial with respect to
   * @p vars. J(0,i) contains ∂f/∂vars(i). @p vars should be an Eigen column
   * vector of symbolic variables.
   */
  template <typename Derived>
  Eigen::Matrix<GenericPolynomial<BasisElement>, 1, Derived::RowsAtCompileTime>
  Jacobian(const Eigen::MatrixBase<Derived>& vars) const {
    static_assert(
        std::is_same_v<typename Derived::Scalar, Variable> &&
            (Derived::ColsAtCompileTime == 1),
        "The argument of GenericPolynomial::Jacobian() should be a vector of "
        "symbolic variables.");
    const VectorX<Expression>::Index n{vars.size()};
    Eigen::Matrix<GenericPolynomial<BasisElement>, 1,
                  Derived::RowsAtCompileTime>
        J{n};
    for (VectorX<Expression>::Index i = 0; i < n; ++i) {
      J(i) = this->Differentiate(vars(i));
    }
    return J;
  }

  /**
   * Evaluates this generic polynomial under a given environment @p env.
   *
   * @throws std::exception if there is a variable in this generic
   * polynomial whose assignment is not provided by @p env.
   */
  [[nodiscard]] double Evaluate(const Environment& env) const;

  /** Partially evaluates this generic polynomial using an environment @p env.
   *
   * @throws std::exception if NaN is detected during evaluation.
   */
  [[nodiscard]] GenericPolynomial<BasisElement> EvaluatePartial(
      const Environment& env) const;

  /** Partially evaluates this generic polynomial by substituting @p var with @p
   * c.

   * @throws std::exception if NaN is detected at any point during
   * evaluation.
   */
  [[nodiscard]] GenericPolynomial<BasisElement> EvaluatePartial(
      const Variable& var, double c) const;

  /** Adds @p coeff * @p m to this generic polynomial. */
  GenericPolynomial<BasisElement>& AddProduct(const Expression& coeff,
                                              const BasisElement& m);

  /** Removes the terms whose absolute value of the coefficients are smaller
   * than or equal to @p coefficient_tol.
   * For example, if the generic polynomial is 2x² + 3xy + 10⁻⁴x - 10⁻⁵,
   * then after calling RemoveTermsWithSmallCoefficients(1e-3), the returned
   * polynomial becomes 2x² + 3xy.
   * @param coefficient_tol A positive scalar.
   * @retval polynomial_cleaned A generic polynomial whose terms with small
   * coefficients are removed.
   */
  [[nodiscard]] GenericPolynomial<BasisElement>
  RemoveTermsWithSmallCoefficients(double coefficient_tol) const;

  GenericPolynomial<BasisElement>& operator+=(
      const GenericPolynomial<BasisElement>& p);
  GenericPolynomial<BasisElement>& operator+=(const BasisElement& m);
  GenericPolynomial<BasisElement>& operator+=(double c);
  GenericPolynomial<BasisElement>& operator+=(const Variable& v);

  GenericPolynomial<BasisElement>& operator-=(
      const GenericPolynomial<BasisElement>& p);
  GenericPolynomial<BasisElement>& operator-=(const BasisElement& m);
  GenericPolynomial<BasisElement>& operator-=(double c);
  GenericPolynomial<BasisElement>& operator-=(const Variable& v);

  GenericPolynomial<BasisElement>& operator*=(
      const GenericPolynomial<BasisElement>& p);
  GenericPolynomial<BasisElement>& operator*=(const BasisElement& m);
  GenericPolynomial<BasisElement>& operator*=(double c);
  GenericPolynomial<BasisElement>& operator*=(const Variable& v);

  GenericPolynomial<BasisElement>& operator/=(double c);

  /** Returns true if this and @p p are structurally equal.
   */
  [[nodiscard]] bool EqualTo(const GenericPolynomial<BasisElement>& p) const;

  /** Returns true if this generic polynomial and @p p are equal after expanding
   * the coefficients. */
  [[nodiscard]] bool EqualToAfterExpansion(
      const GenericPolynomial<BasisElement>& p) const;

  /** Returns true if this polynomial and @p p are almost equal (the difference
   * in the corresponding coefficients are all less than @p tol), after
   * expanding the coefficients.
   */
  bool CoefficientsAlmostEqual(const GenericPolynomial<BasisElement>& p,
                               double tol) const;

  /** Returns a symbolic formula representing the condition where this
   * polynomial and @p p are the same.
   */
  Formula operator==(const GenericPolynomial<BasisElement>& p) const;

  /** Returns a symbolic formula representing the condition where this
   * polynomial and @p p are not the same.
   */
  Formula operator!=(const GenericPolynomial<BasisElement>& p) const;

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(
      HashAlgorithm& hasher,
      const GenericPolynomial<BasisElement>& item) noexcept {
    using drake::hash_append;
    for (const auto& [basis_element, coeff] :
         item.basis_element_to_coefficient_map_) {
      hash_append(hasher, basis_element);
      hash_append(hasher, coeff);
    }
  }

 private:
  // Throws std::exception if there is a variable appeared in both of
  // decision_variables() and indeterminates().
  void CheckInvariant() const;

  MapType basis_element_to_coefficient_map_;
  Variables indeterminates_;
  Variables decision_variables_;
};

/** Defines an explicit SFINAE alias for use with return types to dissuade CTAD
 * from trying to instantiate an invalid GenericElement<> for operator
 * overloads, (if that's actually the case).
 * See discussion for more info:
 * https://github.com/robotlocomotion/drake/pull/14053#pullrequestreview-488744679
 */
template <typename BasisElement>
using GenericPolynomialEnable = std::enable_if_t<
    std::is_base_of_v<PolynomialBasisElement, BasisElement>,
    GenericPolynomial<BasisElement>>;

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(const GenericPolynomial<BasisElement>& p) {
  return -1. * p;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(GenericPolynomial<BasisElement> p1,
          const GenericPolynomial<BasisElement>& p2) {
  return p1 += p2;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(GenericPolynomial<BasisElement> p, const BasisElement& m) {
  return p += m;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(GenericPolynomial<BasisElement> p, double c) {
  return p += c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(const BasisElement& m, GenericPolynomial<BasisElement> p) {
  return p += m;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(const BasisElement& m1, const BasisElement& m2) {
  return GenericPolynomial<BasisElement>(m1) + m2;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(const BasisElement& m, double c) {
  return GenericPolynomial<BasisElement>(m) + c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(double c, GenericPolynomial<BasisElement> p) {
  return p += c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(double c, const BasisElement& m) {
  return GenericPolynomial<BasisElement>(m) + c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(GenericPolynomial<BasisElement> p, const Variable& v) {
  return p += v;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator+(const Variable& v, GenericPolynomial<BasisElement> p) {
  return p += v;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(GenericPolynomial<BasisElement> p1,
          const GenericPolynomial<BasisElement>& p2) {
  return p1 -= p2;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(GenericPolynomial<BasisElement> p, const BasisElement& m) {
  return p -= m;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(GenericPolynomial<BasisElement> p, double c) {
  return p -= c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(const BasisElement& m, GenericPolynomial<BasisElement> p) {
  return p = -1 * p + m;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(const BasisElement& m1, const BasisElement& m2) {
  return GenericPolynomial<BasisElement>(m1) - m2;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(const BasisElement& m, double c) {
  return GenericPolynomial<BasisElement>(m) - c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(double c, GenericPolynomial<BasisElement> p) {
  return p = -p + c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(double c, const BasisElement& m) {
  return c - GenericPolynomial<BasisElement>(m);
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(GenericPolynomial<BasisElement> p, const Variable& v) {
  return p -= v;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator-(const Variable& v, GenericPolynomial<BasisElement> p) {
  return GenericPolynomial<BasisElement>(v, p.indeterminates()) - p;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator*(GenericPolynomial<BasisElement> p1,
          const GenericPolynomial<BasisElement>& p2) {
  return p1 *= p2;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator*(GenericPolynomial<BasisElement> p, const BasisElement& m) {
  return p *= m;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator*(GenericPolynomial<BasisElement> p, double c) {
  return p *= c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator*(const BasisElement& m, GenericPolynomial<BasisElement> p) {
  return p *= m;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator*(const BasisElement& m, double c) {
  return GenericPolynomial<BasisElement>(m) * c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator*(double c, GenericPolynomial<BasisElement> p) {
  return p *= c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator*(double c, const BasisElement& m) {
  return GenericPolynomial<BasisElement>(m) * c;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator*(GenericPolynomial<BasisElement> p, const Variable& v) {
  return p *= v;
}

template <typename BasisElement>
GenericPolynomialEnable<BasisElement>
operator*(const Variable& v, GenericPolynomial<BasisElement> p) {
  return p *= v;
}

/** Returns `p / v`. */
template <typename BasisElement>
GenericPolynomialEnable<BasisElement> operator/(
    GenericPolynomial<BasisElement> p, double v) {
  return p /= v;
}

/** Returns polynomial @p raised to @p n.
 * @param p The base polynomial.
 * @param n The exponent of the power. @pre n>=0.
 * */
template <typename BasisElement>
GenericPolynomialEnable<BasisElement> pow(
    const GenericPolynomial<BasisElement>& p, int n) {
  if (n < 0) {
    throw std::runtime_error(
        fmt::format("pow(): the degree should be non-negative, got {}.", n));
  } else if (n == 0) {
    return GenericPolynomial<BasisElement>(BasisElement());
  } else if (n == 1) {
    return p;
  } else if (n % 2 == 0) {
    const GenericPolynomial<BasisElement> half = pow(p, n / 2);
    return half * half;
  } else {
    const GenericPolynomial<BasisElement> half = pow(p, n / 2);
    return half * half * p;
  }
}

template <typename BasisElement>
std::ostream& operator<<(std::ostream& os,
                         const GenericPolynomial<BasisElement>& p) {
  const typename GenericPolynomial<BasisElement>::MapType& map{
      p.basis_element_to_coefficient_map()};
  if (map.empty()) {
    return os << 0;
  }
  auto it = map.begin();
  os << it->second << "*" << it->first;
  for (++it; it != map.end(); ++it) {
    os << " + " << it->second << "*" << it->first;
  }
  return os;
}

extern template class GenericPolynomial<MonomialBasisElement>;
extern template class GenericPolynomial<ChebyshevBasisElement>;
}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::GenericPolynomial<BasisElement>>. */
template <typename BasisElement>
struct hash<drake::symbolic::GenericPolynomial<BasisElement>>
    : public drake::DefaultHash {};
#if defined(__GLIBCXX__)
// Inform GCC that this hash function is not so fast (i.e. for-loop inside).
// This will enforce caching of hash results. See
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/unordered_associative.html
// for details.
template <typename BasisElement>
struct __is_fast_hash<hash<drake::symbolic::GenericPolynomial<BasisElement>>>
    : std::false_type {};
#endif
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {

// Defines Eigen traits needed for Matrix<drake::symbolic::Polynomial>.
template <>
struct NumTraits<
    drake::symbolic::GenericPolynomial<drake::symbolic::MonomialBasisElement>>
    : GenericNumTraits<drake::symbolic::GenericPolynomial<
          drake::symbolic::MonomialBasisElement>> {
  static inline int digits10() { return 0; }
};

template <>
struct NumTraits<
    drake::symbolic::GenericPolynomial<drake::symbolic::ChebyshevBasisElement>>
    : GenericNumTraits<drake::symbolic::GenericPolynomial<
          drake::symbolic::ChebyshevBasisElement>> {
  static inline int digits10() { return 0; }
};
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
