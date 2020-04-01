#pragma once

#include <complex>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <unsupported/Eigen/Polynomials>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/symbolic.h"

namespace drake {
/** A scalar multi-variate polynomial, modeled after the msspoly in spotless.
 *
 * Polynomial represents a list of additive Monomials, each one of which is a
 * product of a constant coefficient (of T, which by default is double) and any
 * number of distinct Terms (variables raised to positive integer powers).
 *
 * Variables are identified by integer indices rather than symbolic names, but
 * an automatic facility is provided to covert variable names up to four
 * characters into unique integers, provided those variables are named using
 * only lowercase letters and the "@#_." characters followed by a number.  For
 * example, valid names include "dx4" and "m_x".
 *
 * Monomials which have the same variables and powers may be constructed but
 * will be automatically combined: (3 * a * b * a) + (1.5 * b * a**2) will be
 * reduced to (4.5 * b * a**2) internally after construction.
 *
 * Polynomials can be added, subtracted, and multiplied.  They may only be
 * divided by scalars (of T) because Polynomials are not closed under division.
 * 
 * @tparam_default_scalar
 */
template <typename T = double>
class Polynomial {
 public:
  typedef unsigned int VarType;
  /// This should be 'unsigned int' but MSVC considers a call to std::pow(...,
  /// unsigned int) ambiguous because it won't cast unsigned int to int.
  typedef int PowerType;
  typedef typename Eigen::NumTraits<T>::Real RealScalar;
  typedef std::complex<RealScalar> RootType;
  typedef Eigen::Matrix<RootType, Eigen::Dynamic, 1> RootsType;

  template <typename Rhs, typename Lhs>
  struct Product {
    typedef decltype(static_cast<Rhs>(0) * static_cast<Lhs>(0)) type;
  };

  /// An individual variable raised to an integer power; e.g. x**2.
  class Term {
   public:
    VarType var;
    PowerType power;

    bool operator==(const Term& other) const {
      return (var == other.var) && (power == other.power);
    }

    /// A comparison to allow std::lexicographical_compare on this class; does
    /// not reflect any sort of mathematical total order.
    bool operator<(const Term& other) const {
      return ((var < other.var) ||
              ((var == other.var) && (power < other.power)));
    }
  };

  /// An additive atom of a Polynomial: The product of any number of
  /// Terms and a coefficient.
  class Monomial {
   public:
    T coefficient;
    std::vector<Term> terms;  // a list of N variable ids

    bool operator==(const Monomial& other) const {
      return (coefficient == other.coefficient) && (terms == other.terms);
    }

    /// A comparison to allow std::lexicographical_compare on this class; does
    /// not reflect any sort of mathematical total order.
    bool operator<(const Monomial& other) const {
      return ((coefficient < other.coefficient)  ||
              ((coefficient == other.coefficient) && (terms < other.terms)));
    }

    int GetDegree() const;
    int GetDegreeOf(VarType var) const;
    bool HasSameExponents(const Monomial& other) const;

    /// Factors this by other; returns 0 iff other does not divide this.
    Monomial Factor(const Monomial& divisor) const;
  };

 private:
  /// The Monomial atoms of the Polynomial.
  std::vector<Monomial> monomials_;

  /// True iff only 0 or 1 distinct variables appear in the Polynomial.
  bool is_univariate_;

 public:
  /// Construct the vacuous polynomial, "0".
  Polynomial(void) : is_univariate_(true) {}

  /// Construct a Polynomial of a single constant. e.g. "5".
  // This is required for some Eigen operations when used in a
  // polynomial matrix.
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  Polynomial(const T& scalar);

  /// Construct a Polynomial consisting of a single Monomial, e.g. "5xy**3".
  Polynomial(const T coeff, const std::vector<Term>& terms);

  /// Construct a Polynomial from a sequence of Monomials.
  Polynomial(typename std::vector<Monomial>::const_iterator start,
             typename std::vector<Monomial>::const_iterator finish);

  /// Construct a polynomial consisting of a single Monomial of the variable
  /// named varname + num.
  explicit Polynomial(const std::string varname, const unsigned int num = 1);

  /// Construct a single Monomial of the given coefficient and variable.
  Polynomial(const T& coeff, const VarType& v);

  /// A legacy constructor for univariate polynomials:  Takes a vector
  /// of coefficients for the constant, x, x**2, x**3... Monomials.
  template <typename Derived>
  explicit Polynomial(Eigen::MatrixBase<Derived> const& coefficients) {
    VarType v = VariableNameToId("t");
    for (int i = 0; i < coefficients.size(); i++) {
      Monomial m;
      m.coefficient = coefficients(i);
      if (i > 0) {
        Term t;
        t.var = v;
        t.power = i;
        m.terms.push_back(t);
      }
      monomials_.push_back(m);
    }
    is_univariate_ = true;
  }

  /// Returns the number of unique Monomials (and thus the number of
  /// coefficients) in this Polynomial.
  int GetNumberOfCoefficients() const;

  /** Returns the highest degree of any Monomial in this Polynomial.
   *
   * The degree of a multivariate Monomial is the product of the degrees of
   * each of its terms. */
  int GetDegree() const;

  /// Returns true iff this is a sum of terms of degree 1, plus a constant.
  bool IsAffine() const;

  /// If the polynomial is "simple" -- e.g. just a single term with
  /// coefficient 1 -- then returns that variable; otherwise returns 0.
  VarType GetSimpleVariable() const;

  const std::vector<Monomial>& GetMonomials() const;

  Eigen::Matrix<T, Eigen::Dynamic, 1> GetCoefficients() const;

  /// Returns a set of all of the variables present in this Polynomial.
  std::set<VarType> GetVariables() const;

  /** Evaluate a univariate Polynomial at a specific point.
   *
   * Evaluates a univariate Polynomial at the given x.
   * @throws std::runtime_error if this Polynomial is not univariate.
   *
   * @p x may be of any type supporting the ** and + operations (which can be
   * different from both CoefficientsType and RealScalar).
   *
   * This method may also be used for efficient evaluation of the derivatives of
   * the univariate polynomial, evaluated at @p x.  @p derivative_order = 0 (the
   * default) returns the polynomial value without differentiation.  @p
   * derivative_order = 1 returns the first derivative, etc.
   *
   * @pre derivative_order must be non-negative.
   */
  template <typename U>
  typename Product<T, U>::type EvaluateUnivariate(
      const U& x, int derivative_order = 0) const {
    // Note: have to remove_const because Product<AutoDiff, AutoDiff>::type and
    // even Product<double, AutoDiff>::type returns const AutoDiff.
    typedef typename std::remove_const<typename Product<T, U>::type>::type
        ProductType;

    if (!is_univariate_)
      throw std::runtime_error(
          "this method can only be used for univariate polynomials");

    DRAKE_DEMAND(derivative_order >= 0);
    ProductType value = 0;
    using std::pow;
    for (typename std::vector<Monomial>::const_iterator iter =
             monomials_.begin();
         iter != monomials_.end(); iter++) {
      PowerType degree = iter->terms.empty() ? 0 : iter->terms[0].power;
      if (degree < derivative_order) continue;
      T coefficient = iter->coefficient;
      for (int i = 0; i < derivative_order; i++) {
        coefficient *= degree--;
      }
      if (degree == 0) {
        value += coefficient;
      } else if (degree == 1) {
        value += coefficient * x;
      } else {  // degree > 1.
        value += coefficient * pow(static_cast<ProductType>(x), degree);
      }
    }
    return value;
  }

  /** Evaluate a multivariate Polynomial at a specific point.
   *
   * Evaluates a Polynomial with the given values for each variable.
   * @throws std::out_of_range if the Polynomial contains variables for which
   * values were not provided.
   *
   * The provided values may be of any type which is std::is_arithmetic
   * (supporting the std::pow, *, and + operations) and need not be
   * CoefficientsType or RealScalar)
   */
  template <typename U>
  typename Product<T, U>::type EvaluateMultivariate(
      const std::map<VarType, U>& var_values) const {
    using std::pow;
    typedef typename std::remove_const<
      typename Product<T, U>::type>::type ProductType;
    ProductType value = 0;
    for (const Monomial& monomial : monomials_) {
      ProductType monomial_value = monomial.coefficient;
      for (const Term& term : monomial.terms) {
        monomial_value *=
            pow(static_cast<ProductType>(var_values.at(term.var)), term.power);
      }
      value += monomial_value;
    }
    return value;
  }

  /** Substitute values for some but not necessarily all variables of a
   * Polynomial.
   *
   * Analogous to EvaluateMultivariate, but:
   *  (1) Restricted to T, and
   *  (2) Need not map every variable in var_values.
   *
   * Returns a Polynomial in which each variable in var_values has been
   * replaced with its value and constants appropriately combined.
   */
  Polynomial EvaluatePartial(
      const std::map<VarType, T>& var_values) const;

  /// Replaces all instances of variable orig with replacement.
  void Subs(const VarType& orig, const VarType& replacement);

  /** Takes the derivative of this (univariate) Polynomial.
   *
   * Returns a new Polynomial that is the derivative of this one in its sole
   * variable.
   * @throws std::exception if this Polynomial is not univariate.
   *
   * If derivative_order is given, takes the nth derivative of this
   * Polynomial.
   */
  Polynomial Derivative(int derivative_order = 1) const;

  /** Takes the integral of this (univariate, non-constant) Polynomial.
   *
   * Returns a new Polynomial that is the indefinite integral of this one in
   * its sole variable.
   * @throws std::exception if this Polynomial is not univariate, or if it has
   * no variables.
   *
   * If integration_constant is given, adds that constant as the constant
   * term (zeroth-order coefficient) of the resulting Polynomial.
   */
  Polynomial Integral(const T& integration_constant = 0.0) const;

  bool operator==(const Polynomial& other) const;

  Polynomial& operator+=(const Polynomial& other);

  Polynomial& operator-=(const Polynomial& other);

  Polynomial& operator*=(const Polynomial& other);

  Polynomial& operator+=(const T& scalar);

  Polynomial& operator-=(const T& scalar);

  Polynomial& operator*=(const T& scalar);

  Polynomial& operator/=(const T& scalar);

  const Polynomial operator+(const Polynomial& other) const;

  const Polynomial operator-(const Polynomial& other) const;

  const Polynomial operator-() const;

  const Polynomial operator*(const Polynomial& other) const;

  friend const Polynomial operator+(const Polynomial& p,
                                    const T& scalar) {
    Polynomial ret = p;
    ret += scalar;
    return ret;
  }

  friend const Polynomial operator+(const T& scalar,
                                    const Polynomial& p) {
    Polynomial ret = p;
    ret += scalar;
    return ret;
  }

  friend const Polynomial operator-(const Polynomial& p,
                                    const T& scalar) {
    Polynomial ret = p;
    ret -= scalar;
    return ret;
  }

  friend const Polynomial operator-(const T& scalar,
                                    const Polynomial& p) {
    Polynomial ret = -p;
    ret += scalar;
    return ret;
  }

  friend const Polynomial operator*(const Polynomial& p,
                                    const T& scalar) {
    Polynomial ret = p;
    ret *= scalar;
    return ret;
  }
  friend const Polynomial operator*(const T& scalar,
                                    const Polynomial& p) {
    Polynomial ret = p;
    ret *= scalar;
    return ret;
  }

  const Polynomial operator/(const T& scalar) const;

  /// A comparison to allow std::lexicographical_compare on this class; does
  /// not reflect any sort of mathematical total order.
  bool operator<(const Polynomial& other) const {
    // Just delegate to the default vector std::lexicographical_compare.
    return monomials_ < other.monomials_;
  }

  /** Returns the roots of this (univariate) Polynomial.
   *
   * Returns the roots of a univariate Polynomial as an Eigen column vector of
   * complex numbers whose components are of the RealScalar type.
   * @throws std::exception of this Polynomial is not univariate.
   */
  RootsType Roots() const;

  /** Checks if a (univariate) Polynomial is approximately equal to this one.
   *
   * Checks that every coefficient of other is within tol of the
   * corresponding coefficient of this Polynomial.
   * @throws std::exception if either Polynomial is not univariate.
   */
  boolean<T> IsApprox(const Polynomial<T>& other, const RealScalar& tol) const;

  /** Constructs a Polynomial representing the symbolic expression `e`.
   * Note that the ID of a variable is preserved in this translation.
   *
   * @throw std::runtime_error if `e` is not polynomial-convertible.
   * @pre e.is_polynomial() is true.
   */
  static Polynomial<T> FromExpression(const drake::symbolic::Expression& e);

  friend std::ostream& operator<<(std::ostream& os, const Monomial& m) {
    //    if (m.coefficient == 0) return os;

    bool print_star = false;
    if (m.coefficient == -1) {
      os << "-";
    } else if (m.coefficient != 1 || m.terms.empty()) {
      os << '(' << m.coefficient << ")";
      print_star = true;
    }

    for (typename std::vector<Term>::const_iterator iter = m.terms.begin();
         iter != m.terms.end(); iter++) {
      if (print_star)
        os << '*';
      else
        print_star = true;
      os << IdToVariableName(iter->var);
      if (iter->power != 1) {
        os << "^" << iter->power;
      }
    }
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const Polynomial& poly) {
    if (poly.monomials_.empty()) {
      os << "0";
      return os;
    }

    for (typename std::vector<Monomial>::const_iterator iter =
             poly.monomials_.begin();
         iter != poly.monomials_.end(); iter++) {
      os << *iter;
      if (iter + 1 != poly.monomials_.end() && (iter + 1)->coefficient != -1)
        os << '+';
    }
    return os;
  }

  //@{
  /** Variable name/ID conversion facility. */
  static bool IsValidVariableName(const std::string name);

  static VarType VariableNameToId(const std::string name,
                                  const unsigned int m = 1);

  static std::string IdToVariableName(const VarType id);
  //@}

  template <typename U>
  friend Polynomial<U> pow(const Polynomial<U>& p,
                           typename Polynomial<U>::PowerType n);

 private:
  /// Sorts through Monomial list and merges any that have the same powers.
  void MakeMonomialsUnique(void);
};

/** Provides power function for Polynomial. */
template <typename T>
Polynomial<T> pow(
    const Polynomial<T>& base,
    typename Polynomial<T>::PowerType exponent) {
  DRAKE_DEMAND(exponent >= 0);
  if (exponent == 0) {
    return Polynomial<T>{1.0};
  }
  const Polynomial<T> pow_half{pow(base, exponent / 2)};
  if (exponent % 2 == 1) {
    return base * pow_half * pow_half;  // Odd exponent case.
  } else {
    return pow_half * pow_half;  // Even exponent case.
  }
}

template <typename T, int Rows, int Cols>
std::ostream& operator<<(
    std::ostream& os,
    const Eigen::Matrix<Polynomial<T>, Rows, Cols>& poly_mat) {
  for (int i = 0; i < poly_mat.rows(); i++) {
    os << "[ ";
    for (int j = 0; j < poly_mat.cols(); j++) {
      os << poly_mat(i, j);
      if (j < (poly_mat.cols() - 1)) os << " , ";
    }
    os << " ]" << std::endl;
  }
  return os;
}

template <>
boolean<symbolic::Expression> Polynomial<symbolic::Expression>::IsApprox(
    const Polynomial<symbolic::Expression>& other,
    const Polynomial<symbolic::Expression>::RealScalar& tol) const;

#ifndef DRAKE_DOXYGEN_CXX
namespace symbolic {
namespace internal {
// Helper to implement (deprecated) Expression::ToPolynomial.
// TODO(soonho-tri): Remove this on or after 2020-07-01 when we remove
// Expression::ToPolynomial.
inline drake::Polynomial<double> ToPolynomial(
    const drake::symbolic::Expression& e, const ToPolynomialHelperTag&) {
  return drake::Polynomial<double>::FromExpression(e);
}
}  // namespace internal
}  // namespace symbolic
#endif

typedef Polynomial<double> Polynomiald;

/// A column vector of polynomials; used in several optimization classes.
typedef Eigen::Matrix<Polynomiald, Eigen::Dynamic, 1> VectorXPoly;
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::Polynomial)

/** Provides power function for Polynomial. */
template <typename T>
DRAKE_DEPRECATED("2020-07-01", "Use drake::pow instead.")
drake::Polynomial<T> pow(const drake::Polynomial<T>& base,
                         typename drake::Polynomial<T>::PowerType exponent) {
  return drake::pow(base, exponent);
}

template <typename T = double>
using Polynomial DRAKE_DEPRECATED(
    "2020-07-01", "Use drake::Polynomial instead.") = drake::Polynomial<T>;

using Polynomiald DRAKE_DEPRECATED("2020-07-01", "Use drake::Polynomiald.") =
    drake::Polynomial<double>;

using VectorXPoly DRAKE_DEPRECATED("2020-07-01", "Use drake::VectorXPoly.") =
    Eigen::Matrix<drake::Polynomiald, Eigen::Dynamic, 1>;
