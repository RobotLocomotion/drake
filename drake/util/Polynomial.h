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

#include "drake/core/Gradient.h"
#include "drake/drakePolynomial_export.h"

/// A scalar multi-variate polynomial, modeled after the msspoly in spotless.
/**
 * Polynomial represents a list of additive Monomials, each one of which is a
 * product of a constant coefficient (of _CoefficientType, which by default is
 * double) and any number of distinct Terms (variables raised to positive
 * integer powers).
 *
 * Variables are identified by integer indices rather than symbolic names, but
 * an automatic facility is provided to covert variable names up to four
 * characters into unique integers, provided those variables are named using
 * only lowercase letters and the "@#_." characters followed by a number.  For
 * example, valid names include "dx4" and "m_x".
 *
 * Monomials which have the same variables and powers may be constructed but
 * will be automatically combined:
 *   (3 * a * b * a) + (1.5 * b * a**2)
 * will be reduced to
 *   (4.5 * b * a**2)
 * internally after construction.
 *
 * Polynomials can be added, subtracted, and multiplied.  They may only be
 * divided by scalars (of _CoefficientType) because Polynomials are not closed
 * under division.
 */
template <typename _CoefficientType = double>
class DRAKEPOLYNOMIAL_EXPORT Polynomial {
 public:
  typedef _CoefficientType CoefficientType;
  typedef unsigned int VarType;
  /// This should be 'unsigned int' but MSVC considers a call to std::pow(...,
  /// unsigned int) ambiguous because it won't cast unsigned int to int.
  typedef int PowerType;
  typedef typename Eigen::NumTraits<CoefficientType>::Real RealScalar;
  typedef std::complex<RealScalar> RootType;
  typedef Eigen::Matrix<RootType, Eigen::Dynamic, 1> RootsType;

  template <typename Rhs, typename Lhs>
  struct Product {
    typedef decltype((Rhs)0 * (Lhs)0) type;
  };

  /// An individual variable raised to an integer power; e.g. x**2.
  class DRAKEPOLYNOMIAL_EXPORT Term {
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

  /// \brief An additive atom of a Polynomial: The product of any number of
  /// Terms and a coefficient.
  class DRAKEPOLYNOMIAL_EXPORT Monomial {
   public:
    CoefficientType coefficient;
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

    int getDegree() const;
    int getDegreeOf(VarType var) const;
    bool hasSameExponents(const Monomial& other) const;

    /// Factors this by other; returns 0 iff other does not divide this.
    Monomial factor(const Monomial& divisor) const;
  };

 private:
  /// The Monomial atoms of the Polynomial.
  std::vector<Monomial> monomials;

  /// True iff only 0 or 1 distinct variables appear in the Polynomial.
  bool is_univariate;

 public:
  /// Construct the vacuous polynomial, "0".
  Polynomial(void) : is_univariate(true) {}

  /// Construct a Polynomial of a single constant. e.g. "5".
  // This is required for some Eigen operations when used in a
  // polynomial matrix.
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  Polynomial(const CoefficientType& scalar);

  /// Construct a Polynomial consisting of a single Monomial, e.g. "5xy**3".
  Polynomial(const CoefficientType coeff, const std::vector<Term>& terms);

  /// Construct a Polynomial from a sequence of Monomials.
  Polynomial(typename std::vector<Monomial>::const_iterator start,
             typename std::vector<Monomial>::const_iterator finish);

  /// Construct a polynomial consisting of a single Monomial of the variable
  /// named varname + num.
  explicit Polynomial(const std::string varname, const unsigned int num = 1);

  /// Construct a single Monomial of the given coefficient and variable.
  Polynomial(const CoefficientType& coeff, const VarType& v);

  /// A legacy constructor for univariate polynomials:  Takes a vector
  /// of coefficients for the constant, x, x**2, x**3... Monomials.
  template <typename Derived>
  explicit Polynomial(Eigen::MatrixBase<Derived> const& coefficients) {
    VarType v = variableNameToId("t");
    for (int i = 0; i < coefficients.size(); i++) {
      Monomial m;
      m.coefficient = coefficients(i);
      if (i > 0) {
        Term t;
        t.var = v;
        t.power = i;
        m.terms.push_back(t);
      }
      monomials.push_back(m);
    }
    is_univariate = true;
  }

  /// Returns the number of unique Monomials (and thus the number of
  /// coefficients) in this Polynomial.
  int getNumberOfCoefficients() const;

  /// Returns the highest degree of any Monomial in this Polynomial.
  /**
   * The degree of a multivariate Monomial is the product of the degrees of
   * each of its terms. */
  int getDegree() const;

  /// If the polynomial is "simple" -- e.g. just a single term with
  /// coefficient 1 -- then returns that variable; otherwise returns 0.
  VarType getSimpleVariable() const;

  const std::vector<Monomial>& getMonomials() const;

  Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1> getCoefficients() const;

  /// Returns a set of all of the variables present in this Polynomial.
  std::set<VarType> getVariables() const;

  /// Evaluate a univariate Polynomial at a specific point.
  /**
   * Evaluates a univariate Polynomial at the given x.  Throws an
   * exception of this Polynomial is not univariate.
   *
   * x may be of any type supporting the ** and + operations (which can
   * be different from both CoefficientsType and RealScalar)
   */
  template <typename T>
  typename Product<CoefficientType, T>::type evaluateUnivariate(
      const T& x) const {
    typedef typename Product<CoefficientType, T>::type ProductType;

    if (!is_univariate)
      throw std::runtime_error(
          "this method can only be used for univariate polynomials");
    ProductType value = 0;
    for (typename std::vector<Monomial>::const_iterator iter =
             monomials.begin();
         iter != monomials.end(); iter++) {
      if (iter->terms.empty())
        value += iter->coefficient;
      else
        value += iter->coefficient *
                 pow((ProductType)x,
                     (PowerType)iter->terms[0].power);
    }
    return value;
  }

  /// Evaluate a multivariate Polynomial at a specific point.
  /**
   * Evaluates a Polynomial with the given values for each variable.  Throws
   * std::out_of_range if the Polynomial contains variables for which values
   * were not provided.
   *
   * The provided values may be of any type which is std::is_arithmetic
   * (supporting the std::pow, *, and + operations) and need not be
   * CoefficientsType or RealScalar)
   */
  template <typename T>
  typename Product<CoefficientType, T>::type evaluateMultivariate(
      const std::map<VarType, T>& var_values) const {
    typedef typename std::remove_const<
      typename Product<CoefficientType, T>::type>::type ProductType;
    ProductType value = 0;
    for (const Monomial& monomial : monomials) {
      ProductType monomial_value = monomial.coefficient;
      for (const Term& term : monomial.terms) {
        monomial_value *= std::pow(
            static_cast<ProductType>(var_values.at(term.var)),
            term.power);
      }
      value += monomial_value;
    }
    return value;
  }

  /// Specialization of evaluateMultivariate on TaylorVarXd.
  /**
   * Specialize evaluateMultivariate on TaylorVarXd because Eigen autodiffs
   * implement a confusing subset of operators and conversions that makes a
   * strictly generic approach too confusing and unreadable.
   *
   * Note that it is up to the caller to ensure that all of the TaylorVarXds
   * in var_values correctly correspond to one another, because Polynomial has
   * no knowledge of what partial derivative terms the indices of a given
   * TaylorVarXd correspond to.
   */
  Drake::TaylorVarXd evaluateMultivariate(
      const std::map<VarType, Drake::TaylorVarXd>& var_values) const {
    Drake::TaylorVarXd value(0);
    for (const Monomial& monomial : monomials) {
      Drake::TaylorVarXd monomial_value(monomial.coefficient);
      for (const Term& term : monomial.terms) {
        monomial_value *= pow(var_values.at(term.var), term.power);
      }
      value += monomial_value;
    }
    return value;
  }

  /// Substitute values for some but not necessarily all variables of a
  /// polynomial.
  /**
   * Analogous to evaluateMultivariate, but:
   *  (1) Restricted to CoefficientType, and
   *  (2) Need not map every variable in var_values.
   *
   * Returns a Polynomial in which each variable in var_values has been
   * replaced with its value and constants appropriately combined.
   */
  Polynomial evaluatePartial(
      const std::map<VarType, CoefficientType>& var_values) const;

  /// Replaces all instances of variable orig with replacement.
  void subs(const VarType& orig, const VarType& replacement);

  /// Takes the derivative of this (univariate) Polynomial.
  /**
   * Returns a new Polynomial that is the derivative of this one in its sole
   * variable.  Throws an exception of this Polynomial is not univariate.
   *
   * If derivative_order is given, takes the nth derivative of this
   * Polynomial.
   */
  Polynomial derivative(unsigned int derivative_order = 1) const;

  /// Takes the integral of this (univariate, non-constant) Polynomial.
  /**
   * Returns a new Polynomial that is the indefinite integral of this one in
   * its sole variable.  Throws an exception of this Polynomial is not
   * univariate, or if it has no variables.
   *
   * If integration_constant is given, adds that constant as the constant
   * term (zeroth-order coefficient) of the resulting Polynomial.
   */
  Polynomial integral(const CoefficientType& integration_constant = 0.0) const;

  bool operator==(const Polynomial& other) const;

  Polynomial& operator+=(const Polynomial& other);

  Polynomial& operator-=(const Polynomial& other);

  Polynomial& operator*=(const Polynomial& other);

  Polynomial& operator+=(const CoefficientType& scalar);

  Polynomial& operator-=(const CoefficientType& scalar);

  Polynomial& operator*=(const CoefficientType& scalar);

  Polynomial& operator/=(const CoefficientType& scalar);

  const Polynomial operator+(const Polynomial& other) const;

  const Polynomial operator-(const Polynomial& other) const;

  const Polynomial operator-() const;

  const Polynomial operator*(const Polynomial& other) const;

  friend const Polynomial operator+(const Polynomial& p,
                                    const CoefficientType& scalar) {
    Polynomial ret = p;
    ret += scalar;
    return ret;
  }

  friend const Polynomial operator+(const CoefficientType& scalar,
                                    const Polynomial& p) {
    Polynomial ret = p;
    ret += scalar;
    return ret;
  }

  friend const Polynomial operator-(const Polynomial& p,
                                    const CoefficientType& scalar) {
    Polynomial ret = p;
    ret -= scalar;
    return ret;
  }

  friend const Polynomial operator-(const CoefficientType& scalar,
                                    const Polynomial& p) {
    Polynomial ret = -p;
    ret += scalar;
    return ret;
  }

  friend const Polynomial operator*(const Polynomial& p,
                                    const CoefficientType& scalar) {
    Polynomial ret = p;
    ret *= scalar;
    return ret;
  }
  friend const Polynomial operator*(const CoefficientType& scalar,
                                    const Polynomial& p) {
    Polynomial ret = p;
    ret *= scalar;
    return ret;
  }

  const Polynomial operator/(const CoefficientType& scalar) const;

  /// A comparison to allow std::lexicographical_compare on this class; does
  /// not reflect any sort of mathematical total order.
  bool operator<(const Polynomial& other) const {
    // Just delegate to the default vector std::lexicographical_compare.
    return monomials < other.monomials;
  }

  /// Returns the roots of this (univariate) Polynomial.
  /**
   * Returns the roots of a univariate Polynomial as an Eigen column vector of
   * complex numbers whose components are of the RealScalar type.  Throws an
   * exception of this Polynomial is not univariate.
   */
  RootsType roots() const;

  /// Checks if a (univariate) Polynomial is approximately equal to this one.
  /**
   * Checks that every coefficient of other is within tol of the
   * corresponding coefficient of this Polynomial.  Throws an exception if
   * either Polynomial is not univariate.
   */
  bool isApprox(const Polynomial& other, const RealScalar& tol) const;

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
      os << idToVariableName(iter->var);
      if (iter->power != 1) {
        os << "^" << iter->power;
      }
    }
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const Polynomial& poly) {
    if (poly.monomials.empty()) {
      os << "0";
      return os;
    }

    for (typename std::vector<Monomial>::const_iterator iter =
             poly.monomials.begin();
         iter != poly.monomials.end(); iter++) {
      os << *iter;
      if (iter + 1 != poly.monomials.end() && (iter + 1)->coefficient != -1)
        os << '+';
    }
    return os;
  }

  /// Obtains a matrix of random unvariate Polynomials of the specified size.
  static Eigen::Matrix<Polynomial, Eigen::Dynamic, Eigen::Dynamic>
  randomPolynomialMatrix(Eigen::Index num_coefficients_per_polynomial,
                         Eigen::Index rows, Eigen::Index cols) {
    Eigen::Matrix<Polynomial, Eigen::Dynamic, Eigen::Dynamic> mat(rows, cols);
    for (Eigen::Index row = 0; row < mat.rows(); ++row) {
      for (Eigen::Index col = 0; col < mat.cols(); ++col) {
        auto coeffs =
            (Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1>::Random(
                 num_coefficients_per_polynomial)).eval();
        mat(row, col) = Polynomial(coeffs);
      }
    }
    return mat;
  }

 private:
  //@{
  /** Variable name/ID conversion facility. */
  static bool isValidVariableName(const std::string name);

  static VarType variableNameToId(const std::string name,
                                  const unsigned int m = 1);

  static std::string idToVariableName(const VarType id);
  //@}

  //@{
  /// Local version of pow to deal with autodiff.
  /**
   * A version of std::pow that uses std::pow for arithmetic types and
   * repeated multiplication for non-arithmetic types (e.g., autodiff).
   */
  template <bool B, typename T = void>
  using enable_if_t = typename std::enable_if<B, T>::type;
  template <typename Base>
  static Base pow(
      const enable_if_t<std::is_arithmetic<Base>::value, Base>& base,
      const PowerType& exponent) {
    return std::pow(base, exponent);
  }

  template <typename Base>
  static Base pow(const Base& base, const PowerType& exponent) {
    Base result = base;
    for (int i = 1; i < exponent; i++) {
      result = result * base;
    }
    return result;
  }
  //@}

  /// Sorts through Monomial list and merges any that have the same powers.
  void makeMonomialsUnique(void);
};

template <typename CoefficientType, int Rows, int Cols>
std::ostream& operator<<(
    std::ostream& os,
    const Eigen::Matrix<Polynomial<CoefficientType>, Rows, Cols>& poly_mat) {
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

typedef Polynomial<double> Polynomiald;

/// A column vector of polynomials; used in several optimization classes.
typedef Eigen::Matrix<Polynomiald, Eigen::Dynamic, 1> VectorXPoly;
