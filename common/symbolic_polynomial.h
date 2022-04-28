#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <algorithm>
#include <functional>
#include <map>
#include <ostream>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
namespace internal {
// Compares two monomials using the lexicographic order. It is used in
// symbolic::Polynomial::MapType. See
// https://en.wikipedia.org/wiki/Monomial_order for different monomial orders.
struct CompareMonomial {
  bool operator()(const Monomial& m1, const Monomial& m2) const {
    const auto& powers1 = m1.get_powers();
    const auto& powers2 = m2.get_powers();
    return std::lexicographical_compare(
        powers1.begin(), powers1.end(), powers2.begin(), powers2.end(),
        [](const std::pair<const Variable, int>& p1,
           const std::pair<const Variable, int>& p2) {
          const Variable& v1{p1.first};
          const int i1{p1.second};
          const Variable& v2{p2.first};
          const int i2{p2.second};
          if (v1.less(v2)) {
            // m2 does not have the variable v1 explicitly, so we treat it as if
            // it has (v1)⁰. That is, we need "return i1 < 0", but i1 should be
            // positive, so this case always returns false.
            return false;
          }
          if (v2.less(v1)) {
            // m1 does not have the variable v2 explicitly, so we treat it as
            // if it has (v2)⁰. That is, we need "return 0 < i2", but i2 should
            // be positive, so it always returns true.
            return true;
          }
          return i1 < i2;
        });
  }
};
}  // namespace internal

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
// TODO(hongkai.dai) when symbolic::GenericPolynomial is ready, we will
// deprecate symbolic::Polynomial class, and create an alias using
// symbolic::Polynomial=symbolic::GenericPolynomial<MonomialBasisElement>;
// We will copy the unit tests in symbolic_polynomial_test.cc to
// symbolic_generic_polynomial_test.cc
class Polynomial {
 public:
  using MapType = std::map<Monomial, Expression, internal::CompareMonomial>;

  /// Constructs a zero polynomial.
  Polynomial() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Polynomial)

  /** Constructs a default value.  This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit Polynomial(std::nullptr_t) : Polynomial() {}

  /// Constructs a polynomial from a map, Monomial → Expression.
  explicit Polynomial(MapType map);

  /// Constructs a polynomial from a monomial @p m. Note that all variables
  /// in `m` are considered as indeterminates.
  //
  // Note that this implicit conversion is desirable to have a dot product of
  // two Eigen::Vector<Monomial>s return a Polynomial.
  // NOLINTNEXTLINE(runtime/explicit)
  Polynomial(const Monomial& m);

  /// Constructs a polynomial from an expression @p e. Note that all variables
  /// in `e` are considered as indeterminates.
  ///
  /// @throws std::exception if @p e is not a polynomial.
  explicit Polynomial(const Expression& e);

  /// Constructs a polynomial from an expression @p e by decomposing it with
  /// respect to @p indeterminates.
  ///
  /// @note It collects the intersection of the variables appeared in `e` and
  /// the provided @p indeterminates.
  ///
  /// @throws std::exception if @p e is not a polynomial in @p
  /// indeterminates.
  Polynomial(const Expression& e, Variables indeterminates);

  /// Returns the indeterminates of this polynomial.
  const Variables& indeterminates() const;

  /// Returns the decision variables of this polynomial.
  const Variables& decision_variables() const;

  /// Sets the indeterminates to `new_indeterminates`.
  ///
  /// Changing the indeterminates would change `monomial_to_coefficient_map()`,
  /// and also potentially the degree of the polynomial. Here is an example.
  ///
  /// @code
  /// // p is a quadratic polynomial with x being the indeterminates.
  /// symbolic::Polynomial p(a * x * x + b * x + c, {x});
  /// // p.monomial_to_coefficient_map() contains {1: c, x: b, x*x:a}.
  /// std::cout << p.TotalDegree(); // prints 2.
  /// // Now set (a, b, c) to the indeterminates. p becomes a linear
  /// // polynomial of a, b, c.
  /// p.SetIndeterminates({a, b, c});
  /// // p.monomial_to_coefficient_map() now is {a: x * x, b: x, c: 1}.
  /// std::cout << p.TotalDegree(); // prints 1.
  /// @endcode
  void SetIndeterminates(const Variables& new_indeterminates);

  /// Returns the highest degree of this polynomial in a variable @p v.
  int Degree(const Variable& v) const;

  /// Returns the total degree of this polynomial.
  int TotalDegree() const;

  /// Returns the mapping from a Monomial to its corresponding coefficient of
  /// this polynomial.
  const MapType& monomial_to_coefficient_map() const;

  /// Returns an equivalent symbolic expression of this polynomial.
  Expression ToExpression() const;

  /** Differentiates this polynomial with respect to the variable @p x. Note
   * that a variable @p x can be either a decision variable or an indeterminate.
   */
  Polynomial Differentiate(const Variable& x) const;

  /// Computes the Jacobian matrix J of the polynomial with respect to
  /// @p vars. J(0,i) contains ∂f/∂vars(i).
  template <typename Derived>
  Eigen::Matrix<Polynomial, 1, Derived::RowsAtCompileTime> Jacobian(
      const Eigen::MatrixBase<Derived>& vars) const {
    static_assert(std::is_same_v<typename Derived::Scalar, Variable> &&
                      (Derived::ColsAtCompileTime == 1),
                  "The argument of Polynomial::Jacobian should be a vector of "
                  "symbolic variables.");
    const VectorX<Expression>::Index n{vars.size()};
    Eigen::Matrix<Polynomial, 1, Derived::RowsAtCompileTime> J{1, n};
    for (VectorX<Expression>::Index i = 0; i < n; ++i) {
      J(0, i) = Differentiate(vars(i));
    }
    return J;
  }

  /** Integrates this polynomial with respect to an indeterminate @p x.
   * Integration with respect to decision variables is not supported yet. If @p
   * x is not an indeterminate nor decision variable, then it will be added to
   * the list of indeterminates.
   * @throws std::exception if @p x is a decision variable.
   */
  Polynomial Integrate(const Variable& x) const;

  /** Computes the definite integrate of this polynomial with respect to the
   * indeterminate @p x over the domain [a, b].  Integration with respect to
   * decision variables is not supported yet.
   * @throws std::exception if @p x is a decision variable.
   */
  Polynomial Integrate(const Variable& x, double a, double b) const;

  /// Evaluates this polynomial under a given environment @p env.
  ///
  /// @throws std::exception if there is a variable in this polynomial whose
  /// assignment is not provided by @p env.
  double Evaluate(const Environment& env) const;

  /// Partially evaluates this polynomial using an environment @p env.
  ///
  /// @throws std::exception if NaN is detected during evaluation.
  Polynomial EvaluatePartial(const Environment& env) const;

  /// Partially evaluates this polynomial by substituting @p var with @p c.
  ///
  /// @throws std::exception if NaN is detected at any point during
  /// evaluation.
  Polynomial EvaluatePartial(const Variable& var, double c) const;

  /// Evaluates the polynomial at a batch of indeterminates values.
  /// @param[in] indeterminates Must include all this->indeterminates()
  /// @param[in] indeterminates_values Each column of `indeterminates_values`
  /// store one specific values of `indeterminates`.
  /// indeterminates_values.rows() == indeterminates.rows().
  /// @return polynomial_values polynomial_values(j) is obtained by
  /// substituting indeterminates(i) in this polynomial with
  /// indeterminates_values(i, j) for all i.
  /// @note This function only works when all coefficients in this polynomial
  /// are constant. @throw error otherwise.
  Eigen::VectorXd EvaluateIndeterminates(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& indeterminates,
      const Eigen::Ref<const Eigen::MatrixXd>& indeterminates_values) const;

  /// Adds @p coeff * @p m to this polynomial.
  Polynomial& AddProduct(const Expression& coeff, const Monomial& m);

  /// Removes the terms whose absolute value of the coefficients are smaller
  /// than or equal to @p coefficient_tol
  /// For example, if the polynomial is 2x² + 3xy + 10⁻⁴x - 10⁻⁵,
  /// then after calling RemoveTermsWithSmallCoefficients(1e-3), the returned
  /// polynomial becomes 2x² + 3xy.
  /// @param coefficient_tol A positive scalar.
  /// @retval polynomial_cleaned A polynomial whose terms with small
  /// coefficients are removed.
  Polynomial RemoveTermsWithSmallCoefficients(double coefficient_tol) const;

  /// Returns true if the polynomial is even, namely p(x) = p(-x). Meaning that
  /// the coefficient for all odd-degree monomials are 0.
  /// Returns false otherwise.
  /// Note that this is different from the p.TotalDegree() being an even number.
  bool IsEven() const;

  /// Returns true if the polynomial is odd, namely p(x) = -p(-x). Meaning that
  /// the coefficient for all even-degree monomials are 0.
  /// Returns false otherwise.
  /// Note that this is different from the p.TotalDegree() being an odd number.
  bool IsOdd() const;

  Polynomial& operator+=(const Polynomial& p);
  Polynomial& operator+=(const Monomial& m);
  Polynomial& operator+=(double c);
  Polynomial& operator+=(const Variable& v);

  Polynomial& operator-=(const Polynomial& p);
  Polynomial& operator-=(const Monomial& m);
  Polynomial& operator-=(double c);
  Polynomial& operator-=(const Variable& v);

  Polynomial& operator*=(const Polynomial& p);
  Polynomial& operator*=(const Monomial& m);
  Polynomial& operator*=(double c);
  Polynomial& operator*=(const Variable& v);

  /// Returns true if this polynomial and @p p are structurally equal.
  bool EqualTo(const Polynomial& p) const;

  /// Returns true if this polynomial and @p p are equal, after expanding the
  /// coefficients.
  bool EqualToAfterExpansion(const Polynomial& p) const;

  /// Returns true if this polynomial and @p p are almost equal (the difference
  /// in the corresponding coefficients are all less than @p tolerance), after
  /// expanding the coefficients.
  bool CoefficientsAlmostEqual(const Polynomial& p, double tolerance) const;

  /// Returns a symbolic formula representing the condition where this
  /// polynomial and @p p are the same.
  Formula operator==(const Polynomial& p) const;

  /// Returns a symbolic formula representing the condition where this
  /// polynomial and @p p are not the same.
  Formula operator!=(const Polynomial& p) const;

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const Polynomial& item) noexcept {
    using drake::hash_append;
    for (const auto& p : item.monomial_to_coefficient_map_) {
      hash_append(hasher, p.first);
      hash_append(hasher, p.second);
    }
  }
  friend Polynomial operator/(Polynomial p, double v);

 private:
  // Throws std::exception if there is a variable appeared in both of
  // decision_variables() and indeterminates().
  void CheckInvariant() const;

  MapType monomial_to_coefficient_map_;
  Variables indeterminates_;
  Variables decision_variables_;
};

/// Unary minus operation for polynomial.
Polynomial operator-(const Polynomial& p);

Polynomial operator+(Polynomial p1, const Polynomial& p2);
Polynomial operator+(Polynomial p, const Monomial& m);
Polynomial operator+(Polynomial p, double c);
Polynomial operator+(const Monomial& m, Polynomial p);
Polynomial operator+(const Monomial& m1, const Monomial& m2);
Polynomial operator+(const Monomial& m, double c);
Polynomial operator+(double c, Polynomial p);
Polynomial operator+(double c, const Monomial& m);
Polynomial operator+(Polynomial p, const Variable& v);
Polynomial operator+(const Variable& v, Polynomial p);

Polynomial operator-(Polynomial p1, const Polynomial& p2);
Polynomial operator-(Polynomial p, const Monomial& m);
Polynomial operator-(Polynomial p, double c);
Polynomial operator-(const Monomial& m, Polynomial p);
Polynomial operator-(const Monomial& m1, const Monomial& m2);
Polynomial operator-(const Monomial& m, double c);
Polynomial operator-(double c, Polynomial p);
Polynomial operator-(double c, const Monomial& m);
Polynomial operator-(Polynomial p, const Variable& v);
Polynomial operator-(const Variable& v, const Polynomial& p);

Polynomial operator*(Polynomial p1, const Polynomial& p2);
Polynomial operator*(Polynomial p, const Monomial& m);
Polynomial operator*(Polynomial p, double c);
Polynomial operator*(const Monomial& m, Polynomial p);
// Note that `Monomial * Monomial -> Monomial` is provided in
// symbolic_monomial.h file.
Polynomial operator*(const Monomial& m, double c);
Polynomial operator*(double c, Polynomial p);
Polynomial operator*(double c, const Monomial& m);
Polynomial operator*(Polynomial p, const Variable& v);
Polynomial operator*(const Variable& v, Polynomial p);

/// Returns `p / v`.
Polynomial operator/(Polynomial p, double v);

/// Returns polynomial @p rasied to @p n.
Polynomial pow(const Polynomial& p, int n);

std::ostream& operator<<(std::ostream& os, const Polynomial& p);

/// Provides the following seven operations:
///
/// - Matrix<Polynomial> * Matrix<Monomial> => Matrix<Polynomial>
/// - Matrix<Polynomial> * Matrix<double> => Matrix<Polynomial>
/// - Matrix<Monomial> * Matrix<Polynomial> => Matrix<Polynomial>
/// - Matrix<Monomial> * Matrix<Monomial> => Matrix<Polynomial>
/// - Matrix<Monomial> * Matrix<double> => Matrix<Polynomial>
/// - Matrix<double> * Matrix<Polynomial> => Matrix<Polynomial>
/// - Matrix<double> * Matrix<Monomial> => Matrix<Polynomial>
///
/// @note that these operator overloadings are necessary even after providing
/// Eigen::ScalarBinaryOpTraits. See
/// https://stackoverflow.com/questions/41494288/mixing-scalar-types-in-eigen
/// for more information.
#if defined(DRAKE_DOXYGEN_CXX)
template <typename MatrixL, typename MatrixR>
Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
              MatrixR::ColsAtCompileTime>
operator*(const MatrixL& lhs, const MatrixR& rhs);
#else
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        // {Polynomial, Monomial, double} x {Polynomial, Monomial, double}
        (std::is_same_v<typename MatrixL::Scalar, Polynomial> ||
         std::is_same_v<typename MatrixL::Scalar, Monomial> ||
         std::is_same_v<typename MatrixL::Scalar, double>) &&
        (std::is_same_v<typename MatrixR::Scalar, Polynomial> ||
         std::is_same_v<typename MatrixR::Scalar, Monomial> ||
         std::is_same_v<typename MatrixR::Scalar, double>) &&
        // Exclude Polynomial x Polynomial case (because the other seven
        // operations call this case. If we include this case here, we will have
        // self-recursion).
        !(std::is_same_v<typename MatrixL::Scalar, Polynomial> &&
          std::is_same_v<typename MatrixR::Scalar, Polynomial>) &&
        // Exclude double x double case.
        !(std::is_same_v<typename MatrixL::Scalar, double> &&
          std::is_same_v<typename MatrixR::Scalar, double>),
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  // "foo.template cast<Polynomial>()" is redundant if foo is of Polynomial.
  // However, we have checked that `-O2` compiler optimization reduces it into a
  // no-op.
  return lhs.template cast<Polynomial>() * rhs.template cast<Polynomial>();
}
#endif

}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Polynomial>. */
template <>
struct hash<drake::symbolic::Polynomial> : public drake::DefaultHash {};
#if defined(__GLIBCXX__)
// Inform GCC that this hash function is not so fast (i.e. for-loop inside).
// This will enforce caching of hash results. See
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/unordered_associative.html
// for details.
template <>
struct __is_fast_hash<hash<drake::symbolic::Polynomial>> : std::false_type {};
#endif
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {

// Defines Eigen traits needed for Matrix<drake::symbolic::Polynomial>.
template <>
struct NumTraits<drake::symbolic::Polynomial>
    : GenericNumTraits<drake::symbolic::Polynomial> {
  static inline int digits10() { return 0; }
};

// Informs Eigen that BinaryOp(LhsType, RhsType) gets ResultType.
#define DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(LhsType, RhsType, BinaryOp,    \
                                               ResultType)                    \
  template <>                                                                 \
  struct ScalarBinaryOpTraits<LhsType, RhsType, BinaryOp<LhsType, RhsType>> { \
    enum { Defined = 1 };                                                     \
    typedef ResultType ReturnType;                                            \
  };

// Informs Eigen that LhsType op RhsType gets ResultType
// where op ∈ {+, -, *, conj_product}.
#define DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(           \
    LhsType, RhsType, ResultType)                                             \
  DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(LhsType, RhsType,                    \
                                         internal::scalar_sum_op, ResultType) \
  DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(                                     \
      LhsType, RhsType, internal::scalar_difference_op, ResultType)           \
  DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(                                     \
      LhsType, RhsType, internal::scalar_product_op, ResultType)              \
  DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(                                     \
      LhsType, RhsType, internal::scalar_conj_product_op, ResultType)

// Informs Eigen that Polynomial op Monomial gets Polynomial
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Polynomial, drake::symbolic::Monomial,
    drake::symbolic::Polynomial)

// Informs Eigen that Polynomial op double gets Polynomial
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Polynomial, double, drake::symbolic::Polynomial)

// Informs Eigen that Monomial op Polynomial gets Polynomial
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Monomial, drake::symbolic::Polynomial,
    drake::symbolic::Polynomial)

// Informs Eigen that Monomial op Monomial gets Polynomial
// where op ∈ {+, -, *, conj_product}.
//
// Note that we inform Eigen that the return type of Monomial op Monomial is
// Polynomial, not Monomial, while Monomial * Monomial gets a Monomial in our
// implementation. This discrepency is due to the implementation of Eigen's
// dot() method whose return type is scalar_product_op::ReturnType. For more
// information, check line 67 of Eigen/src/Core/Dot.h and line 767 of
// Eigen/src/Core/util/XprHelper.h.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Monomial, drake::symbolic::Monomial,
    drake::symbolic::Polynomial)

// Informs Eigen that Monomial op double gets Polynomial
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Monomial, double, drake::symbolic::Polynomial)

// Informs Eigen that double op Polynomial gets Polynomial
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    double, drake::symbolic::Polynomial, drake::symbolic::Polynomial)

// Informs Eigen that double op Monomial gets Polynomial
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    double, drake::symbolic::Monomial, drake::symbolic::Polynomial)

#undef DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS
#undef DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS

namespace internal {
// Informs Eigen how to cast drake::symbolic::Polynomial to
// drake::symbolic::Expression.
template <>
EIGEN_DEVICE_FUNC inline drake::symbolic::Expression cast(
    const drake::symbolic::Polynomial& p) {
  return p.ToExpression();
}
}  // namespace internal
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)

namespace drake {
namespace symbolic {
/// Evaluates a matrix `m` of symbolic polynomials using `env`.
///
/// @returns a matrix of double whose size is the size of @p m.
/// @throws std::exception if NaN is detected during evaluation.
/// @pydrake_mkdoc_identifier{polynomial}
template <typename Derived>
std::enable_if_t<
    std::is_same_v<typename Derived::Scalar, Polynomial>,
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime, 0, Derived::MaxRowsAtCompileTime,
                  Derived::MaxColsAtCompileTime>>
Evaluate(const Eigen::MatrixBase<Derived>& m, const Environment& env) {
  return m.unaryExpr([&env](const Polynomial& p) { return p.Evaluate(env); });
}

/// Computes the Jacobian matrix J of the vector function @p f with respect to
/// @p vars. J(i,j) contains ∂f(i)/∂vars(j).
///
/// @pre {@p vars is non-empty}.
/// @pydrake_mkdoc_identifier{polynomial}
MatrixX<Polynomial> Jacobian(const Eigen::Ref<const VectorX<Polynomial>>& f,
                             const Eigen::Ref<const VectorX<Variable>>& vars);

}  // namespace symbolic
}  // namespace drake
