#pragma once

#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"
#include "drake/common/symbolic/expression.h"
#define DRAKE_COMMON_SYMBOLIC_POLYNOMIAL_H
#include "drake/common/symbolic/monomial.h"
#undef DRAKE_COMMON_SYMBOLIC_POLYNOMIAL_H

// Remove with deprecation 2026-05-01.
#include <ostream>

namespace drake {
namespace symbolic {
namespace internal {
/* Compares two monomials using the lexicographic order. It is used in
symbolic::Polynomial::MapType. See https://en.wikipedia.org/wiki/Monomial_order
for different monomial orders. */
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

/** Represents symbolic polynomials. A symbolic polynomial keeps a mapping from
a monomial of indeterminates to its coefficient in a symbolic expression.

A polynomial `p` has to satisfy an invariant such that `p.decision_variables() ∩
p.indeterminates() = ∅`. We have CheckInvariant() method to check the invariant.

@anchor polynomial_variable_operation
<h3> Operation between a %Polynomial and a %Variable </h3>

Note that for arithmetic operations ⊕ (where ⊕ can be +,-,*) between a
Polynomial `p` and a Variable `v`, if the variable `v` is an indeterminate of
the polynomial `p`, then we regard `v` as a monomial with indeterminate `v` with
coefficient 1; on the other hand, if the variable `v` is not an indeterminate of
`p`, then we regard `v` as a monomial with 0 degree and coefficient `v`, and `v`
will be appended to decision_variables() of the resulted polynomial. For
example, if p = ax²+y where (x,y) are indeterminates and a is a decision
variable, then in the operation p + y, the result stores the
monomial-to-coefficient mapping as {(x² -> a), (y -> 2)}; on the other hand, in
the operation p + b, b is regarded as a 0-degree monomial with coefficient b,
and p + b stores the monomial-to-coefficient mapping as {(x² -> a), (y -> 1), (1
-> b)}, with (p + b).decision_variables() being {a, b}.  If you want to append
the variable `v` to the indeterminates of the p⊕v, then explicitly convert it to
a monomial as p ⊕ symbolic::Monomial(v).

<!-- TODO(hongkai.dai) when symbolic::GenericPolynomial is ready, we will
deprecate symbolic::Polynomial class, and create an alias using
symbolic::Polynomial=symbolic::GenericPolynomial<MonomialBasisElement>; We will
copy the unit tests in symbolic_polynomial_test.cc to
symbolic_generic_polynomial_test.cc --> */
class Polynomial {
 public:
  using MapType = std::map<Monomial, Expression, internal::CompareMonomial>;

  /** Constructs a zero polynomial. */
  Polynomial() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Polynomial);

  /** Constructs a default value.  This overload is used by Eigen when
  EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled. */
  explicit Polynomial(std::nullptr_t) : Polynomial() {}

  /** Constructs a polynomial from a map, Monomial → Expression. */
  explicit Polynomial(MapType map);

  /** Constructs a polynomial from a monomial `m`. Note that all variables in
  `m` are considered as indeterminates. Note that this implicit conversion is
  desirable to have a dot product of two Eigen::Vector<Monomial>s return a
  Polynomial. */
  // NOLINTNEXTLINE(runtime/explicit)
  Polynomial(const Monomial& m);

  /** Constructs a polynomial from a varaible `v`. Note that v is considered an
  indeterminate. */
  explicit Polynomial(const Variable& v);

  /** Constructs a polynomial from an expression `e`. Note that all variables
  in `e` are considered as indeterminates.
  @throws std::exception if `e` is not a polynomial. */
  explicit Polynomial(const Expression& e);

  /** Constructs a polynomial from an expression `e` by decomposing it with
  respect to `indeterminates`.
  @note It collects the intersection of the variables appeared in `e` and the
  provided `indeterminates`.

  @throws std::exception if `e` is not a polynomial in `indeterminates`. */
  Polynomial(const Expression& e, Variables indeterminates);

  /** Returns the indeterminates of this polynomial. */
  [[nodiscard]] const Variables& indeterminates() const;

  /** Returns the decision variables of this polynomial. */
  [[nodiscard]] const Variables& decision_variables() const;

  /** Sets the indeterminates to `new_indeterminates`.
  Changing the indeterminates would change `monomial_to_coefficient_map()`, and
  also potentially the degree of the polynomial. Here is an example.
  @code
  // p is a quadratic polynomial with x being the indeterminates.
  symbolic::Polynomial p(a * x * x + b * x + c, {x});
  // p.monomial_to_coefficient_map() contains {1: c, x: b, x*x:a}.
  std::cout << p.TotalDegree(); // prints 2.
  // Now set (a, b, c) to the indeterminates. p becomes a linear
  // polynomial of a, b, c.
  p.SetIndeterminates({a, b, c});
  // p.monomial_to_coefficient_map() now is {a: x * x, b: x, c: 1}.
  std::cout << p.TotalDegree(); // prints 1.
  @endcode */
  void SetIndeterminates(const Variables& new_indeterminates);

  /** Returns the highest degree of this polynomial in a variable `v`. */
  [[nodiscard]] int Degree(const Variable& v) const;

  /** Returns the total degree of this polynomial. */
  [[nodiscard]] int TotalDegree() const;

  /** Returns the mapping from a Monomial to its corresponding coefficient of
  this polynomial.  We maintain the invariance that for any [monomial, coeff]
  pair in monomial_to_coefficient_map(), symbolic:is_zero(coeff) is false. */
  [[nodiscard]] const MapType& monomial_to_coefficient_map() const;

  /** Returns an equivalent symbolic expression of this polynomial. */
  [[nodiscard]] Expression ToExpression() const;

  /** Differentiates this polynomial with respect to the variable `x`. Note that
  a variable `x` can be either a decision variable or an indeterminate. */
  [[nodiscard]] Polynomial Differentiate(const Variable& x) const;

  /** Computes the Jacobian matrix J of the polynomial with respect to
  `vars`. J(0,i) contains ∂f/∂vars(i). */
  template <typename Derived>
  [[nodiscard]] Eigen::Matrix<Polynomial, 1, Derived::RowsAtCompileTime>
  Jacobian(const Eigen::MatrixBase<Derived>& vars) const {
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

  /** Integrates this polynomial with respect to an indeterminate `x`.
  Integration with respect to decision variables is not supported yet. If `x` is
  not an indeterminate nor decision variable, then it will be added to the list
  of indeterminates.
  @throws std::exception if `x` is a decision variable. */
  [[nodiscard]] Polynomial Integrate(const Variable& x) const;

  /** Computes the definite integrate of this polynomial with respect to the
  indeterminate `x` over the domain [a, b]. Integration with respect to decision
  variables is not supported yet.
  @throws std::exception if `x` is a decision variable. */
  [[nodiscard]] Polynomial Integrate(const Variable& x, double a,
                                     double b) const;

  /** Evaluates this polynomial under a given environment `env`.
  @throws std::exception if there is a variable in this polynomial whose
  assignment is not provided by `env`. */
  [[nodiscard]] double Evaluate(const Environment& env) const;

  /** Partially evaluates this polynomial using an environment `env`.
  @throws std::exception if NaN is detected during evaluation. */
  [[nodiscard]] Polynomial EvaluatePartial(const Environment& env) const;

  /** Partially evaluates this polynomial by substituting `var` with `c`.
  @throws std::exception if NaN is detected at any point during evaluation. */
  [[nodiscard]] Polynomial EvaluatePartial(const Variable& var, double c) const;

  /** Evaluates the polynomial at a batch of indeterminates values.
  @param[in] indeterminates Must include all `this->indeterminates()`
  @param[in] indeterminates_values Each column of `indeterminates_values` stores
  one specific value of `indeterminates`; `indeterminates_values.rows() ==
  indeterminates.rows()`.
  @return polynomial_values polynomial_values(j) is obtained by substituting
  indeterminates(i) in this polynomial with indeterminates_values(i, j) for all
  i.
  @throw std::exception if any coefficient in this polynomial is not a
  constant. */
  [[nodiscard]] Eigen::VectorXd EvaluateIndeterminates(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& indeterminates,
      const Eigen::Ref<const Eigen::MatrixXd>& indeterminates_values) const;

  /** Evaluates the polynomial at a batch of indeterminate values.
  For a polynomial whose coefficients are affine expressions of decision
  variables, we evaluate this polynomial on a batch of indeterminate
  values, and return the matrix representation of the evaluated affine
  expressions. For example if p(x) = (a+1)x² + b*x where a, b are decision
  variables, if we evaluate this polynomial on x = 1 and x = 2, then p(x) =
  a+b+1 and 4a+2b+4 respectively. We return the evaluation result as A *
  decision_variables + b, where A.row(i) * decision_variables + b(i) is the
  evaluation of the polynomial on indeterminates_values.col(i).
  @param[in] indeterminates Must include all this->indeterminates()
  @param[in] indeterminates_values A matrix representing a batch of values.
  Each column of `indeterminates_values` stores one specific value of
  `indeterminates`, where `indeterminates_values.rows() ==
  indeterminates.rows()`.
  @param[out] A The coefficient of the evaluation results.
  @param[out] decision_variables The decision variables in the evaluation
  results.
  @param[out] b The constant terms in the evaluation results.
  @throw std::exception if the coefficients of this polynomial is not an
  affine expression of its decision variables. For example, the polynomial
  (2+sin(a)) * x² + 1 (where `a` is a decision variable and `x` is a
  indeterminate) doesn't have affine expression as its coefficient 2+sin(a). */
  void EvaluateWithAffineCoefficients(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& indeterminates,
      const Eigen::Ref<const Eigen::MatrixXd>& indeterminates_values,
      Eigen::MatrixXd* A, VectorX<symbolic::Variable>* decision_variables,
      Eigen::VectorXd* b) const;

  /** Adds `coeff` * `m` to this polynomial. */
  Polynomial& AddProduct(const Expression& coeff, const Monomial& m);

  /** An encapsulated data type for use with the method SubstituteAndExpand. */
  struct SubstituteAndExpandCacheData {
   public:
    std::map<Monomial, Polynomial, internal::CompareMonomial>* get_data() {
      return &data_;
    }

   private:
    friend class Polynomial;
    std::map<Monomial, Polynomial, internal::CompareMonomial> data_;
  };

  /** Substitutes the monomials of this polynomial with new polynomial
  expressions and expand the polynomial to the monomial basis. For example,
  consider the substitution x = a(1-y) into the polynomial x¹⁴ +
  (x−1)². Repeatedly expanding the powers of x can take a long time using
  factory methods, so we store intermediate computations in the substitution map
  to avoid recomputing very high powers.

  @param indeterminate_substitution The substitutions of every indeterminate
  with the new desired expression. This map must contain each element of
  `indeterminates()`. For performance reasons, it is recommended that this map
  contains Expanded polynomials as its values, but this is not necessary.

  @param[in,out] substitutions_cached_data A container caching the higher order
  expansions of the `indeterminate_substitutions`. Typically, the first time an
  indeterminate_substitution is performed, this will be empty.  If the same
  indeterminate_substitutions is used for multiple polynomials, passing this
  value will enable the user to re-use the expansions across multiple calls. For
  example, suppose we wish to perform the substitution x = a(1-y) into the
  polynomials p1 = x¹⁴ + (x−1)² and p2 = x⁷. A user may call
  p1.SubstituteAndExpand({x : a(1-y), substitutions_cached_data}) where
  substitutions_cached_data is a pointer to an empty container. As part of
  computing the expansion of p1, the expansion of x⁷ may get computed and stored
  in substitutions_cached_data, and so a subsequent call of
  p2.SubstituteAndExpand({x : a(1-y)}, substitutions_cached_data) would be very
  fast.

  Never reuse substitutions_cached_data if indeterminate_substitutions changes
  as this function will then compute an incorrect result.

  Note that this function is NOT responsible for ensuring that
  `substitutions_cached_data` is consistent i.e. this method will not throw an
  error if substitutions_cached_data contains the inconsistent substitutions
  {x: y, x²: 2y}. To ensure correct results, ensure that the passed
  substitutions_cached_data object is consistent with
  indeterminate_substitutions. The easiest way to do this is to pass a pointer
  to an empty substitutions_cached_data or nullopt to this function. */
  [[nodiscard]] Polynomial SubstituteAndExpand(
      const std::unordered_map<Variable, Polynomial>&
          indeterminate_substitution,
      SubstituteAndExpandCacheData* substitutions_cached_data = nullptr) const;

  /** Expands each coefficient expression and returns the expanded polynomial.
  If any coefficient is equal to 0 after expansion, then remove that term from
  the returned polynomial. */
  [[nodiscard]] Polynomial Expand() const;

  /** Removes the terms whose absolute value of the coefficients are smaller
  than or equal to `coefficient_tol`. For example, if the polynomial is 2x² +
  3xy + 10⁻⁴x - 10⁻⁵, then after calling RemoveTermsWithSmallCoefficients(1e-3),
  the returned polynomial becomes 2x² + 3xy.
  @param coefficient_tol A positive scalar.
  @retval polynomial_cleaned A polynomial whose terms with small coefficients
  are removed. */
  [[nodiscard]] Polynomial RemoveTermsWithSmallCoefficients(
      double coefficient_tol) const;

  /** Returns true if the polynomial is even, namely p(x) = p(-x). Meaning that
  the coefficient for all odd-degree monomials are 0. Returns false otherwise.
  Note that this is different from the p.TotalDegree() being an even number. */
  [[nodiscard]] bool IsEven() const;

  /** Returns true if the polynomial is odd, namely p(x) = -p(-x). Meaning that
  the coefficient for all even-degree monomials are 0. Returns false otherwise.
  Note that this is different from the p.TotalDegree() being an odd number. */
  [[nodiscard]] bool IsOdd() const;

  /** Returns the roots of a _univariate_ polynomial with constant coefficients
  as a column vector. There is no specific guarantee on the order of the
  returned roots.
  @throws std::exception if `this` is not univariate with constant
  coefficients. */
  [[nodiscard]] Eigen::VectorXcd Roots() const;

  Polynomial& operator+=(const Polynomial& p);
  Polynomial& operator+=(const Monomial& m);
  Polynomial& operator+=(double c);
  /** Depending on whether `v` is an indeterminate of this polynomial, this
  operation generates different results. Refer to @ref
  polynomial_variable_operation "the class documentation" for more details. */
  Polynomial& operator+=(const Variable& v);

  Polynomial& operator-=(const Polynomial& p);
  Polynomial& operator-=(const Monomial& m);
  Polynomial& operator-=(double c);
  /** Depending on whether `v` is an indeterminate of this polynomial, this
  operation generates different results. Refer to @ref
  polynomial_variable_operation "the class documentation" for more details. */
  Polynomial& operator-=(const Variable& v);

  Polynomial& operator*=(const Polynomial& p);
  Polynomial& operator*=(const Monomial& m);
  Polynomial& operator*=(double c);
  /** Depending on whether `v` is an indeterminate of this polynomial, this
  operation generates different results. Refer to @ref
  polynomial_variable_operation "the class documentation" for more details. */
  Polynomial& operator*=(const Variable& v);

  /** Returns true if this polynomial and `p` are structurally equal. */
  [[nodiscard]] bool EqualTo(const Polynomial& p) const;

  /** Returns true if this polynomial and `p` are almost equal (the difference
  in the corresponding coefficients are all less than `tolerance`), after
  expanding the coefficients. */
  [[nodiscard]] bool CoefficientsAlmostEqual(const Polynomial& p,
                                             double tolerance) const;

  /** Returns a symbolic formula representing the condition where this
  polynomial and `p` are the same. */
  [[nodiscard]] Formula operator==(const Polynomial& p) const;

  /** Returns a symbolic formula representing the condition where this
  polynomial and `p` are not the same. */
  [[nodiscard]] Formula operator!=(const Polynomial& p) const;

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  // NOLINTNEXTLINE(runtime/references) Per hash_append convention.
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
  // Throws std::exception if any of the condition is true.
  // 1. There is a variable appeared in both of decision_variables() and
  // indeterminates().
  // 2. There is a [monomial, coeff] pair in monomial_to_coefficient_map_ that
  // symbolic::is_zero(coeff) is true.
  void CheckInvariant() const;

  MapType monomial_to_coefficient_map_;
  Variables indeterminates_;
  Variables decision_variables_;
};

/** Unary minus operation for polynomial. */
[[nodiscard]] Polynomial operator-(const Polynomial& p);

[[nodiscard]] Polynomial operator+(Polynomial p1, const Polynomial& p2);
[[nodiscard]] Polynomial operator+(Polynomial p, const Monomial& m);
[[nodiscard]] Polynomial operator+(Polynomial p, double c);
[[nodiscard]] Polynomial operator+(const Monomial& m, Polynomial p);
[[nodiscard]] Polynomial operator+(const Monomial& m1, const Monomial& m2);
[[nodiscard]] Polynomial operator+(const Monomial& m, double c);
[[nodiscard]] Polynomial operator+(double c, Polynomial p);
[[nodiscard]] Polynomial operator+(double c, const Monomial& m);
[[nodiscard]] Polynomial operator+(Polynomial p, const Variable& v);
[[nodiscard]] Polynomial operator+(const Variable& v, Polynomial p);
[[nodiscard]] Expression operator+(const Expression& e, const Polynomial& p);
[[nodiscard]] Expression operator+(const Polynomial& p, const Expression& e);

[[nodiscard]] Polynomial operator-(Polynomial p1, const Polynomial& p2);
[[nodiscard]] Polynomial operator-(Polynomial p, const Monomial& m);
[[nodiscard]] Polynomial operator-(Polynomial p, double c);
[[nodiscard]] Polynomial operator-(const Monomial& m, Polynomial p);
[[nodiscard]] Polynomial operator-(const Monomial& m1, const Monomial& m2);
[[nodiscard]] Polynomial operator-(const Monomial& m, double c);
[[nodiscard]] Polynomial operator-(double c, Polynomial p);
[[nodiscard]] Polynomial operator-(double c, const Monomial& m);
[[nodiscard]] Polynomial operator-(Polynomial p, const Variable& v);
[[nodiscard]] Polynomial operator-(const Variable& v, const Polynomial& p);
[[nodiscard]] Expression operator-(const Expression& e, const Polynomial& p);
[[nodiscard]] Expression operator-(const Polynomial& p, const Expression& e);

[[nodiscard]] Polynomial operator*(Polynomial p1, const Polynomial& p2);
[[nodiscard]] Polynomial operator*(Polynomial p, const Monomial& m);
[[nodiscard]] Polynomial operator*(Polynomial p, double c);
[[nodiscard]] Polynomial operator*(const Monomial& m, Polynomial p);

// Note that `Monomial * Monomial -> Monomial` is provided in
// monomial.h file.
[[nodiscard]] Polynomial operator*(const Monomial& m, double c);
[[nodiscard]] Polynomial operator*(double c, Polynomial p);
[[nodiscard]] Polynomial operator*(double c, const Monomial& m);
[[nodiscard]] Polynomial operator*(Polynomial p, const Variable& v);
[[nodiscard]] Polynomial operator*(const Variable& v, Polynomial p);
[[nodiscard]] Expression operator*(const Expression& e, const Polynomial& p);
[[nodiscard]] Expression operator*(const Polynomial& p, const Expression& e);

/** Returns `p / v`. */
[[nodiscard]] Polynomial operator/(Polynomial p, double v);
[[nodiscard]] Expression operator/(double v, const Polynomial& p);
[[nodiscard]] Expression operator/(const Expression& e, const Polynomial& p);
[[nodiscard]] Expression operator/(const Polynomial& p, const Expression& e);

/** Returns polynomial `p` raised to `n`. */
[[nodiscard]] Polynomial pow(const Polynomial& p, int n);

std::string to_string(const Polynomial& p);

DRAKE_DEPRECATED(
    "2026-05-01",
    "Use fmt functions instead (e.g., fmt::format(), fmt::to_string(), "
    "fmt::print()). Refer to GitHub issue #17742 for more information.")
std::ostream& operator<<(std::ostream& os, const Polynomial& p);

/** Provides the following matrix operations:

- Matrix<Polynomial> * Matrix<Monomial> => Matrix<Polynomial>
- Matrix<Polynomial> * Matrix<Variable> => Matrix<Polynomial>
- Matrix<Polynomial> * Matrix<double> => Matrix<Polynomial>
- Matrix<Monomial> * Matrix<Polynomial> => Matrix<Polynomial>
- Matrix<Monomial> * Matrix<Monomial> => Matrix<Polynomial>
- Matrix<Monomial> * Matrix<Variable> => Matrix<Polynomial>
- Matrix<Monomial> * Matrix<double> => Matrix<Polynomial>
- Matrix<Variable> * Matrix<Polynomial> => Matrix<Polynomial>
- Matrix<Variable> * Matrix<Monomial> => Matrix<Polynomial>
- Matrix<double> * Matrix<Polynomial> => Matrix<Polynomial>
- Matrix<double> * Matrix<Monomial> => Matrix<Polynomial>

@note that these operator overloadings are necessary even after providing
Eigen::ScalarBinaryOpTraits. See
https://stackoverflow.com/questions/41494288/mixing-scalar-types-in-eigen
for more information. */
#if defined(DRAKE_DOXYGEN_CXX)
template <typename MatrixL, typename MatrixR>
Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
              MatrixR::ColsAtCompileTime>
operator*(const MatrixL& lhs, const MatrixR& rhs);
#else
// clang-format off
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
    std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
    // The LHS and RHS must be one of {Polynomial, Monomial, Variable, double}.
    (std::is_same_v<typename MatrixL::Scalar, Polynomial> ||
     std::is_same_v<typename MatrixL::Scalar, Monomial> ||
     std::is_same_v<typename MatrixL::Scalar, Variable> ||
     std::is_same_v<typename MatrixL::Scalar, double>) &&
    (std::is_same_v<typename MatrixR::Scalar, Polynomial> ||
     std::is_same_v<typename MatrixR::Scalar, Monomial> ||
     std::is_same_v<typename MatrixR::Scalar, Variable> ||
     std::is_same_v<typename MatrixR::Scalar, double>) &&
    // The LHS or RHS must be one of {Polynomial, Monomial}.
    (std::is_same_v<typename MatrixL::Scalar, Polynomial> ||
     std::is_same_v<typename MatrixL::Scalar, Monomial> ||
     std::is_same_v<typename MatrixR::Scalar, Polynomial> ||
     std::is_same_v<typename MatrixR::Scalar, Monomial>) &&
    // No specialization for Polynomial x Polynomial.
    !(std::is_same_v<typename MatrixL::Scalar, Polynomial> &&
      std::is_same_v<typename MatrixR::Scalar, Polynomial>),
Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
              MatrixR::ColsAtCompileTime>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Polynomial>() * rhs.template cast<Polynomial>();
}
// clang-format on
#endif

/** Provides the following matrix operations:
- Matrix<Expression> * Matrix<Polynomial> => Matrix<Expression>
- Matrix<Expression> * Matrix<Monomial> => Matrix<Expression>
- Matrix<Polynomial> * Matrix<Expression> => Matrix<Expression>
- Matrix<Monomial> * Matrix<Expression> => Matrix<Expression>
@note that these operator overloadings are necessary even after providing
Eigen::ScalarBinaryOpTraits. See
https://stackoverflow.com/questions/41494288/mixing-scalar-types-in-eigen
for more information. */
#if defined(DRAKE_DOXYGEN_CXX)
template <typename MatrixL, typename MatrixR>
Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
              MatrixR::ColsAtCompileTime>
operator*(const MatrixL& lhs, const MatrixR& rhs);
#else
// clang-format off
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
    std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
    // The LHS and RHS must be one of {Expression, Polynomial, Monomial}.
    (std::is_same_v<typename MatrixL::Scalar, Expression> ||
     std::is_same_v<typename MatrixL::Scalar, Polynomial> ||
     std::is_same_v<typename MatrixL::Scalar, Monomial>) &&
    (std::is_same_v<typename MatrixR::Scalar, Expression> ||
     std::is_same_v<typename MatrixR::Scalar, Polynomial> ||
     std::is_same_v<typename MatrixR::Scalar, Monomial>) &&
    // Exactly one of the two sides must be an Expresion.
    (std::is_same_v<typename MatrixL::Scalar, Expression> ^
     std::is_same_v<typename MatrixR::Scalar, Expression>),
Eigen::Matrix<Expression, MatrixL::RowsAtCompileTime,
              MatrixR::ColsAtCompileTime>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Expression>() * rhs.template cast<Expression>();
}
// clang-format on
#endif

}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Polynomial>. */
template <>
struct hash<drake::symbolic::Polynomial> : public drake::DefaultHash {};
#if defined(__GLIBCXX__)
/* Informs GCC that this hash function is not so fast (i.e. for-loop inside).
This will enforce caching of hash results. See
https://gcc.gnu.org/onlinedocs/libstdc++/manual/unordered_associative.html
for details. */
template <>
struct __is_fast_hash<hash<drake::symbolic::Polynomial>> : std::false_type {};
#endif
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {

/* Defines Eigen traits needed for Matrix<drake::symbolic::Polynomial>. */
template <>
struct NumTraits<drake::symbolic::Polynomial>
    : GenericNumTraits<drake::symbolic::Polynomial> {
  constexpr static int digits() { return 0; }
  constexpr static int digits10() { return 0; }
  constexpr static int max_digits10() { return 0; }
};

/* Informs Eigen that BinaryOp(LhsType, RhsType) gets ResultType. */
#define DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(LhsType, RhsType, BinaryOp,    \
                                               ResultType)                    \
  template <>                                                                 \
  struct ScalarBinaryOpTraits<LhsType, RhsType, BinaryOp<LhsType, RhsType>> { \
    enum { Defined = 1 };                                                     \
    typedef ResultType ReturnType;                                            \
  };

/* Informs Eigen that LhsType op RhsType gets ResultType where op ∈ {+, -, *,
conj_product}. */
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

/* Informs Eigen that Polynomial op Monomial gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Polynomial, drake::symbolic::Monomial,
    drake::symbolic::Polynomial)

/* Informs Eigen that Polynomial op Variable gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Polynomial, drake::symbolic::Variable,
    drake::symbolic::Polynomial)

/* Informs Eigen that Polynomial op double gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Polynomial, double, drake::symbolic::Polynomial)

/* Informs Eigen that Monomial op Polynomial gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Monomial, drake::symbolic::Polynomial,
    drake::symbolic::Polynomial)

/* Informs Eigen that Variable op Polynomial gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Variable, drake::symbolic::Polynomial,
    drake::symbolic::Polynomial)

/* Informs Eigen that Monomial op Monomial gets Polynomial
where op ∈ {+, -, *, conj_product}.

Note that we inform Eigen that the return type of Monomial op Monomial is
Polynomial, not Monomial, while Monomial * Monomial gets a Monomial in our
implementation. This discrepancy is due to the implementation of Eigen's dot()
method whose return type is scalar_product_op::ReturnType. For more information,
check line 67 of Eigen/src/Core/Dot.h and line 767 of
Eigen/src/Core/util/XprHelper.h. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Monomial, drake::symbolic::Monomial,
    drake::symbolic::Polynomial)

/* Informs Eigen that Variable op Monomial gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Variable, drake::symbolic::Monomial,
    drake::symbolic::Polynomial)

/* Informs Eigen that Monomial op Variable gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Monomial, drake::symbolic::Variable,
    drake::symbolic::Polynomial)

/* Informs Eigen that Monomial op double gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Monomial, double, drake::symbolic::Polynomial)

/* Informs Eigen that double op Polynomial gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    double, drake::symbolic::Polynomial, drake::symbolic::Polynomial)

/* Informs Eigen that double op Monomial gets Polynomial
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    double, drake::symbolic::Monomial, drake::symbolic::Polynomial)

/* Informs Eigen that Polynomial op Expression gets Expression
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Polynomial, drake::symbolic::Expression,
    drake::symbolic::Expression)

/* Informs Eigen that Expression op Polynomial gets Expression
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Expression, drake::symbolic::Polynomial,
    drake::symbolic::Expression)

/* Informs Eigen that Monomial op Expression gets Expression
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Monomial, drake::symbolic::Expression,
    drake::symbolic::Expression)

/* Informs Eigen that Expression op Monomial gets Expression
where op ∈ {+, -, *, conj_product}. */
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Expression, drake::symbolic::Monomial,
    drake::symbolic::Expression)

#undef DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS
#undef DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS

namespace internal {
/* Informs Eigen how to cast drake::symbolic::Polynomial to
drake::symbolic::Expression. */
template <>
EIGEN_DEVICE_FUNC inline drake::symbolic::Expression cast(
    const drake::symbolic::Polynomial& p) {
  return p.ToExpression();
}
}  // namespace internal
namespace numext {
template <>
bool equal_strict(const drake::symbolic::Polynomial& x,
                  const drake::symbolic::Polynomial& y);
template <>
EIGEN_STRONG_INLINE bool not_equal_strict(
    const drake::symbolic::Polynomial& x,
    const drake::symbolic::Polynomial& y) {
  return !Eigen::numext::equal_strict(x, y);
}
}  // namespace numext
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)

namespace drake {
namespace symbolic {
/** Evaluates a matrix `m` of symbolic polynomials using `env`.
@returns a matrix of double whose size is the size of `m`.
@throws std::exception if NaN is detected during evaluation.
@pydrake_mkdoc_identifier{polynomial} */
template <typename Derived>
[[nodiscard]] std::enable_if_t<
    std::is_same_v<typename Derived::Scalar, Polynomial>,
    MatrixLikewise<double, Derived>>
Evaluate(const Eigen::MatrixBase<Derived>& m, const Environment& env) {
  return m.unaryExpr([&env](const Polynomial& p) {
    return p.Evaluate(env);
  });
}

/** Computes the Jacobian matrix J of the vector function `f` with respect to
`vars`. J(i,j) contains ∂f(i)/∂vars(j).
@pre `vars` is non-empty.
@pydrake_mkdoc_identifier{polynomial} */
[[nodiscard]] MatrixX<Polynomial> Jacobian(
    const Eigen::Ref<const VectorX<Polynomial>>& f,
    const Eigen::Ref<const VectorX<Variable>>& vars);

/** Returns the polynomial m(x)ᵀ * Q * m(x), where m(x) is the monomial basis,
and Q is the Gram matrix.
@param monomial_basis m(x) in the documentation. A vector of monomials.
@param gram_lower The lower triangular entries in Q, stacked columnwise into a
vector.  */
template <typename Derived1, typename Derived2>
[[nodiscard]] typename std::enable_if<
    is_eigen_vector_of<Derived1, symbolic::Monomial>::value &&
        (is_eigen_vector_of<Derived2, double>::value ||
         is_eigen_vector_of<Derived2, symbolic::Variable>::value ||
         is_eigen_vector_of<Derived2, symbolic::Expression>::value),
    symbolic::Polynomial>::type
CalcPolynomialWLowerTriangularPart(
    const Eigen::MatrixBase<Derived1>& monomial_basis,
    const Eigen::MatrixBase<Derived2>& gram_lower) {
  DRAKE_DEMAND(monomial_basis.rows() * (monomial_basis.rows() + 1) / 2 ==
               gram_lower.rows());
  Polynomial::MapType monomial_coeff_map;
  for (int j = 0; j < monomial_basis.rows(); ++j) {
    for (int i = j; i < monomial_basis.rows(); ++i) {
      // Compute 2 * mᵢ(x) * Qᵢⱼ * mⱼ(x) if i != j, or mᵢ(x)²*Qᵢᵢ
      const auto monomial_product = monomial_basis(i) * monomial_basis(j);
      const auto& Qij =
          gram_lower(monomial_basis.rows() * j + i - (j + 1) * j / 2);
      const symbolic::Expression coeff = i == j ? Qij : 2 * Qij;
      auto it = monomial_coeff_map.find(monomial_product);
      if (it == monomial_coeff_map.end()) {
        monomial_coeff_map.emplace_hint(it, monomial_product, coeff);
      } else {
        it->second += coeff;
      }
    }
  }
  return Polynomial(monomial_coeff_map);
}
}  // namespace symbolic
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::symbolic, Polynomial, x,
                   drake::symbolic::to_string(x))
