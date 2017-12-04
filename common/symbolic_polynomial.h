#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <ostream>
#include <unordered_map>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

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
  //
  // Note that this implicit conversion is desirable to have a dot product of
  // two Eigen::Vector<Monomial>s return a Polynomial.
  // NOLINTNEXTLINE(runtime/explicit)
  Polynomial(const Monomial& m);

  /// Constructs a polynomial from an expression @p e. Note that all variables
  /// in `e` are considered as indeterminates.
  ///
  /// @throws std::runtime_error if @p e is not a polynomial.
  explicit Polynomial(const Expression& e);

  /// Constructs a polynomial from an expression @p e by decomposing it with
  /// respect to @p indeterminates.
  ///
  /// @throws std::runtime_error if @p e is not a polynomial in @p
  /// indeterminates.
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

  /** Differentiates this polynomial with respect to the variable @p x. Note
   * that a variable @p x can be either a decision variable or an indeterminate.
   */
  Polynomial Differentiate(const Variable& x) const;

  /// Computes the Jacobian matrix J of the polynomial with respect to
  /// @p vars. J(0,i) contains ∂f/∂vars(i).
  template <typename Derived>
  Eigen::Matrix<Polynomial, 1, Derived::RowsAtCompileTime> Jacobian(
      const Eigen::MatrixBase<Derived>& vars) const {
    static_assert(std::is_same<typename Derived::Scalar, Variable>::value &&
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

  /// Adds @p coeff * @p m to this polynomial.
  Polynomial& AddProduct(const Expression& coeff, const Monomial& m);

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
  // Throws std::runtime_error if there is a variable appeared in both of
  // decision_variables() and indeterminates().
  void CheckInvariant() const;
  MapType monomial_to_coefficient_map_;
};

/// Unary minus operation for polynomial.
Polynomial operator-(Polynomial p);

Polynomial operator+(Polynomial p1, const Polynomial& p2);
Polynomial operator+(Polynomial p, const Monomial& m);
Polynomial operator+(Polynomial p, double c);
Polynomial operator+(const Monomial& m, Polynomial p);
Polynomial operator+(const Monomial& m1, const Monomial& m2);
Polynomial operator+(const Monomial& m, double c);
Polynomial operator+(double c, Polynomial p);
Polynomial operator+(double c, const Monomial& m);

Polynomial operator-(Polynomial p1, const Polynomial& p2);
Polynomial operator-(Polynomial p, const Monomial& m);
Polynomial operator-(Polynomial p, double c);
Polynomial operator-(const Monomial& m, Polynomial p);
Polynomial operator-(const Monomial& m1, const Monomial& m2);
Polynomial operator-(const Monomial& m, double c);
Polynomial operator-(double c, Polynomial p);
Polynomial operator-(double c, const Monomial& m);

Polynomial operator*(Polynomial p1, const Polynomial& p2);
Polynomial operator*(Polynomial p, const Monomial& m);
Polynomial operator*(Polynomial p, double c);
Polynomial operator*(const Monomial& m, Polynomial p);
// Note that `Monomial * Monomial -> Monomial` is provided in
// symbolic_monomial.h file.
Polynomial operator*(const Monomial& m, double c);
Polynomial operator*(double c, Polynomial p);
Polynomial operator*(double c, const Monomial& m);

/// Returns polynomial @p rasied to @p n.
Polynomial pow(const Polynomial& p, int n);

std::ostream& operator<<(std::ostream& os, const Polynomial& p);

/// Provides the following seven operations:
///  - Matrix<Polynomial> * Matrix<Monomial> => Matrix<Polynomial>
///  - Matrix<Polynomial> * Matrix<double> => Matrix<Polynomial>
///  - Matrix<Monomial> * Matrix<Polynomial> => Matrix<Polynomial>
///  - Matrix<Monomial> * Matrix<Monomial> => Matrix<Polynomial>
///  - Matrix<Monomial> * Matrix<double> => Matrix<Polynomial>
///  - Matrix<double> * Matrix<Polynomial> => Matrix<Polynomial>
///  - Matrix<double> * Matrix<Monomial> => Matrix<Polynomial>
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
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        // {Polynomial, Monomial, double} x {Polynomial, Monomial, double}
        (std::is_same<typename MatrixL::Scalar, Polynomial>::value ||
         std::is_same<typename MatrixL::Scalar, Monomial>::value ||
         std::is_same<typename MatrixL::Scalar, double>::value) &&
        (std::is_same<typename MatrixR::Scalar, Polynomial>::value ||
         std::is_same<typename MatrixR::Scalar, Monomial>::value ||
         std::is_same<typename MatrixR::Scalar, double>::value) &&
        // Exclude Polynomial x Polynomial case (because the other seven
        // operations call this case. If we include this case here, we will have
        // self-recursion).
        !(std::is_same<typename MatrixL::Scalar, Polynomial>::value &&
          std::is_same<typename MatrixR::Scalar, Polynomial>::value) &&
        // Exclude double x double case.
        !(std::is_same<typename MatrixL::Scalar, double>::value &&
          std::is_same<typename MatrixR::Scalar, double>::value),
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  // "foo.template cast<Polynomial>()" is redundant if foo is of Polynomial.
  // However, we have checked that `-O2` compiler optimization reduces it into a
  // no-op.
  return lhs.template cast<Polynomial>() * rhs.template cast<Polynomial>();
}
#endif
}  // namespace symbolic
}  // namespace drake

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
