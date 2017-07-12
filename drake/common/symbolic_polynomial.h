#pragma once

#include <ostream>
#include <unordered_map>

#include <Eigen/Core>

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
  //
  // Note that this implicit conversion is desirable to have a dot product of
  // two Eigen::Vector<Monomial>s return a Polynomial.
  // NOLINTNEXTLINE(runtime/explicit)
  Polynomial(const Monomial& m);

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
// Note that `Monomial * Monomial -> Monomial` is provided in Monomial.h file.
Polynomial operator*(const Monomial& m, double c);
Polynomial operator*(double c, Polynomial p);
Polynomial operator*(double c, const Monomial& m);

/// Returns polynomial @p rasied to @p n.
Polynomial pow(const Polynomial& p, int n);

std::ostream& operator<<(std::ostream& os, const Polynomial& p);

/// Provides Matrix<Polynomial> * Matrix<Monomial> => Matrix<Polynomial>.
/// Note that these operator overloadings are necessary even after providing
/// Eigen::ScalarBinaryOpTraits. See
/// https://stackoverflow.com/questions/41494288/mixing-scalar-types-in-eigen
/// for more information.
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Polynomial>::value &&
        std::is_same<typename MatrixR::Scalar, Monomial>::value,
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs * rhs.template cast<Polynomial>();
}

/// Provides Matrix<Polynomial> * Matrix<double> => Matrix<Polynomial>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Polynomial>::value &&
        std::is_same<typename MatrixR::Scalar, double>::value,
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs * rhs.template cast<Polynomial>();
}

/// Provides Matrix<Monomial> * Matrix<Polynomial> => Matrix<Polynomial>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Monomial>::value &&
        std::is_same<typename MatrixR::Scalar, Polynomial>::value,
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Polynomial>() * rhs;
}

/// Provides Matrix<Monomial> * Matrix<Monomial> => Matrix<Polynomial>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Monomial>::value &&
        std::is_same<typename MatrixR::Scalar, Monomial>::value,
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Polynomial>() * rhs.template cast<Polynomial>();
}

/// Provides Matrix<Monomial> * Matrix<double> => Matrix<Polynomial>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Monomial>::value &&
        std::is_same<typename MatrixR::Scalar, double>::value,
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Polynomial>() * rhs.template cast<Polynomial>();
}

/// Provides Matrix<double> * Matrix<Polynomial> => Matrix<Polynomial>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, double>::value &&
        std::is_same<typename MatrixR::Scalar, Polynomial>::value,
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Polynomial>() * rhs;
}

/// Provides Matrix<double> * Matrix<Monomial> => Matrix<Polynomial>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, double>::value &&
        std::is_same<typename MatrixR::Scalar, Monomial>::value,
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Polynomial>() * rhs.template cast<Polynomial>();
}
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

// Informs Eigen that Polynomial op Monomial gets Polynomial
// where op ∈ {+, -, *, scalar_conj_product_op}.
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Polynomial, drake::symbolic::Monomial,
    internal::scalar_sum_op<drake::symbolic::Polynomial,
                            drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Polynomial, drake::symbolic::Monomial,
    internal::scalar_difference_op<drake::symbolic::Polynomial,
                                   drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Polynomial, drake::symbolic::Monomial,
    internal::scalar_product_op<drake::symbolic::Polynomial,
                                drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Polynomial, drake::symbolic::Monomial,
    internal::scalar_conj_product_op<drake::symbolic::Polynomial,
                                     drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};

// Informs Eigen that Polynomial op double gets Polynomial
// where op ∈ {+, -, *, scalar_conj_product_op}.
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Polynomial, double,
    internal::scalar_sum_op<drake::symbolic::Polynomial, double>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Polynomial, double,
    internal::scalar_difference_op<drake::symbolic::Polynomial, double>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Polynomial, double,
    internal::scalar_product_op<drake::symbolic::Polynomial, double>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Polynomial, double,
    internal::scalar_conj_product_op<drake::symbolic::Polynomial, double>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};

// Informs Eigen that Monomial op Polynomial gets Polynomial
// where op ∈ {+, -, *, scalar_conj_product_op}.
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, drake::symbolic::Polynomial,
    internal::scalar_sum_op<drake::symbolic::Monomial,
                            drake::symbolic::Polynomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, drake::symbolic::Polynomial,
    internal::scalar_difference_op<drake::symbolic::Monomial,
                                   drake::symbolic::Polynomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, drake::symbolic::Polynomial,
    internal::scalar_product_op<drake::symbolic::Monomial,
                                drake::symbolic::Polynomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, drake::symbolic::Polynomial,
    internal::scalar_conj_product_op<drake::symbolic::Monomial,
                                     drake::symbolic::Polynomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};

// Informs Eigen that Monomial op Monomial gets Polynomial
// where op ∈ {+, -, *, scalar_conj_product_op}.
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, drake::symbolic::Monomial,
    internal::scalar_sum_op<drake::symbolic::Monomial,
                            drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, drake::symbolic::Monomial,
    internal::scalar_difference_op<drake::symbolic::Monomial,
                                   drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, drake::symbolic::Monomial,
    internal::scalar_product_op<drake::symbolic::Monomial,
                                drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, drake::symbolic::Monomial,
    internal::scalar_conj_product_op<drake::symbolic::Monomial,
                                     drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  // Note that the return type is Polynomial, not Monomial, while Monomial *
  // Monomial gets a Monomial. This is because Eigen's dot() methods is designed
  // to return scalar_conj_product_op::ReturnType.
  typedef drake::symbolic::Polynomial ReturnType;
};

// Informs Eigen that Monomial op double gets Polynomial
// where op ∈ {+, -, *, scalar_conj_product_op}.
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, double,
    internal::scalar_sum_op<drake::symbolic::Monomial, double>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, double,
    internal::scalar_difference_op<drake::symbolic::Monomial, double>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, double,
    internal::scalar_product_op<drake::symbolic::Monomial, double>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, double,
    internal::scalar_conj_product_op<drake::symbolic::Monomial, double>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};

// Informs Eigen that double op Polynomial gets Polynomial
// where op ∈ {+, -, *, scalar_conj_product_op}.
template <>
struct ScalarBinaryOpTraits<
    double, drake::symbolic::Polynomial,
    internal::scalar_sum_op<double, drake::symbolic::Polynomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    double, drake::symbolic::Polynomial,
    internal::scalar_difference_op<double, drake::symbolic::Polynomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    double, drake::symbolic::Polynomial,
    internal::scalar_product_op<double, drake::symbolic::Polynomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    double, drake::symbolic::Polynomial,
    internal::scalar_conj_product_op<double, drake::symbolic::Polynomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};

// Informs Eigen that double op Monomial gets Polynomial
// where op ∈ {+, -, *, scalar_conj_product_op}.
template <>
struct ScalarBinaryOpTraits<
    double, drake::symbolic::Monomial,
    internal::scalar_sum_op<double, drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    double, drake::symbolic::Monomial,
    internal::scalar_difference_op<double, drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    double, drake::symbolic::Monomial,
    internal::scalar_product_op<double, drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};
template <>
struct ScalarBinaryOpTraits<
    double, drake::symbolic::Monomial,
    internal::scalar_conj_product_op<double, drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};

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
