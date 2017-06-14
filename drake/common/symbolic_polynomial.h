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
class Polynomial {
 public:
  using MapType =
      std::unordered_map<Monomial, Expression, hash_value<Monomial>>;

  /// Constructs a zero polynomial.
  Polynomial() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Polynomial)

  /// Constructs a polynomial from an expression @p e. Note that all variables
  /// in `e` are considered as indeterminates.
  explicit Polynomial(const Expression& e);

  /// Constructs a polynomial from a monomial @p m. Note that all variables
  /// in `m` are considered as indeterminates.
  explicit Polynomial(const Monomial& m);

  /// Constructs a polynomial from an expression @p e by decomposing it with
  /// respect to @p indeterminates.
  Polynomial(const Expression& e, const Variables& indeterminates);

  /// Returns the indeterminates of this polynomial.
  Variables indeterminates() const;

  /// Returns the decision variables of this polynomial.
  Variables decision_variables() const;

  /// Returns the total degree of this polynomial.
  int Degree() const;

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
  Polynomial& operator*=(const Variable& v);
  Polynomial& operator*=(double c);

  /// Returns true if this polynomial and @p p are structurally equal.
  bool EqualTo(const Polynomial& p) const;

  /// Returns a symbolic formula representing the condition where this
  /// polynomial and @p p are the same.
  Formula operator==(Polynomial p) const;

 private:
  /// Add (coeff * m) to this polynomial.
  Polynomial& Add(const Expression& coeff, const Monomial& m);

  /// Checks if there is a variable appeared in both of decision_variables() and
  /// indeterminates(). @throws std::runtime error if there is a case.
  void CheckInvariant() const;

  // Variables indeterminates_;
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
Polynomial operator*(Polynomial p, const Variable& v);
Polynomial operator*(const Variable& v, Polynomial p);
Polynomial operator*(double c, Polynomial p);
Polynomial operator*(Polynomial p, double c);

/// Returns polynomial @p rasied to @p n.
Polynomial pow(Polynomial p, int n);

std::ostream& operator<<(std::ostream& os, const Polynomial& p);

// Matrix<Variable> * Matrix<Monomial> => Matrix<Polynomial>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Variable>::value &&
        std::is_same<typename MatrixR::Scalar, Monomial>::value,
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Polynomial>() * rhs.template cast<Polynomial>();
}

// Matrix<Monomial> * Matrix<Variable> => Matrix<Polynomial>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Monomial>::value &&
        std::is_same<typename MatrixR::Scalar, Variable>::value,
    Eigen::Matrix<Polynomial, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Polynomial>() * rhs.template cast<Polynomial>();
}

// Matrix<Polynomial> * Matrix<Monomial> => Matrix<Polynomial>
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

// Matrix<Monomial> * Matrix<Polynomial> => Matrix<Polynomial>
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

}  // namespace symbolic
}  // namespace drake

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {

// Define Eigen traits needed for Matrix<drake::symbolic::Polynomial>.
template <>
struct NumTraits<drake::symbolic::Polynomial>
    : GenericNumTraits<drake::symbolic::Polynomial> {
  static inline int digits10() { return 0; }
};

// Informs Eigen that Monomial + Monomial gets Polynomial.
template <>
struct ScalarBinaryOpTraits<
    drake::symbolic::Monomial, drake::symbolic::Monomial,
    internal::scalar_sum_op<drake::symbolic::Monomial,
                            drake::symbolic::Monomial>> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};

// Informs Eigen that Monomial op Polynomial gets Polynomial.
template <typename BinaryOp>
struct ScalarBinaryOpTraits<drake::symbolic::Monomial,
                            drake::symbolic::Polynomial, BinaryOp> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};

// Informs Eigen that Polynomial op Monomial gets Polynomial.
template <typename BinaryOp>
struct ScalarBinaryOpTraits<drake::symbolic::Polynomial,
                            drake::symbolic::Monomial, BinaryOp> {
  enum { Defined = 1 };
  typedef drake::symbolic::Polynomial ReturnType;
};

}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
