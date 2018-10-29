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
   * @throws std::logic_error if the precondition is not satisfied.
   */
  RationalFunction(const Polynomial& numerator, const Polynomial& denominator);

  /**
   * Constructs the rational function: p / 1. Note that we use 1 as the
   * denominator.
   * @param p The numerator of the rational function.
   */
  explicit RationalFunction(const Polynomial& p);

  /**
   * Constructs the rational function: c / 1. Note that we use 1 as the
   * denominator.
   * @param c The numerator of the rational function.
   */
  explicit RationalFunction(double c);

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

  /**
   * Returns a symbolic formula representing the condition where this rational
   * function and @p f are the same.
   * If f1 = p1 / q1, f2 = p2 / q2, then f1 == f2 <=> p1 * q2 == p2 * q1
   */
  Formula operator==(const RationalFunction& f) const;

  /**
   * Returns a symbolic formula representing the condition where this rational
   * function and @p f are not the same.
   */
  Formula operator!=(const RationalFunction& f) const;

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
/**
 *  Provides the following operations:
 *
 *  - Matrix<RF>         * Matrix<Polynomial> => Matrix<RF>
 *  - Matrix<RF>         * Matrix<double>     => Matrix<RF>
 *  - Matrix<Polynomial> * Matrix<RF>         => Matrix<RF>
 *  - Matrix<double>     * Matrix<RF>         => Matrix<RF>
 *
 * where RF is a shorthand for RationalFunction.
 *
 * @note that these operator overloadings are necessary even after providing
 * Eigen::ScalarBinaryOpTraits. See
 * https://stackoverflow.com/questions/41494288/mixing-scalar-types-in-eigen
 * for more information
 */
#if defined(DRAKE_DOXYGEN_CXX)
template <typename MatrixL, typename MatrixR>
Eigen::Matrix<RationalFunction, MatrixL::RowsAtCompileTime,
              MatrixR::ColsAtCompileTime>
operator*(const MatrixL& lhs, const MatrixR& rhs);
#else
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        ((std::is_same<typename MatrixL::Scalar, RationalFunction>::value &&
          (std::is_same<typename MatrixR::Scalar, Polynomial>::value ||
           std::is_same<typename MatrixR::Scalar, double>::value)) ||
         (std::is_same<typename MatrixR::Scalar, RationalFunction>::value &&
          (std::is_same<typename MatrixL::Scalar, Polynomial>::value ||
           std::is_same<typename MatrixL::Scalar, double>::value))),
    Eigen::Matrix<RationalFunction, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<RationalFunction>() *
         rhs.template cast<RationalFunction>();
}
#endif
}  // namespace symbolic
}  // namespace drake

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {

// Defines Eigen traits needed for Matrix<drake::symbolic::RationalFunction>.
template <>
struct NumTraits<drake::symbolic::RationalFunction>
    : GenericNumTraits<drake::symbolic::RationalFunction> {
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
// where op ∈ {+, -, *, /, conj_product}.
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

// Informs Eigen that RationalFunction op Polynomial gets RationalFunction
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::RationalFunction, drake::symbolic::Polynomial,
    drake::symbolic::RationalFunction)

// Informs Eigen that Polynomial op RationalFunction gets RationalFunction
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Polynomial, drake::symbolic::RationalFunction,
    drake::symbolic::RationalFunction)

// Informs Eigen that double op RationalFunction gets RationalFunction
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    double, drake::symbolic::RationalFunction,
    drake::symbolic::RationalFunction)

// Informs Eigen that RationalFunction op double gets RationalFunction
// where op ∈ {+, -, *, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::RationalFunction, double,
    drake::symbolic::RationalFunction)
#undef DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS
#undef DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
