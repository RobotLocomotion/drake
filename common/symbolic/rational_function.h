#pragma once

#include <ostream>

#include "drake/common/fmt_ostream.h"
#include "drake/common/symbolic/polynomial.h"

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

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RationalFunction);

  /**
   * Constructs the rational function: numerator / denominator.
   * @param numerator The numerator of the fraction.
   * @param denominator The denominator of the fraction.
   * @pre denominator is not equal to zero.
   * @pre None of the indeterminates in the numerator can be decision variables
   * in the denominator; similarly none of the indeterminates in the denominator
   * can be decision variables in the numerator.
   */
  RationalFunction(Polynomial numerator, Polynomial denominator);

  /**
   * Constructs the rational function: p / 1. Note that we use 1 as the
   * denominator.
   * @param p The numerator of the rational function.
   */
  explicit RationalFunction(const Polynomial& p);

  /**
   * Constructs the rational function: m / 1 for any type which can
   * be cast to a monomial
   * @param m The numerator of the rational function.
   */
  explicit RationalFunction(const Monomial& m);

  /**
   * Constructs the rational function: c / 1. Note that we use 1 as the
   * denominator.
   * @param c The numerator of the rational function.
   */
  explicit RationalFunction(double c);

  /**
   * Evaluates this rational function under a given environment @p env.
   * @throws std::exception if there is a variable in this rational function
   * whose assignment is not provided by @p env.
   */
  [[nodiscard]] double Evaluate(const Environment& env) const;

  ~RationalFunction() = default;

  /// Getter for the numerator.
  [[nodiscard]] const Polynomial& numerator() const { return numerator_; }

  /// Getter for the denominator.
  [[nodiscard]] const Polynomial& denominator() const { return denominator_; }

  RationalFunction& operator+=(const RationalFunction& f);
  RationalFunction& operator+=(const Polynomial& p);
  RationalFunction& operator+=(const Monomial& m);
  RationalFunction& operator+=(double c);

  RationalFunction& operator-=(const RationalFunction& f);
  RationalFunction& operator-=(const Polynomial& p);
  RationalFunction& operator-=(const Monomial& m);
  RationalFunction& operator-=(double c);

  RationalFunction& operator*=(const RationalFunction& f);
  RationalFunction& operator*=(const Polynomial& p);
  RationalFunction& operator*=(const Monomial& m);
  RationalFunction& operator*=(double c);

  /**
   * @throws std::exception if the numerator of the divisor is structurally
   * equal to zero. Note that this does not guarantee that the denominator of
   * the result is not zero after expansion.
   */
  RationalFunction& operator/=(const RationalFunction& f);
  /**
   * @throws std::exception if the divisor is structurally equal to zero.
   * Note that this does not guarantee that the denominator of the result is not
   * zero after expansion.
   */
  RationalFunction& operator/=(const Polynomial& p);
  RationalFunction& operator/=(const Monomial& m);
  /**
   * @throws std::exception if c is 0
   */
  RationalFunction& operator/=(double c);

  /**
   * Unary minus operation for rational function.
   * if f(x) = p(x) / q(x), then -f(x) = (-p(x)) / q(x)
   */
  friend RationalFunction operator-(RationalFunction f);

  /**
   * Returns true if this rational function and f are structurally equal.
   */
  [[nodiscard]] bool EqualTo(const RationalFunction& f) const;

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

  /// Returns an equivalent symbolic expression of this rational function.
  [[nodiscard]] Expression ToExpression() const;

  /// Sets the indeterminates of the numerator and denominator polynomials
  void SetIndeterminates(const Variables& new_indeterminates);

 private:
  // Throws std::exception if an indeterminate of the denominator (numerator,
  // respectively) is a decision variable of the numerator (denominator).
  void CheckIndeterminates() const;
  Polynomial numerator_;
  Polynomial denominator_;
};

RationalFunction operator+(RationalFunction f1, const RationalFunction& f2);
RationalFunction operator+(RationalFunction f, const Polynomial& p);
RationalFunction operator+(const Polynomial& p, RationalFunction f);
RationalFunction operator+(const Monomial& m, RationalFunction f);
RationalFunction operator+(RationalFunction f, const Monomial& m);
RationalFunction operator+(RationalFunction f, double c);
RationalFunction operator+(double c, RationalFunction f);

RationalFunction operator-(RationalFunction f1, const RationalFunction& f2);
RationalFunction operator-(RationalFunction f, const Polynomial& p);
RationalFunction operator-(const Polynomial& p, const RationalFunction& f);
RationalFunction operator-(RationalFunction f, double c);
RationalFunction operator-(double c, RationalFunction f);
RationalFunction operator-(const Monomial& m, RationalFunction f);
RationalFunction operator-(RationalFunction f, const Monomial& m);

RationalFunction operator*(RationalFunction f1, const RationalFunction& f2);
RationalFunction operator*(RationalFunction f, const Polynomial& p);
RationalFunction operator*(const Polynomial& p, RationalFunction f);
RationalFunction operator*(RationalFunction f, double c);
RationalFunction operator*(double c, RationalFunction f);
RationalFunction operator*(const Monomial& m, RationalFunction f);
RationalFunction operator*(RationalFunction f, const Monomial& m);

/**
 * @throws std::exception if the numerator of the divisor is structurally
 * equal to zero. Note that this does not guarantee that the denominator of the
 * result is not zero after expansion.
 */
RationalFunction operator/(RationalFunction f1, const RationalFunction& f2);
/**
 * @throws std::exception if the divisor is structurally equal to zero.
 * Note that this does not guarantee that the denominator of the result is not
 * zero after expansion.
 */
RationalFunction operator/(RationalFunction f, const Polynomial& p);
/**
 * @throws std::exception if the numerator of the divisor is structurally
 * equal to zero. Note that this does not guarantee that the denominator of the
 * result is not zero after expansion.
 */
RationalFunction operator/(const Polynomial& p, const RationalFunction& f);
/**
 * @throws std::exception if c is 0
 */
RationalFunction operator/(RationalFunction f, double c);
/**
 * @throws std::exception if the numerator of  the divisor is structurally
 * equal to zero. Note that this does not guarantee that the denominator of the
 * result is not zero after expansion.
 */
RationalFunction operator/(double c, const RationalFunction& f);
RationalFunction operator/(const Monomial& m, RationalFunction f);
RationalFunction operator/(RationalFunction f, const Monomial& m);

/**
 * Returns the rational function @p f raised to @p n.
 * If n is positive, (f/g)ⁿ = fⁿ / gⁿ;
 * If n is negative, (f/g)ⁿ = g⁻ⁿ / f⁻ⁿ;
 * (f/g)⁰ = 1 / 1.
 */
RationalFunction pow(const RationalFunction& f, int n);
/**
 * Provides the following operations:
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
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        ((std::is_same_v<typename MatrixL::Scalar, RationalFunction> &&
          (std::is_same_v<typename MatrixR::Scalar, Polynomial> ||
           std::is_same_v<typename MatrixR::Scalar, double>)) ||
         (std::is_same_v<typename MatrixR::Scalar, RationalFunction> &&
          (std::is_same_v<typename MatrixL::Scalar, Polynomial> ||
           std::is_same_v<typename MatrixL::Scalar, double>))),
    Eigen::Matrix<RationalFunction, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>
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
  constexpr static int digits() { return 0; }
  constexpr static int digits10() { return 0; }
  constexpr static int max_digits10() { return 0; }
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

namespace numext {
template <>
bool equal_strict(const drake::symbolic::RationalFunction& x,
                  const drake::symbolic::RationalFunction& y);
template <>
EIGEN_STRONG_INLINE bool not_equal_strict(
    const drake::symbolic::RationalFunction& x,
    const drake::symbolic::RationalFunction& y) {
  return !Eigen::numext::equal_strict(x, y);
}
}  // namespace numext
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::symbolic::RationalFunction> : drake::ostream_formatter {
};
}  // namespace fmt
