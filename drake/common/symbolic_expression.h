#pragma once

#include <algorithm>  // for cpplint only
#include <cstddef>
#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/common/hash.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {

namespace symbolic {

/** Kinds of symbolic expressions. */
enum class ExpressionKind {
  Constant,    ///< constant (double)
  Var,         ///< variable
  Neg,         ///< unary minus
  Add,         ///< addition (+)
  Mul,         ///< multiplication (*)
  Div,         ///< division (/)
  Log,         ///< logarithms
  Abs,         ///< absolute value function
  Exp,         ///< exponentiation
  Sqrt,        ///< square root
  Pow,         ///< power function
  Sin,         ///< sine
  Cos,         ///< cosine
  Tan,         ///< tangent
  Asin,        ///< arcsine
  Acos,        ///< arccosine
  Atan,        ///< arctangent
  Atan2,       ///< arctangent2 (atan2(y,x) = atan(y/x))
  Sinh,        ///< hyperbolic sine
  Cosh,        ///< hyperbolic cosine
  Tanh,        ///< hyperbolic tangent
  Min,         ///< min
  Max,         ///< max
  IfThenElse,  ///< if then else
  // TODO(soonho): add Integral
};

/** Total ordering between ExpressionKinds. */
bool operator<(ExpressionKind k1, ExpressionKind k2);

class ExpressionCell;
class Formula;

/** Represents a symbolic form of an expression.

Its syntax tree is as follows:

@verbatim
    E := Var | Constant | - E | E + ... + E | E * ... * E | E / E | log(E)
       | abs(E) | exp(E) | sqrt(E) | pow(E, E) | sin(E) | cos(E) | tan(E)
       | asin(E) | acos(E) | atan(E) | atan2(E, E) | sinh(E) | cosh(E) | tanh(E)
       | min(E, E) | max(E, E) | if_then_else(F, E, E)
@endverbatim

In the implementation, Expression is a simple wrapper including a shared pointer
to ExpressionCell class which is a super-class of different kinds of symbolic
expressions (i.e. ExpressionAdd, ExpressionMul, ExpressionLog,
ExpressionSin). Note that it includes a shared pointer, not a unique pointer, to
allow sharing sub-expressions.

@note The sharing of sub-expressions is not yet implemented.

@note A subtraction E1 - E2 is represented with an addition and unary minus,
that is, E1 + (-E2).

The following simple simplifications are implemented:
@verbatim
    E + 0             ->  E
    0 + E             ->  E
    E - 0             ->  E
    E - E             ->  0
    E * 1             ->  E
    1 * E             ->  E
    E * -1            -> -E
   -1 * E             -> -E
    E * 0             ->  0
    0 * E             ->  0
    E / 1             ->  E
    E / E             ->  1
    pow(E, 0)         ->  1
    pow(E, 1)         ->  E
    E * E             ->  E^2 (= pow(E, 2))
    sqrt(E * E)       ->  |E| (= abs(E))
    sqrt(E) * sqrt(E) -> E
@endverbatim

Constant folding is implemented:
@verbatim
    E(c1) + E(c2)  ->  E(c1 + c2)    // c1, c2 are constants
    E(c1) - E(c2)  ->  E(c1 - c2)
    E(c1) * E(c2)  ->  E(c1 * c2)
    E(c1) / E(c2)  ->  E(c1 / c2)
    f(E(c))        ->  E(f(c))       // c is a constant, f is a math function
@endverbatim

For the math functions which are only defined over a restricted domain (namely,
log, sqrt, pow, asin, acos), we check the domain of argument(s), and throw
std::domain_error exception if a function is not well-defined for a given
argument(s).

Relational operators over expressions (==, !=, <, >, <=, >=) return
symbolic::Formula instead of bool. Those operations are declared in
symbolic_formula.h file. To check structural equality between two expressions a
separate function, Expression::EqualTo, is provided.

symbolic::Expression can be used as a scalar type of Eigen types.
*/
class DRAKE_EXPORT Expression {
 public:
  /** Default constructor. It constructs Zero(). */
  Expression() { *this = Zero(); }

  /** Move-constructs an Expression from an rvalue. */
  Expression(Expression&& e) = default;

  /** Copy-constructs an Expression from an lvalue. */
  Expression(const Expression& e) = default;

  /** Move-assigns an Expression from an rvalue. */
  Expression& operator=(Expression&& e) = default;

  /** Copy-assigns an Expression from an lvalue. */
  Expression& operator=(const Expression& e) = default;

  /** Constructs a constant. */
  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  Expression(double d);
  /** Constructs a variable. */
  explicit Expression(const Variable& name);
  /** Returns expression kind. */
  ExpressionKind get_kind() const;
  /** Returns hash value. */
  size_t get_hash() const;
  /** Collects variables in expression. */
  Variables GetVariables() const;

  /** Checks structural equality. */
  bool EqualTo(const Expression& e) const;
  /** Provides lexicographical ordering between expressions.
      This function is used as a compare function in map<Expression> and
      set<Expression> via std::less<drake::symbolic::Expression>. */
  bool Less(const Expression& e) const;

  /** Evaluates under a given environment (by default, an empty environment). */
  double Evaluate(const Environment& env = Environment{}) const;

  /** Returns string representation of Expression. */
  std::string to_string() const;

  /** Returns zero. */
  static Expression Zero();
  /** Returns one. */
  static Expression One();
  /** Returns Pi, the ratio of a circle’s circumference to its diameter. */
  static Expression Pi();
  /** Return e, the base of natural logarithms. */
  static Expression E();

  friend DRAKE_EXPORT Expression operator+(Expression lhs,
                                           const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator+=(Expression& lhs,
                                             const Expression& rhs);

  /** Provides prefix increment operator (i.e. ++x). */
  Expression& operator++();
  /** Provides postfix increment operator (i.e. x++). */
  Expression operator++(int);

  friend DRAKE_EXPORT Expression operator-(Expression lhs,
                                           const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator-=(Expression& lhs,
                                             const Expression& rhs);

  /** Provides unary minus operator. */
  friend DRAKE_EXPORT Expression operator-(Expression e);
  /** Provides prefix decrement operator (i.e. --x). */
  Expression& operator--();
  /** Provides postfix decrement operator (i.e. x--). */
  Expression operator--(int);

  friend DRAKE_EXPORT Expression operator*(Expression lhs,
                                           const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator*=(Expression& lhs,
                                             const Expression& rhs);

  friend DRAKE_EXPORT Expression operator/(Expression lhs,
                                           const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator/=(Expression& lhs,
                                             const Expression& rhs);

  friend DRAKE_EXPORT Expression log(const Expression& e);
  friend DRAKE_EXPORT Expression abs(const Expression& e);
  friend DRAKE_EXPORT Expression exp(const Expression& e);
  friend DRAKE_EXPORT Expression sqrt(const Expression& e);
  friend DRAKE_EXPORT Expression pow(const Expression& e1,
                                     const Expression& e2);
  friend DRAKE_EXPORT Expression sin(const Expression& e);
  friend DRAKE_EXPORT Expression cos(const Expression& e);
  friend DRAKE_EXPORT Expression tan(const Expression& e);
  friend DRAKE_EXPORT Expression asin(const Expression& e);
  friend DRAKE_EXPORT Expression acos(const Expression& e);
  friend DRAKE_EXPORT Expression atan(const Expression& e);
  friend DRAKE_EXPORT Expression atan2(const Expression& e1,
                                       const Expression& e2);
  friend DRAKE_EXPORT Expression sinh(const Expression& e);
  friend DRAKE_EXPORT Expression cosh(const Expression& e);
  friend DRAKE_EXPORT Expression tanh(const Expression& e);
  friend DRAKE_EXPORT Expression min(const Expression& e1,
                                     const Expression& e2);
  friend DRAKE_EXPORT Expression max(const Expression& e1,
                                     const Expression& e2);
  friend DRAKE_EXPORT Expression if_then_else(const Formula& f_cond,
                                              const Expression& e_then,
                                              const Expression& e_else);

  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream& os,
                                               const Expression& e);
  friend DRAKE_EXPORT void swap(Expression& a, Expression& b) {
    std::swap(a.ptr_, b.ptr_);
  }
  friend class ExpressionAddFactory;
  friend class ExpressionMulFactory;

  /** Checks whether @p v is NaN or not. If @p is NaN, it throws a std::runtime
   * exception. */
  static void check_nan(double v);

 private:
  explicit Expression(const std::shared_ptr<ExpressionCell> ptr);

  std::shared_ptr<ExpressionCell> ptr_;
};

DRAKE_EXPORT Expression operator+(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
DRAKE_EXPORT Expression& operator+=(Expression& lhs, const Expression& rhs);
DRAKE_EXPORT Expression operator-(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
DRAKE_EXPORT Expression& operator-=(Expression& lhs, const Expression& rhs);
DRAKE_EXPORT Expression operator-(Expression e);
DRAKE_EXPORT Expression operator*(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
DRAKE_EXPORT Expression& operator*=(Expression& lhs, const Expression& rhs);
DRAKE_EXPORT Expression operator/(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
DRAKE_EXPORT Expression& operator/=(Expression& lhs, const Expression& rhs);
DRAKE_EXPORT Expression log(const Expression& e);
DRAKE_EXPORT Expression abs(const Expression& e);
DRAKE_EXPORT Expression exp(const Expression& e);
DRAKE_EXPORT Expression sqrt(const Expression& e);
DRAKE_EXPORT Expression pow(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Expression sin(const Expression& e);
DRAKE_EXPORT Expression cos(const Expression& e);
DRAKE_EXPORT Expression tan(const Expression& e);
DRAKE_EXPORT Expression asin(const Expression& e);
DRAKE_EXPORT Expression acos(const Expression& e);
DRAKE_EXPORT Expression atan(const Expression& e);
DRAKE_EXPORT Expression atan2(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Expression sinh(const Expression& e);
DRAKE_EXPORT Expression cosh(const Expression& e);
DRAKE_EXPORT Expression tanh(const Expression& e);
DRAKE_EXPORT Expression min(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Expression max(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Expression if_then_else(const Formula& f_cond,
                                     const Expression& e_then,
                                     const Expression& e_else);

/** Constructs conditional expression (similar to Lisp's cond).

  @verbatim
    cond(cond_1, exp_1,
         cond_2, exp_2,
            ...,   ...,
         cond_n, exp_n,
         exp_{n+1})
  @endverbatim

  The value returned by the above cond expression is @c exp_1 if @c cond_1 is
  true; else if @c cond_2 is true then @c exp_2; ... ; else if @c cond_n is true
  then @c exp_n. If none of the conditions are true, it returns @c exp_{n+1}.
 */
Expression cond(const Expression& e);
template <typename... Rest>
Expression cond(const Formula& f_cond, const Expression& e_then, Rest... rest) {
  return if_then_else(f_cond, e_then, cond(rest...));
}

DRAKE_EXPORT void swap(Expression& a, Expression& b);

std::ostream& operator<<(std::ostream& os, const Expression& e);

/** @relates Expression
 * Return a copy of @p lhs updated to record component-wise multiplication by a
 * constant @p rhs.
 */
template <typename MatrixL>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value,
    typename MatrixL::PlainObject>::type
operator*(const MatrixL& lhs, double rhs) {
  return lhs * Expression{rhs};
}

/** @relates Expression
 * Return a copy of @p rhs updated to record component-wise multiplication by a
 * constant @p rhs.
 */
template <typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixR::Scalar, Expression>::value,
    typename MatrixR::PlainObject>::type
operator*(double lhs, const MatrixR& rhs) {
  return (Expression{lhs}) * rhs;
}

/** @relates Expression
 * Update @p lhs to record component-wise multiplication by a constant @p rhs.
 */
template <typename MatrixL>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value,
    MatrixL&>::type
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
operator*=(MatrixL& lhs, double rhs) {
  return lhs *= Expression{rhs};
}

/** @relates Expression
 * Return a copy of @p lhs updated to record component-wise division by a
 * constant @p rhs.
 */
template <typename MatrixL>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value,
    typename MatrixL::PlainObject>::type
operator/(const MatrixL& lhs, double rhs) {
  return lhs / Expression{rhs};
}

/** @relates Expression
 * Update @p lhs to record component-wise division by a constant @p rhs.
 */
template <typename MatrixL>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value,
    MatrixL&>::type
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
operator/=(MatrixL& lhs, double rhs) {
  return lhs /= Expression{rhs};
}
}  // namespace symbolic

/** Computes the hash value of a symbolic expression. */
template <>
struct hash_value<symbolic::Expression> {
  size_t operator()(const symbolic::Expression& e) const {
    return e.get_hash();
  }
};

/** Specializes is_numeric to be false for symbolic::Expression type. */
template <>
struct DRAKE_EXPORT is_numeric<symbolic::Expression> {
  static constexpr bool value = false;
};
}  // namespace drake

namespace std {
/* Provides std::less<drake::symbolic::Expression>. */
template <>
struct less<drake::symbolic::Expression> {
  bool operator()(const drake::symbolic::Expression& lhs,
                  const drake::symbolic::Expression& rhs) const {
    return lhs.Less(rhs);
  }
};

/* Provides std::equal_to<drake::symbolic::Expression>. */
template <>
struct equal_to<drake::symbolic::Expression> {
  bool operator()(const drake::symbolic::Expression& lhs,
                  const drake::symbolic::Expression& rhs) const {
    return lhs.EqualTo(rhs);
  }
};
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
// Define Eigen traits needed for Matrix<drake::symbolic::Expression>.
namespace Eigen {
// Eigen scalar type traits for Matrix<drake::symbolic::Expression>.
template <>
struct NumTraits<drake::symbolic::Expression>
    : GenericNumTraits<drake::symbolic::Expression> {
  typedef drake::symbolic::Expression Real;
  typedef drake::symbolic::Expression NonInteger;
  typedef drake::symbolic::Expression Nested;
  typedef drake::symbolic::Expression Literal;
  static inline Real epsilon() { return drake::symbolic::Expression::Zero(); }
  static inline Real dummy_precision() {
    return drake::symbolic::Expression::Zero();
  }
  static inline int digits10() { return 0; }
  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 1,
    MulCost = 1
  };
  template <bool Vectorized>
  struct Div {
    enum { Cost = 1 };
  };
};

namespace internal {
// Eigen component-wise Matrix<Expression>::isConstant(Expression).
template <>
struct scalar_fuzzy_impl<drake::symbolic::Expression> {
  static inline bool isApprox(drake::symbolic::Expression x,
                              drake::symbolic::Expression y,
                              drake::symbolic::Expression) {
    return x.EqualTo(y);
  }
};
}  // namespace internal
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
