#pragma once

#include <algorithm>  // for cpplint only
#include <cstddef>
#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "drake/common/cond.h"
#include "drake/common/hash.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic_environment.h"
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

class ExpressionCell;  // In drake/common/symbolic_expression_cell.h
class Formula;         // In drake/common/symbolic_formula.h

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
class Expression {
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

  friend Expression operator+(Expression lhs, const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend Expression& operator+=(Expression& lhs, const Expression& rhs);

  /** Provides prefix increment operator (i.e. ++x). */
  Expression& operator++();
  /** Provides postfix increment operator (i.e. x++). */
  Expression operator++(int);

  friend Expression operator-(Expression lhs, const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend Expression& operator-=(Expression& lhs, const Expression& rhs);

  /** Provides unary minus operator. */
  friend Expression operator-(Expression e);
  /** Provides prefix decrement operator (i.e. --x). */
  Expression& operator--();
  /** Provides postfix decrement operator (i.e. x--). */
  Expression operator--(int);

  friend Expression operator*(Expression lhs, const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend Expression& operator*=(Expression& lhs, const Expression& rhs);

  friend Expression operator/(Expression lhs, const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend Expression& operator/=(Expression& lhs, const Expression& rhs);

  friend Expression log(const Expression& e);
  friend Expression abs(const Expression& e);
  friend Expression exp(const Expression& e);
  friend Expression sqrt(const Expression& e);
  friend Expression pow(const Expression& e1, const Expression& e2);
  friend Expression sin(const Expression& e);
  friend Expression cos(const Expression& e);
  friend Expression tan(const Expression& e);
  friend Expression asin(const Expression& e);
  friend Expression acos(const Expression& e);
  friend Expression atan(const Expression& e);
  friend Expression atan2(const Expression& e1, const Expression& e2);
  friend Expression sinh(const Expression& e);
  friend Expression cosh(const Expression& e);
  friend Expression tanh(const Expression& e);
  friend Expression min(const Expression& e1, const Expression& e2);
  friend Expression max(const Expression& e1, const Expression& e2);

  /** Constructs if-then-else expression.

    @verbatim
      if_then_else(cond, exp_then, exp_else)
    @endverbatim

    The value returned by the above if-then-else expression is @p exp_then if @p
    cond is evaluated to true. Otherwise, it returns @p exp_else.

    The semantics is similar to the C++'s conditional expression constructed by
    its ternary operator, @c ?:. However, there is a key difference between the
    C++'s conditional expression and our @c if_then_else expression in a way the
    arguments are evaluated during the construction.

     - In case of the C++'s conditional expression, <tt> cond ? exp_then :
       exp_else</tt>, the then expression @c exp_then (respectively, the else
       expression @c exp_else) is \b only evaluated when the conditional
       expression @c cond is evaluated to \b true (respectively, when @c cond is
       evaluated to \b false).

     - In case of the symbolic expression, <tt>if_then_else(cond, exp_then,
       exp_else)</tt>, however, \b both arguments @c exp_then and @c exp_else
       are evaluated first and then passed to the @c if_then_else function.

     @note This function returns an \b expression and it is different from the
     C++'s if-then-else \b statement.

     @note While it is still possible to define <tt> min, max, abs</tt> math
     functions using @c if_then_else expression, it is highly \b recommended to
     use the provided native definitions for them because it allows solvers to
     detect specific math functions and to have a room for special
     optimizations.

     @note More information about the C++'s conditional expression and ternary
     operator is available at
     http://en.cppreference.com/w/cpp/language/operator_other#Conditional_operator.
   */
  friend Expression if_then_else(const Formula& f_cond,
                                 const Expression& e_then,
                                 const Expression& e_else);

  friend std::ostream& operator<<(std::ostream& os, const Expression& e);
  friend void swap(Expression& a, Expression& b) { std::swap(a.ptr_, b.ptr_); }
  friend class ExpressionAddFactory;
  friend class ExpressionMulFactory;

  /** Checks whether @p v is NaN or not. If @p is NaN, it throws a std::runtime
   * exception. */
  static void check_nan(double v);

 private:
  explicit Expression(const std::shared_ptr<ExpressionCell> ptr);

  std::shared_ptr<ExpressionCell> ptr_;
};

Expression operator+(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator+=(Expression& lhs, const Expression& rhs);
Expression operator-(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator-=(Expression& lhs, const Expression& rhs);
Expression operator-(Expression e);
Expression operator*(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator*=(Expression& lhs, const Expression& rhs);
Expression operator/(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator/=(Expression& lhs, const Expression& rhs);
Expression log(const Expression& e);
Expression abs(const Expression& e);
Expression exp(const Expression& e);
Expression sqrt(const Expression& e);
Expression pow(const Expression& e1, const Expression& e2);
Expression sin(const Expression& e);
Expression cos(const Expression& e);
Expression tan(const Expression& e);
Expression asin(const Expression& e);
Expression acos(const Expression& e);
Expression atan(const Expression& e);
Expression atan2(const Expression& e1, const Expression& e2);
Expression sinh(const Expression& e);
Expression cosh(const Expression& e);
Expression tanh(const Expression& e);
Expression min(const Expression& e1, const Expression& e2);
Expression max(const Expression& e1, const Expression& e2);
Expression if_then_else(const Formula& f_cond, const Expression& e_then,
                        const Expression& e_else);
void swap(Expression& a, Expression& b);

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

/** Provides specialization of @c cond function defined in drake/common/cond.h
 * file. This specialization is required to handle @c double to @c
 * symbolic::Expression conversion so that we can write one such as <tt>cond(x >
 * 0.0, 1.0, -1.0)</tt>.
*/
template <typename... Rest>
symbolic::Expression cond(const symbolic::Formula& f_cond, double v_then,
                          Rest... rest) {
  return if_then_else(f_cond, symbolic::Expression{v_then}, cond(rest...));
}

/** Computes the hash value of a symbolic expression. */
template <>
struct hash_value<symbolic::Expression> {
  size_t operator()(const symbolic::Expression& e) const {
    return e.get_hash();
  }
};

/** Specializes is_numeric to be false for symbolic::Expression type. */
template <>
struct is_numeric<symbolic::Expression> {
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
