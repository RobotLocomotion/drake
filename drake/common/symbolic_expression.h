#pragma once

#include <algorithm>  // for cpplint only
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/dummy_value.h"
#include "drake/common/hash.h"
#include "drake/common/number_traits.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {

namespace symbolic {

/** Kinds of symbolic expressions. */
enum class ExpressionKind {
  Constant,    ///< constant (double)
  Var,         ///< variable
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
  NaN,         ///< NaN
  // TODO(soonho): add Integral
};

/** Total ordering between ExpressionKinds. */
bool operator<(ExpressionKind k1, ExpressionKind k2);

class ExpressionCell;        // In drake/common/symbolic_expression_cell.h
class ExpressionConstant;    // In drake/common/symbolic_expression_cell.h
class ExpressionVar;         // In drake/common/symbolic_expression_cell.h
class UnaryExpressionCell;   // In drake/common/symbolic_expression_cell.h
class BinaryExpressionCell;  // In drake/common/symbolic_expression_cell.h
class ExpressionAdd;         // In drake/common/symbolic_expression_cell.h
class ExpressionMul;         // In drake/common/symbolic_expression_cell.h
class ExpressionDiv;         // In drake/common/symbolic_expression_cell.h
class ExpressionLog;         // In drake/common/symbolic_expression_cell.h
class ExpressionAbs;         // In drake/common/symbolic_expression_cell.h
class ExpressionExp;         // In drake/common/symbolic_expression_cell.h
class ExpressionSqrt;        // In drake/common/symbolic_expression_cell.h
class ExpressionPow;         // In drake/common/symbolic_expression_cell.h
class ExpressionSin;         // In drake/common/symbolic_expression_cell.h
class ExpressionCos;         // In drake/common/symbolic_expression_cell.h
class ExpressionTan;         // In drake/common/symbolic_expression_cell.h
class ExpressionAsin;        // In drake/common/symbolic_expression_cell.h
class ExpressionAcos;        // In drake/common/symbolic_expression_cell.h
class ExpressionAtan;        // In drake/common/symbolic_expression_cell.h
class ExpressionAtan2;       // In drake/common/symbolic_expression_cell.h
class ExpressionSinh;        // In drake/common/symbolic_expression_cell.h
class ExpressionCosh;        // In drake/common/symbolic_expression_cell.h
class ExpressionTanh;        // In drake/common/symbolic_expression_cell.h
class ExpressionMin;         // In drake/common/symbolic_expression_cell.h
class ExpressionMax;         // In drake/common/symbolic_expression_cell.h
class ExpressionIfThenElse;  // In drake/common/symbolic_expression_cell.h
class Formula;               // In drake/common/symbolic_formula.h

class Expression;

// Substitution is a map from a Variable to a symbolic expression. It is used in
// Expression::Substitute and Formula::Substitute methods as an argument.
using Substitution =
    std::unordered_map<Variable, Expression, hash_value<Variable>>;

/** Represents a symbolic form of an expression.

Its syntax tree is as follows:

@verbatim
    E := Var | Constant | E + ... + E | E * ... * E | E / E | log(E)
       | abs(E) | exp(E) | sqrt(E) | pow(E, E) | sin(E) | cos(E) | tan(E)
       | asin(E) | acos(E) | atan(E) | atan2(E, E) | sinh(E) | cosh(E) | tanh(E)
       | min(E, E) | max(E, E) | if_then_else(F, E, E) | NaN
@endverbatim

In the implementation, Expression is a simple wrapper including a shared pointer
to ExpressionCell class which is a super-class of different kinds of symbolic
expressions (i.e. ExpressionAdd, ExpressionMul, ExpressionLog,
ExpressionSin). Note that it includes a shared pointer, not a unique pointer, to
allow sharing sub-expressions.

@note The sharing of sub-expressions is not yet implemented.

@note -E is represented as -1 * E internally.

@note A subtraction E1 - E2 is represented as E1 + (-1 * E2) internally.

The following simple simplifications are implemented:
@verbatim
    E + 0             ->  E
    0 + E             ->  E
    E - 0             ->  E
    E - E             ->  0
    E * 1             ->  E
    1 * E             ->  E
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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Expression)

  /** Default constructor. It constructs Zero(). */
  Expression() { *this = Zero(); }

  /** Constructs a constant. */
  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  Expression(double d);
  /** Constructs a variable expression from Variable. */
  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  Expression(const Variable& var);
  /** Returns expression kind. */
  ExpressionKind get_kind() const;
  /** Returns hash value. */
  size_t get_hash() const;
  /** Collects variables in expression. */
  Variables GetVariables() const;

  /** Checks structural equality.
   *
   * Two expressions e1 and e2 are structurally equal when they have the same
   * internal AST(abstract-syntax tree) representation. Please note that we can
   * have two computationally (or extensionally) equivalent expressions which
   * are not structurally equal. For example, consider:
   *
   *    e1 = 2 * (x + y)
   *    e2 = 2x + 2y
   *
   * Obviously, we know that e1 and e2 are evaluated to the same value for all
   * assignments to x and y. However, e1 and e2 are not structurally equal by
   * the definition. Note that e1 is a multiplication expression
   * (is_multiplication(e1) is true) while e2 is an addition expression
   * (is_addition(e2) is true).
   *
   * One main reason we use structural equality in EqualTo is due to
   * Richardson's Theorem. It states that checking ∀x. E(x) = F(x) is
   * undecidable when we allow sin, asin, log, exp in E and F. Read
   * https://en.wikipedia.org/wiki/Richardson%27s_theorem for details.
   *
   * Note that for polynomial cases, you can use Expand method and check if two
   * polynomial expressions p1 and p2 are computationally equal. To do so, you
   * check the following:
   *
   *     (p1.Expand() - p2.Expand()).EqualTo(0).
   */
  bool EqualTo(const Expression& e) const;

  /** Provides lexicographical ordering between expressions.
      This function is used as a compare function in map<Expression> and
      set<Expression> via std::less<drake::symbolic::Expression>. */
  bool Less(const Expression& e) const;

  /** Checks if this symbolic expression is convertible to Polynomial. */
  bool is_polynomial() const;

  /** Returns a Polynomial representing this expression.
   *  Note that the ID of a variable is preserved in this translation.
   *  \pre{is_polynomial() is true.}
   */
  Polynomial<double> ToPolynomial() const;

  /** Evaluates under a given environment (by default, an empty environment).
   *  @throws std::runtime_error if NaN is detected during evaluation.
   */
  double Evaluate(const Environment& env = Environment{}) const;

  /** Expands out products and positive integer powers in expression. For
   * example, <tt>(x + 1) * (x - 1)</tt> is expanded to <tt>x^2 - 1</tt> and
   * <tt>(x + y)^2</tt> is expanded to <tt>x^2 + 2xy + y^2</tt>. Note that
   * Expand applies recursively to sub-expressions. For instance, <tt>sin(2 * (x
   * + y))</tt> is expanded to <tt>sin(2x + 2y)</tt>.
   * @throws std::runtime_error if NaN is detected during expansion.
   */
  Expression Expand() const;

  /** Returns a copy of this expression replacing all occurrences of @p var
   * with @p e.
   * @throws std::runtime_error if NaN is detected during substitution.
   */
  Expression Substitute(const Variable& var, const Expression& e) const;

  /** Returns a copy of this expression replacing all occurrences of the
   * variables in @p s with corresponding expressions in @p s. Note that the
   * substitutions occur simultaneously. For example, (x / y).Substitute({{x,
   * y}, {y, x}}) gets (y / x).
   * @throws std::runtime_error if NaN is detected during substitution.
   */
  Expression Substitute(const Substitution& s) const;

  /** Differentiates this symbolic expression with respect to the variable @p
   * var.
   * @throws std::runtime_error if it is not differentiable.
   */
  Expression Differentiate(const Variable& x) const;

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
  /** Returns NaN (Not-a-Number). */
  static Expression NaN();

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
      if_then_else(cond, expr_then, expr_else)
    @endverbatim

    The value returned by the above if-then-else expression is @p expr_then if
    @p cond is evaluated to true. Otherwise, it returns @p expr_else.

    The semantics is similar to the C++'s conditional expression constructed by
    its ternary operator, @c ?:. However, there is a key difference between the
    C++'s conditional expression and our @c if_then_else expression in a way the
    arguments are evaluated during the construction.

     - In case of the C++'s conditional expression, <tt> cond ? expr_then :
       expr_else</tt>, the then expression @c expr_then (respectively, the else
       expression @c expr_else) is \b only evaluated when the conditional
       expression @c cond is evaluated to \b true (respectively, when @c cond is
       evaluated to \b false).

     - In case of the symbolic expression, <tt>if_then_else(cond, expr_then,
       expr_else)</tt>, however, \b both arguments @c expr_then and @c expr_else
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

  friend bool is_constant(const Expression& e);
  friend bool is_variable(const Expression& e);
  friend bool is_addition(const Expression& e);
  friend bool is_multiplication(const Expression& e);
  friend bool is_division(const Expression& e);
  friend bool is_log(const Expression& e);
  friend bool is_abs(const Expression& e);
  friend bool is_exp(const Expression& e);
  friend bool is_sqrt(const Expression& e);
  friend bool is_pow(const Expression& e);
  friend bool is_sin(const Expression& e);
  friend bool is_cos(const Expression& e);
  friend bool is_tan(const Expression& e);
  friend bool is_asin(const Expression& e);
  friend bool is_acos(const Expression& e);
  friend bool is_atan(const Expression& e);
  friend bool is_atan2(const Expression& e);
  friend bool is_sinh(const Expression& e);
  friend bool is_cosh(const Expression& e);
  friend bool is_tanh(const Expression& e);
  friend bool is_min(const Expression& e);
  friend bool is_max(const Expression& e);
  friend bool is_if_then_else(const Expression& e);

  // Note that the following cast functions are only for low-level operations
  // and not exposed to the user of drake/common/symbolic_expression.h
  // header. These functions are declared in
  // drake/common/symbolic_expression_cell.h header.
  friend std::shared_ptr<ExpressionConstant> to_constant(const Expression& e);
  friend std::shared_ptr<ExpressionVar> to_variable(const Expression& e);
  friend std::shared_ptr<UnaryExpressionCell> to_unary(const Expression& e);
  friend std::shared_ptr<BinaryExpressionCell> to_binary(const Expression& e);
  friend std::shared_ptr<ExpressionAdd> to_addition(const Expression& e);
  friend std::shared_ptr<ExpressionMul> to_multiplication(const Expression& e);
  friend std::shared_ptr<ExpressionDiv> to_division(const Expression& e);
  friend std::shared_ptr<ExpressionLog> to_log(const Expression& e);
  friend std::shared_ptr<ExpressionAbs> to_abs(const Expression& e);
  friend std::shared_ptr<ExpressionExp> to_exp(const Expression& e);
  friend std::shared_ptr<ExpressionSqrt> to_sqrt(const Expression& e);
  friend std::shared_ptr<ExpressionPow> to_pow(const Expression& e);
  friend std::shared_ptr<ExpressionSin> to_sin(const Expression& e);
  friend std::shared_ptr<ExpressionCos> to_cos(const Expression& e);
  friend std::shared_ptr<ExpressionTan> to_tan(const Expression& e);
  friend std::shared_ptr<ExpressionAsin> to_asin(const Expression& e);
  friend std::shared_ptr<ExpressionAcos> to_acos(const Expression& e);
  friend std::shared_ptr<ExpressionAtan> to_atan(const Expression& e);
  friend std::shared_ptr<ExpressionAtan2> to_atan2(const Expression& e);
  friend std::shared_ptr<ExpressionSinh> to_sinh(const Expression& e);
  friend std::shared_ptr<ExpressionCosh> to_cosh(const Expression& e);
  friend std::shared_ptr<ExpressionTanh> to_tanh(const Expression& e);
  friend std::shared_ptr<ExpressionMin> to_min(const Expression& e);
  friend std::shared_ptr<ExpressionMax> to_max(const Expression& e);
  friend std::shared_ptr<ExpressionIfThenElse> to_if_then_else(
      const Expression& e);

  friend class ExpressionAddFactory;
  friend class ExpressionMulFactory;

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

/** Checks if @p e is a constant expression. */
bool is_constant(const Expression& e);
/** Checks if @p e is a constant expression representing @p v. */
bool is_constant(const Expression& e, double v);
/** Checks if @p e is 0.0. */
bool is_zero(const Expression& e);
/** Checks if @p e is 1.0. */
bool is_one(const Expression& e);
/** Checks if @p e is -1.0. */
bool is_neg_one(const Expression& e);
/** Checks if @p e is 2.0. */
bool is_two(const Expression& e);
/** Checks if @p e is NaN. */
bool is_nan(const Expression& e);
/** Checks if @p e is a variable expression. */
bool is_variable(const Expression& e);
/** Checks if @p e is an addition expression. */
bool is_addition(const Expression& e);
/** Checks if @p e is a multiplication expression. */
bool is_multiplication(const Expression& e);
/** Checks if @p e is a division expression. */
bool is_division(const Expression& e);
/** Checks if @p e is a log expression. */
bool is_log(const Expression& e);
/** Checks if @p e is an abs expression. */
bool is_abs(const Expression& e);
/** Checks if @p e is an exp expression. */
bool is_exp(const Expression& e);
/** Checks if @p e is a square-root expression. */
bool is_sqrt(const Expression& e);
/** Checks if @p e is a power-function expression. */
bool is_pow(const Expression& e);
/** Checks if @p e is a sine expression. */
bool is_sin(const Expression& e);
/** Checks if @p e is a cosine expression. */
bool is_cos(const Expression& e);
/** Checks if @p e is a tangent expression. */
bool is_tan(const Expression& e);
/** Checks if @p e is an arcsine expression. */
bool is_asin(const Expression& e);
/** Checks if @p e is an arccosine expression. */
bool is_acos(const Expression& e);
/** Checks if @p e is an arctangent expression. */
bool is_atan(const Expression& e);
/** Checks if @p e is an arctangent2 expression. */
bool is_atan2(const Expression& e);
/** Checks if @p e is a hyperbolic-sine expression. */
bool is_sinh(const Expression& e);
/** Checks if @p e is a hyperbolic-cosine expression. */
bool is_cosh(const Expression& e);
/** Checks if @p e is a hyperbolic-tangent expression. */
bool is_tanh(const Expression& e);
/** Checks if @p e is a min expression. */
bool is_min(const Expression& e);
/** Checks if @p e is a max expression. */
bool is_max(const Expression& e);
/** Checks if @p e is an if-then-else expression. */
bool is_if_then_else(const Expression& e);

/** Returns the constant value of the constant expression @p e.
 *  \pre{@p e is a constant expression.}
 */
double get_constant_value(const Expression& e);
/** Returns the embedded variable in the variable expression @p e.
 *  \pre{@p e is a variable expression.}
 */
const Variable& get_variable(const Expression& e);
/** Returns the argument in the unary expression @p e.
 *  \pre{@p e is a unary expression.}
*/
const Expression& get_argument(const Expression& e);
/** Returns the first argument of the binary expression @p e.
 *  \pre{@p e is a binary expression.}
*/
const Expression& get_first_argument(const Expression& e);
/** Returns the second argument of the binary expression @p e.
 *  \pre{@p e is a binary expression.}
*/
const Expression& get_second_argument(const Expression& e);
/** Returns the constant part of the addition expression @p e. For instance,
 *  given 7 + 2 * x + 3 * y, it returns 7.
 *  \pre{@p e is an addition expression.}
*/
double get_constant_in_addition(const Expression& e);
/** Returns the map from an expression to its coefficient in the addition
 *  expression @p e. For instance, given 7 + 2 * x + 3 * y, the return value
 *  maps 'x' to 2 and 'y' to 3.
 *  \pre{@p e is an addition expression.}
*/
const std::map<Expression, double>& get_expr_to_coeff_map_in_addition(
    const Expression& e);
/** Returns the constant part of the multiplication expression @p e. For
 *  instance, given 7 * x^2 * y^3, it returns 7.
 *  \pre{@p e is a multiplication expression.}
*/
double get_constant_in_multiplication(const Expression& e);
/** Returns the map from a base expression to its exponent expression in the
 * multiplication expression @p e. For instance, given 7 * x^2 * y^3 * z^x, the
 * return value maps 'x' to 2, 'y' to 3, and 'z' to 'x'.
 *  \pre{@p e is a multiplication expression.}
*/
const std::map<Expression, Expression>&
get_base_to_exponent_map_in_multiplication(const Expression& e);

// Matrix<Expression> * Matrix<double> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value &&
        std::is_same<typename MatrixR::Scalar, double>::value,
    Eigen::Matrix<Expression, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Expression>() * rhs.template cast<Expression>();
}

// Matrix<double> * Matrix<Expression> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, double>::value &&
        std::is_same<typename MatrixR::Scalar, Expression>::value,
    Eigen::Matrix<Expression, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Expression>() * rhs.template cast<Expression>();
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator+=(Expression& lhs, const Variable& rhs);
Expression operator+(const Variable& lhs, const Variable& rhs);
Expression operator+(Expression lhs, const Variable& rhs);
Expression operator+(const Variable& lhs, Expression rhs);

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator-=(Expression& lhs, const Variable& rhs);
Expression operator-(const Variable& lhs, const Variable& rhs);
Expression operator-(Expression lhs, const Variable& rhs);
Expression operator-(const Variable& lhs, const Expression& rhs);

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator*=(Expression& lhs, const Variable& rhs);
Expression operator*(const Variable& lhs, const Variable& rhs);
Expression operator*(Expression lhs, const Variable& rhs);
Expression operator*(const Variable& lhs, Expression rhs);

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator/=(Expression& lhs, const Variable& rhs);
Expression operator/(const Variable& lhs, const Variable& rhs);
Expression operator/(Expression lhs, const Variable& rhs);
Expression operator/(const Variable& lhs, const Expression& rhs);

Expression operator+(const Variable& var);
Expression operator-(const Variable& var);

// Matrix<Expression> * Matrix<Variable> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value &&
        std::is_same<typename MatrixR::Scalar, Variable>::value,
    Eigen::Matrix<Expression, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs * rhs.template cast<Expression>();
}

// Matrix<Variable> * Matrix<Expression> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Variable>::value &&
        std::is_same<typename MatrixR::Scalar, Expression>::value,
    Eigen::Matrix<Expression, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Expression>() * rhs;
}

// Matrix<Variable> * Matrix<double> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, Variable>::value &&
        std::is_same<typename MatrixR::Scalar, double>::value,
    Eigen::Matrix<Expression, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Expression>() * rhs.template cast<Expression>();
}

// Matrix<double> * Matrix<Variable> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixL::Scalar, double>::value &&
        std::is_same<typename MatrixR::Scalar, Variable>::value,
    Eigen::Matrix<Expression, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<Expression>() * rhs.template cast<Expression>();
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

/// Specializes common/dummy_value.h.
template <>
struct dummy_value<symbolic::Expression> {
  static symbolic::Expression get() { return symbolic::Expression::NaN(); }
};

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

#if !EIGEN_VERSION_AT_LEAST(3, 2, 93)
/// Provides std::max<drake::symbolic::Expression>. There is nothing about this
/// hack that is not horrible.
template <>
inline const drake::symbolic::Expression& max(
    const drake::symbolic::Expression& lhs,
    const drake::symbolic::Expression& rhs) {
  static constexpr char doom[] = R"doom(
Eigen algebra over drake::symbolic::Expressions cannot be safely implemented
using Eigen 3.2. If you need this, use a platform that supports Eigen 3.3 or
later.
)doom";
  DRAKE_ABORT_MSG(doom);
}
#endif  // EIGEN_VERSION_AT_LEAST(3, 2, 93)

}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
// Define Eigen traits needed for Matrix<drake::symbolic::Expression>.
namespace Eigen {
// Eigen scalar type traits for Matrix<drake::symbolic::Expression>.
template <>
struct NumTraits<drake::symbolic::Expression>
    : GenericNumTraits<drake::symbolic::Expression> {
  static inline int digits10() { return 0; }
};

#if EIGEN_VERSION_AT_LEAST(3, 2, 93)
// Informs Eigen that Variable op Variable gets Expression.
template <typename BinaryOp>
struct ScalarBinaryOpTraits<drake::symbolic::Variable,
                            drake::symbolic::Variable, BinaryOp> {
  enum { Defined = 1 };
  typedef drake::symbolic::Expression ReturnType;
};

// Informs Eigen that Variable op Expression gets Expression.
template <typename BinaryOp>
struct ScalarBinaryOpTraits<drake::symbolic::Variable,
                            drake::symbolic::Expression, BinaryOp> {
  enum { Defined = 1 };
  typedef drake::symbolic::Expression ReturnType;
};

// Informs Eigen that Expression op Variable gets Expression.
template <typename BinaryOp>
struct ScalarBinaryOpTraits<drake::symbolic::Expression,
                            drake::symbolic::Variable, BinaryOp> {
  enum { Defined = 1 };
  typedef drake::symbolic::Expression ReturnType;
};

// Informs Eigen that Variable op double gets Expression.
template <typename BinaryOp>
struct ScalarBinaryOpTraits<drake::symbolic::Variable, double, BinaryOp> {
  enum { Defined = 1 };
  typedef drake::symbolic::Expression ReturnType;
};

// Informs Eigen that double op Variable gets Expression.
template <typename BinaryOp>
struct ScalarBinaryOpTraits<double, drake::symbolic::Variable, BinaryOp> {
  enum { Defined = 1 };
  typedef drake::symbolic::Expression ReturnType;
};

// Informs Eigen that Expression op double gets Expression.
template <typename BinaryOp>
struct ScalarBinaryOpTraits<drake::symbolic::Expression, double, BinaryOp> {
  enum { Defined = 1 };
  typedef drake::symbolic::Expression ReturnType;
};

// Informs Eigen that double op Expression gets Expression.
template <typename BinaryOp>
struct ScalarBinaryOpTraits<double, drake::symbolic::Expression, BinaryOp> {
  enum { Defined = 1 };
  typedef drake::symbolic::Expression ReturnType;
};
#endif  // EIGEN_VERSION_AT_LEAST(3, 2, 93)

}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)

namespace drake {
namespace symbolic {
/// Computes the Jacobian matrix J of the vector function @p f with respect to
/// @p vars. J(i,j) contains ∂f(i)/∂vars(j).
///
///  For example, Jacobian([x * cos(y), x * sin(y), x^2], {x, y}) returns the
///  following 3x2 matrix:
///  <pre>
///  = |cos(y)   -x * sin(y)|
///    |sin(y)    x * cos(y)|
///    | 2 * x             0|
///  </pre>
///
/// @pre {@p vars is non-empty}.
MatrixX<Expression> Jacobian(const Eigen::Ref<const VectorX<Expression>>& f,
                             const std::vector<Variable>& vars);

/// Computes the Jacobian matrix J of the vector function @p f with respect to
/// @p vars. J(i,j) contains ∂f(i)/∂vars(j).
///
/// @pre {@p vars is non-empty}.
MatrixX<Expression> Jacobian(const Eigen::Ref<const VectorX<Expression>>& f,
                             const Eigen::Ref<const VectorX<Variable>>& vars);

/// Checks if two Eigen::Matrix<Expression> @p m1 and @p m2 are structurally
/// equal. That is, it returns true if and only if `m1(i, j)` is structurally
/// equal to `m2(i, j)` for all `i`, `j`.
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::MatrixBase<DerivedB>, DerivedB>::value &&
        std::is_same<typename DerivedA::Scalar, Expression>::value &&
        std::is_same<typename DerivedB::Scalar, Expression>::value,
    bool>::type
CheckStructuralEquality(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  // Note that std::equal_to<Expression> calls Expression::EqualTo which checks
  // structural equality between two expressions.
  return m1.binaryExpr(m2, std::equal_to<Expression>{}).all();
}

}  // namespace symbolic
}  // namespace drake
