#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_EXPRESSION_ALL
#error Do not include this file. Use "drake/common/symbolic/expression.h".
#endif

#include <algorithm>  // for cpplint only
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <random>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/dummy_value.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/fmt.h"
#include "drake/common/hash.h"
#include "drake/common/random.h"

namespace drake {
namespace symbolic {

class ExpressionCell;                   // In expression_cell.h
class ExpressionVar;                    // In expression_cell.h
class UnaryExpressionCell;              // In expression_cell.h
class BinaryExpressionCell;             // In expression_cell.h
class ExpressionAdd;                    // In expression_cell.h
class ExpressionMul;                    // In expression_cell.h
class ExpressionDiv;                    // In expression_cell.h
class ExpressionLog;                    // In expression_cell.h
class ExpressionAbs;                    // In expression_cell.h
class ExpressionExp;                    // In expression_cell.h
class ExpressionSqrt;                   // In expression_cell.h
class ExpressionPow;                    // In expression_cell.h
class ExpressionSin;                    // In expression_cell.h
class ExpressionCos;                    // In expression_cell.h
class ExpressionTan;                    // In expression_cell.h
class ExpressionAsin;                   // In expression_cell.h
class ExpressionAcos;                   // In expression_cell.h
class ExpressionAtan;                   // In expression_cell.h
class ExpressionAtan2;                  // In expression_cell.h
class ExpressionSinh;                   // In expression_cell.h
class ExpressionCosh;                   // In expression_cell.h
class ExpressionTanh;                   // In expression_cell.h
class ExpressionMin;                    // In expression_cell.h
class ExpressionMax;                    // In expression_cell.h
class ExpressionCeiling;                // In expression_cell.h
class ExpressionFloor;                  // In expression_cell.h
class ExpressionIfThenElse;             // In expression_cell.h
class ExpressionUninterpretedFunction;  // In expression_cell.h
class Formula;                          // In formula.h
class Expression;

// Substitution is a map from a Variable to a symbolic expression. It is used in
// Expression::Substitute and Formula::Substitute methods as an argument.
using Substitution = std::unordered_map<Variable, Expression>;

namespace internal {
template <bool>
struct Gemm;  // Defined later in this file.
}  // namespace internal

/** Represents a symbolic form of an expression.

Its syntax tree is as follows:

@verbatim
    E := Var | Constant | E + ... + E | E * ... * E | E / E | log(E)
       | abs(E) | exp(E) | sqrt(E) | pow(E, E) | sin(E) | cos(E) | tan(E)
       | asin(E) | acos(E) | atan(E) | atan2(E, E) | sinh(E) | cosh(E) | tanh(E)
       | min(E, E) | max(E, E) | ceil(E) | floor(E) | if_then_else(F, E, E)
       | NaN | uninterpreted_function(name, {v_1, ..., v_n})
@endverbatim

In the implementation, Expression directly stores Constant values inline, but in
all other cases stores a shared pointer to a const ExpressionCell class that is
a super-class of different kinds of symbolic expressions (i.e., ExpressionAdd,
ExpressionMul, ExpressionLog, ExpressionSin), which makes it efficient to copy,
move, and assign to an Expression.

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
symbolic::Formula instead of bool. Those operations are declared in formula.h
file. To check structural equality between two expressions a separate function,
Expression::EqualTo, is provided.

Regarding the arithmetic of an Expression when operating on NaNs, we have the
following rules:
 1. NaN values are extremely rare during typical computations. Because they are
    difficult to handle symbolically, we will round that up to "must never
    occur". We allow the user to form ExpressionNaN cells in a symbolic
    tree. For example, the user can initialize an Expression to NaN and then
    overwrite it later. However, evaluating a tree that has NaN in its evaluated
    sub-trees is an error (see rule (3) below).
 2. It's still valid for code to check `isnan` in order to fail-fast. So we
    provide isnan(const Expression&) for the common case of non-NaN value
    returning False. This way, code can fail-fast with double yet still compile
    with Expression.
 3. If there are expressions that embed separate cases (`if_then_else`), some of
    the sub-expressions may be not used in evaluation when they are in the
    not-taken case (for NaN reasons or any other reason). Bad values within
    those not-taken branches does not cause exceptions.
 4. The isnan check is different than if_then_else. In the latter, the
    ExpressionNaN is within a dead sub-expression branch. In the former, it
    appears in an evaluated trunk. That goes against rule (1) where a NaN
    anywhere in a computation (other than dead code) is an error.

@internal note for Drake developers: under the hood of Expression, we have an
internal::BoxedCell helper class that uses NaN for pointer tagging; that's a
distinct concept from the Expression::NaN() rules enumerated just above.

symbolic::Expression can be used as a scalar type of Eigen types.
*/
class Expression {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Expression)
  ~Expression() = default;

  /** Default constructor. It constructs Zero(). */
  Expression() = default;

  /** Constructs a constant. */
  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  Expression(double constant) : boxed_(std::isnan(constant) ? 0.0 : constant) {
    if (std::isnan(constant)) {
      ConstructExpressionCellNaN();
    }
  }

  /** Constructs an expression from @p var.
   * @pre @p var is not a BOOLEAN variable.
   */
  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  Expression(const Variable& var);

  /** Returns expression kind. */
  [[nodiscard]] ExpressionKind get_kind() const { return boxed_.get_kind(); }

  /** Collects variables in expression. */
  [[nodiscard]] Variables GetVariables() const;

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
   *     p1.Expand().EqualTo(p2.Expand())
   */
  [[nodiscard]] bool EqualTo(const Expression& e) const;

  /** Provides lexicographical ordering between expressions.
      This function is used as a compare function in map<Expression> and
      set<Expression> via std::less<drake::symbolic::Expression>. */
  [[nodiscard]] bool Less(const Expression& e) const;

  /** Checks if this symbolic expression is convertible to Polynomial. */
  [[nodiscard]] bool is_polynomial() const;

  /** Evaluates using a given environment (by default, an empty environment) and
   * a random number generator. If there is a random variable in this expression
   * which is unassigned in @p env, this method uses @p random_generator to
   * sample a value and use the value to substitute all occurrences of the
   * variable in this expression.
   *
   * @throws std::exception if there exists a non-random variable in this
   *                        expression whose assignment is not provided by
   *                        @p env.
   * @throws std::exception if an unassigned random variable is detected
   *                        while @p random_generator is `nullptr`.
   * @throws std::exception if NaN is detected during evaluation.
   */
  double Evaluate(const Environment& env = Environment{},
                  RandomGenerator* random_generator = nullptr) const;

  /** Evaluates using an empty environment and a random number generator. It
   * uses @p random_generator to sample values for the random variables in this
   * expression.
   *
   * See the above overload for the exceptions that it might throw.
   */
  double Evaluate(RandomGenerator* random_generator) const;

  /** Partially evaluates this expression using an environment @p
   * env. Internally, this method promotes @p env into a substitution
   * (Variable → Expression) and call Evaluate::Substitute with it.
   *
   * @throws std::exception if NaN is detected during evaluation.
   */
  [[nodiscard]] Expression EvaluatePartial(const Environment& env) const;

  /** Returns true if this symbolic expression is already
   * expanded. Expression::Expand() uses this flag to avoid calling
   * ExpressionCell::Expand() on an pre-expanded expressions.
   * Expression::Expand() also sets this flag before returning the result.
   *
   * @note This check is conservative in that `false` does not always indicate
   * that the expression is not expanded. This is because exact checks can be
   * costly and we want to avoid the exact check at the construction time.
   */
  [[nodiscard]] bool is_expanded() const;

  /** Expands out products and positive integer powers in expression. For
   * example, `(x + 1) * (x - 1)` is expanded to `x^2 - 1` and `(x + y)^2` is
   * expanded to `x^2 + 2xy + y^2`. Note that Expand applies recursively to
   * sub-expressions. For instance, `sin(2 * (x + y))` is expanded to `sin(2x +
   * 2y)`. It also simplifies "division by constant" cases. See
   * "drake/common/test/symbolic_expansion_test.cc" to find the examples.
   *
   * @throws std::exception if NaN is detected during expansion.
   */
  [[nodiscard]] Expression Expand() const;

  /** Returns a copy of this expression replacing all occurrences of @p var
   * with @p e.
   * @throws std::exception if NaN is detected during substitution.
   */
  [[nodiscard]] Expression Substitute(const Variable& var,
                                      const Expression& e) const;

  /** Returns a copy of this expression replacing all occurrences of the
   * variables in @p s with corresponding expressions in @p s. Note that the
   * substitutions occur simultaneously. For example, (x / y).Substitute({{x,
   * y}, {y, x}}) gets (y / x).
   * @throws std::exception if NaN is detected during substitution.
   */
  [[nodiscard]] Expression Substitute(const Substitution& s) const;

  /** Differentiates this symbolic expression with respect to the variable @p
   * var.
   * @throws std::exception if it is not differentiable.
   */
  [[nodiscard]] Expression Differentiate(const Variable& x) const;

  /** Let `f` be this Expression, computes a row vector of derivatives,
   * `[∂f/∂vars(0), ... , ∂f/∂vars(n-1)]` with respect to the variables
   * @p vars.
   */
  [[nodiscard]] RowVectorX<Expression> Jacobian(
      const Eigen::Ref<const VectorX<Variable>>& vars) const;

  /** Returns string representation of Expression. */
  [[nodiscard]] std::string to_string() const;

  /** Returns zero. */
  static Expression Zero() { return 0.0; }
  /** Returns one. */
  static Expression One() { return 1.0; }
  /** Returns Pi, the ratio of a circle’s circumference to its diameter. */
  static Expression Pi() { return M_PI; }
  /** Return e, the base of natural logarithms. */
  static Expression E() { return M_E; }
  /** Returns NaN (Not-a-Number). */
  static Expression NaN();

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const Expression& item) noexcept {
    DelegatingHasher delegating_hasher(
        [&hasher](const void* data, const size_t length) {
          return hasher(data, length);
        });
    item.HashAppend(&delegating_hasher);
  }

  friend Expression operator+(Expression lhs, const Expression& rhs) {
    lhs += rhs;
    return lhs;
  }
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend Expression& operator+=(Expression& lhs, const Expression& rhs) {
    // Simplification: Expression(c1) + Expression(c2) => Expression(c1 + c2)
    //
    // Recall that BoxedCell can efficiently provide us with a `double` that is
    // either the Constant stored inside the box, or else is NaN.
    //
    // When both the lhs and rhs are constants, we need to perform the addition
    // inline, for performance. For that case, it's more efficient to perform
    // the sum and then check for NaN afterward (returning immediately in case
    // it wasn't, i.e., terms were Constant) rather that doing two separate
    // checks for lhs.is_constant() and rhs.is_constant() ahead of time.
    //
    // Note that operations on infinities might also produce a NaN as the
    // speculative value. That's fine, we'll handle it during AddImpl; that
    // case is rare, so doesn't need to be inline.
    const double speculative_value =
        lhs.boxed_.constant_or_nan() + rhs.boxed_.constant_or_nan();
    if (!std::isnan(speculative_value)) {
      lhs.boxed_.update_constant(speculative_value);
    } else {
      lhs.AddImpl(rhs);
    }
    return lhs;
  }

  /** Provides prefix increment operator (i.e. ++x). */
  Expression& operator++();
  /** Provides postfix increment operator (i.e. x++). */
  Expression operator++(int);
  /** Provides unary plus operator. */
  friend Expression operator+(const Expression& e);

  friend Expression operator-(Expression lhs, const Expression& rhs) {
    lhs -= rhs;
    return lhs;
  }
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend Expression& operator-=(Expression& lhs, const Expression& rhs) {
    // Simplification: Expression(c1) - Expression(c2) => Expression(c1 - c2)
    // Refer to operator+= comment for how the speculative_value works here.
    const double speculative_value =
        lhs.boxed_.constant_or_nan() - rhs.boxed_.constant_or_nan();
    if (!std::isnan(speculative_value)) {
      lhs.boxed_.update_constant(speculative_value);
    } else {
      lhs.SubImpl(rhs);
    }
    return lhs;
  }

  /** Provides unary minus operator. */
  friend Expression operator-(const Expression& e);
  /** Provides prefix decrement operator (i.e. --x). */
  Expression& operator--();
  /** Provides postfix decrement operator (i.e. x--). */
  Expression operator--(int);

  friend Expression operator*(Expression lhs, const Expression& rhs) {
    lhs *= rhs;
    return lhs;
  }
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend Expression& operator*=(Expression& lhs, const Expression& rhs) {
    // Simplification: Expression(c1) * Expression(c2) => Expression(c1 * c2)
    // Refer to operator+= comment for how the speculative_value works here.
    const double speculative_value =
        lhs.boxed_.constant_or_nan() * rhs.boxed_.constant_or_nan();
    if (!std::isnan(speculative_value)) {
      lhs.boxed_.update_constant(speculative_value);
    } else {
      lhs.MulImpl(rhs);
    }
    return lhs;
  }

  friend Expression operator/(Expression lhs, const Expression& rhs) {
    lhs /= rhs;
    return lhs;
  }
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend Expression& operator/=(Expression& lhs, const Expression& rhs) {
    // Simplification: Expression(c1) / Expression(c2) => Expression(c1 / c2)
    // Refer to operator+= comment for how the speculative_value works here.
    //
    // We check rhs for zero here because DivImpl needs to throw an exception
    // in that case. TODO(jwnimmer-tri) I don't understand why we need to throw
    // during division by zero. The result is typically well-defined (infinity).
    const double rhs_or_nan = rhs.boxed_.constant_or_nan();
    const double speculative_value = lhs.boxed_.constant_or_nan() / rhs_or_nan;
    if ((rhs_or_nan != 0.0) && !std::isnan(speculative_value)) {
      lhs.boxed_.update_constant(speculative_value);
    } else {
      lhs.DivImpl(rhs);
    }
    return lhs;
  }

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
  friend Expression clamp(const Expression& v, const Expression& lo,
                          const Expression& hi);
  friend Expression ceil(const Expression& e);
  friend Expression floor(const Expression& e);

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
  friend Expression uninterpreted_function(std::string name,
                                           std::vector<Expression> arguments);

  friend std::ostream& operator<<(std::ostream& os, const Expression& e);
  friend void swap(Expression& a, Expression& b) {
    std::swap(a.boxed_, b.boxed_);
  }

  friend bool is_constant(const Expression& e);
  friend bool is_constant(const Expression& e, double value);
  friend bool is_nan(const Expression& e);
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
  friend bool is_ceil(const Expression& e);
  friend bool is_floor(const Expression& e);
  friend bool is_if_then_else(const Expression& e);
  friend bool is_uninterpreted_function(const Expression& e);
  friend double get_constant_value(const Expression& e);

  // Note that the following cast functions are only for low-level operations
  // and not exposed to the user of drake/common/symbolic/expression.h header.
  // These functions are declared in the expression_cell.h header.
  friend const ExpressionVar& to_variable(const Expression& e);
  friend const UnaryExpressionCell& to_unary(const Expression& e);
  friend const BinaryExpressionCell& to_binary(const Expression& e);
  friend const ExpressionAdd& to_addition(const Expression& e);
  friend const ExpressionMul& to_multiplication(const Expression& e);
  friend const ExpressionDiv& to_division(const Expression& e);
  friend const ExpressionLog& to_log(const Expression& e);
  friend const ExpressionAbs& to_abs(const Expression& e);
  friend const ExpressionExp& to_exp(const Expression& e);
  friend const ExpressionSqrt& to_sqrt(const Expression& e);
  friend const ExpressionPow& to_pow(const Expression& e);
  friend const ExpressionSin& to_sin(const Expression& e);
  friend const ExpressionCos& to_cos(const Expression& e);
  friend const ExpressionTan& to_tan(const Expression& e);
  friend const ExpressionAsin& to_asin(const Expression& e);
  friend const ExpressionAcos& to_acos(const Expression& e);
  friend const ExpressionAtan& to_atan(const Expression& e);
  friend const ExpressionAtan2& to_atan2(const Expression& e);
  friend const ExpressionSinh& to_sinh(const Expression& e);
  friend const ExpressionCosh& to_cosh(const Expression& e);
  friend const ExpressionTanh& to_tanh(const Expression& e);
  friend const ExpressionMin& to_min(const Expression& e);
  friend const ExpressionMax& to_max(const Expression& e);
  friend const ExpressionCeiling& to_ceil(const Expression& e);
  friend const ExpressionFloor& to_floor(const Expression& e);
  friend const ExpressionIfThenElse& to_if_then_else(const Expression& e);
  friend const ExpressionUninterpretedFunction& to_uninterpreted_function(
      const Expression& e);

  // Cast functions which takes a pointer to a non-const Expression.
  friend ExpressionVar& to_variable(Expression* e);
  friend UnaryExpressionCell& to_unary(Expression* e);
  friend BinaryExpressionCell& to_binary(Expression* e);
  friend ExpressionAdd& to_addition(Expression* e);
  friend ExpressionMul& to_multiplication(Expression* e);
  friend ExpressionDiv& to_division(Expression* e);
  friend ExpressionLog& to_log(Expression* e);
  friend ExpressionAbs& to_abs(Expression* e);
  friend ExpressionExp& to_exp(Expression* e);
  friend ExpressionSqrt& to_sqrt(Expression* e);
  friend ExpressionPow& to_pow(Expression* e);
  friend ExpressionSin& to_sin(Expression* e);
  friend ExpressionCos& to_cos(Expression* e);
  friend ExpressionTan& to_tan(Expression* e);
  friend ExpressionAsin& to_asin(Expression* e);
  friend ExpressionAcos& to_acos(Expression* e);
  friend ExpressionAtan& to_atan(Expression* e);
  friend ExpressionAtan2& to_atan2(Expression* e);
  friend ExpressionSinh& to_sinh(Expression* e);
  friend ExpressionCosh& to_cosh(Expression* e);
  friend ExpressionTanh& to_tanh(Expression* e);
  friend ExpressionMin& to_min(Expression* e);
  friend ExpressionMax& to_max(Expression* e);
  friend ExpressionCeiling& to_ceil(Expression* e);
  friend ExpressionFloor& to_floor(Expression* e);
  friend ExpressionIfThenElse& to_if_then_else(Expression* e);
  friend ExpressionUninterpretedFunction& to_uninterpreted_function(
      Expression* e);

  friend class ExpressionAddFactory;
  friend class ExpressionMulFactory;
  template <bool>
  friend struct internal::Gemm;

 private:
  explicit Expression(std::unique_ptr<ExpressionCell> cell);
  void ConstructExpressionCellNaN();

  void HashAppend(DelegatingHasher* hasher) const;

  void AddImpl(const Expression& rhs);
  void SubImpl(const Expression& rhs);
  void MulImpl(const Expression& rhs);
  void DivImpl(const Expression& rhs);

  // Returns a const reference to the owned cell.
  // @pre This expression is not a Constant.
  const ExpressionCell& cell() const { return boxed_.cell(); }

  // Returns a mutable reference to the owned cell. This function may only be
  // called when this object is the sole owner of the cell (use_count == 1).
  // @pre This expression is not an ExpressionKind::Constant.
  ExpressionCell& mutable_cell();

  internal::BoxedCell boxed_;
};

Expression operator+(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator+=(Expression& lhs, const Expression& rhs);
Expression operator+(const Expression& e);
Expression operator-(Expression lhs, const Expression& rhs);
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator-=(Expression& lhs, const Expression& rhs);
Expression operator-(const Expression& e);
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
Expression clamp(const Expression& v, const Expression& lo,
                 const Expression& hi);
Expression ceil(const Expression& e);
Expression floor(const Expression& e);
Expression if_then_else(const Formula& f_cond, const Expression& e_then,
                        const Expression& e_else);

/** Constructs an uninterpreted-function expression with @p name and @p
 * arguments. An uninterpreted function is an opaque function that has no other
 * property than its name and a list of its arguments. This is useful to
 * applications where it is good enough to provide abstract information of a
 * function without exposing full details. Declaring sparsity of a system is a
 * typical example.
 */
Expression uninterpreted_function(std::string name,
                                  std::vector<Expression> arguments);
void swap(Expression& a, Expression& b);

std::ostream& operator<<(std::ostream& os, const Expression& e);

/** Checks if @p e is a constant expression. */
inline bool is_constant(const Expression& e) {
  return e.boxed_.is_constant();
}
/** Checks if @p e is a constant expression representing @p v. */
inline bool is_constant(const Expression& e, double v) {
  // N.B. This correctly returns `false` even when comparing e's cell against
  // a `v` value of NaN, because NaN == NaN is false.
  return e.boxed_.constant_or_nan() == v;
}
/** Checks if @p e is 0.0. */
inline bool is_zero(const Expression& e) {
  return is_constant(e, 0.0);
}
/** Checks if @p e is 1.0. */
inline bool is_one(const Expression& e) {
  return is_constant(e, 1.0);
}
/** Checks if @p e is -1.0. */
inline bool is_neg_one(const Expression& e) {
  return is_constant(e, -1.0);
}
/** Checks if @p e is 2.0. */
inline bool is_two(const Expression& e) {
  return is_constant(e, 2.0);
}
/** Checks if @p e is NaN. */
inline bool is_nan(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::NaN>();
}
/** Checks if @p e is a variable expression. */
inline bool is_variable(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Var>();
}
/** Checks if @p e is an addition expression. */
inline bool is_addition(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Add>();
}
/** Checks if @p e is a multiplication expression. */
inline bool is_multiplication(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Mul>();
}
/** Checks if @p e is a division expression. */
inline bool is_division(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Div>();
}
/** Checks if @p e is a log expression. */
inline bool is_log(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Log>();
}
/** Checks if @p e is an abs expression. */
inline bool is_abs(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Abs>();
}
/** Checks if @p e is an exp expression. */
inline bool is_exp(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Exp>();
}
/** Checks if @p e is a square-root expression. */
inline bool is_sqrt(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Sqrt>();
}
/** Checks if @p e is a power-function expression. */
inline bool is_pow(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Pow>();
}
/** Checks if @p e is a sine expression. */
inline bool is_sin(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Sin>();
}
/** Checks if @p e is a cosine expression. */
inline bool is_cos(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Cos>();
}
/** Checks if @p e is a tangent expression. */
inline bool is_tan(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Tan>();
}
/** Checks if @p e is an arcsine expression. */
inline bool is_asin(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Asin>();
}
/** Checks if @p e is an arccosine expression. */
inline bool is_acos(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Acos>();
}
/** Checks if @p e is an arctangent expression. */
inline bool is_atan(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Atan>();
}
/** Checks if @p e is an arctangent2 expression. */
inline bool is_atan2(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Atan2>();
}
/** Checks if @p e is a hyperbolic-sine expression. */
inline bool is_sinh(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Sinh>();
}
/** Checks if @p e is a hyperbolic-cosine expression. */
inline bool is_cosh(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Cosh>();
}
/** Checks if @p e is a hyperbolic-tangent expression. */
inline bool is_tanh(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Tanh>();
}
/** Checks if @p e is a min expression. */
inline bool is_min(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Min>();
}
/** Checks if @p e is a max expression. */
inline bool is_max(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Max>();
}
/** Checks if @p e is a ceil expression. */
inline bool is_ceil(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Ceil>();
}
/** Checks if @p e is a floor expression. */
inline bool is_floor(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::Floor>();
}
/** Checks if @p e is an if-then-else expression. */
inline bool is_if_then_else(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::IfThenElse>();
}
/** Checks if @p e is an uninterpreted-function expression. */
inline bool is_uninterpreted_function(const Expression& e) {
  return e.boxed_.is_kind<ExpressionKind::UninterpretedFunction>();
}

/** Returns the constant value of the constant expression @p e.
 *  \pre{@p e is a constant expression.}
 */
inline double get_constant_value(const Expression& e) {
  return e.boxed_.constant();
}
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

/** Returns the name of an uninterpreted-function expression @p e.
 *  \pre @p e is an uninterpreted-function expression.
 */
const std::string& get_uninterpreted_function_name(const Expression& e);

/** Returns the arguments of an uninterpreted-function expression @p e.
 *  \pre @p e is an uninterpreted-function expression.
 */
const std::vector<Expression>& get_uninterpreted_function_arguments(
    const Expression& e);

/** Returns the conditional formula in the if-then-else expression @p e.
 * @pre @p e is an if-then-else expression.
 */
const Formula& get_conditional_formula(const Expression& e);

/** Returns the 'then' expression in the if-then-else expression @p e.
 * @pre @p e is an if-then-else expression.
 */
const Expression& get_then_expression(const Expression& e);

/** Returns the 'else' expression in the if-then-else expression @p e.
 * @pre @p e is an if-then-else expression.
 */
const Expression& get_else_expression(const Expression& e);

Expression operator+(const Variable& var);
Expression operator-(const Variable& var);

/// Returns the Taylor series expansion of `f` around `a` of order `order`.
///
/// @param[in] f     Symbolic expression to approximate using Taylor series
///                  expansion.
/// @param[in] a     Symbolic environment which specifies the point of
///                  approximation. If a partial environment is provided,
///                  the unspecified variables are treated as symbolic
///                  variables (e.g. decision variable).
/// @param[in] order Positive integer which specifies the maximum order of the
///                  resulting polynomial approximating `f` around `a`.
Expression TaylorExpand(const Expression& f, const Environment& a, int order);

}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Expression>. */
template <>
struct hash<drake::symbolic::Expression> : public drake::DefaultHash {};
#if defined(__GLIBCXX__)
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/unordered_associative.html
template <>
struct __is_fast_hash<hash<drake::symbolic::Expression>> : std::false_type {};
#endif

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

/* Provides std::numeric_limits<drake::symbolic::Expression>. */
template <>
struct numeric_limits<drake::symbolic::Expression>
    : public std::numeric_limits<double> {};

/// Provides std::uniform_real_distribution, U(a, b), for symbolic expressions.
///
/// When operator() is called, it returns a symbolic expression `a + (b - a) *
/// v` where v is a symbolic random variable associated with the standard
/// uniform distribution.
///
/// @see std::normal_distribution<drake::symbolic::Expression> for the internal
/// representation of this implementation.
template <>
class uniform_real_distribution<drake::symbolic::Expression> {
 public:
  using RealType = drake::symbolic::Expression;
  using result_type = RealType;

  /// Constructs a new distribution object with a minimum value @p a and a
  /// maximum value @p b.
  ///
  /// @throws std::exception if a and b are constant expressions but a > b.
  explicit uniform_real_distribution(RealType a, RealType b = 1.0)
      : a_{std::move(a)},
        b_{std::move(b)},
        random_variables_{std::make_shared<std::vector<Variable>>()} {
    if (is_constant(a_) && is_constant(b_) &&
        get_constant_value(a_) > get_constant_value(b_)) {
      throw std::runtime_error(
          "In constructing a uniform_real_distribution<Expression>, we "
          "detected that the minimum distribution parameter " +
          a_.to_string() +
          " is greater than the maximum distribution parameter " +
          b_.to_string() + ".");
    }
  }

  /// Constructs a new distribution object with a = 0.0 and b = 1.0.
  uniform_real_distribution() : uniform_real_distribution{0.0} {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(uniform_real_distribution);

  /// Resets the internal state of the distribution object.
  void reset() { index_ = 0; }

  /// Generates a symbolic expression representing a random value that is
  /// distributed according to the associated probability function.
  result_type operator()() {
    if (random_variables_->size() == index_) {
      random_variables_->emplace_back(
          "random_uniform_" + std::to_string(index_),
          drake::symbolic::Variable::Type::RANDOM_UNIFORM);
    }
    const drake::symbolic::Variable& v{(*random_variables_)[index_++]};
    return a_ + (b_ - a_) * v;
  }

  /// Generates a symbolic expression representing a random value that is
  /// distributed according to the associated probability function.
  ///
  /// @note We provide this method, which takes a random generator, for
  /// compatibility with the std::uniform_real_distribution::operator().
  template <class Generator>
  result_type operator()(Generator&) {
    return (*this)();
  }

  /// Returns the minimum value a.
  [[nodiscard]] RealType a() const { return a_; }
  /// Returns the maximum value b.
  [[nodiscard]] RealType b() const { return b_; }

  /// Returns the minimum potentially generated value.
  [[nodiscard]] result_type min() const { return a_; }
  /// Returns the maximum potentially generated value.
  [[nodiscard]] result_type max() const { return b_; }

 private:
  using Variable = drake::symbolic::Variable;

  RealType a_;
  RealType b_;
  std::shared_ptr<std::vector<Variable>> random_variables_;
  std::vector<Variable>::size_type index_{0};

  friend bool operator==(
      const uniform_real_distribution<drake::symbolic::Expression>& lhs,
      const uniform_real_distribution<drake::symbolic::Expression>& rhs) {
    return lhs.a().EqualTo(rhs.a()) && lhs.b().EqualTo(rhs.b()) &&
           (lhs.index_ == rhs.index_) &&
           (lhs.random_variables_ == rhs.random_variables_);
  }
};

inline bool operator!=(
    const uniform_real_distribution<drake::symbolic::Expression>& lhs,
    const uniform_real_distribution<drake::symbolic::Expression>& rhs) {
  return !(lhs == rhs);
}

// TODO(jwnimmer-tri) Rewrite this as a fmt::formatter specialization.
inline std::ostream& operator<<(
    std::ostream& os,
    const uniform_real_distribution<drake::symbolic::Expression>& d) {
  return os << d.a() << " " << d.b();
}

/// Provides std::normal_distribution, N(μ, σ), for symbolic expressions.
///
/// When operator() is called, it returns a symbolic expression `μ + σ * v`
/// where v is a symbolic random variable associated with the standard normal
/// (Gaussian) distribution.
///
/// It keeps a shared pointer to the vector of symbolic random variables that
/// has been created for the following purposes:
///
///  - When `reset()` is called, it rewinds `index_` to zero so that the next
///    operator (re)-uses the first symbolic random variable.
///    @code
///        random_device rd;
///        RandomGenerator g{rd()};
///        std::normal_distribution<Expression> d(0.0, 1.0);
///
///        const Expression e1{d(g)};
///        const Expression e2{d(g)};
///        d.reset();
///        const Expression e3{d(g)};
///
///        EXPECT_FALSE(e1.EqualTo(e2));
///        EXPECT_TRUE(e1.EqualTo(e3));
///    @endcode
///
///  - When an instance of this class is copied, the original and copied
///    distributions share the vector of symbolic random variables. We want to
///    make sure that the two generate identical sequences of elements.
///    @code
///        random_device rd;
///        RandomGenerator g{rd()};
///
///        std::normal_distribution<Expression> d1(0.0, 1.0);
///        std::normal_distribution<Expression> d2(d1);
///        const Expression e1_1{d1(g)};
///        const Expression e1_2{d1(g)};
///
///        const Expression e2_1{d2(g)};
///        const Expression e2_2{d2(g)};
///
///        EXPECT_TRUE(e1_1.EqualTo(e2_1));
///        EXPECT_TRUE(e1_2.EqualTo(e2_2));
///    @endcode
template <>
class normal_distribution<drake::symbolic::Expression> {
 public:
  using RealType = drake::symbolic::Expression;
  using result_type = RealType;

  /// Constructs a new distribution object with @p mean and @p stddev.
  ///
  /// @throws std::exception if stddev is a non-positive constant expression.
  explicit normal_distribution(RealType mean, RealType stddev = 1.0)
      : mean_{std::move(mean)},
        stddev_{std::move(stddev)},
        random_variables_{std::make_shared<std::vector<Variable>>()} {
    if (is_constant(stddev_) && get_constant_value(stddev_) <= 0) {
      throw std::runtime_error(
          "In constructing a normal_distribution<Expression>, we "
          "detected that the stddev distribution parameter " +
          stddev_.to_string() + " is non-positive.");
    }
  }

  /// Constructs a new distribution object with mean = 0.0 and stddev = 1.0.
  normal_distribution() : normal_distribution{0.0} {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(normal_distribution);

  /// Resets the internal state of the distribution object.
  void reset() { index_ = 0; }

  /// Generates a symbolic expression representing a random value that is
  /// distributed according to the associated probability function.
  result_type operator()() {
    if (random_variables_->size() == index_) {
      random_variables_->emplace_back(
          "random_gaussian_" + std::to_string(index_),
          drake::symbolic::Variable::Type::RANDOM_GAUSSIAN);
    }
    const drake::symbolic::Variable& v{(*random_variables_)[index_++]};
    return mean_ + stddev_ * v;
  }

  /// Generates a symbolic expression representing a random value that is
  /// distributed according to the associated probability function.
  ///
  /// @note We provide this method, which takes a random generator, for
  /// compatibility with the std::normal_distribution::operator().
  template <class Generator>
  result_type operator()(Generator&) {
    return (*this)();
  }

  /// Returns the mean μ distribution parameter.
  [[nodiscard]] RealType mean() const { return mean_; }
  /// Returns the deviation σ distribution parameter.
  [[nodiscard]] RealType stddev() const { return stddev_; }

  /// Returns the minimum potentially generated value.
  ///
  /// @note In libstdc++ std::normal_distribution<> defines min() and max() to
  /// return -DBL_MAX and DBL_MAX while the one in libc++ returns -INFINITY and
  /// INFINITY. We follows libc++ and return -INFINITY and INFINITY.
  [[nodiscard]] result_type min() const {
    return -std::numeric_limits<double>::infinity();
  }
  /// Returns the maximum potentially generated value.o
  [[nodiscard]] result_type max() const {
    return std::numeric_limits<double>::infinity();
  }

 private:
  using Variable = drake::symbolic::Variable;

  RealType mean_;
  RealType stddev_;
  std::shared_ptr<std::vector<Variable>> random_variables_;
  std::vector<Variable>::size_type index_{0};

  friend bool operator==(
      const normal_distribution<drake::symbolic::Expression>& lhs,
      const normal_distribution<drake::symbolic::Expression>& rhs) {
    return lhs.mean().EqualTo(rhs.mean()) &&
           lhs.stddev().EqualTo(rhs.stddev()) && (lhs.index_ == rhs.index_) &&
           (lhs.random_variables_ == rhs.random_variables_);
  }
};

inline bool operator!=(
    const normal_distribution<drake::symbolic::Expression>& lhs,
    const normal_distribution<drake::symbolic::Expression>& rhs) {
  return !(lhs == rhs);
}

// TODO(jwnimmer-tri) Rewrite this as a fmt::formatter specialization.
inline std::ostream& operator<<(
    std::ostream& os,
    const normal_distribution<drake::symbolic::Expression>& d) {
  return os << d.mean() << " " << d.stddev();
}

/// Provides std::exponential_distribution, Exp(λ), for symbolic expressions.
///
/// When operator() is called, it returns a symbolic expression `v / λ` where v
/// is a symbolic random variable associated with the standard exponential
/// distribution (λ = 1).
///
/// @see std::normal_distribution<drake::symbolic::Expression> for the internal
/// representation of this implementation.
template <>
class exponential_distribution<drake::symbolic::Expression> {
 public:
  using RealType = drake::symbolic::Expression;
  using result_type = RealType;

  /// Constructs a new distribution object with @p lambda.
  ///
  /// @throws std::exception if lambda is a non-positive constant expression.
  explicit exponential_distribution(RealType lambda)
      : lambda_{std::move(lambda)},
        random_variables_{std::make_shared<std::vector<Variable>>()} {
    if (is_constant(lambda_) && get_constant_value(lambda_) <= 0) {
      throw std::runtime_error(
          "In constructing an exponential_distribution<Expression>, we "
          "detected that the lambda distribution parameter " +
          lambda_.to_string() + " is non-positive.");
    }
  }

  /// Constructs a new distribution object with lambda = 1.0.
  exponential_distribution() : exponential_distribution{1.0} {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(exponential_distribution);

  /// Resets the internal state of the distribution object.
  void reset() { index_ = 0; }

  /// Generates a symbolic expression representing a random value that is
  /// distributed according to the associated probability function.
  result_type operator()() {
    if (random_variables_->size() == index_) {
      random_variables_->emplace_back(
          "random_exponential_" + std::to_string(index_),
          drake::symbolic::Variable::Type::RANDOM_EXPONENTIAL);
    }
    const drake::symbolic::Variable& v{(*random_variables_)[index_++]};
    return v / lambda_;
  }

  /// Generates a symbolic expression representing a random value that is
  /// distributed according to the associated probability function.
  ///
  /// @note We provide this method, which takes a random generator, for
  /// compatibility with the std::exponential_distribution::operator().
  template <class Generator>
  result_type operator()(Generator&) {
    return (*this)();
  }

  /// Returns the lambda λ distribution parameter.
  [[nodiscard]] RealType lambda() const { return lambda_; }
  /// Returns the minimum potentially generated value.
  [[nodiscard]] result_type min() const { return 0.0; }

  /// Returns the maximum potentially generated value.
  /// @note that in libstdc++ exponential_distribution<>::max() returns DBL_MAX
  /// while the one in libc++ returns INFINITY. We follows libc++ and return
  /// INFINITY.
  [[nodiscard]] result_type max() const {
    return std::numeric_limits<double>::infinity();
  }

 private:
  using Variable = drake::symbolic::Variable;

  RealType lambda_;
  std::shared_ptr<std::vector<Variable>> random_variables_;
  std::vector<Variable>::size_type index_{0};

  friend bool operator==(
      const exponential_distribution<drake::symbolic::Expression>& lhs,
      const exponential_distribution<drake::symbolic::Expression>& rhs) {
    return lhs.lambda().EqualTo(rhs.lambda()) && (lhs.index_ == rhs.index_) &&
           (lhs.random_variables_ == rhs.random_variables_);
  }
};

inline bool operator!=(
    const exponential_distribution<drake::symbolic::Expression>& lhs,
    const exponential_distribution<drake::symbolic::Expression>& rhs) {
  return !(lhs == rhs);
}

// TODO(jwnimmer-tri) Rewrite this as a fmt::formatter specialization.
inline std::ostream& operator<<(
    std::ostream& os,
    const exponential_distribution<drake::symbolic::Expression>& d) {
  return os << d.lambda();
}

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

}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)

namespace drake {
namespace symbolic {
namespace internal {

/* Optimized implementations of BLAS GEMM for symbolic types to take advantage
of scalar type specializations. With our current mechanism for hooking this into
Eigen, we only need to support the simplified form C ⇐ A@B rather than the more
general C ⇐ αA@B+βC of typical GEMM; if we figure out how to hook into Eigen's
expression templates, we could expand to the more general form. We group these
functions using a struct so that the friendship declaration with Expression can
be straightforward.
@tparam reverse When true, calculates B@A instead of A@B. */
template <bool reverse>
struct Gemm {
  Gemm() = delete;
  // Allow for passing numpy.ndarray without copies.
  template <typename T>
  using MatrixRef = Eigen::Ref<const MatrixX<T>, 0, StrideX>;
  // Matrix product for double, Variable.
  // When reverse == false, sets result to D * V.
  // When reverse == true, sets result to V * D.
  static void CalcDV(const MatrixRef<double>& D, const MatrixRef<Variable>& V,
                     EigenPtr<MatrixX<Expression>> result);
  // Matrix product for Variable, Variable.
  // When reverse == false, sets result to A * B.
  // When reverse == true, sets result to B * A.
  static void CalcVV(const MatrixRef<Variable>& A, const MatrixRef<Variable>& B,
                     EigenPtr<MatrixX<Expression>> result);
  // Matrix product for double, Expression.
  // When reverse == false, sets result to D * E.
  // When reverse == true, sets result to E * D.
  static void CalcDE(const MatrixRef<double>& D, const MatrixRef<Expression>& E,
                     EigenPtr<MatrixX<Expression>> result);
  // Matrix product for Variable, Expression.
  // When reverse == false, sets result to V * E.
  // When reverse == true, sets result to E * V.
  static void CalcVE(const MatrixRef<Variable>& V,
                     const MatrixRef<Expression>& E,
                     EigenPtr<MatrixX<Expression>> result);
  // Matrix product for Expression, Expression.
  // When reverse == false, sets result to A * B.
  // When reverse == true, sets result to B * A.
  static void CalcEE(const MatrixRef<Expression>& A,
                     const MatrixRef<Expression>& B,
                     EigenPtr<MatrixX<Expression>> result);
};

/* Eigen promises "automatic conversion of the inner product to a scalar", so
when we calculate an inner product we need to return this magic type that acts
like both a Matrix1<Expression> and an Expression. There does not appear to be
any practical way to mimic what Eigen does, other than multiple inheritance. */
class ExpressionInnerProduct
    : public Expression,
      public Eigen::Map<Eigen::Matrix<Expression, 1, 1>> {
 public:
  ExpressionInnerProduct() : Map(this) {}
  void resize(int rows, int cols) { DRAKE_ASSERT(rows == 1 && cols == 1); }
};

/* Helper to look up the return type we'll use for an Expression matmul. */
template <typename MatrixL, typename MatrixR>
using ExpressionMatMulResult =
    std::conditional_t<MatrixL::RowsAtCompileTime == 1 &&
                           MatrixR::ColsAtCompileTime == 1,
                       ExpressionInnerProduct,
                       Eigen::Matrix<Expression, MatrixL::RowsAtCompileTime,
                                     MatrixR::ColsAtCompileTime>>;

}  // namespace internal

// Matrix<Expression> * Matrix<double> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        std::is_same_v<typename MatrixL::Scalar, Expression> &&
        std::is_same_v<typename MatrixR::Scalar, double>,
    internal::ExpressionMatMulResult<MatrixL, MatrixR>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  internal::ExpressionMatMulResult<MatrixL, MatrixR> result;
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = true;
  internal::Gemm<reverse>::CalcDE(rhs, lhs, &result);
  return result;
}

// Matrix<double> * Matrix<Expression> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        std::is_same_v<typename MatrixL::Scalar, double> &&
        std::is_same_v<typename MatrixR::Scalar, Expression>,
    internal::ExpressionMatMulResult<MatrixL, MatrixR>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  internal::ExpressionMatMulResult<MatrixL, MatrixR> result;
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = false;
  internal::Gemm<reverse>::CalcDE(lhs, rhs, &result);
  return result;
}

// Matrix<Expression> * Matrix<Variable> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        std::is_same_v<typename MatrixL::Scalar, Expression> &&
        std::is_same_v<typename MatrixR::Scalar, Variable>,
    internal::ExpressionMatMulResult<MatrixL, MatrixR>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  internal::ExpressionMatMulResult<MatrixL, MatrixR> result;
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = true;
  internal::Gemm<reverse>::CalcVE(rhs, lhs, &result);
  return result;
}

// Matrix<Variable> * Matrix<Expression> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        std::is_same_v<typename MatrixL::Scalar, Variable> &&
        std::is_same_v<typename MatrixR::Scalar, Expression>,
    internal::ExpressionMatMulResult<MatrixL, MatrixR>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  internal::ExpressionMatMulResult<MatrixL, MatrixR> result;
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = false;
  internal::Gemm<reverse>::CalcVE(lhs, rhs, &result);
  return result;
}

// Matrix<Variable> * Matrix<double> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        std::is_same_v<typename MatrixL::Scalar, Variable> &&
        std::is_same_v<typename MatrixR::Scalar, double>,
    internal::ExpressionMatMulResult<MatrixL, MatrixR>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  internal::ExpressionMatMulResult<MatrixL, MatrixR> result;
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = true;
  internal::Gemm<reverse>::CalcDV(rhs, lhs, &result);
  return result;
}

// Matrix<double> * Matrix<Variable> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        std::is_same_v<typename MatrixL::Scalar, double> &&
        std::is_same_v<typename MatrixR::Scalar, Variable>,
    internal::ExpressionMatMulResult<MatrixL, MatrixR>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  internal::ExpressionMatMulResult<MatrixL, MatrixR> result;
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = false;
  internal::Gemm<reverse>::CalcDV(lhs, rhs, &result);
  return result;
}

// Matrix<Variable> * Matrix<Variable> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        std::is_same_v<typename MatrixL::Scalar, Variable> &&
        std::is_same_v<typename MatrixR::Scalar, Variable>,
    internal::ExpressionMatMulResult<MatrixL, MatrixR>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  internal::ExpressionMatMulResult<MatrixL, MatrixR> result;
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = false;
  internal::Gemm<reverse>::CalcVV(lhs, rhs, &result);
  return result;
}

// Matrix<Expression> * Matrix<Expression> => Matrix<Expression>
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        std::is_same_v<typename MatrixL::Scalar, Expression> &&
        std::is_same_v<typename MatrixR::Scalar, Expression>,
    internal::ExpressionMatMulResult<MatrixL, MatrixR>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  internal::ExpressionMatMulResult<MatrixL, MatrixR> result;
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = false;
  internal::Gemm<reverse>::CalcEE(lhs, rhs, &result);
  return result;
}

/// Transform<double> * Transform<Expression> => Transform<Expression>
template <int Dim, int LhsMode, int RhsMode, int LhsOptions, int RhsOptions>
auto operator*(const Eigen::Transform<Expression, Dim, LhsMode, LhsOptions>& t1,
               const Eigen::Transform<double, Dim, RhsMode, RhsOptions>& t2) {
  return t1 * t2.template cast<Expression>();
}

/// Transform<Expression> * Transform<double> => Transform<Expression>
template <int Dim, int LhsMode, int RhsMode, int LhsOptions, int RhsOptions>
auto operator*(
    const Eigen::Transform<double, Dim, LhsMode, LhsOptions>& t1,
    const Eigen::Transform<Expression, Dim, RhsMode, RhsOptions>& t2) {
  return t1.template cast<Expression>() * t2;
}

/// Evaluates a symbolic matrix @p m using @p env and @p random_generator.
///
/// If there is a random variable in @p m which is unassigned in @p env, this
/// function uses @p random_generator to sample a value and use the value to
/// substitute all occurrences of the random variable in @p m.
///
/// @returns a matrix of double whose size is the size of @p m.
/// @throws std::exception if NaN is detected during evaluation.
/// @throws std::exception if @p m includes unassigned random variables but
///                           @p random_generator is `nullptr`.
/// @pydrake_mkdoc_identifier{expression}
template <typename Derived>
std::enable_if_t<std::is_same_v<typename Derived::Scalar, Expression>,
                 MatrixLikewise<double, Derived>>
Evaluate(const Eigen::MatrixBase<Derived>& m,
         const Environment& env = Environment{},
         RandomGenerator* random_generator = nullptr) {
  // Note that the return type is written out explicitly to help gcc 5 (on
  // ubuntu).  Previously the implementation used `auto`, and placed  an `
  // .eval()` at the end to prevent lazy evaluation.
  if (random_generator == nullptr) {
    return m.unaryExpr([&env](const Expression& e) {
      return e.Evaluate(env);
    });
  } else {
    // Construct an environment by extending `env` by sampling values for the
    // random variables in `m` which are unassigned in `env`.
    const Environment env_with_random_variables{PopulateRandomVariables(
        env, GetDistinctVariables(m), random_generator)};
    return m.unaryExpr([&env_with_random_variables](const Expression& e) {
      return e.Evaluate(env_with_random_variables);
    });
  }
}

/** Evaluates @p m using a given environment (by default, an empty environment).
 *
 * @throws std::exception if there exists a variable in @p m whose value is
 *                        not provided by @p env.
 * @throws std::exception if NaN is detected during evaluation.
 */
Eigen::SparseMatrix<double> Evaluate(
    const Eigen::Ref<const Eigen::SparseMatrix<Expression>>& m,
    const Environment& env = Environment{});

/// Substitutes a symbolic matrix @p m using a given substitution @p subst.
///
/// @returns a matrix of symbolic expressions whose size is the size of @p m.
/// @throws std::exception if NaN is detected during substitution.
template <typename Derived>
MatrixLikewise<Expression, Derived> Substitute(
    const Eigen::MatrixBase<Derived>& m, const Substitution& subst) {
  static_assert(std::is_same_v<typename Derived::Scalar, Expression>,
                "Substitute only accepts a symbolic matrix.");
  // Note that the return type is written out explicitly to help gcc 5 (on
  // ubuntu).
  return m.unaryExpr([&subst](const Expression& e) {
    return e.Substitute(subst);
  });
}

/// Substitutes @p var with @p e in a symbolic matrix @p m.
///
/// @returns a matrix of symbolic expressions whose size is the size of @p m.
/// @throws std::exception if NaN is detected during substitution.
template <typename Derived>
MatrixLikewise<Expression, Derived> Substitute(
    const Eigen::MatrixBase<Derived>& m, const Variable& var,
    const Expression& e) {
  static_assert(std::is_same_v<typename Derived::Scalar, Expression>,
                "Substitute only accepts a symbolic matrix.");
  // Note that the return type is written out explicitly to help gcc 5 (on
  // ubuntu).
  return Substitute(m, Substitution{{var, e}});
}

/// Constructs a vector of variables from the vector of variable expressions.
/// @throws std::exception if there is an expression in @p vec which is not a
/// variable.
VectorX<Variable> GetVariableVector(
    const Eigen::Ref<const VectorX<Expression>>& expressions);

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
/// @pydrake_mkdoc_identifier{expression}
MatrixX<Expression> Jacobian(const Eigen::Ref<const VectorX<Expression>>& f,
                             const Eigen::Ref<const VectorX<Variable>>& vars);

/// Returns the distinct variables in the matrix of expressions.
Variables GetDistinctVariables(const Eigen::Ref<const MatrixX<Expression>>& v);

/// Checks if two Eigen::Matrix<Expression> @p m1 and @p m2 are structurally
/// equal. That is, it returns true if and only if `m1(i, j)` is structurally
/// equal to `m2(i, j)` for all `i`, `j`.
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<DerivedA>, DerivedA> &&
        std::is_base_of_v<Eigen::MatrixBase<DerivedB>, DerivedB> &&
        std::is_same_v<typename DerivedA::Scalar, Expression> &&
        std::is_same_v<typename DerivedB::Scalar, Expression>,
    bool>
CheckStructuralEquality(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  // Note that std::equal_to<Expression> calls Expression::EqualTo which checks
  // structural equality between two expressions.
  return m1.binaryExpr(m2, std::equal_to<Expression>{}).all();
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

/// Returns the symbolic expression's value() as a double.
///
/// @throws std::exception if it is not possible to evaluate the symbolic
/// expression with an empty environment.
double ExtractDoubleOrThrow(const symbolic::Expression& e);

/// Returns @p matrix as an Eigen::Matrix<double, ...> with the same size
/// allocation as @p matrix.  Calls ExtractDoubleOrThrow on each element of the
/// matrix, and therefore throws if any one of the extractions fail.
template <typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Derived::Scalar, symbolic::Expression>,
    MatrixLikewise<double, Derived>>
ExtractDoubleOrThrow(const Eigen::MatrixBase<Derived>& matrix) {
  return matrix
      .unaryExpr([](const typename Derived::Scalar& value) {
        return ExtractDoubleOrThrow(value);
      })
      .eval();
}

/*
 * Determine if two EigenBase<> types are matrices (non-column-vectors) of
 * Expressions and doubles, to then form an implicit formulas.
 */
template <typename DerivedV, typename DerivedB>
struct is_eigen_nonvector_expression_double_pair
    : std::bool_constant<
          is_eigen_nonvector_of<DerivedV, symbolic::Expression>::value &&
          is_eigen_nonvector_of<DerivedB, double>::value> {};

/*
 * Determine if two EigenBase<> types are vectors of Expressions and doubles
 * that could make a formula.
 */
template <typename DerivedV, typename DerivedB>
struct is_eigen_vector_expression_double_pair
    : std::bool_constant<
          is_eigen_vector_of<DerivedV, symbolic::Expression>::value &&
          is_eigen_vector_of<DerivedB, double>::value> {};

}  // namespace drake

DRAKE_FORMATTER_AS(, drake::symbolic, Expression, e, e.to_string())
