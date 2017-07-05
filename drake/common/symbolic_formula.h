#pragma once

#include <functional>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

/** Kinds of symbolic formulas. */
enum class FormulaKind {
  False,                 ///< ⊥
  True,                  ///< ⊤
  Var,                   ///< Boolean Variable
  Eq,                    ///< =
  Neq,                   ///< !=
  Gt,                    ///< >
  Geq,                   ///< >=
  Lt,                    ///< <
  Leq,                   ///< <=
  And,                   ///< Conjunction (∧)
  Or,                    ///< Disjunction (∨)
  Not,                   ///< Negation (¬)
  Forall,                ///< Universal quantification (∀)
  Isnan,                 ///< NaN check predicate
  PositiveSemidefinite,  ///< Positive semidefinite matrix
};

// Total ordering between FormulaKinds
bool operator<(FormulaKind k1, FormulaKind k2);

class FormulaCell;                  // In drake/common/symbolic_formula_cell.h
class FormulaFalse;                 // In drake/common/symbolic_formula_cell.h
class FormulaTrue;                  // In drake/common/symbolic_formula_cell.h
class FormulaVar;                   // In drake/common/symbolic_formula_cell.h
class RelationalFormulaCell;        // In drake/common/symbolic_formula_cell.h
class FormulaEq;                    // In drake/common/symbolic_formula_cell.h
class FormulaNeq;                   // In drake/common/symbolic_formula_cell.h
class FormulaGt;                    // In drake/common/symbolic_formula_cell.h
class FormulaGeq;                   // In drake/common/symbolic_formula_cell.h
class FormulaLt;                    // In drake/common/symbolic_formula_cell.h
class FormulaLeq;                   // In drake/common/symbolic_formula_cell.h
class NaryFormulaCell;              // In drake/common/symbolic_formula_cell.h
class FormulaNot;                   // In drake/common/symbolic_formula_cell.h
class FormulaAnd;                   // In drake/common/symbolic_formula_cell.h
class FormulaOr;                    // In drake/common/symbolic_formula_cell.h
class FormulaForall;                // In drake/common/symbolic_formula_cell.h
class FormulaIsnan;                 // In drake/common/symbolic_formula_cell.h
class FormulaPositiveSemidefinite;  // In drake/common/symbolic_formula_cell.h

/** Represents a symbolic form of a first-order logic formula.

It has the following grammar:

\verbatim
    F := ⊥ | ⊤ | Var | E = E | E ≠ E | E > E | E ≥ E | E < E | E ≤ E
       | E ∧ ... ∧ E | E ∨ ... ∨ E | ¬F | ∀ x₁, ..., xn. F
\endverbatim

In the implementation, Formula is a simple wrapper including a shared
pointer to FormulaCell class which is a super-class of different kinds
of symbolic formulas (i.e. FormulaAnd, FormulaOr, FormulaEq). Note
that it includes a shared pointer, not a unique pointer, to allow
sharing sub-expressions.

\note The sharing of sub-expressions is not yet implemented.

The following simple simplifications are implemented:
\verbatim
    E1 = E2        ->  True    (if E1 and E2 are structurally equal)
    E1 ≠ E2        ->  False   (if E1 and E2 are structurally equal)
    E1 > E2        ->  False   (if E1 and E2 are structurally equal)
    E1 ≥ E2        ->  True    (if E1 and E2 are structurally equal)
    E1 < E2        ->  False   (if E1 and E2 are structurally equal)
    E1 ≤ E2        ->  True    (if E1 and E2 are structurally equal)
    F1 ∧ F2        ->  False   (if either F1 or F2 is False)
    F1 ∨ F2        ->  True    (if either F1 or F2 is True)
    ¬(¬(F))        ->  F
\endverbatim

We flatten nested conjunctions (or disjunctions) at the construction. A
conjunction (resp. disjunction) takes a set of conjuncts (resp. disjuncts). Note
that any duplicated conjunct/disjunct is removed. For example, both of `f1 &&
(f2 && f1)` and `(f1 && f2) && f1` are flattened to `f1 && f2 && f1` and
simplified into `f1 && f2`. As a result, the two are identified as the same
formula.

\note Formula class has an explicit conversion operator to bool. It evaluates a
symbolic formula under an empty environment. If a symbolic formula includes
variables, the conversion operator throws an exception. This operator is only
intended for third-party code doing things like <tt>(imag(SymbolicExpression(0))
== SymbolicExpression(0)) { ... };<tt> that we found in Eigen3 codebase. In
general, a user of this class should explicitly call \c Evaluate from within
Drake for readability.

*/
class Formula {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Formula)

  /** Default constructor. */
  Formula() { *this = True(); }

  explicit Formula(std::shared_ptr<FormulaCell> ptr);

  /** Constructs a formula from @p var.
   * @pre @p var is of BOOLEAN type and not a dummy variable.
   */
  explicit Formula(const Variable& var);

  FormulaKind get_kind() const;
  size_t get_hash() const;
  /** Gets free variables (unquantified variables). */
  Variables GetFreeVariables() const;
  /** Checks structural equality*/
  bool EqualTo(const Formula& f) const;
  /** Checks lexicographical ordering between this and @p e.
   *
   * If the two formulas f1 and f2 have different kinds k1 and k2 respectively,
   * f1.Less(f2) is equal to k1 < k2. If f1 and f2 are expressions of the same
   * kind, we check the ordering between f1 and f2 by comparing their elements
   * lexicographically.
   *
   * For example, in case of And, let f1 and f2 be
   *
   *     f1 = f_1,1 ∧ ... ∧ f_1,n
   *     f2 = f_2,1 ∧ ... ∧ f_2,m
   *
   * f1.Less(f2) is true if there exists an index i (<= n, m) such that
   * for all j < i, we have
   *
   *     ¬(f_1_j.Less(f_2_j)) ∧ ¬(f_2_j.Less(f_1_j))
   *
   * and f_1_i.Less(f_2_i) holds.
   *
   * This function is used as a compare function in
   * std::map<symbolic::Formula> and std::set<symbolic::Formula> via
   * std::less<symbolic::Formula>. */
  bool Less(const Formula& f) const;

  /** Evaluates under a given environment (by default, an empty environment).
   *
   * @throws runtime_error if a variable `v` is needed for an evaluation but not
   * provided by @p env.
   *
   * Note that for an equality e₁ = e₂ and an inequality e₁ ≠ e₂, this method
   * partially evaluates e₁ and e₂ and checks the structural equality of the two
   * results if @p env does not provide complete information to call Evaluate on
   * e₁ and e₂.
   */
  bool Evaluate(const Environment& env = Environment{}) const;

  /** Returns a copy of this formula replacing all occurrences of @p var
   * with @p e.
   * @throws std::runtime_error if NaN is detected during substitution.
   */
  Formula Substitute(const Variable& var, const Expression& e) const;

  /** Returns a copy of this formula replacing all occurrences of the
   * variables in @p s with corresponding expressions in @p s. Note that the
   * substitutions occur simultaneously. For example, (x / y >
   * 0).Substitute({{x, y}, {y, x}}) gets (y / x > 0).
   * @throws std::runtime_error if NaN is detected during substitution.
   */
  Formula Substitute(const Substitution& s) const;

  /** Returns string representation of Formula. */
  std::string to_string() const;

  static Formula True();
  static Formula False();

  /** Conversion to bool. */
  explicit operator bool() const { return Evaluate(); }

  friend std::ostream& operator<<(std::ostream& os, const Formula& f);
  friend void swap(Formula& a, Formula& b) { std::swap(a.ptr_, b.ptr_); }

  friend bool is_false(const Formula& f);
  friend bool is_true(const Formula& f);
  friend bool is_variable(const Formula& f);
  friend bool is_equal_to(const Formula& f);
  friend bool is_not_equal_to(const Formula& f);
  friend bool is_greater_than(const Formula& f);
  friend bool is_greater_than_or_equal_to(const Formula& f);
  friend bool is_less_than(const Formula& f);
  friend bool is_less_than_or_equal_to(const Formula& f);
  friend bool is_relational(const Formula& f);
  friend bool is_conjunction(const Formula& f);
  friend bool is_disjunction(const Formula& f);
  friend bool is_negation(const Formula& f);
  friend bool is_forall(const Formula& f);
  friend bool is_isnan(const Formula& f);
  friend bool is_positive_semidefinite(const Formula& f);

  // Note that the following cast functions are only for low-level operations
  // and not exposed to the user of symbolic_formula.h. These functions are
  // declared in symbolic_formula_cell.h header.
  friend std::shared_ptr<FormulaFalse> to_false(const Formula& f);
  friend std::shared_ptr<FormulaTrue> to_true(const Formula& f);
  friend std::shared_ptr<FormulaVar> to_variable(const Formula& f);
  friend std::shared_ptr<RelationalFormulaCell> to_relational(const Formula& f);
  friend std::shared_ptr<FormulaEq> to_equal_to(const Formula& f);
  friend std::shared_ptr<FormulaNeq> to_not_equal_to(const Formula& f);
  friend std::shared_ptr<FormulaGt> to_greater_than(const Formula& f);
  friend std::shared_ptr<FormulaGeq> to_greater_than_or_equal_to(
      const Formula& f);
  friend std::shared_ptr<FormulaLt> to_less_than(const Formula& f);
  friend std::shared_ptr<FormulaLeq> to_less_than_or_equal_to(const Formula& f);
  friend std::shared_ptr<NaryFormulaCell> to_nary(const Formula& f);
  friend std::shared_ptr<FormulaAnd> to_conjunction(const Formula& f);
  friend std::shared_ptr<FormulaOr> to_disjunction(const Formula& f);
  friend std::shared_ptr<FormulaNot> to_negation(const Formula& f);
  friend std::shared_ptr<FormulaForall> to_forall(const Formula& f);
  friend std::shared_ptr<FormulaIsnan> to_isnan(const Formula& f);
  friend std::shared_ptr<FormulaPositiveSemidefinite> to_positive_semidefinite(
      const Formula& f);

 private:
  std::shared_ptr<FormulaCell> ptr_;
};

/** Returns a formula @p f, universally quantified by variables @p vars. */
Formula forall(const Variables& vars, const Formula& f);

/** Returns a conjunction of @p formulas. It performs the following
 * simplification:
 *
 * - make_conjunction({}) returns True.
 * - make_conjunction({f₁}) returns f₁.
 * - If False ∈ @p formulas, it returns False.
 * - If True ∈ @p formulas, it will not appear in the return value.
 * - Nested conjunctions will be flattened. For example, make_conjunction({f₁,
 *   f₂ ∧ f₃}) returns f₁ ∧ f₂ ∧ f₃.
 */
Formula make_conjunction(const std::set<Formula>& formulas);
Formula operator&&(const Formula& f1, const Formula& f2);
Formula operator&&(const Variable& v, const Formula& f);
Formula operator&&(const Formula& f, const Variable& v);
Formula operator&&(const Variable& v1, const Variable& v2);

/** Returns a disjunction of @p formulas. It performs the following
 * simplification:
 *
 * - make_disjunction({}) returns False.
 * - make_disjunction({f₁}) returns f₁.
 * - If True ∈ @p formulas, it returns True.
 * - If False ∈ @p formulas, it will not appear in the return value.
 * - Nested disjunctions will be flattened. For example, make_disjunction({f₁,
 *   f₂ ∨ f₃}) returns f₁ ∨ f₂ ∨ f₃.
 */
Formula make_disjunction(const std::set<Formula>& formulas);
Formula operator||(const Formula& f1, const Formula& f2);
Formula operator||(const Variable& v, const Formula& f);
Formula operator||(const Formula& f, const Variable& v);
Formula operator||(const Variable& v1, const Variable& v2);
Formula operator!(const Formula& f);
Formula operator!(const Variable& v);
Formula operator==(const Expression& e1, const Expression& e2);
Formula operator!=(const Expression& e1, const Expression& e2);
Formula operator<(const Expression& e1, const Expression& e2);
Formula operator<=(const Expression& e1, const Expression& e2);
Formula operator>(const Expression& e1, const Expression& e2);
Formula operator>=(const Expression& e1, const Expression& e2);

/** Returns a Formula for the predicate isnan(e) to the given expression. This
 * serves as the argument-dependent lookup related to std::isnan(double). When
 * evaluated, this Formula will return false when the e.Evaluate() is not NaN.
 * @throws std::runtime_error if NaN is detected during evaluation.
 */
Formula isnan(const Expression& e);

/** Returns a symbolic formula constraining @p m to be a positive-semidefinite
 * matrix. By definition, a symmetric matrix @p m is positive-semidefinte if xᵀ
 * m x ≥ 0 for all vector x ∈ ℝⁿ.
 *
 * @throws std::runtime_error if @p m is not symmetric.
 *
 * @note This method checks if @p m is symmetric by calling `math::IsSymmetric`
 * function which can be costly. If you want to avoid it, please consider using
 * `positive_semidefinite(m.triangularView<Eigen::Lower>())` or
 * `positive_semidefinite(m.triangularView<Eigen::Upper>())` instead of
 * `positive_semidefinite(m)`.
 */
Formula positive_semidefinite(const Eigen::Ref<const MatrixX<Expression>>& m);

/** Constructs and returns a symbolic positive-semidefinite formula from @p
 * m. If @p mode is Eigen::Lower, it's using the lower-triangular part of @p m
 * to construct a positive-semidefinite formula. If @p mode is Eigen::Upper, the
 * upper-triangular part of @p m is used. It throws std::runtime_error if @p has
 * other values. See the following code snippet.
 *
 * @code
 * Eigen::Matrix<Expression, 2, 2> m;
 * m << 1.0, 2.0,
 *      3.0, 4.0;
 *
 * const Formula psd_l{positive_semidefinite(m, Eigen::Lower)};
 * // psd_l includes [1.0 3.0]
 * //                [3.0 4.0].
 *
 * const Formula psd_u{positive_semidefinite(m, Eigen::Upper)};
 * // psd_u includes [1.0 2.0]
 * //                [2.0 4.0].
 * @endcode
 */
Formula positive_semidefinite(const MatrixX<Expression>& m,
                              Eigen::UpLoType mode);

/** Constructs and returns a symbolic positive-semidefinite formula from a lower
 * triangular-view @p l. See the following code snippet.
 *
 * @code
 * Eigen::Matrix<Expression, 2, 2> m;
 * m << 1.0, 2.0,
 *      3.0, 4.0;
 *
 * Formula psd{positive_semidefinite(m.triangularView<Eigen::Lower>())};
 * // psd includes [1.0 3.0]
 * //              [3.0 4.0].
 * @endcode
 */
template <typename Derived>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::MatrixXpr>::value &&
        std::is_same<typename Derived::Scalar, Expression>::value,
    Formula>::type
positive_semidefinite(const Eigen::TriangularView<Derived, Eigen::Lower>& l) {
  return positive_semidefinite(l, Eigen::Lower);
}

/** Constructs and returns a symbolic positive-semidefinite formula from an
 * upper triangular-view @p u. See the following code snippet.
 *
 * @code
 * Eigen::Matrix<Expression, 2, 2> m;
 * m << 1.0, 2.0,
 *      3.0, 4.0;
 *
 * Formula psd{positive_semidefinite(m.triangularView<Eigen::Upper>())};
 * // psd includes [1.0 2.0]
 * //              [2.0 4.0].
 * @endcode
 */
template <typename Derived>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::MatrixXpr>::value &&
        std::is_same<typename Derived::Scalar, Expression>::value,
    Formula>::type
positive_semidefinite(const Eigen::TriangularView<Derived, Eigen::Upper>& u) {
  return positive_semidefinite(u, Eigen::Upper);
}

std::ostream& operator<<(std::ostream& os, const Formula& f);

/** Checks if @p f is structurally equal to False formula. */
bool is_false(const Formula& f);
/** Checks if @p f is structurally equal to True formula. */
bool is_true(const Formula& f);
/** Checks if @p f is a variable formula. */
bool is_variable(const Formula& f);
/** Checks if @p f is a formula representing equality (==). */
bool is_equal_to(const Formula& f);
/** Checks if @p f is a formula representing disequality (!=). */
bool is_not_equal_to(const Formula& f);
/** Checks if @p f is a formula representing greater-than (>). */
bool is_greater_than(const Formula& f);
/** Checks if @p f is a formula representing greater-than-or-equal-to (>=). */
bool is_greater_than_or_equal_to(const Formula& f);
/** Checks if @p f is a formula representing less-than (<). */
bool is_less_than(const Formula& f);
/** Checks if @p f is a formula representing less-than-or-equal-to (<=). */
bool is_less_than_or_equal_to(const Formula& f);
/** Checks if @p f is a relational formula ({==, !=, >, >=, <, <=}). */
bool is_relational(const Formula& f);
/** Checks if @p f is a conjunction (∧). */
bool is_conjunction(const Formula& f);
/** Checks if @p f is a disjunction (∨). */
bool is_disjunction(const Formula& f);
/** Checks if @p f is a n-ary formula ({∧, ∨}). */
bool is_nary(const Formula& f);
/** Checks if @p f is a negation (¬). */
bool is_negation(const Formula& f);
/** Checks if @p f is a Forall formula (∀). */
bool is_forall(const Formula& f);
/** Checks if @p f is an isnan formula. */
bool is_isnan(const Formula& f);
/** Checks if @p f is a positive-semidefinite formula. */
bool is_positive_semidefinite(const Formula& f);

/** Returns the embedded variable in the variable formula @p f.
 *  @pre @p f is a variable formula.
 */
const Variable& get_variable(const Formula& f);

/** Returns the lhs-argument of a relational formula @p f.
 *  \pre{@p f is a relational formula.}
 */
const Expression& get_lhs_expression(const Formula& f);

/** Returns the rhs-argument of a relational formula @p f.
 *  \pre{@p f is a relational formula.}
 */
const Expression& get_rhs_expression(const Formula& f);

/** Returns the set of formulas in a n-ary formula @p f.
 *  \pre{@p f is a n-ary formula.}
 */
const std::set<Formula>& get_operands(const Formula& f);

/** Returns the formula in a negation formula @p f.
 *  \pre{@p f is a negation formula.}
 */
const Formula& get_operand(const Formula& f);

/** Returns the quantified variables in a forall formula @p f.
 *  \pre{@p f is a forall formula.}
 */
const Variables& get_quantified_variables(const Formula& f);

/** Returns the quantified formula in a forall formula @p f.
 *  \pre{@p f is a forall formula.}
 */
const Formula& get_quantified_formula(const Formula& f);

/** Returns the matrix in a positive-semidefinite formula @p f.
 *  \pre{@p f is a positive-semidefinite formula.}
 */
const MatrixX<Expression>& get_matrix_in_positive_semidefinite(
    const Formula& f);

namespace detail {
/// Provides a return type of relational operations (=, ≠, ≤, <, ≥, >) between
/// `Eigen::Array`s.
///
/// @tparam DerivedA A derived type of Eigen::ArrayBase.
/// @tparam DerivedB A derived type of Eigen::ArrayBase.
/// @pre The type of (DerivedA::Scalar() == DerivedB::Scalar()) is symbolic
/// formula.
template <typename DerivedA, typename DerivedB,
          typename = std::enable_if<
              std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                           Eigen::ArrayXpr>::value &&
              std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                           Eigen::ArrayXpr>::value &&
              std::is_same<decltype(typename DerivedA::Scalar() ==
                                    typename DerivedB::Scalar()),
                           Formula>::value>>
struct RelationalOpTraits {
  using ReturnType =
      Eigen::Array<Formula,
                   EigenSizeMinPreferFixed<DerivedA::RowsAtCompileTime,
                                           DerivedB::RowsAtCompileTime>::value,
                   EigenSizeMinPreferFixed<DerivedA::ColsAtCompileTime,
                                           DerivedB::ColsAtCompileTime>::value>;
};
/// Returns @p f1 ∧ @p f2. We have it because gcc-4.8 does not have
/// `std::logical_and`.
inline Formula logic_and(const Formula& f1, const Formula& f2) {
  return f1 && f2;
}
}  // namespace detail

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise symbolic-equality of two arrays @p m1 and @p m2.
///
/// The following table describes the return type of @p m1 == @p m2.
///
///    LHS \ RHS    | EA<Expression> | EA<Variable> | EA<double>
/// ----------------|----------------|--------------|--------------
///  EA<Expression> | EA<Formula>    | EA<Formula>  | EA<Formula>
///  EA<Variable>   | EA<Formula>    | EA<Formula>  | EA<Formula>
///  EA<double>     | EA<Formula>    | EA<Formula>  | EA<bool>
///
/// In the table, `EA` is a short-hand of `Eigen::Array`.
///
/// Note that this function does *not* provide operator overloading for the
/// following case. It returns `Eigen::Array<bool>` and is provided by Eigen.
///
///    - Eigen::Array<double> == Eigen::Array<double>
///
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() ==
                              typename DerivedB::Scalar()),
                     Formula>::value,
    typename detail::RelationalOpTraits<DerivedA, DerivedB>::ReturnType>::type
operator==(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  return a1.binaryExpr(a2, std::equal_to<void>());
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between an array @p a and a scalar @p v using
/// equal-to operator (==). That is, for all i and j, the (i, j)-th entry of `(a
/// == v)` has a symbolic formula `a(i, j) == v`.
///
/// Here is an example using this operator overloading.
/// @code
///     Eigen::Array<Variable, 2, 2> a;
///     a << Variable{"x"}, Variable{"y"},
///          Variable{"z"}, Variable{"w"};
///     Eigen::Array<Formula, 2, 2> f = (a == 3.5);
///     // Here f = |(x == 3.5)  (y == 3.5)|
///     //          |(z == 3.5)  (w == 3.5)|.
/// @endcode
template <typename Derived, typename ScalarType>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename Derived::Scalar() == ScalarType()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator==(const Derived& a, const ScalarType& v) {
  return a.unaryExpr(
      [&v](const typename Derived::Scalar& x) { return x == v; });
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between a scalar @p v and an array @p using equal-to
/// operator (==). That is, for all i and j, the (i, j)-th entry of `(v == a)`
/// has a symbolic formula `v == a(i, j)`.
///
/// Here is an example using this operator overloading.
/// @code
///     Eigen::Array<Variable, 2, 2> a;
///     a << Variable{"x"}, Variable{"y"},
///          Variable{"z"}, Variable{"w"};
///     Eigen::Array<Formula, 2, 2> f = (3.5 == a);
///     // Here f = |(3.5 == x)  (3.5 == y)|
///     //          |(3.5 == z)  (3.5 == w)|.
/// @endcode
template <typename ScalarType, typename Derived>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(ScalarType() == typename Derived::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator==(const ScalarType& v, const Derived& a) {
  return a.unaryExpr(
      [&v](const typename Derived::Scalar& x) { return v == x; });
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using
/// less-than-or-equal operator (<=).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() <=
                              typename DerivedB::Scalar()),
                     Formula>::value,
    typename detail::RelationalOpTraits<DerivedA, DerivedB>::ReturnType>::type
operator<=(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  return a1.binaryExpr(a2, std::less_equal<void>());
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between an array @p a and a scalar @p v using
/// less-than-or-equal operator (<=). That is, for all i and j, the (i, j)-th
/// entry of `(a <= v)` has a symbolic formula `a(i, j) <= v`.
template <typename Derived, typename ScalarType>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename Derived::Scalar() <= ScalarType()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator<=(const Derived& a, const ScalarType& v) {
  return a.unaryExpr(
      [&v](const typename Derived::Scalar& x) { return x <= v; });
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between a scalar @p v and an array @p using
/// less-than-or-equal operator (<=). That is, for all i and j, the (i, j)-th
/// entry of `(v <= a)` has a symbolic formula `v <= a(i, j)`.
template <typename ScalarType, typename Derived>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(ScalarType() <= typename Derived::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator<=(const ScalarType& v, const Derived& a) {
  return a.unaryExpr(
      [&v](const typename Derived::Scalar& x) { return v <= x; });
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using less-than
/// operator (<).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() <
                              typename DerivedB::Scalar()),
                     Formula>::value,
    typename detail::RelationalOpTraits<DerivedA, DerivedB>::ReturnType>::type
operator<(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  return a1.binaryExpr(a2, std::less<void>());
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between an array @p a and a scalar @p v using
/// less-than operator (<). That is, for all i and j, the (i, j)-th
/// entry of `(a < v)` has a symbolic formula `a(i, j) < v`.
template <typename Derived, typename ScalarType>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename Derived::Scalar() < ScalarType()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator<(const Derived& a, const ScalarType& v) {
  return a.unaryExpr([&v](const typename Derived::Scalar& x) { return x < v; });
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between a scalar @p v and an array @p using
/// less-than operator (<). That is, for all i and j, the (i, j)-th
/// entry of `(v < a)` has a symbolic formula `v < a(i, j)`.
template <typename ScalarType, typename Derived>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(ScalarType() < typename Derived::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator<(const ScalarType& v, const Derived& a) {
  return a.unaryExpr([&v](const typename Derived::Scalar& x) { return v < x; });
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using
/// greater-than-or-equal operator (>=).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() >=
                              typename DerivedB::Scalar()),
                     Formula>::value,
    typename detail::RelationalOpTraits<DerivedA, DerivedB>::ReturnType>::type
operator>=(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  return a1.binaryExpr(a2, std::greater_equal<void>());
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between an array @p a and a scalar @p v using
/// greater-than-or-equal operator (>=). That is, for all i and j, the (i, j)-th
/// entry of `(a >= v)` has a symbolic formula `a(i, j) >= v`.
template <typename Derived, typename ScalarType>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename Derived::Scalar() >= ScalarType()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator>=(const Derived& a, const ScalarType& v) {
  return a.unaryExpr(
      [&v](const typename Derived::Scalar& x) { return x >= v; });
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between a scalar @p v and an array @p using
/// less-than-or-equal operator (<=) instead of greater-than-or-equal operator
/// (>=). That is, for all i and j, the (i, j)-th entry of `(v >= a)` has a
/// symbolic formula `a(i, j) <= v`.
///
/// Note that given `v >= a`, this methods returns the result of `a <= v`. First
/// of all, this formulation is mathematically equivalent to the original
/// formulation. We implement this method in this way to be consistent with
/// Eigen's semantics. See the definition of `EIGEN_MAKE_CWISE_COMP_R_OP` in
/// ArrayCwiseBinaryOps.h file in Eigen.
template <typename ScalarType, typename Derived>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(ScalarType() >= typename Derived::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator>=(const ScalarType& v, const Derived& a) {
  return a <= v;
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using greater-than
/// operator (>).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() >
                              typename DerivedB::Scalar()),
                     Formula>::value,
    typename detail::RelationalOpTraits<DerivedA, DerivedB>::ReturnType>::type
operator>(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  return a1.binaryExpr(a2, std::greater<void>());
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between an array @p a and a scalar @p v using
/// greater-than operator (>). That is, for all i and j, the (i, j)-th
/// entry of `(a > v)` has a symbolic formula `a(i, j) > v`.
template <typename Derived, typename ScalarType>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename Derived::Scalar() > ScalarType()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator>(const Derived& a, const ScalarType& v) {
  return a.unaryExpr([&v](const typename Derived::Scalar& x) { return x > v; });
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between a scalar @p v and an array @p using
/// less-than operator (<) instead of greater-than operator (>). That is, for
/// all i and j, the (i, j)-th entry of `(v > a)` has a symbolic formula `a(i,
/// j) < v`.
///
/// Note that given `v > a`, this methods returns the result of `a < v`. First
/// of all, this formulation is mathematically equivalent to the original
/// formulation. We implement this method in this way to be consistent with
/// Eigen's semantics. See the definition of `EIGEN_MAKE_CWISE_COMP_R_OP` in
/// ArrayCwiseBinaryOps.h file in Eigen.
template <typename ScalarType, typename Derived>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(ScalarType() > typename Derived::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator>(const ScalarType& v, const Derived& a) {
  return a < v;
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using not-equal
/// operator (!=).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() !=
                              typename DerivedB::Scalar()),
                     Formula>::value,
    typename detail::RelationalOpTraits<DerivedA, DerivedB>::ReturnType>::type
operator!=(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  return a1.binaryExpr(a2, std::not_equal_to<void>());
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between an array @p a and a scalar @p v using
/// not-equal operator (!=). That is, for all i and j, the (i, j)-th
/// entry of `(a != v)` has a symbolic formula `a(i, j) != v`.
template <typename Derived, typename ScalarType>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(typename Derived::Scalar() != ScalarType()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator!=(const Derived& a, const ScalarType& v) {
  return a.unaryExpr(
      [&v](const typename Derived::Scalar& x) { return x != v; });
}

/// Returns an Eigen array of symbolic formulas where each element includes
/// element-wise comparison between a scalar @p v and an array @p using
/// not-equal operator (!=). That is, for all i and j, the (i, j)-th
/// entry of `(v != a)` has a symbolic formula `v != a(i, j)`.
template <typename ScalarType, typename Derived>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<Derived>::XprKind,
                 Eigen::ArrayXpr>::value &&
        std::is_same<decltype(ScalarType() != typename Derived::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula, Derived::RowsAtCompileTime,
                 Derived::ColsAtCompileTime>>::type
operator!=(const ScalarType& v, const Derived& a) {
  return a.unaryExpr(
      [&v](const typename Derived::Scalar& x) { return v != x; });
}

/// Returns a symbolic formula checking if two matrices @p m1 and @p m2 are
/// equal.
///
/// The following table describes the return type of @p m1 == @p m2.
///
///    LHS \ RHS    | EM<Expression> | EM<Variable> | EM<double>
/// ----------------|----------------|--------------|------------
///  EM<Expression> | Formula        | Formula      | Formula
///  EM<Variable>   | Formula        | Formula      | Formula
///  EM<double>     | Formula        | Formula      | bool
///
/// In the table, `EM` is a short-hand of `Eigen::Matrix`.
///
/// Note that this function does *not* provide operator overloading for the
/// following case. It returns `bool` and is provided by Eigen.
///
///    - Eigen::Matrix<double> == Eigen::Matrix<double>
///
/// Note that this method returns a conjunctive formula which keeps its
/// conjuncts as `std::set<Formula>` internally. This set is ordered by
/// `Formula::Less` and this ordering can be *different* from the one in
/// inputs. Also, any duplicated formulas are removed in construction.  Please
/// check the following example.
///
/// @code
///     // set up v1 = [y x y] and v2 = [1 2 1]
///     VectorX<Expression> v1{3};
///     VectorX<Expression> v2{3};
///     const Variable x{"x"};
///     const Variable y{"y"};
///     v1 << y, x, y;
///     v2 << 1, 2, 1;
///     // Here v1_eq_v2 = ((x = 2) ∧ (y = 1))
///     const Formula v1_eq_v2{v1 == v2};
///     const std::set<Formula> conjuncts{get_operands(v1_eq_v2)};
///     for (const Formula& f : conjuncts) {
///       std::cerr << f << std::endl;
///     }
///     // The outcome of the above loop is:
///     (x = 2)
///     (y = 1)
/// @endcode
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::MatrixXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::MatrixXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() ==
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Formula>::type
operator==(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  return m1.binaryExpr(m2, std::equal_to<void>()).redux(detail::logic_and);
}

/// Returns a symbolic formula representing element-wise comparison between two
/// matrices @p m1 and @p m2 using not-equal (!=) operator.
///
/// The following table describes the return type of @p m1 != @p m2.
///
///    LHS \ RHS    | EM<Expression> | EM<Variable> | EM<double>
/// ----------------|----------------|--------------|------------
///  EM<Expression> | Formula        | Formula      | Formula
///  EM<Variable>   | Formula        | Formula      | Formula
///  EM<double>     | Formula        | Formula      | bool
///
/// In the table, `EM` is a short-hand of `Eigen::Matrix`.
///
/// Note that this function does *not* provide operator overloading for the
/// following case. It returns `bool` and is provided by Eigen.
///
///    - Eigen::Matrix<double> != Eigen::Matrix<double>
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::MatrixXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::MatrixXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() !=
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Formula>::type
operator!=(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  return m1.binaryExpr(m2, std::not_equal_to<void>()).redux(detail::logic_and);
}

/// Returns a symbolic formula representing element-wise comparison between two
/// matrices @p m1 and @p m2 using less-than (<) operator.
///
/// The following table describes the return type of @p m1 < @p m2.
///
///    LHS \ RHS    | EM<Expression> | EM<Variable> | EM<double>
/// ----------------|----------------|--------------|------------
///  EM<Expression> | Formula        | Formula      | Formula
///  EM<Variable>   | Formula        | Formula      | Formula
///  EM<double>     | Formula        | Formula      | N/A
///
/// In the table, `EM` is a short-hand of `Eigen::Matrix`.
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::MatrixXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::MatrixXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() <
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Formula>::type
operator<(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  return m1.binaryExpr(m2, std::less<void>()).redux(detail::logic_and);
}

/// Returns a symbolic formula representing element-wise comparison between two
/// matrices @p m1 and @p m2 using less-than-or-equal operator (<=).
///
/// The following table describes the return type of @p m1 <= @p m2.
///
///    LHS \ RHS    | EM<Expression> | EM<Variable> | EM<double>
/// ----------------|----------------|--------------|------------
///  EM<Expression> | Formula        | Formula      | Formula
///  EM<Variable>   | Formula        | Formula      | Formula
///  EM<double>     | Formula        | Formula      | N/A
///
/// In the table, `EM` is a short-hand of `Eigen::Matrix`.
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::MatrixXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::MatrixXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() <=
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Formula>::type
operator<=(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  return m1.binaryExpr(m2, std::less_equal<void>()).redux(detail::logic_and);
}

/// Returns a symbolic formula representing element-wise comparison between two
/// matrices @p m1 and @p m2 using greater-than operator (>).
///
/// The following table describes the return type of @p m1 > @p m2.
///
///    LHS \ RHS    | EM<Expression> | EM<Variable> | EM<double>
/// ----------------|----------------|--------------|------------
///  EM<Expression> | Formula        | Formula      | Formula
///  EM<Variable>   | Formula        | Formula      | Formula
///  EM<double>     | Formula        | Formula      | N/A
///
/// In the table, `EM` is a short-hand of `Eigen::Matrix`.
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::MatrixXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::MatrixXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() >
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Formula>::type
operator>(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  return m1.binaryExpr(m2, std::greater<void>()).redux(detail::logic_and);
}

/// Returns a symbolic formula representing element-wise comparison between two
/// matrices @p m1 and @p m2 using greater-than-or-equal operator (>=).
///
/// The following table describes the return type of @p m1 >= @p m2.
///
///    LHS \ RHS    | EM<Expression> | EM<Variable> | EM<double>
/// ----------------|----------------|--------------|------------
///  EM<Expression> | Formula        | Formula      | Formula
///  EM<Variable>   | Formula        | Formula      | Formula
///  EM<double>     | Formula        | Formula      | N/A
///
/// In the table, `EM` is a short-hand of `Eigen::Matrix`.
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same<typename Eigen::internal::traits<DerivedA>::XprKind,
                 Eigen::MatrixXpr>::value &&
        std::is_same<typename Eigen::internal::traits<DerivedB>::XprKind,
                     Eigen::MatrixXpr>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() >=
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Formula>::type
operator>=(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  return m1.binaryExpr(m2, std::greater_equal<void>()).redux(detail::logic_and);
}

}  // namespace symbolic

/** Computes the hash value of a symbolic formula. */
template <>
struct hash_value<symbolic::Formula> {
  size_t operator()(const symbolic::Formula& f) const { return f.get_hash(); }
};

namespace assert {
/* We allow assertion-like statements to receive a Formula.  Given the typical
 * uses of assertions and Formulas, rather than trying to be clever and, e.g.,
 * capture assertion data for later use or partially-solve the Formula to find
 * counterexamples, instead we've decided to ignore assertions for the purpose
 * of Formula.  They are syntax-checked, but always pass. */
template <>
struct ConditionTraits<symbolic::Formula> {
  static constexpr bool is_valid = true;
  static bool Evaluate(const symbolic::Formula&) { return true; }
};
}  // namespace assert

}  // namespace drake

namespace std {
/* Provides std::less<drake::symbolic::Formula>. */
template <>
struct less<drake::symbolic::Formula> {
  bool operator()(const drake::symbolic::Formula& lhs,
                  const drake::symbolic::Formula& rhs) const {
    return lhs.Less(rhs);
  }
};

/* Provides std::equal_to<drake::symbolic::Formula>. */
template <>
struct equal_to<drake::symbolic::Formula> {
  bool operator()(const drake::symbolic::Formula& lhs,
                  const drake::symbolic::Formula& rhs) const {
    return lhs.EqualTo(rhs);
  }
};
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
// Define Eigen traits needed for Matrix<drake::symbolic::Formula>.
namespace Eigen {
// Eigen scalar type traits for Matrix<drake::symbolic::Formula>.
template <>
struct NumTraits<drake::symbolic::Formula>
    : GenericNumTraits<drake::symbolic::Formula> {
  static inline int digits10() { return 0; }
};

namespace internal {

/// Provides specialization for scalar_cmp_op to handle the case "Expr == Expr"
template <>
struct scalar_cmp_op<drake::symbolic::Expression, drake::symbolic::Expression,
                     cmp_EQ>
    : binary_op_base<drake::symbolic::Expression, drake::symbolic::Expression> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Expression& a,
             const drake::symbolic::Expression& b) const {
    return a == b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Expr < Expr".
template <>
struct scalar_cmp_op<drake::symbolic::Expression, drake::symbolic::Expression,
                     cmp_LT>
    : binary_op_base<drake::symbolic::Expression, drake::symbolic::Expression> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Expression& a,
             const drake::symbolic::Expression& b) const {
    return a < b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Expr <= Expr".
template <>
struct scalar_cmp_op<drake::symbolic::Expression, drake::symbolic::Expression,
                     cmp_LE>
    : binary_op_base<drake::symbolic::Expression, drake::symbolic::Expression> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Expression& a,
             const drake::symbolic::Expression& b) const {
    return a <= b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Expr > Expr".
template <>
struct scalar_cmp_op<drake::symbolic::Expression, drake::symbolic::Expression,
                     cmp_GT>
    : binary_op_base<drake::symbolic::Expression, drake::symbolic::Expression> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Expression& a,
             const drake::symbolic::Expression& b) const {
    return a > b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Expr >= Expr".
template <>
struct scalar_cmp_op<drake::symbolic::Expression, drake::symbolic::Expression,
                     cmp_GE>
    : binary_op_base<drake::symbolic::Expression, drake::symbolic::Expression> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Expression& a,
             const drake::symbolic::Expression& b) const {
    return a >= b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Expr != Expr".
template <>
struct scalar_cmp_op<drake::symbolic::Expression, drake::symbolic::Expression,
                     cmp_NEQ>
    : binary_op_base<drake::symbolic::Expression, drake::symbolic::Expression> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Expression& a,
             const drake::symbolic::Expression& b) const {
    return a != b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Var == Var".
template <>
struct scalar_cmp_op<drake::symbolic::Variable, drake::symbolic::Variable,
                     cmp_EQ>
    : binary_op_base<drake::symbolic::Variable, drake::symbolic::Variable> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Variable& a,
             const drake::symbolic::Variable& b) const {
    return a == b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Var < Var".
template <>
struct scalar_cmp_op<drake::symbolic::Variable, drake::symbolic::Variable,
                     cmp_LT>
    : binary_op_base<drake::symbolic::Variable, drake::symbolic::Variable> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Variable& a,
             const drake::symbolic::Variable& b) const {
    return a < b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Var <= Var".
template <>
struct scalar_cmp_op<drake::symbolic::Variable, drake::symbolic::Variable,
                     cmp_LE>
    : binary_op_base<drake::symbolic::Variable, drake::symbolic::Variable> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Variable& a,
             const drake::symbolic::Variable& b) const {
    return a <= b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Var > Var".
template <>
struct scalar_cmp_op<drake::symbolic::Variable, drake::symbolic::Variable,
                     cmp_GT>
    : binary_op_base<drake::symbolic::Variable, drake::symbolic::Variable> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Variable& a,
             const drake::symbolic::Variable& b) const {
    return a > b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Var >= Var".
template <>
struct scalar_cmp_op<drake::symbolic::Variable, drake::symbolic::Variable,
                     cmp_GE>
    : binary_op_base<drake::symbolic::Variable, drake::symbolic::Variable> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Variable& a,
             const drake::symbolic::Variable& b) const {
    return a >= b;
  }
};

/// Provides specialization for scalar_cmp_op to handle the case "Var != Var".
template <>
struct scalar_cmp_op<drake::symbolic::Variable, drake::symbolic::Variable,
                     cmp_NEQ>
    : binary_op_base<drake::symbolic::Variable, drake::symbolic::Variable> {
  typedef drake::symbolic::Formula result_type;
  EIGEN_EMPTY_STRUCT_CTOR(scalar_cmp_op)
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE result_type
  operator()(const drake::symbolic::Variable& a,
             const drake::symbolic::Variable& b) const {
    return a != b;
  }
};

}  // namespace internal
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
