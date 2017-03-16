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
#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

/** Kinds of symbolic formulas. */
enum class FormulaKind {
  False,   ///< ⊥
  True,    ///< ⊤
  Eq,      ///< =
  Neq,     ///< !=
  Gt,      ///< >
  Geq,     ///< >=
  Lt,      ///< <
  Leq,     ///< <=
  And,     ///< Conjunction (∧)
  Or,      ///< Disjunction (∨)
  Not,     ///< Negation (¬)
  Forall,  ///< Universal quantification (∀)
  Isnan,   ///< NaN check predicate
};

// Total ordering between FormulaKinds
bool operator<(FormulaKind k1, FormulaKind k2);

class FormulaCell;            // In drake/common/symbolic_formula_cell.h
class RelationalFormulaCell;  // In drake/common/symbolic_formula_cell.h
class NaryFormulaCell;        // In drake/common/symbolic_formula_cell.h
class FormulaNot;             // In drake/common/symbolic_formula_cell.h
class FormulaForall;          // In drake/common/symbolic_formula_cell.h
class FormulaIsnan;           // In drake/common/symbolic_formula_cell.h

/** Represents a symbolic form of a first-order logic formula.

It has the following grammar:

\verbatim
    F := ⊥ | ⊤ | E = E | E ≠ E | E > E | E ≥ E | E < E | E ≤ E
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
\endverbatim

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

  explicit Formula(const std::shared_ptr<FormulaCell> ptr);

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
  /** Evaluates under a given environment (by default, an empty environment)*/
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

  friend Formula operator&&(const Formula& f1, const Formula& f2);
  friend Formula operator||(const Formula& f1, const Formula& f2);
  friend Formula operator!(const Formula& f);
  friend Formula operator==(const Expression& e1, const Expression& e2);
  friend Formula operator!=(const Expression& e1, const Expression& e2);
  friend Formula operator<(const Expression& e1, const Expression& e2);
  friend Formula operator<=(const Expression& e1, const Expression& e2);
  friend Formula operator>(const Expression& e1, const Expression& e2);
  friend Formula operator>=(const Expression& e1, const Expression& e2);

  friend std::ostream& operator<<(std::ostream& os, const Formula& f);
  friend void swap(Formula& a, Formula& b) { std::swap(a.ptr_, b.ptr_); }

  friend bool is_false(const Formula& f);
  friend bool is_true(const Formula& f);
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

  // Note that the following cast functions are only for low-level operations
  // and not exposed to the user of symbolic_formula.h. These functions are
  // declared in symbolic_formula_cell.h header.
  friend std::shared_ptr<RelationalFormulaCell> to_relational(const Formula& f);
  friend std::shared_ptr<NaryFormulaCell> to_nary(const Formula& f);
  friend std::shared_ptr<FormulaNot> to_negation(const Formula& f);
  friend std::shared_ptr<FormulaForall> to_forall(const Formula& f);
  friend std::shared_ptr<FormulaIsnan> to_isnan(const Formula& f);

 private:
  std::shared_ptr<FormulaCell> ptr_;
};

/** Returns a formula @p f, universally quantified by variables @p vars. */
Formula forall(const Variables& vars, const Formula& f);

Formula operator&&(const Formula& f1, const Formula& f2);
Formula operator||(const Formula& f1, const Formula& f2);
Formula operator!(const Formula& f);
Formula operator==(const Expression& e1, const Expression& e2);
Formula operator!=(const Expression& e1, const Expression& e2);
Formula operator<(const Expression& e1, const Expression& e2);
Formula operator<=(const Expression& e1, const Expression& e2);
Formula operator>(const Expression& e1, const Expression& e2);
Formula operator>=(const Expression& e1, const Expression& e2);

Formula operator==(const Variable& v1, const Variable& v2);
Formula operator!=(const Variable& v1, const Variable& v2);
Formula operator<(const Variable& v1, const Variable& v2);
Formula operator<=(const Variable& v1, const Variable& v2);
Formula operator>(const Variable& v1, const Variable& v2);
Formula operator>=(const Variable& v1, const Variable& v2);

/** Returns a Formula for the predicate isnan(e) to the given expression. This
 * serves as the argument-dependent lookup related to std::isnan(double). When
 * evaluated, this Formula will return false when the e.Evaluate() is not NaN.
 * @throws std::runtime_error if NaN is detected during evaluation.
 */
Formula isnan(const Expression& e);

std::ostream& operator<<(std::ostream& os, const Formula& f);

/** Checks if @p f is structurally equal to False formula. */
bool is_false(const Formula& f);
/** Checks if @p f is structurally equal to True formula. */
bool is_true(const Formula& f);
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

/// Returns an Eigen array of symbolic formula where each element includes
/// element-wise symbolic-equality of two arrays @p m1 and @p m2.
///
/// The following table describes the return type of @p m1 == @p m2.
/// +--------------------------------------------------------------+
/// |   LHS \ RHS    | EA<Expression> | EA<Variable> | EA<double>  |
/// -----------------+----------------+--------------+--------------
/// | EA<Expression> | EA<Formula>    | EA<Formula>  | EA<Formula> |
/// -----------------+----------------+--------------+--------------
/// | EA<Variable>   | EA<Formula>    | EA<Formula>  | EA<Formula> |
/// -----------------+----------------+--------------+--------------
/// | EA<double>     | EA<Formula>    | EA<Formula>  | EA<bool>    |
/// +--------------------------------------------------------------+
///                (EA is a short-hand of Eigen::Array)
///
/// Note that this function does *not* provide operator overloading for the
/// following case. It returns `Eigen::Array<bool>` and is provided by Eigen.
///
///    - Eigen::Array<double> == Eigen::Array<double>
///
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() ==
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula,
                 DerivedA::RowsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::RowsAtCompileTime
                     : DerivedA::RowsAtCompileTime,
                 DerivedA::ColsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::ColsAtCompileTime
                     : DerivedA::ColsAtCompileTime>>::type
operator==(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  const auto equal = [](const auto& e1, const auto& e2) { return e1 == e2; };
  return a1.binaryExpr(a2, equal);
}

/// Returns an Eigen array of symbolic formula where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using
/// less-than-or-equal operator (<=).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() >=
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula,
                 DerivedA::RowsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::RowsAtCompileTime
                     : DerivedA::RowsAtCompileTime,
                 DerivedA::ColsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::ColsAtCompileTime
                     : DerivedA::ColsAtCompileTime>>::type
operator<=(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  const auto lte = [](const auto& e1, const auto& e2) { return e1 <= e2; };
  return a1.binaryExpr(a2, lte);
}

/// Returns an Eigen array of symbolic formula where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using
/// less-than operator (<).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() >
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula,
                 DerivedA::RowsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::RowsAtCompileTime
                     : DerivedA::RowsAtCompileTime,
                 DerivedA::ColsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::ColsAtCompileTime
                     : DerivedA::ColsAtCompileTime>>::type
operator<(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  const auto lt = [](const auto& e1, const auto& e2) { return e1 < e2; };
  return a1.binaryExpr(a2, lt);
}

/// Returns an Eigen array of symbolic formula where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using
/// greater-than-or-equal operator (>=).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() >=
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula,
                 DerivedA::RowsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::RowsAtCompileTime
                     : DerivedA::RowsAtCompileTime,
                 DerivedA::ColsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::ColsAtCompileTime
                     : DerivedA::ColsAtCompileTime>>::type
operator>=(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  const auto gte = [](const auto& e1, const auto& e2) { return e1 >= e2; };
  return a1.binaryExpr(a2, gte);
}

/// Returns an Eigen array of symbolic formula where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using
/// greater-than operator (>).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() >
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula,
                 DerivedA::RowsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::RowsAtCompileTime
                     : DerivedA::RowsAtCompileTime,
                 DerivedA::ColsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::ColsAtCompileTime
                     : DerivedA::ColsAtCompileTime>>::type
operator>(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  const auto gt = [](const auto& e1, const auto& e2) { return e1 > e2; };
  return a1.binaryExpr(a2, gt);
}

/// Returns an Eigen array of symbolic formula where each element includes
/// element-wise comparison of two arrays @p a1 and @p a2 using
/// not-equal operator (!=).
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::ArrayBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::ArrayBase<DerivedB>, DerivedB>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() >
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Eigen::Array<Formula,
                 DerivedA::RowsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::RowsAtCompileTime
                     : DerivedA::RowsAtCompileTime,
                 DerivedA::ColsAtCompileTime == Eigen::Dynamic
                     ? DerivedB::ColsAtCompileTime
                     : DerivedA::ColsAtCompileTime>>::type
operator!=(const DerivedA& a1, const DerivedB& a2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(a1.rows() == a2.rows() && a1.cols() == a2.cols());
  const auto neq = [](const auto& e1, const auto& e2) { return e1 != e2; };
  return a1.binaryExpr(a2, neq);
}

/// Returns a symbolic formula checking if two matrices @p m1 and @p m2 are
/// equal.
///
/// The following table describes the return type of @p m1 == @p m2.
/// +-------------------------------------------------------------+
/// |   LHS \ RHS    | EM<Expression> | EM<Variable> | EM<double> |
/// -----------------+----------------+--------------+-------------
/// | EM<Expression> | Formula        | Formula      | Formula    |
/// -----------------+----------------+--------------+-------------
/// | EM<Variable>   | Formula        | Formula      | Formula    |
/// -----------------+----------------+--------------+-------------
/// | EM<double>     | Formula        | Formula      | bool       |
/// +-------------------------------------------------------------+
///                (EM is a short-hand of Eigen::Matrix)
///
/// Note that this function does *not* provide operator overloading for the
/// following case. It returns `bool` and is provided by Eigen.
///
///    - Eigen::Matrix<double> == Eigen::Matrix<double>
///
template <typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<DerivedA>, DerivedA>::value &&
        std::is_base_of<Eigen::MatrixBase<DerivedB>, DerivedB>::value &&
        std::is_same<decltype(typename DerivedA::Scalar() ==
                              typename DerivedB::Scalar()),
                     Formula>::value,
    Formula>::type
operator==(const DerivedA& m1, const DerivedB& m2) {
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DerivedA, DerivedB);
  DRAKE_DEMAND(m1.rows() == m2.rows() && m1.cols() == m2.cols());
  const auto equal = [](const auto& e1, const auto& e2) { return e1 == e2; };
  const auto logic_and = [](const Formula& f1, const Formula& f2) {
    return f1 && f2;
  };
  return m1.binaryExpr(m2, equal).redux(logic_and);
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
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
