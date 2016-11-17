#pragma once

#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

/** Kinds of symbolic formulas. */
enum class FormulaKind {
  False,  ///< ⊥
  True,   ///< ⊤
  Eq,     ///< =
  Neq,    ///< !=
  Gt,     ///< >
  Geq,    ///< >=
  Lt,     ///< <
  Leq,    ///< <=
  And,    ///< Conjunction (∧)
  Or,     ///< Disjunction (∨)
  Not,    ///< Negation (¬)
  Forall  ///< Universal quantification (∀)
};

// Total ordering between FormulaKinds
bool operator<(FormulaKind k1, FormulaKind k2);

class FormulaCell;  // In drake/common/symbolic_formula_cell.h

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
*/
class Formula {
 public:
  /** Default constructor (deleted). */
  Formula() = delete;

  /** Move-construct a set from an rvalue. */
  Formula(Formula&& f) = default;

  /** Copy-construct a set from an lvalue. */
  Formula(const Formula& f) = default;

  /** Move-assign a set from an rvalue. */
  Formula& operator=(Formula&& f) = default;

  /** Copy-assign a set from an lvalue. */
  Formula& operator=(const Formula& f) = default;

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

  /** Returns string representation of Formula. */
  std::string to_string() const;

  static Formula True();
  static Formula False();

  friend Formula operator&&(const Formula& f1, const Formula& f2);
  friend Formula operator||(const Formula& f1, const Formula& f2);
  friend Formula operator!(const Formula& f);
  friend Formula operator==(const Expression& e1, const Expression& e2);
  friend Formula operator==(double v1, const Expression& e2);
  friend Formula operator==(const Expression& e1, double v2);
  friend Formula operator!=(const Expression& e1, const Expression& e2);
  friend Formula operator!=(double v1, const Expression& e2);
  friend Formula operator!=(const Expression& e1, double v2);
  friend Formula operator<(const Expression& e1, const Expression& e2);
  friend Formula operator<(double v1, const Expression& e2);
  friend Formula operator<(const Expression& e1, double v2);
  friend Formula operator<=(const Expression& e1, const Expression& e2);
  friend Formula operator<=(double v1, const Expression& e2);
  friend Formula operator<=(const Expression& e1, double v2);
  friend Formula operator>(const Expression& e1, const Expression& e2);
  friend Formula operator>(double v1, const Expression& e2);
  friend Formula operator>(const Expression& e1, double v2);
  friend Formula operator>=(const Expression& e1, const Expression& e2);
  friend Formula operator>=(double v1, const Expression& e2);
  friend Formula operator>=(const Expression& e1, double v2);

  friend std::ostream& operator<<(std::ostream& os, const Formula& f);
  friend void swap(Formula& a, Formula& b) { std::swap(a.ptr_, b.ptr_); }

 private:
  std::shared_ptr<FormulaCell> ptr_;
};

/** Returns a formula @p f, universally quantified by variables @p vars. */
Formula forall(const Variables& vars, const Formula& f);

Formula operator&&(const Formula& f1, const Formula& f2);
Formula operator||(const Formula& f1, const Formula& f2);
Formula operator!(const Formula& f);
Formula operator==(const Expression& e1, const Expression& e2);
Formula operator==(double v1, const Expression& e2);
Formula operator==(const Expression& e1, double v2);
Formula operator!=(const Expression& e1, const Expression& e2);
Formula operator!=(double v1, const Expression& e2);
Formula operator!=(const Expression& e1, double v2);
Formula operator<(const Expression& e1, const Expression& e2);
Formula operator<(double v1, const Expression& e2);
Formula operator<(const Expression& e1, double v2);
Formula operator<=(const Expression& e1, const Expression& e2);
Formula operator<=(double v1, const Expression& e2);
Formula operator<=(const Expression& e1, double v2);
Formula operator>(const Expression& e1, const Expression& e2);
Formula operator>(double v1, const Expression& e2);
Formula operator>(const Expression& e1, double v2);
Formula operator>=(const Expression& e1, const Expression& e2);
Formula operator>=(double v1, const Expression& e2);
Formula operator>=(const Expression& e1, double v2);

std::ostream& operator<<(std::ostream& os, const Formula& e);

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
