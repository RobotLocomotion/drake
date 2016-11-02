#pragma once

#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

/** Kinds of symbolic formulas. */
enum class FormulaKind {
  True,   ///< ⊤
  False,  ///< ⊥
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

class FormulaCell;

/** Represents a symbolic form of a first-order logic formula.

It has the following grammar:

\verbatim
    F := ⊤ | ⊥ | E = E | E ≠ E | E > E | E ≥ E | E < E | E ≤ E
       | E ∧ E | E ∨ E | ¬F | ∀ x₁, ..., xn. F
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
class DRAKE_EXPORT Formula {
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
  /** Evaluates under a given environment (by default, an empty environment)*/
  bool Evaluate(const Environment& env = Environment{}) const;

  /** Returns string representation of Formula. */
  std::string to_string() const;

  static Formula True();
  static Formula False();

  friend DRAKE_EXPORT Formula operator&&(const Formula& f1, const Formula& f2);
  friend DRAKE_EXPORT Formula operator||(const Formula& f1, const Formula& f2);
  friend DRAKE_EXPORT Formula operator!(const Formula& f);
  friend DRAKE_EXPORT bool operator==(const Formula& f1, const Formula& f2);
  friend DRAKE_EXPORT Formula operator==(const Expression& e1,
                                         const Expression& e2);
  friend DRAKE_EXPORT Formula operator==(double v1, const Expression& e2);
  friend DRAKE_EXPORT Formula operator==(const Expression& e1, double v2);
  friend DRAKE_EXPORT bool operator!=(const Formula& f1, const Formula& f2);
  friend DRAKE_EXPORT Formula operator!=(const Expression& e1,
                                         const Expression& e2);
  friend DRAKE_EXPORT Formula operator!=(double v1, const Expression& e2);
  friend DRAKE_EXPORT Formula operator!=(const Expression& e1, double v2);
  friend DRAKE_EXPORT Formula operator<(const Expression& e1,
                                        const Expression& e2);
  friend DRAKE_EXPORT Formula operator<(double v1, const Expression& e2);
  friend DRAKE_EXPORT Formula operator<(const Expression& e1, double v2);
  friend DRAKE_EXPORT Formula operator<=(const Expression& e1,
                                         const Expression& e2);
  friend DRAKE_EXPORT Formula operator<=(double v1, const Expression& e2);
  friend DRAKE_EXPORT Formula operator<=(const Expression& e1, double v2);
  friend DRAKE_EXPORT Formula operator>(const Expression& e1,
                                        const Expression& e2);
  friend DRAKE_EXPORT Formula operator>(double v1, const Expression& e2);
  friend DRAKE_EXPORT Formula operator>(const Expression& e1, double v2);
  friend DRAKE_EXPORT Formula operator>=(const Expression& e1,
                                         const Expression& e2);
  friend DRAKE_EXPORT Formula operator>=(double v1, const Expression& e2);
  friend DRAKE_EXPORT Formula operator>=(const Expression& e1, double v2);

  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream& os,
                                               const Formula& f);
  friend DRAKE_EXPORT void swap(Formula& a, Formula& b) {
    std::swap(a.ptr_, b.ptr_);
  }

 private:
  std::shared_ptr<FormulaCell> ptr_;
};

/** Returns a formula \p f, universally quantified by variables \p vars. */
DRAKE_EXPORT Formula forall(const Variables& vars, const Formula& f);

DRAKE_EXPORT Formula operator==(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Formula operator==(double v1, const Expression& e2);
DRAKE_EXPORT Formula operator==(const Expression& e1, double v2);
DRAKE_EXPORT Formula operator!=(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Formula operator!=(double v1, const Expression& e2);
DRAKE_EXPORT Formula operator!=(const Expression& e1, double v2);
DRAKE_EXPORT Formula operator<(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Formula operator<(double v1, const Expression& e2);
DRAKE_EXPORT Formula operator<(const Expression& e1, double v2);
DRAKE_EXPORT Formula operator<=(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Formula operator<=(double v1, const Expression& e2);
DRAKE_EXPORT Formula operator<=(const Expression& e1, double v2);
DRAKE_EXPORT Formula operator>(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Formula operator>(double v1, const Expression& e2);
DRAKE_EXPORT Formula operator>(const Expression& e1, double v2);
DRAKE_EXPORT Formula operator>=(const Expression& e1, const Expression& e2);
DRAKE_EXPORT Formula operator>=(double v1, const Expression& e2);
DRAKE_EXPORT Formula operator>=(const Expression& e1, double v2);

/** Represents an abstract class which is the base of concrete symbolic-formula
 * classes (i.e. symbolic::FormulaAnd, symbolic::FormulaEq).
 *
 * \note It provides virtual function, FormulaCell::Display,
 * because operator<< is not allowed to be a virtual function.
 */
class FormulaCell {
 public:
  /** Default constructor (deleted). */
  FormulaCell() = delete;
  /** Move-construct a set from an rvalue. */
  FormulaCell(FormulaCell&& f) = default;
  /** Copy-construct a set from an lvalue. */
  FormulaCell(const FormulaCell& f) = default;
  /** Move-assign (DELETED). */
  FormulaCell& operator=(FormulaCell&& f) = delete;
  /** Copy-assign (DELETED). */
  FormulaCell& operator=(const FormulaCell& f) = delete;
  /** Construct FormulaCell of kind \p k with \p hash. */
  FormulaCell(FormulaKind k, size_t hash);
  /** Returns kind of formula. */
  FormulaKind get_kind() const { return kind_; }
  /** Returns hash of formula. */
  size_t get_hash() const { return hash_; }
  /** Returns set of free variables in formula. */
  virtual Variables GetFreeVariables() const = 0;
  /** Checks structural equality. */
  virtual bool EqualTo(const FormulaCell& c) const = 0;
  /** Evaluates under a given environment. */
  virtual bool Evaluate(const Environment& env) const = 0;
  /** Output string representation of formula into output stream \p os. */
  virtual std::ostream& Display(std::ostream& os) const = 0;

 private:
  const FormulaKind kind_{};
  const size_t hash_{};
};

/** Represents the base class for relational operators (==, !=, <, <=, >, >=).
 */
class RelationalFormulaCell : public FormulaCell {
 protected:
  /** Default constructor (deleted). */
  RelationalFormulaCell() = delete;
  /** Move-construct a set from an rvalue. */
  RelationalFormulaCell(RelationalFormulaCell&& f) = default;
  /** Copy-construct a set from an lvalue. */
  RelationalFormulaCell(const RelationalFormulaCell& f) = default;
  /** Move-assign (DELETED). */
  RelationalFormulaCell& operator=(RelationalFormulaCell&& f) = delete;
  /** Copy-assign (DELETED). */
  RelationalFormulaCell& operator=(const RelationalFormulaCell& f) = delete;
  /** Construct RelationalFormulaCell of kind \p k with \p hash. */
  RelationalFormulaCell(FormulaKind k, const Expression& e1,
                        const Expression& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;

  /** Returns the first expression. */
  const Expression& get_1st_expression() const { return e1_; }
  /** Returns the second expression. */
  const Expression& get_2nd_expression() const { return e2_; }

 private:
  const Expression e1_;
  const Expression e2_;
};

/** Represents the base class for binary logic operators (∧ and ∨).
 */
class BinaryFormulaCell : public FormulaCell {
 protected:
  /** Default constructor (deleted). */
  BinaryFormulaCell() = delete;
  /** Move-construct a set from an rvalue. */
  BinaryFormulaCell(BinaryFormulaCell&& f) = default;
  /** Copy-construct a set from an lvalue. */
  BinaryFormulaCell(const BinaryFormulaCell& f) = default;
  /** Move-assign (DELETED). */
  BinaryFormulaCell& operator=(BinaryFormulaCell&& f) = delete;
  /** Copy-assign (DELETED). */
  BinaryFormulaCell& operator=(const BinaryFormulaCell& f) = delete;
  /** Construct BinaryFormulaCell of kind \p k with \p hash. */
  BinaryFormulaCell(FormulaKind k, const Formula& f1, const Formula& f2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  /** Returns the first formula. */
  const Formula& get_1st_formula() const { return f1_; }
  /** Returns the second formula. */
  const Formula& get_2nd_formula() const { return f2_; }

 private:
  const Formula f1_;
  const Formula f2_;
};

/** Symbolic formula representing true. */
class FormulaTrue : public FormulaCell {
 public:
  FormulaTrue();
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing false. */
class FormulaFalse : public FormulaCell {
 public:
  FormulaFalse();
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing equality (e1 = e2). */
class FormulaEq : public RelationalFormulaCell {
 public:
  FormulaEq(const Expression& e1, const Expression& e2);
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing disequality (e1 ≠ e2). */
class FormulaNeq : public RelationalFormulaCell {
 public:
  FormulaNeq(const Expression& e1, const Expression& e2);
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing 'greater-than' (e1 > e2). */
class FormulaGt : public RelationalFormulaCell {
 public:
  FormulaGt(const Expression& e1, const Expression& e2);
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing 'greater-than-or-equal-to' (e1 ≥ e2). */
class FormulaGeq : public RelationalFormulaCell {
 public:
  FormulaGeq(const Expression& e1, const Expression& e2);
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing 'less-than' (e1 < e2). */
class FormulaLt : public RelationalFormulaCell {
 public:
  FormulaLt(const Expression& e1, const Expression& e2);
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing 'less-than-or-equal-to' (e1 ≤ e2). */
class FormulaLeq : public RelationalFormulaCell {
 public:
  FormulaLeq(const Expression& e1, const Expression& e2);
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing conjunctions (f1 ∧ f2). */
class FormulaAnd : public BinaryFormulaCell {
 public:
  FormulaAnd(const Formula& f1, const Formula& f2);
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing disjunctions (f1 ∨ f2). */
class FormulaOr : public BinaryFormulaCell {
 public:
  FormulaOr(const Formula& f1, const Formula& f2);
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing negations (¬f). */
class FormulaNot : public FormulaCell {
 public:
  explicit FormulaNot(const Formula& f);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Formula f_;
};

/** Symbolic formula representing universal quantifications (∀ x₁, ...,
 * xn. F). */
class FormulaForall : public FormulaCell {
 public:
  FormulaForall(const Variables& vars, const Formula& f);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Variables vars_;  // Quantified variables.
  const Formula f_;       // Quantified formula.
};

std::ostream& operator<<(std::ostream& os, const Formula& e);

}  // namespace symbolic
namespace assert {
/* We allow assertion-like statements to receive a Formula.  Given the typical
 * uses of assertions and Formulas, rather than trying to be clever and, e.g.,
 * capture assertion data for later use or partially-solve the Formula to find
 * counterexamples, instead we've decided to ignore assertions for the purpose
 * of Formula.  They are syntax-checked, but always pass. */
template <>
struct ConditionTraits<symbolic::Formula> {
  static constexpr bool is_valid = true;
  static bool Evaluate(const symbolic::Formula&) {
    return true;
  }
};
}  // namespace assert
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Formula>. */
template <>
struct hash<drake::symbolic::Formula> {
  size_t operator()(const drake::symbolic::Formula& e) const {
    return e.get_hash();
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
