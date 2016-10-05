#pragma once

#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"
#include "drake/drakeCommon_export.h"

namespace drake {
namespace symbolic {
enum class FormulaKind {
  True,   // ⊤
  False,  // ⊥
  Eq,     // =
  Neq,    // !=
  Gt,     // >
  Geq,    // >=
  Lt,     // <
  Leq,    // <=
  And,    // Conjunction (∧)
  Or,     // Disjunction (∨)
  Not,    // Negation (¬)
  Forall  // Universal quantification (∀)
};

class FormulaCell;

/** Represents a symbolic form of a firt-order logic formula.

It has the following grammar:

\verbatim
    F := ⊤ | ⊥ | E = E | E ≠ E | E > E | E ≥ E | E < E | E ≤ E
       | E ∧ E | E ∨ E | ¬E | ∀ x₁, ..., xn. F
\endverbatim

In the implementation, Formula is a simple wrapper including a shared pointer to
FormulaCell class which is a super-class of different kinds of symbolic formulas
(i.e. FormulaAnd, FormulaOr, FormulaEq). Note that it includes a shared pointer,
not a unique pointer, to allow sharing sub-expressions.

\note The sharing of sub-expressions is not yet implemented.

The following simple simplifications are implemented:
\verbatim
    E1 = E2        ->  True    (if E1 and E2 are structually equal)
    E1 ≠ E2        ->  False   (if E1 and E2 are structually equal)
    E1 > E2        ->  False   (if E1 and E2 are structually equal)
    E1 ≥ E2        ->  True    (if E1 and E2 are structually equal)
    E1 < E2        ->  False   (if E1 and E2 are structually equal)
    E1 ≤ E2        ->  True    (if E1 and E2 are structually equal)
    F1 ∧ F2        ->  False   (if either F1 or F2 is False)
    F1 ∨ F2        ->  True    (if either F1 or F2 is True)
\endverbatim
*/

class DRAKECOMMON_EXPORT Formula {
 public:
  /** Default constructor. */
  Formula() = default;

  /** Move-construct a set from an rvalue. */
  Formula(Formula&& f) = default;

  /** Copy-construct a set from an lvalue. */
  Formula(const Formula& f) = default;

  /** Move-assign a set from an rvalue. */
  Formula& operator=(Formula&& f) = default;

  /** Copy-assign a set from an lvalue. */
  Formula& operator=(const Formula& f) = default;

  explicit Formula(std::shared_ptr<FormulaCell> const ptr);

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

  friend DRAKECOMMON_EXPORT Formula operator&&(const Formula& f1,
                                               const Formula& f2);
  friend DRAKECOMMON_EXPORT Formula operator||(const Formula& f1,
                                               const Formula& f2);
  friend DRAKECOMMON_EXPORT Formula operator!(const Formula& f);
  friend DRAKECOMMON_EXPORT Formula operator==(const Expression& e1,
                                               const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator==(const double v1,
                                               const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator==(const Expression& e1,
                                               const double v2);
  friend DRAKECOMMON_EXPORT Formula operator!=(const Expression& e1,
                                               const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator!=(const double v1,
                                               const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator!=(const Expression& e1,
                                               const double v2);
  friend DRAKECOMMON_EXPORT Formula operator<(const Expression& e1,
                                              const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator<(const double v1,
                                              const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator<(const Expression& e1,
                                              const double v2);
  friend DRAKECOMMON_EXPORT Formula operator<=(const Expression& e1,
                                               const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator<=(const double v1,
                                               const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator<=(const Expression& e1,
                                               const double v2);
  friend DRAKECOMMON_EXPORT Formula operator>(const Expression& e1,
                                              const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator>(const double v1,
                                              const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator>(const Expression& e1,
                                              const double v2);
  friend DRAKECOMMON_EXPORT Formula operator>=(const Expression& e1,
                                               const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator>=(const double v1,
                                               const Expression& e2);
  friend DRAKECOMMON_EXPORT Formula operator>=(const Expression& e1,
                                               const double v2);

  friend DRAKECOMMON_EXPORT std::ostream& operator<<(std::ostream& os,
                                                     const Formula& f);
  friend DRAKECOMMON_EXPORT void swap(Formula& a, Formula& b) {
    std::swap(a.ptr_, b.ptr_);
  }

 private:
  std::shared_ptr<FormulaCell> ptr_;
};

DRAKECOMMON_EXPORT Formula forall(const Variables& vars, const Formula& f);

DRAKECOMMON_EXPORT Formula operator==(const Expression& e1,
                                      const Expression& e2);
DRAKECOMMON_EXPORT Formula operator==(const double v1, const Expression& e2);
DRAKECOMMON_EXPORT Formula operator==(const Expression& e1, const double v2);
DRAKECOMMON_EXPORT Formula operator!=(const Expression& e1,
                                      const Expression& e2);
DRAKECOMMON_EXPORT Formula operator!=(const double v1, const Expression& e2);
DRAKECOMMON_EXPORT Formula operator!=(const Expression& e1, const double v2);
DRAKECOMMON_EXPORT Formula operator<(const Expression& e1,
                                     const Expression& e2);
DRAKECOMMON_EXPORT Formula operator<(const double v1, const Expression& e2);
DRAKECOMMON_EXPORT Formula operator<(const Expression& e1, const double v2);
DRAKECOMMON_EXPORT Formula operator<=(const Expression& e1,
                                      const Expression& e2);
DRAKECOMMON_EXPORT Formula operator<=(const double v1, const Expression& e2);
DRAKECOMMON_EXPORT Formula operator<=(const Expression& e1, const double v2);
DRAKECOMMON_EXPORT Formula operator>(const Expression& e1,
                                     const Expression& e2);
DRAKECOMMON_EXPORT Formula operator>(const double v1, const Expression& e2);
DRAKECOMMON_EXPORT Formula operator>(const Expression& e1, const double v2);
DRAKECOMMON_EXPORT Formula operator>=(const Expression& e1,
                                      const Expression& e2);
DRAKECOMMON_EXPORT Formula operator>=(const double v1, const Expression& e2);
DRAKECOMMON_EXPORT Formula operator>=(const Expression& e1, const double v2);

class FormulaCell {
 public:
  FormulaCell(FormulaKind const k, size_t const hash);
  FormulaKind get_kind() const { return kind_; }
  size_t get_hash() const { return hash_; }
  virtual Variables GetFreeVariables() const = 0;
  /** Checks structural equality. */
  virtual bool EqualTo(const FormulaCell& c) const = 0;
  /** Evaluates under a given environment (by default, an empty environment). */
  virtual bool Evaluate(const Environment& env) const = 0;
  virtual std::ostream& Display(std::ostream& os) const = 0;

 protected:
  const FormulaKind kind_{};
  const size_t hash_{};
};

class FormulaTrue : public FormulaCell {
 public:
  FormulaTrue();
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaFalse : public FormulaCell {
 public:
  FormulaFalse();
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaEq : public FormulaCell {
 public:
  FormulaEq(const Expression& e1, const Expression& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Expression e1_;
  const Expression e2_;
};

class FormulaNeq : public FormulaCell {
 public:
  FormulaNeq(const Expression& e1, const Expression& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Expression e1_;
  const Expression e2_;
};

class FormulaGt : public FormulaCell {
 public:
  FormulaGt(const Expression& e1, const Expression& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Expression e1_;
  const Expression e2_;
};

class FormulaGeq : public FormulaCell {
 public:
  FormulaGeq(const Expression& e1, const Expression& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Expression e1_;
  const Expression e2_;
};

class FormulaLt : public FormulaCell {
 public:
  FormulaLt(const Expression& e1, const Expression& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Expression e1_;
  const Expression e2_;
};

class FormulaLeq : public FormulaCell {
 public:
  FormulaLeq(const Expression& e1, const Expression& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Expression e1_;
  const Expression e2_;
};

class FormulaAnd : public FormulaCell {
 public:
  FormulaAnd(const Formula& f1, const Formula& f2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Formula f1_;
  const Formula f2_;
};

class FormulaOr : public FormulaCell {
 public:
  FormulaOr(const Formula& f1, const Formula& f2);
  Variables GetFreeVariables() const override;
  bool EqualTo(const FormulaCell& f) const override;
  bool Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Formula f1_;
  const Formula f2_;
};

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
}  // namespace drake
}  // namespace symbolic

/** Provides std::hash<drake::symbolic::Formula>. */
namespace std {
template <>
struct hash<drake::symbolic::Formula> {
  size_t operator()(const drake::symbolic::Formula& e) const {
    return e.get_hash();
  }
};

/** Provides std::equal_to<drake::symbolic::Formula>. */
template <>
struct equal_to<drake::symbolic::Formula> {
  bool operator()(const drake::symbolic::Formula& lhs,
                  const drake::symbolic::Formula& rhs) const {
    return lhs.EqualTo(rhs);
  }
};
}  // namespace std
