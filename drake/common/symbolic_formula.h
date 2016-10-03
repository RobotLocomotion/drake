#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <string>

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

/** \brief Represent a symbolic form of a firt-order logic formula.

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
 private:
  std::shared_ptr<FormulaCell> ptr_;

 public:
  /** Default constructor. */
  Formula() = default;

  /** Move-construct a set from an rvalue. */
  Formula(Formula&& f) = default;

  /** Copy-construct a set from an lvalue. */
  Formula(Formula const& f) = default;

  /** Move-assign a set from an rvalue. */
  Formula& operator=(Formula&& f) = default;

  /** Copy-assign a set from an lvalue. */
  Formula& operator=(Formula const& f) = default;

  explicit Formula(std::shared_ptr<FormulaCell> const ptr);

  FormulaKind get_kind() const;
  size_t get_hash() const;
  /** Get free variables*/
  Variables GetFreeVariables() const;
  /** Check structural equality*/
  bool EqualTo(Formula const& f) const;
  /** Evaluate under a given environment (by default, an empty environment)*/
  bool Evaluate(Environment const& env = Environment{}) const;

  static Formula True();
  static Formula False();

  friend DRAKECOMMON_EXPORT Formula operator&&(Formula const& f1,
                                               Formula const& f2);
  friend DRAKECOMMON_EXPORT Formula operator||(Formula const& f1,
                                               Formula const& f2);
  friend DRAKECOMMON_EXPORT Formula operator!(Formula const& f);
  friend DRAKECOMMON_EXPORT Formula operator==(Expression const& e1,
                                               Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator==(double const v1,
                                               Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator==(Expression const& e1,
                                               double const v2);
  friend DRAKECOMMON_EXPORT Formula operator!=(Expression const& e1,
                                               Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator!=(double const v1,
                                               Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator!=(Expression const& e1,
                                               double const v2);
  friend DRAKECOMMON_EXPORT Formula operator<(Expression const& e1,
                                              Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator<(double const v1,
                                              Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator<(Expression const& e1,
                                              double const v2);
  friend DRAKECOMMON_EXPORT Formula operator<=(Expression const& e1,
                                               Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator<=(double const v1,
                                               Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator<=(Expression const& e1,
                                               double const v2);
  friend DRAKECOMMON_EXPORT Formula operator>(Expression const& e1,
                                              Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator>(double const v1,
                                              Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator>(Expression const& e1,
                                              double const v2);
  friend DRAKECOMMON_EXPORT Formula operator>=(Expression const& e1,
                                               Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator>=(double const v1,
                                               Expression const& e2);
  friend DRAKECOMMON_EXPORT Formula operator>=(Expression const& e1,
                                               double const v2);

  friend DRAKECOMMON_EXPORT std::ostream& operator<<(std::ostream& os,
                                                     Formula const& f);
  friend DRAKECOMMON_EXPORT void swap(Formula& a, Formula& b) {
    std::swap(a.ptr_, b.ptr_);
  }
};

DRAKECOMMON_EXPORT Formula forall(Variables const& vars, Formula const& f);

DRAKECOMMON_EXPORT Formula operator==(Expression const& e1,
                                      Expression const& e2);
DRAKECOMMON_EXPORT Formula operator==(double const v1, Expression const& e2);
DRAKECOMMON_EXPORT Formula operator==(Expression const& e1, double const v2);
DRAKECOMMON_EXPORT Formula operator!=(Expression const& e1,
                                      Expression const& e2);
DRAKECOMMON_EXPORT Formula operator!=(double const v1, Expression const& e2);
DRAKECOMMON_EXPORT Formula operator!=(Expression const& e1, double const v2);
DRAKECOMMON_EXPORT Formula operator<(Expression const& e1,
                                     Expression const& e2);
DRAKECOMMON_EXPORT Formula operator<(double const v1, Expression const& e2);
DRAKECOMMON_EXPORT Formula operator<(Expression const& e1, double const v2);
DRAKECOMMON_EXPORT Formula operator<=(Expression const& e1,
                                      Expression const& e2);
DRAKECOMMON_EXPORT Formula operator<=(double const v1, Expression const& e2);
DRAKECOMMON_EXPORT Formula operator<=(Expression const& e1, double const v2);
DRAKECOMMON_EXPORT Formula operator>(Expression const& e1,
                                     Expression const& e2);
DRAKECOMMON_EXPORT Formula operator>(double const v1, Expression const& e2);
DRAKECOMMON_EXPORT Formula operator>(Expression const& e1, double const v2);
DRAKECOMMON_EXPORT Formula operator>=(Expression const& e1,
                                      Expression const& e2);
DRAKECOMMON_EXPORT Formula operator>=(double const v1, Expression const& e2);
DRAKECOMMON_EXPORT Formula operator>=(Expression const& e1, double const v2);

class FormulaCell {
 protected:
  FormulaKind kind_;
  size_t hash_;

 public:
  FormulaCell(FormulaKind const k, size_t const hash);
  FormulaKind get_kind() const { return kind_; }
  size_t get_hash() const { return hash_; }
  virtual Variables GetFreeVariables() const = 0;
  /** Check structural equality*/
  virtual bool EqualTo(FormulaCell const& c) const = 0;
  /** Evaluate under a given environment (by default, an empty environment)*/
  virtual bool Evaluate(Environment const& env) const = 0;
  virtual std::ostream& Display(std::ostream& os) const = 0;
};

class FormulaTrue : public FormulaCell {
 public:
  FormulaTrue();
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaFalse : public FormulaCell {
 public:
  FormulaFalse();
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaEq : public FormulaCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  FormulaEq(Expression const& e1, Expression const& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaNeq : public FormulaCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  FormulaNeq(Expression const& e1, Expression const& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaGt : public FormulaCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  FormulaGt(Expression const& e1, Expression const& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaGeq : public FormulaCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  FormulaGeq(Expression const& e1, Expression const& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaLt : public FormulaCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  FormulaLt(Expression const& e1, Expression const& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaLeq : public FormulaCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  FormulaLeq(Expression const& e1, Expression const& e2);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaAnd : public FormulaCell {
 private:
  Formula f1_;
  Formula f2_;

 public:
  FormulaAnd(Formula const& f1, Formula const& f2);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaOr : public FormulaCell {
 private:
  Formula f1_;
  Formula f2_;

 public:
  FormulaOr(Formula const& f1, Formula const& f2);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaNot : public FormulaCell {
 private:
  Formula f_;

 public:
  explicit FormulaNot(Formula const& f);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

class FormulaForall : public FormulaCell {
 private:
  Variables vars_; /** quantified variables*/
  Formula f_;      /** quantified formula*/

 public:
  FormulaForall(Variables const& vars, Formula const& f);
  Variables GetFreeVariables() const override;
  bool EqualTo(FormulaCell const& f) const override;
  bool Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

std::ostream& operator<<(std::ostream& os, Formula const& e);
}  // namespace drake
}  // namespace symbolic

/** Provide std::hash<drake::symbolic::Formula>. */
namespace std {
template <>
struct hash<drake::symbolic::Formula> {
  size_t operator()(drake::symbolic::Formula const& e) const {
    return e.get_hash();
  }
};

/** Provide std::equal_to<drake::symbolic::Formula>. */
template <>
struct equal_to<drake::symbolic::Formula> {
  bool operator()(drake::symbolic::Formula const& lhs,
                  drake::symbolic::Formula const& rhs) const {
    return lhs.EqualTo(rhs);
  }
};

/** Provide std::to_string for drake::symbolic::Formula. */
DRAKECOMMON_EXPORT std::string to_string(drake::symbolic::Formula const& f);
}  // namespace std
