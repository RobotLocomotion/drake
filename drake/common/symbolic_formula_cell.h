#pragma once

#include <functional>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/variable.h"
#include "drake/common/variables.h"

namespace drake {
namespace symbolic {

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
  /** Construct FormulaCell of kind @p k with @p hash. */
  FormulaCell(FormulaKind k, size_t hash);
  /** Returns kind of formula. */
  FormulaKind get_kind() const { return kind_; }
  /** Returns hash of formula. */
  size_t get_hash() const { return hash_; }
  /** Returns set of free variables in formula. */
  virtual Variables GetFreeVariables() const = 0;
  /** Checks structural equality. */
  virtual bool EqualTo(const FormulaCell& c) const = 0;
  /** Checks ordering. */
  virtual bool Less(const FormulaCell& c) const = 0;
  /** Evaluates under a given environment. */
  virtual bool Evaluate(const Environment& env) const = 0;
  /** Outputs string representation of formula into output stream @p os. */
  virtual std::ostream& Display(std::ostream& os) const = 0;

 private:
  const FormulaKind kind_{};
  const size_t hash_{};
};

/** Represents the base class for relational operators (==, !=, <, <=, >, >=).
 */
class RelationalFormulaCell : public FormulaCell {
 public:
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
  /** Construct RelationalFormulaCell of kind @p k with @p hash. */
  RelationalFormulaCell(FormulaKind k, const Expression& e1,
                        const Expression& e2);
  /** Returns set of free variables in formula. */
  Variables GetFreeVariables() const override;
  /** Checks structural equality. */
  bool EqualTo(const FormulaCell& f) const override;
  /** Checks ordering. */
  bool Less(const FormulaCell& f) const override;

  /** Returns the expression on left-hand-side. */
  const Expression& get_lhs_expression() const { return e_lhs_; }
  /** Returns the expression on right-hand-side. */
  const Expression& get_rhs_expression() const { return e_rhs_; }

 private:
  const Expression e_lhs_;
  const Expression e_rhs_;
};

/** Represents the base class for N-ary logic operators (∧ and ∨).
 *
 * @note Internally this class maintains a set of symbolic formulas to avoid
 * duplicated elements (i.e. f1 ∧ ... ∧ f1).
 */
class NaryFormulaCell : public FormulaCell {
 public:
  /** Default constructor (deleted). */
  NaryFormulaCell() = delete;
  /** Move-construct a set from an rvalue. */
  NaryFormulaCell(NaryFormulaCell&& f) = default;
  /** Copy-construct a set from an lvalue. */
  NaryFormulaCell(const NaryFormulaCell& f) = default;
  /** Move-assign (DELETED). */
  NaryFormulaCell& operator=(NaryFormulaCell&& f) = delete;
  /** Copy-assign (DELETED). */
  NaryFormulaCell& operator=(const NaryFormulaCell& f) = delete;
  /** Construct NaryFormulaCell of kind @p k with @p hash. */
  NaryFormulaCell(FormulaKind k, const std::set<Formula>& formulas);
  /** Returns free variables in formula. */
  Variables GetFreeVariables() const override;
  /** Checks structural equality. */
  bool EqualTo(const FormulaCell& f) const override;
  /** Checks ordering. */
  bool Less(const FormulaCell& f) const override;
  /** Returns the formulas. */
  const std::set<Formula>& get_operands() const { return formulas_; }

 protected:
  std::ostream& DisplayWithOp(std::ostream& os, const std::string& op) const;

 private:
  const std::set<Formula> formulas_;
};

/** Symbolic formula representing true. */
class FormulaTrue : public FormulaCell {
 public:
  /** Default Constructor. */
  FormulaTrue();
  /** Returns set of free variables in formula. */
  Variables GetFreeVariables() const override;
  /** Checks structural equality. */
  bool EqualTo(const FormulaCell& f) const override;
  /** Checks ordering. */
  bool Less(const FormulaCell& f) const override;
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing false. */
class FormulaFalse : public FormulaCell {
 public:
  /** Default Constructor. */
  FormulaFalse();
  /** Returns set of free variables in formula. */
  Variables GetFreeVariables() const override;
  /** Checks structural equality. */
  bool EqualTo(const FormulaCell& f) const override;
  /** Checks ordering. */
  bool Less(const FormulaCell& f) const override;
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing equality (e1 = e2). */
class FormulaEq : public RelationalFormulaCell {
 public:
  /** Constructs from @p e1 and @p e2. */
  FormulaEq(const Expression& e1, const Expression& e2);
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing disequality (e1 ≠ e2). */
class FormulaNeq : public RelationalFormulaCell {
 public:
  /** Constructs from @p e1 and @p e2. */
  FormulaNeq(const Expression& e1, const Expression& e2);
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing 'greater-than' (e1 > e2). */
class FormulaGt : public RelationalFormulaCell {
 public:
  /** Constructs from @p e1 and @p e2. */
  FormulaGt(const Expression& e1, const Expression& e2);
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing 'greater-than-or-equal-to' (e1 ≥ e2). */
class FormulaGeq : public RelationalFormulaCell {
 public:
  /** Constructs from @p e1 and @p e2. */
  FormulaGeq(const Expression& e1, const Expression& e2);
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing 'less-than' (e1 < e2). */
class FormulaLt : public RelationalFormulaCell {
 public:
  /** Constructs from @p e1 and @p e2. */
  FormulaLt(const Expression& e1, const Expression& e2);
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing 'less-than-or-equal-to' (e1 ≤ e2). */
class FormulaLeq : public RelationalFormulaCell {
 public:
  /** Constructs from @p e1 and @p e2. */
  FormulaLeq(const Expression& e1, const Expression& e2);
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing conjunctions (f1 ∧ ... ∧ fn). */
class FormulaAnd : public NaryFormulaCell {
 public:
  /** Constructs from @p formulas. */
  explicit FormulaAnd(const std::set<Formula>& formulas);
  /** Constructs @p f1 ∧ @p f2. */
  FormulaAnd(const Formula& f1, const Formula& f2);
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing disjunctions (f1 ∨ ... ∨ fn). */
class FormulaOr : public NaryFormulaCell {
 public:
  /** Constructs from @p formulas. */
  explicit FormulaOr(const std::set<Formula>& formula);
  /** Constructs @p f1 ∨ @p f2. */
  FormulaOr(const Formula& f1, const Formula& f2);
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic formula representing negations (¬f). */
class FormulaNot : public FormulaCell {
 public:
  /** Constructs from @p f. */
  explicit FormulaNot(const Formula& f);
  /** Returns set of free variables in formula. */
  Variables GetFreeVariables() const override;
  /** Checks structural equality. */
  bool EqualTo(const FormulaCell& f) const override;
  /** Checks ordering. */
  bool Less(const FormulaCell& f) const override;
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
  /** Returns the operand. */
  const Formula& get_operand() const { return f_; }

 private:
  const Formula f_;
};

/** Symbolic formula representing universal quantifications
 *  (∀ x₁, ..., * xn. F).
 */
class FormulaForall : public FormulaCell {
 public:
  /** Constructs from @p vars and @p f. */
  FormulaForall(const Variables& vars, const Formula& f);
  /** Returns set of free variables in formula. */
  Variables GetFreeVariables() const override;
  /** Checks structural equality. */
  bool EqualTo(const FormulaCell& f) const override;
  /** Checks ordering. */
  bool Less(const FormulaCell& f) const override;
  /** Evaluates under a given environment. */
  bool Evaluate(const Environment& env) const override;
  /** Outputs string representation of formula into output stream @p os. */
  std::ostream& Display(std::ostream& os) const override;
  /** Returns the quantified variables. */
  const Variables& get_quantified_variables() const { return vars_; }
  /** Returns the quantified formula. */
  const Formula& get_quantified_formula() const { return f_; }

 private:
  const Variables vars_;  // Quantified variables.
  const Formula f_;       // Quantified formula.
};

/** Checks if @p f is structurally equal to False formula. */
bool is_false(const FormulaCell& f);
/** Checks if @p f is structurally equal to True formula. */
bool is_true(const FormulaCell& f);
/** Checks if @p f is a formula representing equality (==). */
bool is_equal_to(const FormulaCell& f);
/** Checks if @p f is a formula representing disequality (!=). */
bool is_not_equal_to(const FormulaCell& f);
/** Checks if @p f is a formula representing greater-than (>). */
bool is_greater_than(const FormulaCell& f);
/** Checks if @p f is a formula representing greater-than-or-equal-to (>=). */
bool is_greater_than_or_equal_to(const FormulaCell& f);
/** Checks if @p f is a formula representing less-than (<). */
bool is_less_than(const FormulaCell& f);
/** Checks if @p f is a formula representing less-than-or-equal-to (<=). */
bool is_less_than_or_equal_to(const FormulaCell& f);
/** Checks if @p f is a relational formula ({==, !=, >, >=, <, <=}). */
bool is_relational(const FormulaCell& f);
/** Checks if @p f is a conjunction (∧). */
bool is_conjunction(const FormulaCell& f);
/** Checks if @p f is a disjunction (∨). */
bool is_disjunction(const FormulaCell& f);
/** Checks if @p f is a negation (¬). */
bool is_negation(const FormulaCell& f);
/** Checks if @p f is a Forall formula (∀). */
bool is_forall(const FormulaCell& f);

/** Casts @p f_ptr of shared_ptr<FormulaCell> to
 * @c shared_ptr<RelationalFormulaCell>.
 *  \pre{@c is_relational(*f_ptr) is true.}
 */
std::shared_ptr<RelationalFormulaCell> to_relational(
    const std::shared_ptr<FormulaCell> f_ptr);

/** Casts @p f of Formula to
 * @c shared_ptr<RelationalFormulaCell>.
 *  \pre{@c is_relational(f) is true.}
 */
std::shared_ptr<RelationalFormulaCell> to_relational(const Formula& f);

/** Casts @p f_ptr of shared_ptr<FormulaCell>
 * to @c shared_ptr<NaryFormulaCell>.
 *  \pre{@c is_nary(*f_ptr) is true.}
 */
std::shared_ptr<NaryFormulaCell> to_nary(
    const std::shared_ptr<FormulaCell> f_ptr);

/** Casts @p f of Formula to @c shared_ptr<NaryFormulaCell>.
 *  \pre{@c is_nary(f) is true.}
 */
std::shared_ptr<NaryFormulaCell> to_nary(const Formula& f);

/** Casts @p f_ptr of shared_ptr<FormulaCell> to @c shared_ptr<FormulaNot>.
 *  \pre{@c is_negation(*f_ptr) is true.}
 */
std::shared_ptr<FormulaNot> to_negation(
    const std::shared_ptr<FormulaCell> f_ptr);

/** Casts @p f of Formula to @c shared_ptr<FormulaNot>.
 *  \pre{@c is_negation(f) is true.}
 */
std::shared_ptr<FormulaNot> to_negation(const Formula& f);

/** Casts @p f_ptr of shared_ptr<FormulaCell> to @c shared_ptr<FormulaForall>.
 *  \pre{@c is_forall(*f_ptr) is true.}
 */
std::shared_ptr<FormulaForall> to_forall(
    const std::shared_ptr<FormulaCell> f_ptr);

/** Casts @p f of Formula to @c shared_ptr<FormulaForall>.
 *  \pre{@c is_forall(f) is true.}
 */
std::shared_ptr<FormulaForall> to_forall(const Formula& f);

}  // namespace symbolic
}  // namespace drake
