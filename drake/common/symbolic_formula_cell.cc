#include "drake/common/symbolic_formula_cell.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_compat.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::equal;
using std::hash;
using std::ostream;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;

FormulaCell::FormulaCell(const FormulaKind k, const size_t hash)
    : kind_{k}, hash_{hash_combine(hash, static_cast<size_t>(kind_))} {}

RelationalFormulaCell::RelationalFormulaCell(const FormulaKind k,
                                             const Expression& lhs,
                                             const Expression& rhs)
    : FormulaCell{k, hash_combine(lhs.get_hash(), rhs)},
      e_lhs_{lhs},
      e_rhs_{rhs} {}

Variables RelationalFormulaCell::GetFreeVariables() const {
  Variables ret{e_lhs_.GetVariables()};
  ret.insert(e_rhs_.GetVariables());
  return ret;
}

bool RelationalFormulaCell::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const RelationalFormulaCell& rel_f =
      static_cast<const RelationalFormulaCell&>(f);
  return e_lhs_.EqualTo(rel_f.e_lhs_) && e_rhs_.EqualTo(rel_f.e_rhs_);
}

bool RelationalFormulaCell::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const RelationalFormulaCell& rel_f =
      static_cast<const RelationalFormulaCell&>(f);
  if (e_lhs_.Less(rel_f.e_lhs_)) {
    return true;
  }
  if (rel_f.e_lhs_.Less(e_lhs_)) {
    return false;
  }
  return e_rhs_.Less(rel_f.e_rhs_);
}

NaryFormulaCell::NaryFormulaCell(const FormulaKind k,
                                 const set<Formula>& formulas)
    : FormulaCell{k, hash_value<set<Formula>>{}(formulas)},
      formulas_{formulas} {}

Variables NaryFormulaCell::GetFreeVariables() const {
  Variables ret{};
  for (const auto& f : formulas_) {
    ret.insert(f.GetFreeVariables());
  }
  return ret;
}

bool NaryFormulaCell::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const NaryFormulaCell& nary_f = static_cast<const NaryFormulaCell&>(f);
  return equal(
      formulas_.cbegin(), formulas_.cend(), nary_f.formulas_.cbegin(),
      nary_f.formulas_.cend(),
      [](const Formula& f1, const Formula& f2) { return f1.EqualTo(f2); });
}

bool NaryFormulaCell::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const NaryFormulaCell& nary_f = static_cast<const NaryFormulaCell&>(f);
  return lexicographical_compare(
      formulas_.cbegin(), formulas_.cend(), nary_f.formulas_.cbegin(),
      nary_f.formulas_.cend(),
      [](const Formula& f1, const Formula& f2) { return f1.Less(f2); });
}

ostream& NaryFormulaCell::DisplayWithOp(ostream& os, const string& op) const {
  const set<Formula>& formulas{get_operands()};
  auto it(formulas.cbegin());
  DRAKE_ASSERT(formulas.size() > 1u);
  os << "(";
  os << *it;
  ++it;
  while (it != formulas.cend()) {
    os << " " << op << " " << *it;
    ++it;
  }
  os << ")";
  return os;
}

FormulaTrue::FormulaTrue()
    : FormulaCell{FormulaKind::True, hash<string>{}("True")} {}

Variables FormulaTrue::GetFreeVariables() const { return Variables{}; }

bool FormulaTrue::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  return true;  // There is only one instance of this kind.
}

bool FormulaTrue::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  // True < True ==> false
  return false;
}

bool FormulaTrue::Evaluate(const Environment& env) const { return true; }

Formula FormulaTrue::Substitute(const Substitution& s) const {
  return Formula::True();
}

ostream& FormulaTrue::Display(ostream& os) const { return os << "True"; }

FormulaFalse::FormulaFalse()
    : FormulaCell{FormulaKind::False, hash<string>{}("False")} {}

Variables FormulaFalse::GetFreeVariables() const { return Variables{}; }

bool FormulaFalse::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  return true;  // There is only one instance of this kind.
}

bool FormulaFalse::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  // False < False ==> false
  return false;
}

bool FormulaFalse::Evaluate(const Environment& env) const { return false; }

Formula FormulaFalse::Substitute(const Substitution& s) const {
  return Formula::False();
}

ostream& FormulaFalse::Display(ostream& os) const { return os << "False"; }

FormulaEq::FormulaEq(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Eq, e1, e2} {}

bool FormulaEq::Evaluate(const Environment& env) const {
  return get_lhs_expression().Evaluate(env) ==
         get_rhs_expression().Evaluate(env);
}

Formula FormulaEq::Substitute(const Substitution& s) const {
  return get_lhs_expression().Substitute(s) ==
         get_rhs_expression().Substitute(s);
}

ostream& FormulaEq::Display(ostream& os) const {
  return os << "(" << get_lhs_expression() << " = " << get_rhs_expression()
            << ")";
}

FormulaNeq::FormulaNeq(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Neq, e1, e2} {}

bool FormulaNeq::Evaluate(const Environment& env) const {
  return get_lhs_expression().Evaluate(env) !=
         get_rhs_expression().Evaluate(env);
}

Formula FormulaNeq::Substitute(const Substitution& s) const {
  return get_lhs_expression().Substitute(s) !=
         get_rhs_expression().Substitute(s);
}

ostream& FormulaNeq::Display(ostream& os) const {
  return os << "(" << get_lhs_expression() << " != " << get_rhs_expression()
            << ")";
}

FormulaGt::FormulaGt(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Gt, e1, e2} {}

bool FormulaGt::Evaluate(const Environment& env) const {
  return get_lhs_expression().Evaluate(env) >
         get_rhs_expression().Evaluate(env);
}

Formula FormulaGt::Substitute(const Substitution& s) const {
  return get_lhs_expression().Substitute(s) >
         get_rhs_expression().Substitute(s);
}

ostream& FormulaGt::Display(ostream& os) const {
  return os << "(" << get_lhs_expression() << " > " << get_rhs_expression()
            << ")";
}

FormulaGeq::FormulaGeq(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Geq, e1, e2} {}

bool FormulaGeq::Evaluate(const Environment& env) const {
  return get_lhs_expression().Evaluate(env) >=
         get_rhs_expression().Evaluate(env);
}

Formula FormulaGeq::Substitute(const Substitution& s) const {
  return get_lhs_expression().Substitute(s) >=
         get_rhs_expression().Substitute(s);
}

ostream& FormulaGeq::Display(ostream& os) const {
  return os << "(" << get_lhs_expression() << " >= " << get_rhs_expression()
            << ")";
}

FormulaLt::FormulaLt(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Lt, e1, e2} {}

bool FormulaLt::Evaluate(const Environment& env) const {
  return get_lhs_expression().Evaluate(env) <
         get_rhs_expression().Evaluate(env);
}

Formula FormulaLt::Substitute(const Substitution& s) const {
  return get_lhs_expression().Substitute(s) <
         get_rhs_expression().Substitute(s);
}

ostream& FormulaLt::Display(ostream& os) const {
  return os << "(" << get_lhs_expression() << " < " << get_rhs_expression()
            << ")";
}

FormulaLeq::FormulaLeq(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Leq, e1, e2} {}

bool FormulaLeq::Evaluate(const Environment& env) const {
  return get_lhs_expression().Evaluate(env) <=
         get_rhs_expression().Evaluate(env);
}

Formula FormulaLeq::Substitute(const Substitution& s) const {
  return get_lhs_expression().Substitute(s) <=
         get_rhs_expression().Substitute(s);
}

ostream& FormulaLeq::Display(ostream& os) const {
  return os << "(" << get_lhs_expression() << " <= " << get_rhs_expression()
            << ")";
}

FormulaAnd::FormulaAnd(const set<Formula>& formulas)
    : NaryFormulaCell{FormulaKind::And, formulas} {
  DRAKE_ASSERT(get_operands().size() > 1u);
}

FormulaAnd::FormulaAnd(const Formula& f1, const Formula& f2)
    : NaryFormulaCell{FormulaKind::And, set<Formula>{f1, f2}} {}

bool FormulaAnd::Evaluate(const Environment& env) const {
  for (const auto& f : get_operands()) {
    if (!f.Evaluate(env)) {
      return false;
    }
  }
  return true;
}

Formula FormulaAnd::Substitute(const Substitution& s) const {
  Formula ret{Formula::True()};
  for (const auto& f : get_operands()) {
    ret = ret && f.Substitute(s);
    // short-circuiting
    if (is_false(ret)) {
      return ret;
    }
  }
  return ret;
}

ostream& FormulaAnd::Display(ostream& os) const {
  return DisplayWithOp(os, "and");
}

FormulaOr::FormulaOr(const set<Formula>& formulas)
    : NaryFormulaCell{FormulaKind::Or, formulas} {
  DRAKE_ASSERT(get_operands().size() > 1u);
}

FormulaOr::FormulaOr(const Formula& f1, const Formula& f2)
    : NaryFormulaCell{FormulaKind::Or, set<Formula>{f1, f2}} {}

bool FormulaOr::Evaluate(const Environment& env) const {
  for (const auto& f : get_operands()) {
    if (f.Evaluate(env)) {
      return true;
    }
  }
  return false;
}

Formula FormulaOr::Substitute(const Substitution& s) const {
  Formula ret{Formula::False()};
  for (const auto& f : get_operands()) {
    ret = ret || f.Substitute(s);
    // short-circuiting
    if (is_true(ret)) {
      return ret;
    }
  }
  return ret;
}

ostream& FormulaOr::Display(ostream& os) const {
  return DisplayWithOp(os, "or");
}

FormulaNot::FormulaNot(const Formula& f)
    : FormulaCell{FormulaKind::Not, f.get_hash()}, f_{f} {}

Variables FormulaNot::GetFreeVariables() const { return f_.GetFreeVariables(); }

bool FormulaNot::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaNot& f_not{static_cast<const FormulaNot&>(f)};
  return f_.EqualTo(f_not.f_);
}

bool FormulaNot::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaNot& not_f{static_cast<const FormulaNot&>(f)};
  return f_.Less(not_f.f_);
}

bool FormulaNot::Evaluate(const Environment& env) const {
  return !f_.Evaluate(env);
}

Formula FormulaNot::Substitute(const Substitution& s) const {
  return !f_.Substitute(s);
}

ostream& FormulaNot::Display(ostream& os) const {
  return os << "!(" << f_ << ")";
}

FormulaForall::FormulaForall(const Variables& vars, const Formula& f)
    : FormulaCell{FormulaKind::Forall, hash_combine(vars.get_hash(), f)},
      vars_{vars},
      f_{f} {}

Variables FormulaForall::GetFreeVariables() const {
  return f_.GetFreeVariables() - vars_;
}

bool FormulaForall::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaForall& f_forall{static_cast<const FormulaForall&>(f)};
  return vars_ == f_forall.vars_ && f_.EqualTo(f_forall.f_);
}

bool FormulaForall::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaForall& forall_f{static_cast<const FormulaForall&>(f)};
  if (vars_ < forall_f.vars_) {
    return true;
  }
  if (forall_f.vars_ < vars_) {
    return false;
  }
  return this->f_.Less(forall_f.f_);
}

bool FormulaForall::Evaluate(const Environment& env) const {
  // Given ∀ x1, ..., xn. F, check if there is a counterexample satisfying
  // ¬F. If exists, it returns false. Otherwise, return true.
  // That is, it returns !check(∃ x1, ..., xn. ¬F)

  throw runtime_error("not implemented yet");
}

Formula FormulaForall::Substitute(const Substitution& s) const {
  // Quantified variables are already bound and should not be substituted by s.
  // We construct a new substitution new_s from s by removing the entries of
  // bound variables.
  Substitution new_s{s};
  for (const Variable& var : vars_) {
    new_s.erase(var);
  }
  return forall(vars_, f_.Substitute(new_s));
}

ostream& FormulaForall::Display(ostream& os) const {
  return os << "forall(" << vars_ << ". " << f_ << ")";
}

FormulaIsnan::FormulaIsnan(const Expression& e)
    : FormulaCell{FormulaKind::Isnan, e.get_hash()}, e_{e} {}

Variables FormulaIsnan::GetFreeVariables() const { return e_.GetVariables(); }

bool FormulaIsnan::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaIsnan& f_isnan{static_cast<const FormulaIsnan&>(f)};
  return e_.EqualTo(f_isnan.e_);
}

bool FormulaIsnan::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaIsnan& f_isnan{static_cast<const FormulaIsnan&>(f)};
  return e_.Less(f_isnan.e_);
}

bool FormulaIsnan::Evaluate(const Environment& env) const {
  // Note that it throws std::runtime_error if it detects NaN during evaluation.
  return std::isnan(e_.Evaluate(env));
}

Formula FormulaIsnan::Substitute(const Substitution& s) const {
  return isnan(e_.Substitute(s));
}

ostream& FormulaIsnan::Display(ostream& os) const {
  return os << "isnan(" << e_ << ")";
}

bool is_false(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::False;
}

bool is_true(const FormulaCell& f) { return f.get_kind() == FormulaKind::True; }

bool is_equal_to(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Eq;
}

bool is_not_equal_to(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Neq;
}

bool is_greater_than(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Gt;
}

bool is_greater_than_or_equal_to(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Geq;
}

bool is_less_than(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Lt;
}

bool is_less_than_or_equal_to(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Leq;
}

bool is_relational(const FormulaCell& f) {
  return is_equal_to(f) || is_not_equal_to(f) || is_greater_than(f) ||
         is_greater_than_or_equal_to(f) || is_less_than(f) ||
         is_less_than_or_equal_to(f);
}

bool is_conjunction(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::And;
}

bool is_disjunction(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Or;
}

bool is_nary(const FormulaCell& f) {
  return is_conjunction(f) || is_disjunction(f);
}

bool is_negation(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Not;
}

bool is_forall(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Forall;
}

bool is_isnan(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Isnan;
}

shared_ptr<RelationalFormulaCell> to_relational(
    const shared_ptr<FormulaCell> f_ptr) {
  DRAKE_ASSERT(is_relational(*f_ptr));
  return static_pointer_cast<RelationalFormulaCell>(f_ptr);
}

shared_ptr<RelationalFormulaCell> to_relational(const Formula& f) {
  return to_relational(f.ptr_);
}

shared_ptr<NaryFormulaCell> to_nary(const shared_ptr<FormulaCell> f_ptr) {
  DRAKE_ASSERT(is_nary(*f_ptr));
  return static_pointer_cast<NaryFormulaCell>(f_ptr);
}

shared_ptr<NaryFormulaCell> to_nary(const Formula& f) {
  return to_nary(f.ptr_);
}

shared_ptr<FormulaNot> to_negation(const shared_ptr<FormulaCell> f_ptr) {
  DRAKE_ASSERT(is_negation(*f_ptr));
  return static_pointer_cast<FormulaNot>(f_ptr);
}

shared_ptr<FormulaNot> to_negation(const Formula& f) {
  return to_negation(f.ptr_);
}

shared_ptr<FormulaForall> to_forall(const shared_ptr<FormulaCell> f_ptr) {
  DRAKE_ASSERT(is_forall(*f_ptr));
  return static_pointer_cast<FormulaForall>(f_ptr);
}

shared_ptr<FormulaForall> to_forall(const Formula& f) {
  return to_forall(f.ptr_);
}

shared_ptr<FormulaIsnan> to_isnan(const shared_ptr<FormulaCell> f_ptr) {
  DRAKE_ASSERT(is_isnan(*f_ptr));
  return static_pointer_cast<FormulaIsnan>(f_ptr);
}

shared_ptr<FormulaIsnan> to_isnan(const Formula& f) { return to_isnan(f.ptr_); }

}  // namespace symbolic
}  // namespace drake
