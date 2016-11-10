#include "drake/common/symbolic_formula.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::equal;
using std::hash;
using std::lexicographical_compare;
using std::make_shared;
using std::ostream;
using std::ostringstream;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;

bool operator<(FormulaKind k1, FormulaKind k2) {
  return static_cast<int>(k1) < static_cast<int>(k2);
}

Formula::Formula(const shared_ptr<FormulaCell> ptr) : ptr_{ptr} {}

FormulaKind Formula::get_kind() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->get_kind();
}

size_t Formula::get_hash() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->get_hash();
}

symbolic::Variables Formula::GetFreeVariables() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->GetFreeVariables();
}

bool Formula::EqualTo(const Formula& f) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  DRAKE_ASSERT(f.ptr_ != nullptr);
  if (ptr_ == f.ptr_) {
    // pointer equality
    return true;
  }
  if (get_kind() != f.get_kind()) {
    return false;
  }
  if (get_hash() != f.get_hash()) {
    return false;
  }
  // Same kind/hash, but it could be the result of hash collision,
  // check structural equality.
  return ptr_->EqualTo(*(f.ptr_));
}

bool Formula::Less(const Formula& f) const {
  const FormulaKind k1{get_kind()};
  const FormulaKind k2{f.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  return ptr_->Less(*(f.ptr_));
}

bool Formula::Evaluate(const Environment& env) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->Evaluate(env);
}

string Formula::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

Formula Formula::True() {
  static Formula tt{make_shared<FormulaTrue>()};
  return tt;
}
Formula Formula::False() {
  static Formula ff{make_shared<FormulaFalse>()};
  return ff;
}

Formula forall(const Variables& vars, const Formula& f) {
  return Formula{make_shared<FormulaForall>(vars, f)};
}

Formula operator&&(const Formula& f1, const Formula& f2) {
  // ff && x => ff    x && ff => ff
  if (f1.EqualTo(Formula::False()) || f2.EqualTo(Formula::False())) {
    return Formula::False();
  }
  // tt && f2 => f2
  if (f1.EqualTo(Formula::True())) {
    return f2;
  }
  // f1 && tt => f1
  if (f2.EqualTo(Formula::True())) {
    return f1;
  }
  // Flattening
  if (f1.get_kind() == FormulaKind::And) {
    set<Formula> formulas1{
        static_pointer_cast<FormulaAnd>(f1.ptr_)->get_formulas()};
    if (f2.get_kind() == FormulaKind::And) {
      // (f1,1 ∧ ... f1,n) ∧ (f2,1 ∧ ... f2,m)
      // => (f1,1 ∧ ... f1,n ∧ f2,1 ∧ ... f2,m)
      const set<Formula>& formulas2{
          static_pointer_cast<FormulaAnd>(f2.ptr_)->get_formulas()};
      formulas1.insert(formulas2.begin(), formulas2.end());
    } else {
      // (f1,1 ∧ ... f1,n) ∧ f2
      // => (f1,1 ∧ ... f1,n ∧ f2)
      formulas1.insert(f2);
    }
    return Formula{make_shared<FormulaAnd>(formulas1)};
  } else {
    if (f2.get_kind() == FormulaKind::And) {
      // f1 ∧ (f2,1 ∧ ... f2,m)
      // => (f1 ∧ f2,1 ∧ ... f2,m)
      set<Formula> formulas2{
          static_pointer_cast<FormulaAnd>(f2.ptr_)->get_formulas()};
      formulas2.insert(f1);
      return Formula{make_shared<FormulaAnd>(formulas2)};
    } else {
      // Nothing to flatten.
      return Formula{make_shared<FormulaAnd>(f1, f2)};
    }
  }
}

Formula operator||(const Formula& f1, const Formula& f2) {
  // tt || x => tt    x || tt => tt
  if (f1.EqualTo(Formula::True()) || f2.EqualTo(Formula::True())) {
    return Formula::True();
  }
  // ff || f2 => f2
  if (f1.EqualTo(Formula::False())) {
    return f2;
  }
  // f1 || ff => f1
  if (f2.EqualTo(Formula::False())) {
    return f1;
  }
  // Flattening
  if (f1.get_kind() == FormulaKind::Or) {
    set<Formula> formulas{
        static_pointer_cast<FormulaOr>(f1.ptr_)->get_formulas()};
    if (f2.get_kind() == FormulaKind::Or) {
      // (f1,1 ∨ ... f1,n) ∨ (f2,1 ∨ ... f2,m)
      // => (f1,1 ∨ ... f1,n ∨ f2,1 ∨ ... f2,m)
      const set<Formula>& formulas2{
          static_pointer_cast<FormulaOr>(f2.ptr_)->get_formulas()};
      formulas.insert(formulas2.begin(), formulas2.end());
    } else {
      // (f1,1 ∨ ... f1,n) ∨ f2
      // => (f1,1 ∨ ... f1,n ∨ f2)
      formulas.insert(f2);
    }
    return Formula{make_shared<FormulaOr>(formulas)};
  } else {
    if (f2.get_kind() == FormulaKind::Or) {
      // f1 ∨ (f2,1 ∨ ... f2,m)
      // => (f1 ∨ f2,1 ∨ ... f2,m)
      set<Formula> formulas{
          static_pointer_cast<FormulaOr>(f2.ptr_)->get_formulas()};
      formulas.insert(f1);
      return Formula{make_shared<FormulaOr>(formulas)};
    } else {
      // Nothing to flatten.
      return Formula{make_shared<FormulaOr>(f1, f2)};
    }
  }
}

Formula operator!(const Formula& f) {
  if (f.EqualTo(Formula::True())) {
    return Formula::False();
  }
  if (f.EqualTo(Formula::False())) {
    return Formula::True();
  }
  return Formula{make_shared<FormulaNot>(f)};
}

ostream& operator<<(ostream& os, const Formula& e) {
  DRAKE_ASSERT(e.ptr_ != nullptr);
  return e.ptr_->Display(os);
}

Formula operator==(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 == 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() == 0.0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaEq>(e1, e2)};
}

Formula operator==(const double v1, const Expression& e2) {
  // Uses () to avoid a conflict between cpplint and clang-format.
  return (Expression{v1}) == e2;
}

Formula operator==(const Expression& e1, const double v2) {
  return e1 == Expression{v2};
}

Formula operator!=(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 != 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() != 0.0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaNeq>(e1, e2)};
}
Formula operator!=(const double v1, const Expression& e2) {
  // Uses () to avoid a conflict between cpplint and clang-format.
  return (Expression{v1}) != e2;
}
Formula operator!=(const Expression& e1, const double v2) {
  return e1 != Expression{v2};
}

Formula operator<(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 < 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() < 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaLt>(e1, e2)};
}
Formula operator<(const double v1, const Expression& e2) {
  return Expression{v1} < e2;
}
Formula operator<(const Expression& e1, const double v2) {
  return e1 < Expression{v2};
}

Formula operator<=(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 <= 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() <= 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaLeq>(e1, e2)};
}
Formula operator<=(const double v1, const Expression& e2) {
  return Expression{v1} <= e2;
}
Formula operator<=(const Expression& e1, const double v2) {
  return e1 <= Expression{v2};
}

Formula operator>(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 > 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() > 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaGt>(e1, e2)};
}
Formula operator>(const double v1, const Expression& e2) {
  return Expression{v1} > e2;
}
Formula operator>(const Expression& e1, const double v2) {
  return e1 > Expression{v2};
}

Formula operator>=(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 >= 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() >= 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaGeq>(e1, e2)};
}
Formula operator>=(const double v1, const Expression& e2) {
  return Expression{v1} >= e2;
}
Formula operator>=(const Expression& e1, const double v2) {
  return e1 >= Expression{v2};
}

FormulaCell::FormulaCell(const FormulaKind k, const size_t hash)
    : kind_{k}, hash_{hash_combine(static_cast<size_t>(kind_), hash)} {}

RelationalFormulaCell::RelationalFormulaCell(const FormulaKind k,
                                             const Expression& e1,
                                             const Expression& e2)
    : FormulaCell{k, hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

Variables RelationalFormulaCell::GetFreeVariables() const {
  Variables ret{e1_.GetVariables()};
  const Variables res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool RelationalFormulaCell::EqualTo(const FormulaCell& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  const RelationalFormulaCell& rel_f{
      static_cast<const RelationalFormulaCell&>(f)};
  return e1_.EqualTo(rel_f.e1_) && e2_.EqualTo(rel_f.e2_);
}

bool RelationalFormulaCell::Less(const FormulaCell& f) const {
  const FormulaKind k1{get_kind()};
  const FormulaKind k2{f.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  const RelationalFormulaCell& rel_f{
      static_cast<const RelationalFormulaCell&>(f)};
  if (e1_.Less(rel_f.e1_)) {
    return true;
  }
  if (rel_f.e1_.Less(e1_)) {
    return false;
  }
  return e2_.Less(rel_f.e2_);
}

NaryFormulaCell::NaryFormulaCell(const FormulaKind k,
                                 const set<Formula>& formulas)
    : FormulaCell{k, hash_value<set<Formula>>{}(formulas)},
      formulas_{formulas} {}

Variables NaryFormulaCell::GetFreeVariables() const {
  Variables ret{};
  for (const auto& f : formulas_) {
    const Variables res_from_f{f.GetFreeVariables()};
    ret.insert(res_from_f.begin(), res_from_f.end());
  }
  return ret;
}

bool NaryFormulaCell::EqualTo(const FormulaCell& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  const NaryFormulaCell& nary_f{static_cast<const NaryFormulaCell&>(f)};
  return equal(
      formulas_.cbegin(), formulas_.cend(), nary_f.formulas_.cbegin(),
      nary_f.formulas_.cend(),
      [](const Formula& f1, const Formula& f2) { return f1.EqualTo(f2); });
}

bool NaryFormulaCell::Less(const FormulaCell& f) const {
  const FormulaKind k1{get_kind()};
  const FormulaKind k2{f.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  const NaryFormulaCell& nary_f{static_cast<const NaryFormulaCell&>(f)};
  return lexicographical_compare(
      formulas_.cbegin(), formulas_.cend(), nary_f.formulas_.cbegin(),
      nary_f.formulas_.cend(),
      [](const Formula& f1, const Formula& f2) { return f1.Less(f2); });
}

ostream& NaryFormulaCell::DisplayWithOp(ostream& os, const string& op) const {
  const set<Formula>& formulas{get_formulas()};
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

symbolic::Variables FormulaTrue::GetFreeVariables() const {
  return Variables{};
}

bool FormulaTrue::EqualTo(const FormulaCell& f) const {
  return f.get_kind() == f.get_kind();
}

bool FormulaTrue::Less(const FormulaCell& f) const {
  const FormulaKind k1{get_kind()};
  const FormulaKind k2{f.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  // True < True ==> false
  return false;
}

bool FormulaTrue::Evaluate(const Environment& env) const { return true; }

ostream& FormulaTrue::Display(ostream& os) const {
  os << "True";
  return os;
}

FormulaFalse::FormulaFalse()
    : FormulaCell{FormulaKind::False, hash<string>{}("False")} {}

symbolic::Variables FormulaFalse::GetFreeVariables() const {
  return Variables{};
}

bool FormulaFalse::EqualTo(const FormulaCell& f) const {
  return get_kind() == f.get_kind();
}

bool FormulaFalse::Less(const FormulaCell& f) const {
  const FormulaKind k1{get_kind()};
  const FormulaKind k2{f.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  // False < False ==> false
  return false;
}

bool FormulaFalse::Evaluate(const Environment& env) const { return false; }

ostream& FormulaFalse::Display(ostream& os) const {
  os << "False";
  return os;
}

FormulaEq::FormulaEq(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Eq, e1, e2} {}

bool FormulaEq::Evaluate(const Environment& env) const {
  return get_1st_expression().Evaluate(env) ==
         get_2nd_expression().Evaluate(env);
}

ostream& FormulaEq::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " = " << get_2nd_expression() << ")";
  return os;
}

FormulaNeq::FormulaNeq(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Neq, e1, e2} {}

bool FormulaNeq::Evaluate(const Environment& env) const {
  return get_1st_expression().Evaluate(env) !=
         get_2nd_expression().Evaluate(env);
}

ostream& FormulaNeq::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " != " << get_2nd_expression() << ")";
  return os;
}

FormulaGt::FormulaGt(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Gt, e1, e2} {}

bool FormulaGt::Evaluate(const Environment& env) const {
  return get_1st_expression().Evaluate(env) >
         get_2nd_expression().Evaluate(env);
}

ostream& FormulaGt::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " > " << get_2nd_expression() << ")";
  return os;
}

FormulaGeq::FormulaGeq(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Geq, e1, e2} {}

bool FormulaGeq::Evaluate(const Environment& env) const {
  return get_1st_expression().Evaluate(env) >=
         get_2nd_expression().Evaluate(env);
}

ostream& FormulaGeq::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " >= " << get_2nd_expression() << ")";
  return os;
}

FormulaLt::FormulaLt(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Lt, e1, e2} {}

bool FormulaLt::Evaluate(const Environment& env) const {
  return get_1st_expression().Evaluate(env) <
         get_2nd_expression().Evaluate(env);
}

ostream& FormulaLt::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " < " << get_2nd_expression() << ")";
  return os;
}

FormulaLeq::FormulaLeq(const Expression& e1, const Expression& e2)
    : RelationalFormulaCell{FormulaKind::Leq, e1, e2} {}

bool FormulaLeq::Evaluate(const Environment& env) const {
  return get_1st_expression().Evaluate(env) <=
         get_2nd_expression().Evaluate(env);
}

ostream& FormulaLeq::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " <= " << get_2nd_expression() << ")";
  return os;
}

FormulaAnd::FormulaAnd(const set<Formula>& formulas)
    : NaryFormulaCell{FormulaKind::And, formulas} {
  DRAKE_ASSERT(get_formulas().size() > 1u);
}

FormulaAnd::FormulaAnd(const Formula& f1, const Formula& f2)
    : NaryFormulaCell{FormulaKind::And, set<Formula>{f1, f2}} {}

bool FormulaAnd::Evaluate(const Environment& env) const {
  for (const auto& f : get_formulas()) {
    if (!f.Evaluate(env)) {
      return false;
    }
  }
  return true;
}

ostream& FormulaAnd::Display(ostream& os) const {
  return DisplayWithOp(os, "and");
}

FormulaOr::FormulaOr(const set<Formula>& formulas)
    : NaryFormulaCell{FormulaKind::Or, formulas} {
  DRAKE_ASSERT(get_formulas().size() > 1u);
}

FormulaOr::FormulaOr(const Formula& f1, const Formula& f2)
    : NaryFormulaCell{FormulaKind::Or, set<Formula>{f1, f2}} {}

bool FormulaOr::Evaluate(const Environment& env) const {
  for (const auto& f : get_formulas()) {
    if (f.Evaluate(env)) {
      return true;
    }
  }
  return false;
}

ostream& FormulaOr::Display(ostream& os) const {
  return DisplayWithOp(os, "or");
}

FormulaNot::FormulaNot(const Formula& f)
    : FormulaCell{FormulaKind::Not, f.get_hash()}, f_{f} {}

symbolic::Variables FormulaNot::GetFreeVariables() const {
  return f_.GetFreeVariables();
}

bool FormulaNot::EqualTo(const FormulaCell& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  const FormulaNot& f_not{static_cast<const FormulaNot&>(f)};
  return f_.EqualTo(f_not.f_);
}

bool FormulaNot::Less(const FormulaCell& f) const {
  const FormulaKind k1{get_kind()};
  const FormulaKind k2{f.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  const FormulaNot& not_f{static_cast<const FormulaNot&>(f)};
  return f_.Less(not_f.f_);
}

bool FormulaNot::Evaluate(const Environment& env) const {
  return !f_.Evaluate(env);
}

ostream& FormulaNot::Display(ostream& os) const {
  os << "!(" << f_ << ")";
  return os;
}

FormulaForall::FormulaForall(const Variables& vars, const Formula& f)
    : FormulaCell{FormulaKind::Forall,
                  hash_combine(vars.get_hash(), f.get_hash())},
      vars_{vars},
      f_{f} {}

symbolic::Variables FormulaForall::GetFreeVariables() const {
  return f_.GetFreeVariables() - vars_;
}

bool FormulaForall::EqualTo(const FormulaCell& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  const FormulaForall& f_forall{static_cast<const FormulaForall&>(f)};
  return vars_ == f_forall.vars_ && f_.EqualTo(f_forall.f_);
}

bool FormulaForall::Less(const FormulaCell& f) const {
  const FormulaKind k1{get_kind()};
  const FormulaKind k2{f.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
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

ostream& FormulaForall::Display(ostream& os) const {
  os << "forall(" << vars_ << ". " << f_ << ")";
  return os;
}
}  // namespace symbolic
}  // namespace drake
