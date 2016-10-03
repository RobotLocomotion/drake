#include "drake/common/symbolic_formula.h"

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/hash_combine.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::hash;
using std::make_shared;
using std::ostream;
using std::ostringstream;
using std::runtime_error;
using std::shared_ptr;
using std::string;

Formula::Formula(shared_ptr<FormulaCell> const ptr) : ptr_{ptr} {}

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

bool Formula::EqualTo(Formula const& f) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  DRAKE_ASSERT(f.ptr_ != nullptr);
  if (ptr_ == f.ptr_) {
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

bool Formula::Evaluate(Environment const& env) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->Evaluate(env);
}

Formula Formula::True() {
  static Formula tt{make_shared<FormulaTrue>()};
  return tt;
}
Formula Formula::False() {
  static Formula ff{make_shared<FormulaFalse>()};
  return ff;
}

Formula forall(Variables const& vars, Formula const& f) {
  return Formula{make_shared<FormulaForall>(vars, f)};
}

Formula operator&&(Formula const& f1, Formula const& f2) {
  // ff && x = ff    x && ff == ff
  if (f1.get_kind() == FormulaKind::False ||
      f2.get_kind() == FormulaKind::False) {
    return Formula::False();
  }
  // tt && f2 = f2
  if (f1.get_kind() == FormulaKind::True) {
    return f2;
  }
  // f1 && tt = f1
  if (f2.get_kind() == FormulaKind::True) {
    return f1;
  }
  return Formula{make_shared<FormulaAnd>(f1, f2)};
}

Formula operator||(Formula const& f1, Formula const& f2) {
  // tt || x = tt    x && tt == tt
  if (f1.get_kind() == FormulaKind::True ||
      f2.get_kind() == FormulaKind::True) {
    return Formula::True();
  }
  // ff && f2 = f2
  if (f1.get_kind() == FormulaKind::False) {
    return f2;
  }
  // f1 && ff = f1
  if (f2.get_kind() == FormulaKind::False) {
    return f1;
  }
  return Formula{make_shared<FormulaOr>(f1, f2)};
}

Formula operator!(Formula const& f) {
  if (f.get_kind() == FormulaKind::True) {
    return Formula::False();
  }
  if (f.get_kind() == FormulaKind::False) {
    return Formula::True();
  }
  return Formula{make_shared<FormulaNot>(f)};
}

ostream& operator<<(ostream& os, Formula const& e) {
  DRAKE_ASSERT(e.ptr_ != nullptr);
  return e.ptr_->Display(os);
}

Formula operator==(Expression const& e1, Expression const& e2) {
  // If e1 and e2 are structurally equal, simplify e1 == e2 to true.
  if (e1.EqualTo(e2)) {
    return Formula::True();
  }
  return Formula{make_shared<FormulaEq>(e1, e2)};
}
Formula operator==(double const v1, Expression const& e2) {
  return Expression{v1} == e2;
}
Formula operator==(Expression const& e1, double const v2) {
  return e1 == Expression{v2};
}

Formula operator!=(Expression const& e1, Expression const& e2) {
  // If e1 and e2 are structurally equal, simplify e1 != e2 to false.
  if (e1.EqualTo(e2)) {
    return Formula::False();
  }
  return Formula{make_shared<FormulaNeq>(e1, e2)};
}
Formula operator!=(double const v1, Expression const& e2) {
  return Expression{v1} != e2;
}
Formula operator!=(Expression const& e1, double const v2) {
  return e1 != Expression{v2};
}

Formula operator<(Expression const& e1, Expression const& e2) {
  // simplification: E < E  -->  False
  if (e1.EqualTo(e2)) {
    return Formula::False();
  }
  return Formula{make_shared<FormulaLt>(e1, e2)};
}
Formula operator<(double const v1, Expression const& e2) {
  return Expression{v1} < e2;
}
Formula operator<(Expression const& e1, double const v2) {
  return e1 < Expression{v2};
}

Formula operator<=(Expression const& e1, Expression const& e2) {
  // simplification: E <= E  -->  True
  if (e1.EqualTo(e2)) {
    return Formula::True();
  }
  return Formula{make_shared<FormulaLeq>(e1, e2)};
}
Formula operator<=(double const v1, Expression const& e2) {
  return Expression{v1} <= e2;
}
Formula operator<=(Expression const& e1, double const v2) {
  return e1 <= Expression{v2};
}

Formula operator>(Expression const& e1, Expression const& e2) {
  // simplification: E > E  -->  False
  if (e1.EqualTo(e2)) {
    return Formula::False();
  }
  return Formula{make_shared<FormulaGt>(e1, e2)};
}
Formula operator>(double const v1, Expression const& e2) {
  return Expression{v1} > e2;
}
Formula operator>(Expression const& e1, double const v2) {
  return e1 > Expression{v2};
}

Formula operator>=(Expression const& e1, Expression const& e2) {
  // simplification: E >= E  -->  True
  if (e1.EqualTo(e2)) {
    return Formula::True();
  }
  return Formula{make_shared<FormulaGeq>(e1, e2)};
}
Formula operator>=(double const v1, Expression const& e2) {
  return Expression{v1} >= e2;
}
Formula operator>=(Expression const& e1, double const v2) {
  return e1 >= Expression{v2};
}

FormulaCell::FormulaCell(FormulaKind const k, size_t const hash)
    : kind_{k}, hash_{hash_combine(static_cast<size_t>(kind_), hash)} {}

FormulaTrue::FormulaTrue()
    : FormulaCell{FormulaKind::True, hash<string>{}("True")} {}

symbolic::Variables FormulaTrue::GetFreeVariables() const {
  return symbolic::Variables{};
}

bool FormulaTrue::EqualTo(FormulaCell const& f) const {
  return get_kind() == f.get_kind();
}

bool FormulaTrue::Evaluate(Environment const& env) const { return true; }

ostream& FormulaTrue::Display(ostream& os) const {
  os << "True";
  return os;
}

FormulaFalse::FormulaFalse()
    : FormulaCell{FormulaKind::False, hash<string>{}("False")} {}

symbolic::Variables FormulaFalse::GetFreeVariables() const {
  return symbolic::Variables{};
}

bool FormulaFalse::EqualTo(FormulaCell const& f) const {
  return get_kind() == f.get_kind();
}

bool FormulaFalse::Evaluate(Environment const& env) const { return false; }

ostream& FormulaFalse::Display(ostream& os) const {
  os << "False";
  return os;
}

FormulaEq::FormulaEq(Expression const& e1, Expression const& e2)
    : FormulaCell{FormulaKind::Eq, hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

symbolic::Variables FormulaEq::GetFreeVariables() const {
  symbolic::Variables ret{e1_.GetVariables()};
  symbolic::Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool FormulaEq::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaEq const& f_eq = static_cast<FormulaEq const&>(f);
  return e1_.EqualTo(f_eq.e1_) && e2_.EqualTo(f_eq.e2_);
}

bool FormulaEq::Evaluate(Environment const& env) const {
  return e1_.Evaluate(env) == e2_.Evaluate(env);
}

ostream& FormulaEq::Display(ostream& os) const {
  os << "(" << e1_ << " = " << e2_ << ")";
  return os;
}

FormulaNeq::FormulaNeq(Expression const& e1, Expression const& e2)
    : FormulaCell{FormulaKind::Neq, hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

symbolic::Variables FormulaNeq::GetFreeVariables() const {
  symbolic::Variables ret{e1_.GetVariables()};
  symbolic::Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool FormulaNeq::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaNeq const& f_neq = static_cast<FormulaNeq const&>(f);
  return e1_.EqualTo(f_neq.e1_) && e2_.EqualTo(f_neq.e2_);
}

bool FormulaNeq::Evaluate(Environment const& env) const {
  return e1_.Evaluate(env) != e2_.Evaluate(env);
}

ostream& FormulaNeq::Display(ostream& os) const {
  os << "(" << e1_ << " = " << e2_ << ")";
  return os;
}

FormulaGt::FormulaGt(Expression const& e1, Expression const& e2)
    : FormulaCell{FormulaKind::Gt, hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

symbolic::Variables FormulaGt::GetFreeVariables() const {
  symbolic::Variables ret{e1_.GetVariables()};
  symbolic::Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool FormulaGt::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaGt const& f_gt = static_cast<FormulaGt const&>(f);
  return e1_.EqualTo(f_gt.e1_) && e2_.EqualTo(f_gt.e2_);
}

bool FormulaGt::Evaluate(Environment const& env) const {
  return e1_.Evaluate(env) > e2_.Evaluate(env);
}

ostream& FormulaGt::Display(ostream& os) const {
  os << "(" << e1_ << " > " << e2_ << ")";
  return os;
}

FormulaGeq::FormulaGeq(Expression const& e1, Expression const& e2)
    : FormulaCell{FormulaKind::Geq, hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

symbolic::Variables FormulaGeq::GetFreeVariables() const {
  symbolic::Variables ret{e1_.GetVariables()};
  symbolic::Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool FormulaGeq::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaGeq const& f_geq = static_cast<FormulaGeq const&>(f);
  return e1_.EqualTo(f_geq.e1_) && e2_.EqualTo(f_geq.e2_);
}

bool FormulaGeq::Evaluate(Environment const& env) const {
  return e1_.Evaluate(env) >= e2_.Evaluate(env);
}

ostream& FormulaGeq::Display(ostream& os) const {
  os << "(" << e1_ << " >= " << e2_ << ")";
  return os;
}

FormulaLt::FormulaLt(Expression const& e1, Expression const& e2)
    : FormulaCell{FormulaKind::Lt, hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

symbolic::Variables FormulaLt::GetFreeVariables() const {
  symbolic::Variables ret{e1_.GetVariables()};
  symbolic::Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool FormulaLt::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaLt const& f_gt = static_cast<FormulaLt const&>(f);
  return e1_.EqualTo(f_gt.e1_) && e2_.EqualTo(f_gt.e2_);
}

bool FormulaLt::Evaluate(Environment const& env) const {
  return e1_.Evaluate(env) < e2_.Evaluate(env);
}

ostream& FormulaLt::Display(ostream& os) const {
  os << "(" << e1_ << " < " << e2_ << ")";
  return os;
}

FormulaLeq::FormulaLeq(Expression const& e1, Expression const& e2)
    : FormulaCell{FormulaKind::Leq, hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

symbolic::Variables FormulaLeq::GetFreeVariables() const {
  symbolic::Variables ret{e1_.GetVariables()};
  symbolic::Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool FormulaLeq::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaLeq const& f_geq = static_cast<FormulaLeq const&>(f);
  return e1_.EqualTo(f_geq.e1_) && e2_.EqualTo(f_geq.e2_);
}

bool FormulaLeq::Evaluate(Environment const& env) const {
  return e1_.Evaluate(env) <= e2_.Evaluate(env);
}

ostream& FormulaLeq::Display(ostream& os) const {
  os << "(" << e1_ << " <= " << e2_ << ")";
  return os;
}

FormulaAnd::FormulaAnd(Formula const& f1, Formula const& f2)
    : FormulaCell{FormulaKind::And, hash_combine(f1.get_hash(), f2.get_hash())},
      f1_{f1},
      f2_{f2} {}

symbolic::Variables FormulaAnd::GetFreeVariables() const {
  symbolic::Variables ret{f1_.GetFreeVariables()};
  symbolic::Variables const res_from_f2{f2_.GetFreeVariables()};
  ret.insert(res_from_f2.begin(), res_from_f2.end());
  return ret;
}

bool FormulaAnd::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaAnd const& f_and = static_cast<FormulaAnd const&>(f);
  return f1_.EqualTo(f_and.f1_) && f2_.EqualTo(f_and.f2_);
}

bool FormulaAnd::Evaluate(Environment const& env) const {
  return f1_.Evaluate(env) && f2_.Evaluate(env);
}

ostream& FormulaAnd::Display(ostream& os) const {
  os << "(" << f1_ << " and " << f2_ << ")";
  return os;
}

FormulaOr::FormulaOr(Formula const& f1, Formula const& f2)
    : FormulaCell{FormulaKind::Or, hash_combine(f1.get_hash(), f2.get_hash())},
      f1_{f1},
      f2_{f2} {}

symbolic::Variables FormulaOr::GetFreeVariables() const {
  symbolic::Variables ret{f1_.GetFreeVariables()};
  symbolic::Variables const res_from_f2{f2_.GetFreeVariables()};
  ret.insert(res_from_f2.begin(), res_from_f2.end());
  return ret;
}

bool FormulaOr::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaOr const& f_or = static_cast<FormulaOr const&>(f);
  return f1_.EqualTo(f_or.f1_) && f2_.EqualTo(f_or.f2_);
}

bool FormulaOr::Evaluate(Environment const& env) const {
  return f1_.Evaluate(env) || f2_.Evaluate(env);
}

ostream& FormulaOr::Display(ostream& os) const {
  os << "(" << f1_ << " or " << f2_ << ")";
  return os;
}

FormulaNot::FormulaNot(Formula const& f)
    : FormulaCell{FormulaKind::Not, f.get_hash()}, f_{f} {}

symbolic::Variables FormulaNot::GetFreeVariables() const {
  return f_.GetFreeVariables();
}

bool FormulaNot::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaNot const& f_not = static_cast<FormulaNot const&>(f);
  return f_.EqualTo(f_not.f_);
}

bool FormulaNot::Evaluate(Environment const& env) const {
  return !f_.Evaluate(env);
}

ostream& FormulaNot::Display(ostream& os) const {
  os << "!(" << f_ << ")";
  return os;
}

FormulaForall::FormulaForall(Variables const& vars, Formula const& f)
    : FormulaCell{FormulaKind::Forall,
                  hash_combine(vars.get_hash(), f.get_hash())},
      vars_{vars},
      f_{f} {}

symbolic::Variables FormulaForall::GetFreeVariables() const {
  return f_.GetFreeVariables() - vars_;
}

bool FormulaForall::EqualTo(FormulaCell const& f) const {
  if (get_kind() != f.get_kind()) {
    return false;
  }
  FormulaForall const& f_forall = static_cast<FormulaForall const&>(f);
  return vars_ == f_forall.vars_ && f_.EqualTo(f_forall.f_);
}

bool FormulaForall::Evaluate(Environment const& env) const {
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

namespace std {
string to_string(drake::symbolic::Formula const& f) {
  ostringstream oss;
  oss << f;
  return oss.str();
}
}  // namespace std
