#include "drake/common/symbolic_formula.h"

#include <cstddef>
#include <iostream>
#include <memory>
#include <set>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula_cell.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::make_shared;
using std::ostream;
using std::ostringstream;
using std::set;
using std::shared_ptr;
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

Variables Formula::GetFreeVariables() const {
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

Formula Formula::Substitute(const Variable& var, const Expression& e) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return Formula{ptr_->Substitute({{var, e}})};
}

Formula Formula::Substitute(const Substitution& s) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  if (s.size() > 0) {
    return Formula{ptr_->Substitute(s)};
  } else {
    return *this;
  }
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
  if (is_conjunction(f1)) {
    set<Formula> formulas{get_operands(f1)};
    if (is_conjunction(f2)) {
      // (f1,1 ∧ ... f1,n) ∧ (f2,1 ∧ ... f2,m)
      // => (f1,1 ∧ ... f1,n ∧ f2,1 ∧ ... f2,m)
      const set<Formula>& formulas2{get_operands(f2)};
      formulas.insert(formulas2.begin(), formulas2.end());
    } else {
      // (f1,1 ∧ ... f1,n) ∧ f2
      // => (f1,1 ∧ ... f1,n ∧ f2)
      formulas.insert(f2);
    }
    return Formula{make_shared<FormulaAnd>(formulas)};
  } else {
    if (is_conjunction(f2)) {
      // f1 ∧ (f2,1 ∧ ... f2,m)
      // => (f1 ∧ f2,1 ∧ ... f2,m)
      set<Formula> formulas{get_operands(f2)};
      formulas.insert(f1);
      return Formula{make_shared<FormulaAnd>(formulas)};
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
  if (is_disjunction(f1)) {
    set<Formula> formulas{get_operands(f1)};
    if (is_disjunction(f2)) {
      // (f1,1 ∨ ... f1,n) ∨ (f2,1 ∨ ... f2,m)
      // => (f1,1 ∨ ... f1,n ∨ f2,1 ∨ ... f2,m)
      const set<Formula>& formulas2{get_operands(f2)};
      formulas.insert(formulas2.begin(), formulas2.end());
    } else {
      // (f1,1 ∨ ... f1,n) ∨ f2
      // => (f1,1 ∨ ... f1,n ∨ f2)
      formulas.insert(f2);
    }
    return Formula{make_shared<FormulaOr>(formulas)};
  } else {
    if (is_disjunction(f2)) {
      // f1 ∨ (f2,1 ∨ ... f2,m)
      // => (f1 ∨ f2,1 ∨ ... f2,m)
      set<Formula> formulas{get_operands(f2)};
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

ostream& operator<<(ostream& os, const Formula& f) {
  DRAKE_ASSERT(f.ptr_ != nullptr);
  return f.ptr_->Display(os);
}

Formula operator==(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 == 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() == 0.0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaEq>(e1, e2)};
}

Formula operator!=(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 != 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() != 0.0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaNeq>(e1, e2)};
}

Formula operator<(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 < 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() < 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaLt>(e1, e2)};
}

Formula operator<=(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 <= 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() <= 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaLeq>(e1, e2)};
}

Formula operator>(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 > 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() > 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaGt>(e1, e2)};
}

Formula operator>=(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 >= 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() >= 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<FormulaGeq>(e1, e2)};
}

Formula isnan(const Expression& e) {
  return Formula{make_shared<FormulaIsnan>(e)};
}

Formula operator==(const Variable& v1, const Variable& v2) {
  return Expression{v1} == Expression{v2};
}
Formula operator!=(const Variable& v1, const Variable& v2) {
  return Expression{v1} != Expression{v2};
}
Formula operator<(const Variable& v1, const Variable& v2) {
  return Expression{v1} < Expression{v2};
}
Formula operator<=(const Variable& v1, const Variable& v2) {
  return Expression{v1} <= Expression{v2};
}
Formula operator>(const Variable& v1, const Variable& v2) {
  return Expression{v1} > Expression{v2};
}
Formula operator>=(const Variable& v1, const Variable& v2) {
  return Expression{v1} >= Expression{v2};
}

bool is_false(const Formula& f) { return is_false(*f.ptr_); }
bool is_true(const Formula& f) { return is_true(*f.ptr_); }
bool is_equal_to(const Formula& f) { return is_equal_to(*f.ptr_); }
bool is_not_equal_to(const Formula& f) { return is_not_equal_to(*f.ptr_); }
bool is_greater_than(const Formula& f) { return is_greater_than(*f.ptr_); }
bool is_greater_than_or_equal_to(const Formula& f) {
  return is_greater_than_or_equal_to(*f.ptr_);
}
bool is_less_than(const Formula& f) { return is_less_than(*f.ptr_); }
bool is_less_than_or_equal_to(const Formula& f) {
  return is_less_than_or_equal_to(*f.ptr_);
}
bool is_relational(const Formula& f) {
  return is_equal_to(f) || is_not_equal_to(f) || is_greater_than(f) ||
         is_greater_than_or_equal_to(f) || is_less_than(f) ||
         is_less_than_or_equal_to(f);
}
bool is_conjunction(const Formula& f) { return is_conjunction(*f.ptr_); }
bool is_disjunction(const Formula& f) { return is_disjunction(*f.ptr_); }
bool is_nary(const Formula& f) {
  return is_conjunction(f) || is_disjunction(f);
}
bool is_negation(const Formula& f) { return is_negation(*f.ptr_); }
bool is_forall(const Formula& f) { return is_forall(*f.ptr_); }
bool is_isnan(const Formula& f) { return is_isnan(*f.ptr_); }

const Expression& get_lhs_expression(const Formula& f) {
  DRAKE_ASSERT(is_relational(f));
  return to_relational(f)->get_lhs_expression();
}
const Expression& get_rhs_expression(const Formula& f) {
  DRAKE_ASSERT(is_relational(f));
  return to_relational(f)->get_rhs_expression();
}

const set<Formula>& get_operands(const Formula& f) {
  return to_nary(f)->get_operands();
}

const Formula& get_operand(const Formula& f) {
  return to_negation(f)->get_operand();
}

const Variables& get_quantified_variables(const Formula& f) {
  return to_forall(f)->get_quantified_variables();
}

const Formula& get_quantified_formula(const Formula& f) {
  return to_forall(f)->get_quantified_formula();
}

}  // namespace symbolic
}  // namespace drake
