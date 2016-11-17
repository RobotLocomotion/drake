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
}  // namespace symbolic
}  // namespace drake
