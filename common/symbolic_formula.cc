// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <cstddef>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic.h"
#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_formula_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

namespace drake {
namespace symbolic {

using std::make_shared;
using std::numeric_limits;
using std::ostream;
using std::ostringstream;
using std::set;
using std::shared_ptr;
using std::string;

bool operator<(FormulaKind k1, FormulaKind k2) {
  return static_cast<int>(k1) < static_cast<int>(k2);
}

Formula::Formula(std::shared_ptr<const FormulaCell> ptr)
    : ptr_{std::move(ptr)} {}

Formula::Formula(const Variable& var)
    : ptr_{make_shared<const FormulaVar>(var)} {}

FormulaKind Formula::get_kind() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->get_kind();
}

void Formula::HashAppend(DelegatingHasher* hasher) const {
  using drake::hash_append;
  hash_append(*hasher, get_kind());
  ptr_->HashAppendDetail(hasher);
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

bool Formula::Evaluate(const Environment& env,
                       RandomGenerator* const random_generator) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  if (random_generator == nullptr) {
    return ptr_->Evaluate(env);
  } else {
    return ptr_->Evaluate(
        PopulateRandomVariables(env, GetFreeVariables(), random_generator));
  }
}

bool Formula::Evaluate(RandomGenerator* const random_generator) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return Evaluate(Environment{}, random_generator);
}

Formula Formula::Substitute(const Variable& var, const Expression& e) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return Formula{ptr_->Substitute({{var, e}})};
}

Formula Formula::Substitute(const Substitution& s) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  if (!s.empty()) {
    return Formula{ptr_->Substitute(s)};
  }
  return *this;
}

string Formula::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

Formula Formula::True() {
  static Formula tt{make_shared<const FormulaTrue>()};
  return tt;
}
Formula Formula::False() {
  static Formula ff{make_shared<const FormulaFalse>()};
  return ff;
}

Formula forall(const Variables& vars, const Formula& f) {
  return Formula{make_shared<const FormulaForall>(vars, f)};
}

Formula make_conjunction(const set<Formula>& formulas) {
  set<Formula> operands;
  for (const Formula& f : formulas) {
    if (is_false(f)) {
      // Short-circuits to False.
      // f₁ ∧ ... ∧ False ∧ ... ∧ fₙ => False
      return Formula::False();
    }
    if (is_true(f)) {
      // Drop redundant True.
      // f₁ ∧ ... ∧ True ∧ ... ∧ fₙ => f₁ ∧ ... ∧ fₙ
      continue;
    }
    if (is_conjunction(f)) {
      // Flattening.
      //    f₁ ∧ ... ∧ (fᵢ₁ ∧ ... ∧ fᵢₘ) ∧ ... ∧ fₙ
      // => f₁ ∧ ... ∧ fᵢ₁ ∧ ... ∧ fᵢₘ ∧ ... ∧ fₙ
      const auto& operands_in_f = get_operands(f);
      operands.insert(operands_in_f.cbegin(), operands_in_f.cend());
    } else {
      operands.insert(f);
    }
  }
  if (operands.empty()) {
    // ⋀{} = True
    return Formula::True();
  }
  if (operands.size() == 1) {
    return *(operands.begin());
  }
  // TODO(soonho-tri): Returns False if both f and ¬f appear in operands.
  return Formula{make_shared<const FormulaAnd>(operands)};
}

Formula operator&&(const Formula& f1, const Formula& f2) {
  return make_conjunction({f1, f2});
}

Formula operator&&(const Variable& v, const Formula& f) {
  return Formula(v) && f;
}
Formula operator&&(const Formula& f, const Variable& v) {
  return f && Formula(v);
}
Formula operator&&(const Variable& v1, const Variable& v2) {
  return Formula(v1) && Formula(v2);
}

Formula make_disjunction(const set<Formula>& formulas) {
  set<Formula> operands;
  for (const Formula& f : formulas) {
    if (is_true(f)) {
      // Short-circuits to True.
      // f₁ ∨ ... ∨ True ∨ ... ∨ fₙ => True
      return Formula::True();
    }
    if (is_false(f)) {
      // Drop redundant False.
      // f₁ ∨ ... ∨ False ∨ ... ∨ fₙ => f₁ ∨ ... ∨ fₙ
      continue;
    }
    if (is_disjunction(f)) {
      // Flattening.
      //    f₁ ∨ ... ∨ (fᵢ₁ ∨ ... ∨ fᵢₘ) ∨ ... ∨ fₙ
      // => f₁ ∨ ... ∨ fᵢ₁ ∨ ... ∨ fᵢₘ ∨ ... ∨ fₙ
      const auto& operands_in_f = get_operands(f);
      operands.insert(operands_in_f.cbegin(), operands_in_f.cend());
    } else {
      operands.insert(f);
    }
  }
  if (operands.empty()) {
    // ⋁{} = False
    return Formula::False();
  }
  if (operands.size() == 1) {
    return *(operands.begin());
  }
  // TODO(soonho-tri): Returns True if both f and ¬f appear in operands.
  return Formula{make_shared<const FormulaOr>(operands)};
}

Formula operator||(const Formula& f1, const Formula& f2) {
  return make_disjunction({f1, f2});
}
Formula operator||(const Variable& v, const Formula& f) {
  return Formula(v) || f;
}
Formula operator||(const Formula& f, const Variable& v) {
  return f || Formula(v);
}
Formula operator||(const Variable& v1, const Variable& v2) {
  return Formula(v1) || Formula(v2);
}

Formula operator!(const Formula& f) {
  if (f.EqualTo(Formula::True())) {
    return Formula::False();
  }
  if (f.EqualTo(Formula::False())) {
    return Formula::True();
  }
  // Simplification: ¬(¬f₁)  =>  f₁
  if (is_negation(f)) {
    return get_operand(f);
  }
  return Formula{make_shared<const FormulaNot>(f)};
}

Formula operator!(const Variable& v) { return !Formula(v); }

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
  return Formula{make_shared<const FormulaEq>(e1, e2)};
}

Formula operator!=(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 != 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() != 0.0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<const FormulaNeq>(e1, e2)};
}

Formula operator<(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 < 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() < 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<const FormulaLt>(e1, e2)};
}

Formula operator<=(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 <= 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() <= 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<const FormulaLeq>(e1, e2)};
}

Formula operator>(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 > 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() > 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<const FormulaGt>(e1, e2)};
}

Formula operator>=(const Expression& e1, const Expression& e2) {
  // Simplification: E1 - E2 >= 0  =>  True
  const Expression diff{e1 - e2};
  if (diff.get_kind() == ExpressionKind::Constant) {
    return diff.Evaluate() >= 0 ? Formula::True() : Formula::False();
  }
  return Formula{make_shared<const FormulaGeq>(e1, e2)};
}

Formula isnan(const Expression& e) {
  return Formula{make_shared<const FormulaIsnan>(e)};
}

Formula isinf(const Expression& e) {
  const double inf{numeric_limits<double>::infinity()};
  return (-inf == e) || (e == inf);
}

Formula isfinite(const Expression& e) {
  const double inf{numeric_limits<double>::infinity()};
  return (-inf < e) && (e < inf);
}

Formula positive_semidefinite(const Eigen::Ref<const MatrixX<Expression>>& m) {
  return Formula{make_shared<const FormulaPositiveSemidefinite>(m)};
}

Formula positive_semidefinite(const MatrixX<Expression>& m,
                              const Eigen::UpLoType mode) {
  switch (mode) {
    case Eigen::Lower:
      return Formula{make_shared<const FormulaPositiveSemidefinite>(
          m.triangularView<Eigen::Lower>())};
    case Eigen::Upper:
      return Formula{make_shared<const FormulaPositiveSemidefinite>(
          m.triangularView<Eigen::Upper>())};
    default:
      throw std::runtime_error(
          "positive_semidefinite is called with a mode which is neither "
          "Eigen::Lower nor Eigen::Upper.");
  }
}

bool is_false(const Formula& f) { return is_false(*f.ptr_); }
bool is_true(const Formula& f) { return is_true(*f.ptr_); }
bool is_variable(const Formula& f) { return is_variable(*f.ptr_); }
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
bool is_positive_semidefinite(const Formula& f) {
  return is_positive_semidefinite(*f.ptr_);
}

const Variable& get_variable(const Formula& f) {
  DRAKE_ASSERT(is_variable(f));
  return to_variable(f)->get_variable();
}

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

const MatrixX<Expression>& get_matrix_in_positive_semidefinite(
    const Formula& f) {
  return to_positive_semidefinite(f)->get_matrix();
}
}  // namespace symbolic
}  // namespace drake
