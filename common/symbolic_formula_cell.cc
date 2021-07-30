#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_formula_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

using std::equal;
using std::lexicographical_compare;
using std::ostream;
using std::ostringstream;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;

FormulaCell::FormulaCell(const FormulaKind k) : kind_{k} {}

RelationalFormulaCell::RelationalFormulaCell(const FormulaKind k,
                                             Expression lhs, Expression rhs)
    : FormulaCell{k}, e_lhs_{std::move(lhs)}, e_rhs_{std::move(rhs)} {}

void RelationalFormulaCell::HashAppendDetail(DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  hash_append(*hasher, e_lhs_);
  hash_append(*hasher, e_rhs_);
}

Variables RelationalFormulaCell::GetFreeVariables() const {
  Variables ret{e_lhs_.GetVariables()};
  ret.insert(e_rhs_.GetVariables());
  return ret;
}

bool RelationalFormulaCell::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const auto& rel_f = static_cast<const RelationalFormulaCell&>(f);
  return e_lhs_.EqualTo(rel_f.e_lhs_) && e_rhs_.EqualTo(rel_f.e_rhs_);
}

bool RelationalFormulaCell::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const auto& rel_f = static_cast<const RelationalFormulaCell&>(f);
  if (e_lhs_.Less(rel_f.e_lhs_)) {
    return true;
  }
  if (rel_f.e_lhs_.Less(e_lhs_)) {
    return false;
  }
  return e_rhs_.Less(rel_f.e_rhs_);
}

NaryFormulaCell::NaryFormulaCell(const FormulaKind k, set<Formula> formulas)
    : FormulaCell{k}, formulas_{std::move(formulas)} {}

void NaryFormulaCell::HashAppendDetail(DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  hash_append(*hasher, formulas_);
}

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
  const auto& nary_f = static_cast<const NaryFormulaCell&>(f);
  return equal(
      formulas_.cbegin(), formulas_.cend(), nary_f.formulas_.cbegin(),
      nary_f.formulas_.cend(),
      [](const Formula& f1, const Formula& f2) { return f1.EqualTo(f2); });
}

bool NaryFormulaCell::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const auto& nary_f = static_cast<const NaryFormulaCell&>(f);
  return lexicographical_compare(
      formulas_.cbegin(), formulas_.cend(), nary_f.formulas_.cbegin(),
      nary_f.formulas_.cend(),
      [](const Formula& f1, const Formula& f2) { return f1.Less(f2); });
}

ostream& NaryFormulaCell::DisplayWithOp(ostream& os, const string& op) const {
  const set<Formula>& formulas{get_operands()};
  auto it(formulas.cbegin());
  DRAKE_ASSERT(formulas.size() > 1U);
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

FormulaTrue::FormulaTrue() : FormulaCell{FormulaKind::True} {}

void FormulaTrue::HashAppendDetail(DelegatingHasher*) const {}

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

bool FormulaTrue::Evaluate(const Environment&) const { return true; }

Formula FormulaTrue::Substitute(const Substitution&) const {
  return Formula::True();
}

ostream& FormulaTrue::Display(ostream& os) const { return os << "True"; }

FormulaFalse::FormulaFalse() : FormulaCell{FormulaKind::False} {}

void FormulaFalse::HashAppendDetail(DelegatingHasher*) const {}

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

bool FormulaFalse::Evaluate(const Environment&) const { return false; }

Formula FormulaFalse::Substitute(const Substitution&) const {
  return Formula::False();
}

ostream& FormulaFalse::Display(ostream& os) const { return os << "False"; }

FormulaVar::FormulaVar(Variable v)
    : FormulaCell{FormulaKind::Var}, var_{std::move(v)} {
  // Dummy symbolic variable (ID = 0) should not be used in constructing
  // symbolic formulas.
  DRAKE_DEMAND(!var_.is_dummy());
  DRAKE_DEMAND(var_.get_type() == Variable::Type::BOOLEAN);
}

void FormulaVar::HashAppendDetail(DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  hash_append(*hasher, var_);
}

Variables FormulaVar::GetFreeVariables() const { return Variables{var_}; }

bool FormulaVar::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaVar& f_var{static_cast<const FormulaVar&>(f)};
  return var_.equal_to(f_var.var_);
}

bool FormulaVar::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaVar& f_var{static_cast<const FormulaVar&>(f)};
  return var_.less(f_var.var_);
}

bool FormulaVar::Evaluate(const Environment& env) const {
  const Environment::const_iterator it{env.find(var_)};
  if (it != env.cend()) {
    return static_cast<bool>(it->second);
  } else {
    ostringstream oss;
    oss << "The following environment does not have an entry for the "
           "variable "
        << var_ << "\n";
    oss << env << "\n";
    throw runtime_error(oss.str());
  }
}

Formula FormulaVar::Substitute(const Substitution&) const {
  // TODO(soonho-tri): Add a substitution (Variable -> Formula) and use it
  // here. For now, `Substitute` does nothing for Boolean variables.
  return Formula{var_};
}

ostream& FormulaVar::Display(ostream& os) const { return os << var_; }

const Variable& FormulaVar::get_variable() const { return var_; }

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
  return os << "(" << get_lhs_expression() << " == " << get_rhs_expression()
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
  DRAKE_ASSERT(get_operands().size() > 1U);
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
  DRAKE_ASSERT(get_operands().size() > 1U);
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

FormulaNot::FormulaNot(Formula f)
    : FormulaCell{FormulaKind::Not}, f_{std::move(f)} {}

void FormulaNot::HashAppendDetail(DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  hash_append(*hasher, f_);
}

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

FormulaForall::FormulaForall(Variables vars, Formula f)
    : FormulaCell{FormulaKind::Forall},
      vars_{std::move(vars)},
      f_{std::move(f)} {}

void FormulaForall::HashAppendDetail(DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  hash_append(*hasher, vars_);
  hash_append(*hasher, f_);
}

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

bool FormulaForall::Evaluate(const Environment&) const {
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

FormulaIsnan::FormulaIsnan(Expression e)
    : FormulaCell{FormulaKind::Isnan}, e_{std::move(e)} {}

void FormulaIsnan::HashAppendDetail(DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  hash_append(*hasher, e_);
}

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

namespace {
bool IsSymmetric(const Eigen::Ref<const MatrixX<Expression>>& m) {
  const int dim = m.rows();
  if (m.cols() != dim) {
    return false;
  }
  for (int i = 0; i < dim; ++i) {
    for (int j = i + 1; j < dim; ++j) {
      if (!m(i, j).EqualTo(m(j, i))) {
        return false;
      }
    }
  }
  return true;
}
}  // namespace

FormulaPositiveSemidefinite::FormulaPositiveSemidefinite(
    const Eigen::Ref<const MatrixX<Expression>>& m)
    : FormulaCell{FormulaKind::PositiveSemidefinite}, m_{m} {
  if (!IsSymmetric(m)) {
    ostringstream oss;
    oss << "The following matrix is not symmetric and cannot be used to "
           "construct drake::symbolic::FormulaPositiveSemidefinite:\n"
        << m;
    throw std::runtime_error(oss.str());
  }
}

namespace {
// Helper Eigen-visitor class that we use to implement
// FormulaPositiveSemidefinite::GetFreeVariables().
struct VariablesCollector {
  using Index = Eigen::Index;

  // Called for the first coefficient.
  void init(const Expression& e, Index i, Index j) {
    DRAKE_ASSERT(vars_.empty());
    return operator()(e, i, j);
  }
  // Called for all other coefficients.
  void operator()(const Expression& e, Index /* i */, Index /* j */) {
    vars_ += e.GetVariables();
  }

  Variables vars_;
};
}  // namespace

void FormulaPositiveSemidefinite::HashAppendDetail(
    DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  // Computes a hash of a matrix only using its lower-triangular part.
  for (int i = 0; i < m_.rows(); ++i) {
    for (int j = 0; j <= i; ++j) {
      hash_append(*hasher, m_(i, j));
    }
  }
  hash_append(*hasher, m_.size());
}

Variables FormulaPositiveSemidefinite::GetFreeVariables() const {
  VariablesCollector vc;
  m_.visit(vc);
  return vc.vars_;
}

bool FormulaPositiveSemidefinite::EqualTo(const FormulaCell& f) const {
  // Formula::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaPositiveSemidefinite& f_psd{
      static_cast<const FormulaPositiveSemidefinite&>(f)};
  return (m_.rows() == f_psd.m_.rows()) && (m_.cols() == f_psd.m_.cols()) &&
         CheckStructuralEquality(m_, f_psd.m_);
}

bool FormulaPositiveSemidefinite::Less(const FormulaCell& f) const {
  // Formula::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == f.get_kind());
  const FormulaPositiveSemidefinite& f_psd{
      static_cast<const FormulaPositiveSemidefinite&>(f)};

  // Compare rows.
  if (m_.rows() < f_psd.m_.rows()) {
    return true;
  }
  if (f_psd.m_.rows() < m_.rows()) {
    return false;
  }

  // No need to compare cols since m_ and f_psd.m_ are square matrices.
  DRAKE_ASSERT(m_.rows() == f_psd.m_.rows() && m_.cols() == f_psd.m_.cols());

  // Element-wise comparison.
  const int num_of_elements = m_.rows() * m_.cols();
  // clang-format off
  return lexicographical_compare(
      m_.data(), m_.data() + num_of_elements,
      f_psd.m_.data(), f_psd.m_.data() + num_of_elements,
      [](const Expression& e1, const Expression& e2) { return e1.Less(e2); });
  // clang-format on
}

bool FormulaPositiveSemidefinite::Evaluate(const Environment&) const {
  // Need to check if xᵀ m x ≥ * 0 for all vector x ∈ ℝⁿ.
  // TODO(Soonho): implement this when we have SMT/delta-SMT support.
  throw runtime_error(
      "Checking positive_semidefinite(M) is not yet implemented.");
}

Formula FormulaPositiveSemidefinite::Substitute(const Substitution& s) const {
  return positive_semidefinite(
      m_.unaryExpr([&s](const Expression& e) { return e.Substitute(s); }));
}

ostream& FormulaPositiveSemidefinite::Display(ostream& os) const {
  return os << "positive_semidefinite(" << m_ << ")";
}

bool is_false(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::False;
}

bool is_true(const FormulaCell& f) { return f.get_kind() == FormulaKind::True; }

bool is_variable(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::Var;
}

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

bool is_positive_semidefinite(const FormulaCell& f) {
  return f.get_kind() == FormulaKind::PositiveSemidefinite;
}

shared_ptr<const FormulaFalse> to_false(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_false(*f_ptr));
  return static_pointer_cast<const FormulaFalse>(f_ptr);
}

shared_ptr<const FormulaFalse> to_false(const Formula& f) {
  return to_false(f.ptr_);
}

shared_ptr<const FormulaTrue> to_true(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_true(*f_ptr));
  return static_pointer_cast<const FormulaTrue>(f_ptr);
}

shared_ptr<const FormulaTrue> to_true(const Formula& f) {
  return to_true(f.ptr_);
}

shared_ptr<const FormulaVar> to_variable(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_variable(*f_ptr));
  return static_pointer_cast<const FormulaVar>(f_ptr);
}

shared_ptr<const FormulaVar> to_variable(const Formula& f) {
  return to_variable(f.ptr_);
}

shared_ptr<const RelationalFormulaCell> to_relational(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_relational(*f_ptr));
  return static_pointer_cast<const RelationalFormulaCell>(f_ptr);
}

shared_ptr<const RelationalFormulaCell> to_relational(const Formula& f) {
  return to_relational(f.ptr_);
}

shared_ptr<const FormulaEq> to_equal_to(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_equal_to(*f_ptr));
  return static_pointer_cast<const FormulaEq>(f_ptr);
}

shared_ptr<const FormulaEq> to_equal_to(const Formula& f) {
  return to_equal_to(f.ptr_);
}

shared_ptr<const FormulaNeq> to_not_equal_to(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_not_equal_to(*f_ptr));
  return static_pointer_cast<const FormulaNeq>(f_ptr);
}

shared_ptr<const FormulaNeq> to_not_equal_to(const Formula& f) {
  return to_not_equal_to(f.ptr_);
}

shared_ptr<const FormulaGt> to_greater_than(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_greater_than(*f_ptr));
  return static_pointer_cast<const FormulaGt>(f_ptr);
}

shared_ptr<const FormulaGt> to_greater_than(const Formula& f) {
  return to_greater_than(f.ptr_);
}

shared_ptr<const FormulaGeq> to_greater_than_or_equal_to(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_greater_than_or_equal_to(*f_ptr));
  return static_pointer_cast<const FormulaGeq>(f_ptr);
}

shared_ptr<const FormulaGeq> to_greater_than_or_equal_to(const Formula& f) {
  return to_greater_than_or_equal_to(f.ptr_);
}

shared_ptr<const FormulaLt> to_less_than(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_less_than(*f_ptr));
  return static_pointer_cast<const FormulaLt>(f_ptr);
}

shared_ptr<const FormulaLt> to_less_than(const Formula& f) {
  return to_less_than(f.ptr_);
}

shared_ptr<const FormulaLeq> to_less_than_or_equal_to(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_less_than_or_equal_to(*f_ptr));
  return static_pointer_cast<const FormulaLeq>(f_ptr);
}

shared_ptr<const FormulaLeq> to_less_than_or_equal_to(const Formula& f) {
  return to_less_than_or_equal_to(f.ptr_);
}

shared_ptr<const NaryFormulaCell> to_nary(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_nary(*f_ptr));
  return static_pointer_cast<const NaryFormulaCell>(f_ptr);
}

shared_ptr<const NaryFormulaCell> to_nary(const Formula& f) {
  return to_nary(f.ptr_);
}

shared_ptr<const FormulaAnd> to_conjunction(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_conjunction(*f_ptr));
  return static_pointer_cast<const FormulaAnd>(f_ptr);
}

shared_ptr<const FormulaAnd> to_conjunction(const Formula& f) {
  return to_conjunction(f.ptr_);
}

shared_ptr<const FormulaOr> to_disjunction(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_disjunction(*f_ptr));
  return static_pointer_cast<const FormulaOr>(f_ptr);
}

shared_ptr<const FormulaOr> to_disjunction(const Formula& f) {
  return to_disjunction(f.ptr_);
}

shared_ptr<const FormulaNot> to_negation(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_negation(*f_ptr));
  return static_pointer_cast<const FormulaNot>(f_ptr);
}

shared_ptr<const FormulaNot> to_negation(const Formula& f) {
  return to_negation(f.ptr_);
}

shared_ptr<const FormulaForall> to_forall(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_forall(*f_ptr));
  return static_pointer_cast<const FormulaForall>(f_ptr);
}

shared_ptr<const FormulaForall> to_forall(const Formula& f) {
  return to_forall(f.ptr_);
}

shared_ptr<const FormulaIsnan> to_isnan(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_isnan(*f_ptr));
  return static_pointer_cast<const FormulaIsnan>(f_ptr);
}

shared_ptr<const FormulaIsnan> to_isnan(const Formula& f) {
  return to_isnan(f.ptr_);
}

shared_ptr<const FormulaPositiveSemidefinite> to_positive_semidefinite(
    const shared_ptr<const FormulaCell>& f_ptr) {
  DRAKE_ASSERT(is_positive_semidefinite(*f_ptr));
  return static_pointer_cast<const FormulaPositiveSemidefinite>(f_ptr);
}

shared_ptr<const FormulaPositiveSemidefinite> to_positive_semidefinite(
    const Formula& f) {
  return to_positive_semidefinite(f.ptr_);
}

}  // namespace symbolic
}  // namespace drake
