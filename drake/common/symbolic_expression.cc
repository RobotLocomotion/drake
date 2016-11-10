#include "drake/common/symbolic_expression.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::accumulate;
using std::copy;
using std::domain_error;
using std::endl;
using std::equal;
using std::hash;
using std::invalid_argument;
using std::lexicographical_compare;
using std::make_pair;
using std::make_shared;
using std::map;
using std::numeric_limits;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::setprecision;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;

bool operator<(ExpressionKind k1, ExpressionKind k2) {
  return static_cast<int>(k1) < static_cast<int>(k2);
}

Expression::Expression(const Variable& name)
    : ptr_{make_shared<ExpressionVar>(name)} {}
Expression::Expression(const double d)
    : ptr_{make_shared<ExpressionConstant>(d)} {}
Expression::Expression(const shared_ptr<ExpressionCell> ptr) : ptr_{ptr} {}

ExpressionKind Expression::get_kind() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->get_kind();
}
size_t Expression::get_hash() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->get_hash();
}

Expression Expression::Zero() {
  static const Expression zero{0.0};
  return zero;
}

Expression Expression::One() {
  static const Expression one{1.0};
  return one;
}

Expression Expression::Pi() {
  static const Expression pi{M_PI};
  return pi;
}

Expression Expression::E() {
  static const Expression e{M_E};
  return e;
}

Variables Expression::GetVariables() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->GetVariables();
}

bool Expression::EqualTo(const Expression& e) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  DRAKE_ASSERT(e.ptr_ != nullptr);
  if (ptr_ == e.ptr_) {
    return true;
  }
  if (get_kind() != e.get_kind()) {
    return false;
  }
  if (get_hash() != e.get_hash()) {
    return false;
  }
  // Same kind/hash, but it could be the result of hash collision,
  // check structural equality.
  return ptr_->EqualTo(*(e.ptr_));
}

bool Expression::Less(const Expression& e) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  DRAKE_ASSERT(e.ptr_ != nullptr);
  if (ptr_ == e.ptr_) {
    return false;  // this equals to e, not less-than.
  }
  const ExpressionKind k1{get_kind()};
  const ExpressionKind k2{e.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  // k1 == k2
  return ptr_->Less(*(e.ptr_));
}

double Expression::Evaluate(const Environment& env) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  const double res{ptr_->Evaluate(env)};
  check_nan(res);
  return res;
}

string Expression::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

Expression operator+(Expression lhs, const Expression& rhs) {
  lhs += rhs;
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator+=(Expression& lhs, const Expression& rhs) {
  // Simplification: 0 + x => x
  if (lhs.EqualTo(Expression::Zero())) {
    lhs = rhs;
    return lhs;
  }
  // Simplification: x + 0 => x
  if (rhs.EqualTo(Expression::Zero())) {
    return lhs;
  }
  // Simplification: Expression(c1) + Expression(c2) => Expression(c1 + c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    const double v1{
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value()};
    const double v2{
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value()};
    lhs = Expression{v1 + v2};
    return lhs;
  }
  // Simplification: flattening. To build a new expression, we use
  // ExpressionAddFactory which holds intermediate terms and does
  // simplifications internally.
  ExpressionAddFactory add_factory{};
  if (lhs.get_kind() == ExpressionKind::Add) {
    // 1. (e_1 + ... + e_n) + rhs
    add_factory = static_pointer_cast<ExpressionAdd>(lhs.ptr_);
    // Note: AddExpression method takes care of the special case where `rhs` is
    // of ExpressionAdd.
    add_factory.AddExpression(rhs);
  } else {
    if (rhs.get_kind() == ExpressionKind::Add) {
      // 2. lhs + (e_1 + ... + e_n)
      add_factory = static_pointer_cast<ExpressionAdd>(rhs.ptr_);
      add_factory.AddExpression(lhs);
    } else {
      // nothing to flatten: return lhs + rhs
      add_factory.AddExpression(lhs);
      add_factory.AddExpression(rhs);
    }
  }
  // Extract an expression from factory
  lhs = add_factory.GetExpression();
  return lhs;
}

Expression& Expression::operator++() {
  *this += Expression::One();
  return *this;
}

Expression Expression::operator++(int) {
  Expression copy(*this);
  ++*this;
  return copy;
}

Expression operator-(Expression lhs, const Expression& rhs) {
  lhs -= rhs;
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator-=(Expression& lhs, const Expression& rhs) {
  // Simplification: E - E => 0
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might cause 0/0 during evaluation.
  if (lhs.EqualTo(rhs)) {
    lhs = Expression::Zero();
    return lhs;
  }
  // Simplification: x - 0 => x
  if (rhs.EqualTo(Expression::Zero())) {
    return lhs;
  }
  // Simplification: Expression(c1) - Expression(c2) => Expression(c1 - c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    const double v1{
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value()};
    const double v2{
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value()};
    lhs = Expression{v1 - v2};
    return lhs;
  }
  // x - y => x + (-y)
  lhs += -rhs;
  return lhs;
}

// Unary minus case: -(E)
Expression operator-(Expression e) {
  // Simplification: constant folding
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{-v};
  }
  // Simplification: -(-(E))  =>  E
  if (e.get_kind() == ExpressionKind::Neg) {
    return static_pointer_cast<ExpressionNeg>(e.ptr_)->get_expression();
  }
  // Simplification: push '-' inside over '+'.
  // -(E_1 + ... + E_n) => (-E_1 + ... + -E_n)
  if (e.get_kind() == ExpressionKind::Add) {
    return ExpressionAddFactory{static_pointer_cast<ExpressionAdd>(e.ptr_)}
        .Negate()
        .GetExpression();
  }
  return Expression{make_shared<ExpressionNeg>(e)};
}

Expression& Expression::operator--() {
  *this -= Expression::One();
  return *this;
}

Expression Expression::operator--(int) {
  const Expression copy(*this);
  --*this;
  return copy;
}

Expression operator*(Expression lhs, const Expression& rhs) {
  lhs *= rhs;
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator*=(Expression& lhs, const Expression& rhs) {
  // Simplification: 1 * x => x
  if (lhs.EqualTo(Expression::One())) {
    lhs = rhs;
    return lhs;
  }
  // Simplification: x * 1 => x
  if (rhs.EqualTo(Expression::One())) {
    return lhs;
  }
  // Simplification: -1 * x => -x
  if (lhs.EqualTo(-Expression::One())) {
    lhs = -rhs;
    return lhs;
  }
  // Simplification: x * -1 => -x
  if (rhs.EqualTo(-Expression::One())) {
    lhs = -lhs;
    return lhs;
  }
  // Simplification: 0 * E => 0
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might cause 0/0 during evaluation.
  if (lhs.EqualTo(Expression::Zero())) {
    return lhs;
  }
  // Simplification: E * 0 => 0
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might cause 0/0 during evaluation.
  if (rhs.EqualTo(Expression::Zero())) {
    lhs = Expression::Zero();
    return lhs;
  }
  // Pow-related simplifications.
  if (lhs.get_kind() == ExpressionKind::Pow) {
    const auto lhs_ptr(static_pointer_cast<ExpressionPow>(lhs.ptr_));
    const Expression& e1{lhs_ptr->get_first_expression()};
    if (rhs.get_kind() == ExpressionKind::Pow) {
      const auto rhs_ptr(static_pointer_cast<ExpressionPow>(rhs.ptr_));
      const Expression& e3{rhs_ptr->get_first_expression()};
      if (e1.EqualTo(e3)) {
        // Simplification: pow(e1, e2) * pow(e1, e4) => pow(e1, e2 + e4)
        // TODO(soonho-tri): This simplification is not sound. For example, x^4
        // * x^(-3) => x. The original expression `x^4 * x^(-3)` is evaluated to
        // `nan` when x = 0 while the simplified expression `x` is evaluated to
        // 0.
        const Expression& e2{lhs_ptr->get_second_expression()};
        const Expression& e4{rhs_ptr->get_second_expression()};
        lhs = pow(e1, e2 + e4);
        return lhs;
      }
    }
    if (e1.EqualTo(rhs)) {
      // Simplification: pow(e1, e2) * e1 => pow(e1, e2 + 1)
      // TODO(soonho-tri): This simplification is not sound.
      const Expression& e2{lhs_ptr->get_second_expression()};
      lhs = pow(e1, e2 + 1);
      return lhs;
    }
  } else {
    if (rhs.get_kind() == ExpressionKind::Pow) {
      const auto rhs_ptr(static_pointer_cast<ExpressionPow>(rhs.ptr_));
      const Expression& e1{rhs_ptr->get_first_expression()};
      if (e1.EqualTo(lhs)) {
        // Simplification: (lhs * rhs == e1 * pow(e1, e2)) => pow(e1, 1 + e2)
        // TODO(soonho-tri): This simplification is not sound.
        const Expression& e2{rhs_ptr->get_second_expression()};
        lhs = pow(e1, 1 + e2);
        return lhs;
      }
    }
  }
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    // Simplification: Expression(c1) * Expression(c2) => Expression(c1 * c2)
    const double v1{
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value()};
    const double v2{
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value()};
    lhs = Expression{v1 * v2};
    return lhs;
  }
  // Simplification: flattening
  ExpressionMulFactory mul_factory{};
  if (lhs.get_kind() == ExpressionKind::Mul) {
    // (e_1 * ... * e_n) * rhs
    mul_factory = static_pointer_cast<ExpressionMul>(lhs.ptr_);
    // Note: AddExpression method takes care of the special case where `rhs` is
    // of ExpressionMul.
    mul_factory.AddExpression(rhs);
  } else {
    if (rhs.get_kind() == ExpressionKind::Mul) {
      // e_1 * (e_2 * ... * e_n) -> (e_2 * ... * e_n * e_1)
      //
      // Note that we do not preserve the original ordering because * is
      // associative.
      mul_factory = static_pointer_cast<ExpressionMul>(rhs.ptr_);
      mul_factory.AddExpression(lhs);
    } else {
      // Simplification: x * x => x^2 (=pow(x,2))
      if (lhs.EqualTo(rhs)) {
        lhs = pow(lhs, 2.0);
        return lhs;
      } else {
        // nothing to flatten
        mul_factory.AddExpression(lhs);
        mul_factory.AddExpression(rhs);
      }
    }
  }
  lhs = mul_factory.GetExpression();
  return lhs;
}

Expression operator/(Expression lhs, const Expression& rhs) {
  lhs /= rhs;
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator/=(Expression& lhs, const Expression& rhs) {
  // Simplification: x / 1 => x
  if (rhs.EqualTo(Expression::One())) {
    return lhs;
  }
  // Simplification: Expression(c1) / Expression(c2) => Expression(c1 / c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    const double v1{
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value()};
    const double v2{
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value()};
    if (v2 == 0.0) {
      ostringstream oss{};
      oss << "Division by zero: " << v1 << "/" << v2;
      throw runtime_error(oss.str());
    }
    lhs = Expression{v1 / v2};
    return lhs;
  }
  // Simplification: E / E => 1
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might contain 0/0 problems.
  if (lhs.EqualTo(rhs)) {
    lhs = Expression::One();
    return lhs;
  }
  lhs.ptr_ = make_shared<ExpressionDiv>(lhs, rhs);
  return lhs;
}

ExpressionCell::ExpressionCell(ExpressionKind const k, size_t const hash)
    : kind_{k}, hash_{hash_combine(static_cast<size_t>(kind_), hash)} {}

void Expression::check_nan(const double v) {
  if (std::isnan(v)) {
    throw runtime_error("NaN is detected during Symbolic computation.");
  }
}

UnaryExpressionCell::UnaryExpressionCell(const ExpressionKind k,
                                         const Expression& e)
    : ExpressionCell{k, e.get_hash()}, e_{e} {}

Variables UnaryExpressionCell::GetVariables() const {
  return e_.GetVariables();
}

bool UnaryExpressionCell::EqualTo(const ExpressionCell& e) const {
  if (get_kind() != e.get_kind()) {
    return false;
  }
  const UnaryExpressionCell& unary_e{
      static_cast<const UnaryExpressionCell&>(e)};
  return e_.EqualTo(unary_e.e_);
}

bool UnaryExpressionCell::Less(const ExpressionCell& e) const {
  const ExpressionKind k1{get_kind()};
  const ExpressionKind k2{e.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  const UnaryExpressionCell& unary_e{
      static_cast<const UnaryExpressionCell&>(e)};
  return e_.Less(unary_e.e_);
}

double UnaryExpressionCell::Evaluate(const Environment& env) const {
  const double v{e_.Evaluate(env)};
  return DoEvaluate(v);
}

BinaryExpressionCell::BinaryExpressionCell(const ExpressionKind k,
                                           const Expression& e1,
                                           const Expression& e2)
    : ExpressionCell{k, hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

Variables BinaryExpressionCell::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  const Variables res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool BinaryExpressionCell::EqualTo(const ExpressionCell& e) const {
  if (get_kind() != e.get_kind()) {
    return false;
  }
  const BinaryExpressionCell& binary_e{
      static_cast<const BinaryExpressionCell&>(e)};
  return e1_.EqualTo(binary_e.e1_) && e2_.EqualTo(binary_e.e2_);
}

bool BinaryExpressionCell::Less(const ExpressionCell& e) const {
  const ExpressionKind k1{get_kind()};
  const ExpressionKind k2{e.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  const BinaryExpressionCell& binary_e{
      static_cast<const BinaryExpressionCell&>(e)};
  if (e1_.Less(binary_e.e1_)) {
    return true;
  }
  if (binary_e.e1_.Less(e1_)) {
    return false;
  }
  // e1_ equals to binary_e.e1_, compare e2_ and binary_e.e2_
  return e2_.Less(binary_e.e2_);
}

double BinaryExpressionCell::Evaluate(const Environment& env) const {
  const double v1{e1_.Evaluate(env)};
  const double v2{e2_.Evaluate(env)};
  return DoEvaluate(v1, v2);
}

ExpressionVar::ExpressionVar(const Variable& v)
    : ExpressionCell{ExpressionKind::Var, hash_value<Variable>{}(v)}, var_{v} {}

Variables ExpressionVar::GetVariables() const { return {get_variable()}; }

bool ExpressionVar::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Var) {
    return false;
  }
  return var_ == static_cast<const ExpressionVar&>(e).var_;
}

bool ExpressionVar::Less(const ExpressionCell& e) const {
  const ExpressionKind k1{get_kind()};
  const ExpressionKind k2{e.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  // Note the below is using the overloaded operator< between ExpressionVar
  // which is based on variable IDs.
  return var_ < static_cast<const ExpressionVar&>(e).var_;
}

double ExpressionVar::Evaluate(const Environment& env) const {
  Environment::const_iterator const it{env.find(var_)};
  if (it != env.cend()) {
    DRAKE_ASSERT(!std::isnan(it->second));
    return it->second;
  } else {
    ostringstream oss;
    oss << "The following environment does not have an entry for the "
           "variable "
        << var_ << endl;
    oss << env << endl;
    throw runtime_error(oss.str());
  }
}

ostream& ExpressionVar::Display(ostream& os) const {
  os << var_;
  return os;
}

ExpressionConstant::ExpressionConstant(const double v)
    : ExpressionCell{ExpressionKind::Constant, hash<double>{}(v)}, v_{v} {
  Expression::check_nan(v_);
}

Variables ExpressionConstant::GetVariables() const { return Variables{}; }

bool ExpressionConstant::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Constant) {
    return false;
  }
  return v_ == static_cast<const ExpressionConstant&>(e).v_;
}

bool ExpressionConstant::Less(const ExpressionCell& e) const {
  const ExpressionKind k1{get_kind()};
  const ExpressionKind k2{e.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  return v_ < static_cast<const ExpressionConstant&>(e).v_;
}

double ExpressionConstant::Evaluate(const Environment& env) const {
  DRAKE_ASSERT(!std::isnan(v_));
  return v_;
}

ostream& ExpressionConstant::Display(ostream& os) const {
  ostringstream oss;
  oss << setprecision(numeric_limits<double>::max_digits10) << v_;
  os << oss.str();
  return os;
}

ExpressionNeg::ExpressionNeg(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Neg, e} {}

ostream& ExpressionNeg::Display(ostream& os) const {
  os << "-(" << get_expression() << ")";
  return os;
}

double ExpressionNeg::DoEvaluate(const double v) const { return -v; }

ExpressionAdd::ExpressionAdd(const double constant_term,
                             const map<Expression, double>& term_to_coeff_map)
    : ExpressionCell{ExpressionKind::Add,
                     hash_combine(constant_term,
                                  hash_value<map<Expression, double>>{}(
                                      term_to_coeff_map))},
      constant_term_(constant_term),
      term_to_coeff_map_(term_to_coeff_map) {
  DRAKE_ASSERT(!term_to_coeff_map_.empty());
}

Variables ExpressionAdd::GetVariables() const {
  Variables ret{};
  for (const auto& p : term_to_coeff_map_) {
    const Variables vars_in_term{p.first.GetVariables()};
    ret.insert(vars_in_term.begin(), vars_in_term.end());
  }
  return ret;
}

bool ExpressionAdd::EqualTo(const ExpressionCell& e) const {
  if (get_kind() != e.get_kind()) {
    return false;
  }
  const ExpressionAdd& add_e{static_cast<const ExpressionAdd&>(e)};
  // Compare constant_term.
  if (constant_term_ != add_e.constant_term_) {
    return false;
  }
  return equal(term_to_coeff_map_.cbegin(), term_to_coeff_map_.cend(),
               add_e.term_to_coeff_map_.cbegin(),
               add_e.term_to_coeff_map_.cend(),
               [](const pair<Expression, double>& p1,
                  const pair<Expression, double>& p2) {
                 return p1.first.EqualTo(p2.first) && p1.second == p2.second;
               });
}

bool ExpressionAdd::Less(const ExpressionCell& e) const {
  const ExpressionKind k1{get_kind()};
  const ExpressionKind k2{e.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  DRAKE_ASSERT(e.get_kind() == ExpressionKind::Add);
  const ExpressionAdd& add_e{static_cast<const ExpressionAdd&>(e)};
  // Compare the constant_terms.
  if (constant_term_ < add_e.constant_term_) {
    return true;
  }
  if (add_e.constant_term_ < constant_term_) {
    return false;
  }
  // Compare the two maps.
  return lexicographical_compare(
      term_to_coeff_map_.cbegin(), term_to_coeff_map_.cend(),
      add_e.term_to_coeff_map_.cbegin(), add_e.term_to_coeff_map_.cend(),
      [](const pair<Expression, double>& p1,
         const pair<Expression, double>& p2) {
        const double coeff1{p1.second};
        const double coeff2{p2.second};
        if (coeff1 < coeff2) {
          return true;
        }
        if (coeff2 < coeff1) {
          return false;
        }
        const Expression& term1{p1.first};
        const Expression& term2{p2.first};
        return term1.Less(term2);
      });
}

double ExpressionAdd::Evaluate(const Environment& env) const {
  return accumulate(
      term_to_coeff_map_.begin(), term_to_coeff_map_.end(), constant_term_,
      [&env](const double init, const pair<Expression, double>& p) {
        return init + p.first.Evaluate(env) * p.second;
      });
}

ostream& ExpressionAdd::Display(ostream& os) const {
  DRAKE_ASSERT(!term_to_coeff_map_.empty());
  bool print_plus{false};
  os << "(";
  if (constant_term_ != 0.0) {
    os << constant_term_;
    print_plus = true;
  }
  for (auto& p : term_to_coeff_map_) {
    DisplayTerm(os, print_plus, p.second, p.first);
    print_plus = true;
  }
  os << ")";
  return os;
}

ostream& ExpressionAdd::DisplayTerm(ostream& os, const bool print_plus,
                                    const double coeff,
                                    const Expression& term) const {
  DRAKE_ASSERT(coeff != 0.0);
  if (coeff > 0.0) {
    if (print_plus) {
      os << " + ";
    }
    // Do not print "1 * t"
    if (coeff != 1.0) {
      os << coeff << " * ";
    }
  } else {
    // Instead of printing "+ (- E)", just print "- E".
    os << " - ";
    if (coeff != -1.0) {
      os << (-coeff) << " * ";
    }
  }
  os << term;
  return os;
}

ExpressionAddFactory::ExpressionAddFactory(
    const double constant_term,
    const map<Expression, double>& term_to_coeff_map)
    : constant_term_{constant_term}, term_to_coeff_map_{term_to_coeff_map} {}

ExpressionAddFactory::ExpressionAddFactory(
    const shared_ptr<const ExpressionAdd> ptr)
    : constant_term_{ptr->get_constant_term()},
      term_to_coeff_map_{ptr->get_term_to_coeff_map()} {}

void ExpressionAddFactory::AddExpression(const Expression& e) {
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return AddConstant(v);
  }
  if (e.get_kind() == ExpressionKind::Add) {
    // Flattening
    return Add(static_pointer_cast<ExpressionAdd>(e.ptr_));
  }
  if (e.get_kind() == ExpressionKind::Mul) {
    const auto ptr(static_pointer_cast<ExpressionMul>(e.ptr_));
    const double constant_factor{ptr->get_constant_factor()};
    if (constant_factor != 1.0) {
      // Instead of adding (1.0 * (constant_factor * b1^t1 ... bn^tn)),
      // add (constant_factor, 1.0 * b1^t1 ... bn^tn).
      return AddTerm(constant_factor,
                     ExpressionMulFactory(1.0, ptr->get_term_to_exp_map())
                         .GetExpression());
    }
  }
  return AddTerm(1.0, e);
}

void ExpressionAddFactory::Add(const shared_ptr<const ExpressionAdd> ptr) {
  AddConstant(ptr->get_constant_term());
  AddMap(ptr->get_term_to_coeff_map());
}

ExpressionAddFactory& ExpressionAddFactory::operator=(
    const shared_ptr<ExpressionAdd> ptr) {
  constant_term_ = ptr->get_constant_term();
  term_to_coeff_map_ = ptr->get_term_to_coeff_map();
  return *this;
}

ExpressionAddFactory& ExpressionAddFactory::Negate() {
  constant_term_ = -constant_term_;
  for (auto& p : term_to_coeff_map_) {
    p.second = -p.second;
  }
  return *this;
}

Expression ExpressionAddFactory::GetExpression() const {
  if (term_to_coeff_map_.empty()) {
    return Expression{constant_term_};
  }
  if (constant_term_ == 0.0 && term_to_coeff_map_.size() == 1u) {
    // 0.0 + c1 * t1 -> c1 * t1
    const auto it(term_to_coeff_map_.cbegin());
    return it->first * it->second;
  }
  return Expression{
      make_shared<ExpressionAdd>(constant_term_, term_to_coeff_map_)};
}

void ExpressionAddFactory::AddConstant(const double constant_term) {
  constant_term_ += constant_term;
}

void ExpressionAddFactory::AddTerm(const double coeff, const Expression& term) {
  DRAKE_ASSERT(term.get_kind() != ExpressionKind::Constant);

  // If (term, coeff) = (-E, coeff), add (E, -coeff) by recursive call.
  if (term.get_kind() == ExpressionKind::Neg) {
    return AddTerm(
        -coeff,
        static_pointer_cast<ExpressionNeg>(term.ptr_)->get_expression());
  }

  const auto it(term_to_coeff_map_.find(term));
  if (it != term_to_coeff_map_.end()) {
    // Case1: term is already in the map
    double& this_coeff{it->second};
    this_coeff += coeff;
    if (this_coeff == 0.0) {
      // If the coefficient becomes zero, remove the entry.
      // TODO(soonho-tri): The following operation is not sound since it cancels
      // `term` which might contain 0/0 problems.
      term_to_coeff_map_.erase(it);
    }
  } else {
    // Case2: term is not found in term_to_coeff_map_.
    // Add the entry (term, coeff).
    term_to_coeff_map_.emplace(term, coeff);
  }
}

void ExpressionAddFactory::AddMap(
    const map<Expression, double> term_to_coeff_map) {
  for (const auto& p : term_to_coeff_map) {
    AddTerm(p.second, p.first);
  }
}

ExpressionMul::ExpressionMul(const double constant_factor,
                             const map<Expression, Expression>& term_to_exp_map)
    : ExpressionCell{ExpressionKind::Mul,
                     hash_combine(constant_factor,
                                  hash_value<map<Expression, Expression>>{}(
                                      term_to_exp_map))},
      constant_factor_(constant_factor),
      term_to_exp_map_(term_to_exp_map) {
  DRAKE_ASSERT(!term_to_exp_map_.empty());
}

Variables ExpressionMul::GetVariables() const {
  Variables ret{};
  for (const auto& p : term_to_exp_map_) {
    const Variables vars_in_base{p.first.GetVariables()};
    const Variables vars_in_exp{p.second.GetVariables()};
    ret.insert(vars_in_base.begin(), vars_in_base.end());
    ret.insert(vars_in_exp.begin(), vars_in_exp.end());
  }
  return ret;
}

bool ExpressionMul::EqualTo(const ExpressionCell& e) const {
  if (get_kind() != e.get_kind()) {
    return false;
  }
  const ExpressionMul& mul_e{static_cast<const ExpressionMul&>(e)};
  // Compare constant_factor.
  if (constant_factor_ != mul_e.constant_factor_) {
    return false;
  }
  // Check each (term, coeff) pairs in two maps.
  return equal(term_to_exp_map_.cbegin(), term_to_exp_map_.cend(),
               mul_e.term_to_exp_map_.cbegin(), mul_e.term_to_exp_map_.cend(),
               [](const pair<Expression, Expression>& p1,
                  const pair<Expression, Expression>& p2) {
                 return p1.first.EqualTo(p2.first) &&
                        p1.second.EqualTo(p2.second);
               });
}

bool ExpressionMul::Less(const ExpressionCell& e) const {
  const ExpressionKind k1{get_kind()};
  const ExpressionKind k2{e.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  DRAKE_ASSERT(e.get_kind() == ExpressionKind::Mul);
  const ExpressionMul& mul_e{static_cast<const ExpressionMul&>(e)};
  // Compare the constant_factors.
  if (constant_factor_ < mul_e.constant_factor_) {
    return true;
  }
  if (mul_e.constant_factor_ < constant_factor_) {
    return false;
  }
  // Compare the two maps.
  return lexicographical_compare(
      term_to_exp_map_.cbegin(), term_to_exp_map_.cend(),
      mul_e.term_to_exp_map_.cbegin(), mul_e.term_to_exp_map_.cend(),
      [](const pair<Expression, Expression>& p1,
         const pair<Expression, Expression>& p2) {
        const Expression& base1{p1.first};
        const Expression& base2{p2.first};
        if (base1.Less(base2)) {
          return true;
        }
        if (base2.Less(base1)) {
          return false;
        }
        const Expression& exp1{p1.second};
        const Expression& exp2{p2.second};
        return exp1.Less(exp2);
      });
}

double ExpressionMul::Evaluate(const Environment& env) const {
  return accumulate(
      term_to_exp_map_.begin(), term_to_exp_map_.end(), constant_factor_,
      [&env](const double init, const pair<Expression, Expression>& p) {
        return init * std::pow(p.first.Evaluate(env), p.second.Evaluate(env));
      });
}

ostream& ExpressionMul::Display(ostream& os) const {
  DRAKE_ASSERT(!term_to_exp_map_.empty());
  bool print_mul{false};
  os << "(";
  if (constant_factor_ != 1.0) {
    os << constant_factor_;
    print_mul = true;
  }
  for (auto& p : term_to_exp_map_) {
    DisplayTerm(os, print_mul, p.first, p.second);
    print_mul = true;
  }
  os << ")";
  return os;
}

ostream& ExpressionMul::DisplayTerm(ostream& os, const bool print_mul,
                                    const Expression& base,
                                    const Expression& exponent) const {
  // Print " * pow(base, exponent)" if print_mul is true
  // Print "pow(base, exponent)" if print_mul is false
  // Print "base" instead of "pow(base, exponent)" if exponent == 1.0
  if (print_mul) {
    os << " * ";
  }
  if (exponent.EqualTo(Expression::One())) {
    os << base;
  } else {
    os << "pow(" << base << ", " << exponent << ")";
  }
  return os;
}

ExpressionMulFactory::ExpressionMulFactory(
    const double constant_factor,
    const map<Expression, Expression>& term_to_exp_map)
    : constant_factor_{constant_factor}, term_to_exp_map_{term_to_exp_map} {}

void ExpressionMulFactory::AddExpression(const Expression& e) {
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return AddConstant(v);
  }
  if (e.get_kind() == ExpressionKind::Mul) {
    // Flattening
    return Add(static_pointer_cast<ExpressionMul>(e.ptr_));
  }
  // Add e^1
  return AddTerm(e, Expression{1.0});
}

void ExpressionMulFactory::Add(const shared_ptr<const ExpressionMul> ptr) {
  AddConstant(ptr->get_constant_factor());
  AddMap(ptr->get_term_to_exp_map());
}

ExpressionMulFactory& ExpressionMulFactory::operator=(
    const shared_ptr<ExpressionMul> ptr) {
  constant_factor_ = ptr->get_constant_factor();
  term_to_exp_map_ = ptr->get_term_to_exp_map();
  return *this;
}

Expression ExpressionMulFactory::GetExpression() const {
  if (term_to_exp_map_.empty()) {
    return Expression{constant_factor_};
  }
  if (constant_factor_ == 1.0 && term_to_exp_map_.size() == 1u) {
    // 1.0 * c1^t1 -> c1^t1
    const auto it(term_to_exp_map_.cbegin());
    return pow(it->first, it->second);
  }
  return Expression{
      make_shared<ExpressionMul>(constant_factor_, term_to_exp_map_)};
}

void ExpressionMulFactory::AddConstant(const double constant_factor) {
  constant_factor_ *= constant_factor;
}

void ExpressionMulFactory::AddTerm(const Expression& base,
                                   const Expression& exponent) {
  // Case: both of base and exponent are constant, multiply this by pow(base,
  // exponent).
  // Example: (4 * x^2) * (3^2) => (4 * (3^2)) * x^2
  if (base.get_kind() == ExpressionKind::Constant &&
      exponent.get_kind() == ExpressionKind::Constant) {
    constant_factor_ *= pow(base, exponent).Evaluate();
    return;
  }
  const auto it(term_to_exp_map_.find(base));
  if (it != term_to_exp_map_.end()) {
    // base is already in map.
    // (= b1^e1 * ... * (base^this_exponent) * ... * en^bn).
    // Update it to be (... * (base^(this_exponent + exponent)) * ...)
    // Example: x^3 * x^2 => x^5
    Expression& this_exponent{it->second};
    this_exponent += exponent;
    if (this_exponent.EqualTo(Expression::Zero())) {
      // If it ends up with base^0 (= 1.0) then remove this entry from the map.
      // TODO(soonho-tri): The following operation is not sound since it can
      // cancels `base` which might include 0/0 problems.
      term_to_exp_map_.erase(it);
    }
  } else {
    // Product is not found in term_to_exp_map_. Add the entry (base, exponent).
    if (base.get_kind() == ExpressionKind::Pow) {
      // If (base, exponent) = (pow(e1, e2), exponent)), then add (e1, e2 *
      // exponent)
      // Example: (x^2)^3 => x^(2 * 3)
      const Expression& e1{static_pointer_cast<ExpressionPow>(base.ptr_)
                               ->get_first_expression()};
      const Expression& e2{static_pointer_cast<ExpressionPow>(base.ptr_)
                               ->get_second_expression()};
      term_to_exp_map_.emplace(e1, e2 * exponent);
    } else {
      term_to_exp_map_.emplace(base, exponent);
    }
  }
}

void ExpressionMulFactory::AddMap(
    const map<Expression, Expression> term_to_exp_map) {
  for (const auto& p : term_to_exp_map) {
    AddTerm(p.first, p.second);
  }
}

ExpressionDiv::ExpressionDiv(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Div, e1, e2} {}

ostream& ExpressionDiv::Display(ostream& os) const {
  os << "(" << get_first_expression() << " / " << get_second_expression()
     << ")";
  return os;
}

double ExpressionDiv::DoEvaluate(const double v1, const double v2) const {
  if (v2 == 0.0) {
    ostringstream oss;
    oss << "Division by zero: " << v1 << " / " << v2;
    this->Display(oss) << endl;
    throw runtime_error(oss.str());
  }
  return v1 / v2;
}

ExpressionLog::ExpressionLog(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Log, e} {}

void ExpressionLog::check_domain(const double v) {
  if (!(v >= 0)) {
    ostringstream oss;
    oss << "log(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

ostream& ExpressionLog::Display(ostream& os) const {
  os << "log(" << get_expression() << ")";
  return os;
}

double ExpressionLog::DoEvaluate(const double v) const {
  check_domain(v);
  return std::log(v);
}

ExpressionAbs::ExpressionAbs(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Abs, e} {}

ostream& ExpressionAbs::Display(ostream& os) const {
  os << "abs(" << get_expression() << ")";
  return os;
}

double ExpressionAbs::DoEvaluate(const double v) const { return std::fabs(v); }

ExpressionExp::ExpressionExp(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Exp, e} {}

ostream& ExpressionExp::Display(ostream& os) const {
  os << "exp(" << get_expression() << ")";
  return os;
}

double ExpressionExp::DoEvaluate(const double v) const { return std::exp(v); }

ExpressionSqrt::ExpressionSqrt(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Sqrt, e} {}

void ExpressionSqrt::check_domain(const double v) {
  if (!(v >= 0)) {
    ostringstream oss;
    oss << "sqrt(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

ostream& ExpressionSqrt::Display(ostream& os) const {
  os << "sqrt(" << get_expression() << ")";
  return os;
}

double ExpressionSqrt::DoEvaluate(const double v) const {
  check_domain(v);
  return std::sqrt(v);
}

ExpressionPow::ExpressionPow(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Pow, e1, e2} {}

static bool is_int(const double v) {
  double intpart;  // dummy variable
  return modf(v, &intpart) == 0.0;
}

void ExpressionPow::check_domain(const double v1, const double v2) {
  if (std::isfinite(v1) && (v1 < 0.0) && std::isfinite(v2) && !is_int(v2)) {
    ostringstream oss;
    oss << "pow(" << v1 << ", " << v2
        << ") : numerical argument out of domain. " << v1
        << " is finite negative and " << v2 << " is finite non-integer."
        << endl;
    throw domain_error(oss.str());
  }
}

ostream& ExpressionPow::Display(ostream& os) const {
  os << "pow(" << get_first_expression() << ", " << get_second_expression()
     << ")";
  return os;
}

double ExpressionPow::DoEvaluate(const double v1, const double v2) const {
  check_domain(v1, v2);
  return std::pow(v1, v2);
}

ExpressionSin::ExpressionSin(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Sin, e} {}

ostream& ExpressionSin::Display(ostream& os) const {
  os << "sin(" << get_expression() << ")";
  return os;
}

double ExpressionSin::DoEvaluate(const double v) const { return std::sin(v); }

ExpressionCos::ExpressionCos(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Cos, e} {}

ostream& ExpressionCos::Display(ostream& os) const {
  os << "cos(" << get_expression() << ")";
  return os;
}

double ExpressionCos::DoEvaluate(const double v) const { return std::cos(v); }

ExpressionTan::ExpressionTan(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Tan, e} {}

ostream& ExpressionTan::Display(ostream& os) const {
  os << "tan(" << get_expression() << ")";
  return os;
}

double ExpressionTan::DoEvaluate(const double v) const { return std::tan(v); }

ExpressionAsin::ExpressionAsin(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Asin, e} {}

void ExpressionAsin::check_domain(const double v) {
  if (!((v >= -1.0) && (v <= 1.0))) {
    ostringstream oss;
    oss << "asin(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

ostream& ExpressionAsin::Display(ostream& os) const {
  os << "asin(" << get_expression() << ")";
  return os;
}

double ExpressionAsin::DoEvaluate(const double v) const {
  check_domain(v);
  return std::asin(v);
}

ExpressionAcos::ExpressionAcos(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Acos, e} {}

void ExpressionAcos::check_domain(const double v) {
  if (!((v >= -1.0) && (v <= 1.0))) {
    ostringstream oss;
    oss << "acos(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

ostream& ExpressionAcos::Display(ostream& os) const {
  os << "acos(" << get_expression() << ")";
  return os;
}

double ExpressionAcos::DoEvaluate(const double v) const {
  check_domain(v);
  return std::acos(v);
}

ExpressionAtan::ExpressionAtan(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Atan, e} {}

ostream& ExpressionAtan::Display(ostream& os) const {
  os << "atan(" << get_expression() << ")";
  return os;
}

double ExpressionAtan::DoEvaluate(const double v) const { return std::atan(v); }

ExpressionAtan2::ExpressionAtan2(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Atan2, e1, e2} {}

ostream& ExpressionAtan2::Display(ostream& os) const {
  os << "atan2(" << get_first_expression() << ", " << get_second_expression()
     << ")";
  return os;
}

double ExpressionAtan2::DoEvaluate(const double v1, const double v2) const {
  return std::atan2(v1, v2);
}

ExpressionSinh::ExpressionSinh(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Sinh, e} {}

double ExpressionSinh::Evaluate(const Environment& env) const {
  return std::sinh(get_expression().Evaluate(env));
}

ostream& ExpressionSinh::Display(ostream& os) const {
  os << "sinh(" << get_expression() << ")";
  return os;
}

double ExpressionSinh::DoEvaluate(const double v) const { return std::sinh(v); }

ExpressionCosh::ExpressionCosh(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Cosh, e} {}

ostream& ExpressionCosh::Display(ostream& os) const {
  os << "cosh(" << get_expression() << ")";
  return os;
}

double ExpressionCosh::DoEvaluate(const double v) const { return std::cosh(v); }

ExpressionTanh::ExpressionTanh(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Tanh, e} {}

ostream& ExpressionTanh::Display(ostream& os) const {
  os << "tanh(" << get_expression() << ")";
  return os;
}

double ExpressionTanh::DoEvaluate(const double v) const { return std::tanh(v); }

ExpressionMin::ExpressionMin(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Min, e1, e2} {}

ostream& ExpressionMin::Display(ostream& os) const {
  os << "min(" << get_first_expression() << ", " << get_second_expression()
     << ")";
  return os;
}

double ExpressionMin::DoEvaluate(const double v1, const double v2) const {
  return std::min(v1, v2);
}

ExpressionMax::ExpressionMax(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Max, e1, e2} {}

ostream& ExpressionMax::Display(ostream& os) const {
  os << "max(" << get_first_expression() << ", " << get_second_expression()
     << ")";
  return os;
}

double ExpressionMax::DoEvaluate(const double v1, const double v2) const {
  return std::max(v1, v2);
}

ostream& operator<<(ostream& os, const Expression& e) {
  DRAKE_ASSERT(e.ptr_ != nullptr);
  return e.ptr_->Display(os);
}

Expression log(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    ExpressionLog::check_domain(v);
    return Expression{std::log(v)};
  }
  return Expression{make_shared<ExpressionLog>(e)};
}

Expression abs(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{std::fabs(v)};
  }
  return Expression{make_shared<ExpressionAbs>(e)};
}

Expression exp(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{std::exp(v)};
  }
  return Expression{make_shared<ExpressionExp>(e)};
}

Expression sqrt(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    ExpressionSqrt::check_domain(v);
    return Expression{std::sqrt(v)};
  }
  // Simplification: sqrt(pow(x, 2)) => abs(x)
  if (e.get_kind() == ExpressionKind::Pow) {
    const auto e_pow(static_pointer_cast<ExpressionPow>(e.ptr_));
    if (e_pow->get_second_expression().EqualTo(Expression{2.0})) {
      return abs(e_pow->get_first_expression());
    }
  }
  return Expression{make_shared<ExpressionSqrt>(e)};
}

Expression pow(const Expression& e1, const Expression& e2) {
  // Simplification
  if (e2.get_kind() == ExpressionKind::Constant) {
    const double v2{
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value()};
    if (e1.get_kind() == ExpressionKind::Constant) {
      // Constant folding
      const double v1{
          static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value()};
      ExpressionPow::check_domain(v1, v2);
      return Expression{std::pow(v1, v2)};
    }
    // pow(E, 0) => 1
    // TODO(soonho-tri): This simplification is not sound since it cancels `E`
    // which might contain 0/0 problems.
    if (v2 == 0.0) {
      return Expression::One();
    }
    // pow(E, 1) => E
    if (v2 == 1.0) {
      return e1;
    }
  }
  if (e1.get_kind() == ExpressionKind::Pow) {
    // pow(base, exponent) ^ e2 => pow(base, exponent * e2)
    const Expression& base{
        static_pointer_cast<ExpressionPow>(e1.ptr_)->get_first_expression()};
    const Expression& exponent{
        static_pointer_cast<ExpressionPow>(e1.ptr_)->get_second_expression()};
    return Expression{make_shared<ExpressionPow>(base, exponent * e2)};
  }
  return Expression{make_shared<ExpressionPow>(e1, e2)};
}

Expression pow(const Expression& e1, const double v2) {
  return pow(e1, Expression{v2});
}

Expression pow(const double v1, const Expression& e2) {
  return pow(Expression{v1}, e2);
}

Expression sin(const Expression& e) {
  // simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{std::sin(v)};
  }
  return Expression{make_shared<ExpressionSin>(e)};
}

Expression cos(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{std::cos(v)};
  }

  return Expression{make_shared<ExpressionCos>(e)};
}

Expression tan(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{std::tan(v)};
  }
  return Expression{make_shared<ExpressionTan>(e)};
}

Expression asin(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    ExpressionAsin::check_domain(v);
    return Expression{std::asin(v)};
  }
  return Expression{make_shared<ExpressionAsin>(e)};
}

Expression acos(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    ExpressionAcos::check_domain(v);
    return Expression{std::acos(v)};
  }
  return Expression{make_shared<ExpressionAcos>(e)};
}

Expression atan(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{std::atan(v)};
  }
  return Expression{make_shared<ExpressionAtan>(e)};
}

Expression atan2(const Expression& e1, const Expression& e2) {
  // Simplification: constant folding.
  if (e1.get_kind() == ExpressionKind::Constant &&
      e2.get_kind() == ExpressionKind::Constant) {
    const double v1{
        static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value()};
    const double v2{
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value()};
    return Expression{std::atan2(v1, v2)};
  }
  return Expression{make_shared<ExpressionAtan2>(e1, e2)};
}

Expression atan2(const Expression& e1, const double v2) {
  return atan2(e1, Expression{v2});
}

Expression atan2(const double v1, const Expression& e2) {
  return atan2(Expression{v1}, e2);
}

Expression sinh(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{std::sinh(v)};
  }
  return Expression{make_shared<ExpressionSinh>(e)};
}

Expression cosh(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{std::cosh(v)};
  }
  return Expression{make_shared<ExpressionCosh>(e)};
}

Expression tanh(const Expression& e) {
  // Simplification: constant folding.
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v{
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value()};
    return Expression{std::tanh(v)};
  }
  return Expression{make_shared<ExpressionTanh>(e)};
}

Expression min(const Expression& e1, const Expression& e2) {
  // simplification: min(x, x) => x
  if (e1.EqualTo(e2)) {
    return e1;
  }
  // Simplification: constant folding.
  if (e1.get_kind() == ExpressionKind::Constant &&
      e2.get_kind() == ExpressionKind::Constant) {
    const double v1{
        static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value()};
    const double v2{
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value()};
    return Expression{std::min(v1, v2)};
  }
  return Expression{make_shared<ExpressionMin>(e1, e2)};
}

Expression max(const Expression& e1, const Expression& e2) {
  // Simplification: max(x, x) => x
  if (e1.EqualTo(e2)) {
    return e1;
  }
  // Simplification: constant folding
  if (e1.get_kind() == ExpressionKind::Constant &&
      e2.get_kind() == ExpressionKind::Constant) {
    const double v1{
        static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value()};
    const double v2{
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value()};
    return Expression{std::max(v1, v2)};
  }
  return Expression{make_shared<ExpressionMax>(e1, e2)};
}

}  // namespace symbolic
}  // namespace drake
