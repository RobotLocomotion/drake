#include "drake/common/symbolic_expression_cell.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::accumulate;
using std::domain_error;
using std::endl;
using std::equal;
using std::hash;
using std::lexicographical_compare;
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

ExpressionCell::ExpressionCell(ExpressionKind const k, size_t const hash)
    : kind_{k}, hash_{hash_combine(static_cast<size_t>(kind_), hash)} {}

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
    : ExpressionCell{k, hash_combine(e1.get_hash(), e2)}, e1_{e1}, e2_{e2} {}

Variables BinaryExpressionCell::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  ret.insert(e2_.GetVariables());
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
                     hash_combine(hash<double>{}(constant_term),
                                  term_to_coeff_map)},
      constant_term_(constant_term),
      term_to_coeff_map_(term_to_coeff_map) {
  DRAKE_ASSERT(!term_to_coeff_map_.empty());
}

Variables ExpressionAdd::GetVariables() const {
  Variables ret{};
  for (const auto& p : term_to_coeff_map_) {
    ret.insert(p.first.GetVariables());
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
                     hash_combine(hash<double>{}(constant_factor),
                                  term_to_exp_map)},
      constant_factor_(constant_factor),
      term_to_exp_map_(term_to_exp_map) {
  DRAKE_ASSERT(!term_to_exp_map_.empty());
}

Variables ExpressionMul::GetVariables() const {
  Variables ret{};
  for (const auto& p : term_to_exp_map_) {
    ret.insert(p.first.GetVariables());
    ret.insert(p.second.GetVariables());
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

// ExpressionIfThenElse
// --------------------
ExpressionIfThenElse::ExpressionIfThenElse(const Formula& f_cond,
                                           const Expression& e_then,
                                           const Expression& e_else)
    : ExpressionCell{ExpressionKind::IfThenElse,
                     hash_combine(hash_value<Formula>{}(f_cond), e_then,
                                  e_else)},
      f_cond_{f_cond},
      e_then_{e_then},
      e_else_{e_else} {}

Variables ExpressionIfThenElse::GetVariables() const {
  Variables ret{f_cond_.GetFreeVariables()};
  ret.insert(e_then_.GetVariables());
  ret.insert(e_else_.GetVariables());
  return ret;
}

bool ExpressionIfThenElse::EqualTo(const ExpressionCell& e) const {
  if (get_kind() != e.get_kind()) {
    return false;
  }
  const ExpressionIfThenElse& ite_e{
      static_cast<const ExpressionIfThenElse&>(e)};
  return f_cond_.EqualTo(ite_e.f_cond_) && e_then_.EqualTo(ite_e.e_then_) &&
         e_else_.EqualTo(ite_e.e_else_);
}

bool ExpressionIfThenElse::Less(const ExpressionCell& e) const {
  const ExpressionKind k1{get_kind()};
  const ExpressionKind k2{e.get_kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  const ExpressionIfThenElse& ite_e{
      static_cast<const ExpressionIfThenElse&>(e)};
  if (f_cond_.Less(ite_e.f_cond_)) {
    return true;
  }
  if (ite_e.f_cond_.Less(f_cond_)) {
    return false;
  }
  if (e_then_.Less(ite_e.e_then_)) {
    return true;
  }
  if (ite_e.e_then_.Less(e_then_)) {
    return false;
  }
  return e_else_.Less(ite_e.e_else_);
}

double ExpressionIfThenElse::Evaluate(const Environment& env) const {
  if (f_cond_.Evaluate(env)) {
    return e_then_.Evaluate(env);
  } else {
    return e_else_.Evaluate(env);
  }
}

ostream& ExpressionIfThenElse::Display(ostream& os) const {
  os << "(if " << f_cond_ << " then " << e_then_ << " else " << e_else_ << ")";
  return os;
}

}  // namespace symbolic
}  // namespace drake
