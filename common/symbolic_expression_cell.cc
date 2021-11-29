#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

using std::accumulate;
using std::all_of;
using std::domain_error;
using std::endl;
using std::equal;
using std::lexicographical_compare;
using std::make_shared;
using std::map;
using std::numeric_limits;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::string;
using std::vector;

bool is_integer(const double v) {
  // v should be in [int_min, int_max].
  if (!((numeric_limits<int>::lowest() <= v) &&
        (v <= numeric_limits<int>::max()))) {
    return false;
  }

  double intpart{};  // dummy variable
  return modf(v, &intpart) == 0.0;
}

bool is_positive_integer(const double v) { return (v > 0) && is_integer(v); }

bool is_non_negative_integer(const double v) {
  return (v >= 0) && is_integer(v);
}

namespace {

// Determines if the summation represented by term_to_coeff_map is
// polynomial-convertible or not. This function is used in the
// constructor of ExpressionAdd.
bool determine_polynomial(
    const std::map<Expression, double>& term_to_coeff_map) {
  return all_of(term_to_coeff_map.begin(), term_to_coeff_map.end(),
                [](const pair<const Expression, double>& p) {
                  return p.first.is_polynomial();
                });
}

// Determines if the product represented by term_to_coeff_map is
// polynomial-convertible or not. This function is used in the
// constructor of ExpressionMul.
bool determine_polynomial(
    const std::map<Expression, Expression>& base_to_exponent_map) {
  return all_of(base_to_exponent_map.begin(), base_to_exponent_map.end(),
                [](const pair<const Expression, Expression>& p) {
                  // For each base^exponent, it has to satisfy the following
                  // conditions:
                  //     - base is polynomial-convertible.
                  //     - exponent is a non-negative integer.
                  const Expression& base{p.first};
                  const Expression& exponent{p.second};
                  if (!base.is_polynomial() || !is_constant(exponent)) {
                    return false;
                  }
                  const double e{get_constant_value(exponent)};
                  return is_non_negative_integer(e);
                });
}

// Determines if pow(base, exponent) is polynomial-convertible or not. This
// function is used in constructor of ExpressionPow.
bool determine_polynomial(const Expression& base, const Expression& exponent) {
  // base ^ exponent is polynomial-convertible if the following hold:
  //    - base is polynomial-convertible.
  //    - exponent is a non-negative integer.
  if (!(base.is_polynomial() && is_constant(exponent))) {
    return false;
  }
  const double e{get_constant_value(exponent)};
  return is_non_negative_integer(e);
}

Expression ExpandMultiplication(const Expression& e1, const Expression& e2,
                                const Expression& e3);

// Helper function expanding (e1 * e2). It assumes that both of e1 and e2 are
// already expanded.
Expression ExpandMultiplication(const Expression& e1, const Expression& e2) {
  // Precondition: e1 and e2 are already expanded.
  DRAKE_ASSERT(e1.EqualTo(e1.Expand()));
  DRAKE_ASSERT(e2.EqualTo(e2.Expand()));

  if (is_addition(e1)) {
    //   (c0 + c1 * e_{1,1} + ... + c_n * e_{1, n}) * e2
    // = c0 * e2 + c1 * e_{1,1} * e2 + ... + c_n * e_{1,n} * e2
    const double c0{get_constant_in_addition(e1)};
    const map<Expression, double>& m1{get_expr_to_coeff_map_in_addition(e1)};
    ExpressionAddFactory fac;
    fac.AddExpression(ExpandMultiplication(c0, e2));
    for (const pair<const Expression, double>& p : m1) {
      fac.AddExpression(ExpandMultiplication(p.second, p.first, e2));
    }
    return fac.GetExpression();
  }
  if (is_addition(e2)) {
    //   e1 * (c0 + c1 * e_{2,1} + ... + c_n * e_{2, n})
    // = e1 * c0 + e1 * c1 * e_{2,1} + ... + e1 * c_n * e_{2,n}
    const double c0{get_constant_in_addition(e2)};
    const map<Expression, double>& m1{get_expr_to_coeff_map_in_addition(e2)};
    ExpressionAddFactory fac;
    fac.AddExpression(ExpandMultiplication(e1, c0));
    for (const pair<const Expression, double>& p : m1) {
      fac.AddExpression(ExpandMultiplication(e1, p.second, p.first));
    }
    return fac.GetExpression();
  }
  if (is_division(e1)) {
    const Expression& e1_1{get_first_argument(e1)};
    const Expression& e1_2{get_second_argument(e1)};
    if (is_division(e2)) {
      //    ((e1_1 / e1_2) * (e2_1 / e2_2)).Expand()
      // => (e1_1 * e2_1).Expand() / (e1_2 * e2_2).Expand().
      //
      // Note that e1_1, e1_2, e2_1, and e_2 are already expanded by the
      // precondition.
      const Expression& e2_1{get_first_argument(e2)};
      const Expression& e2_2{get_second_argument(e2)};
      return ExpandMultiplication(e1_1, e2_1) /
             ExpandMultiplication(e1_2, e2_2);
    }
    //    ((e1_1 / e1_2) * e2).Expand()
    // => (e1_1 * e2).Expand() / e2.
    //
    // Note that e1_1, e1_2, and e_2 are already expanded by the precondition.
    return ExpandMultiplication(e1_1, e2) / e1_2;
  }
  if (is_division(e2)) {
    //    (e1 * (e2_1 / e2_2)).Expand()
    // => (e1 * e2_1).Expand() / e2_2.
    //
    // Note that e1, e2_1, and e2_2 are already expanded by the precondition.
    const Expression& e2_1{get_first_argument(e2)};
    const Expression& e2_2{get_second_argument(e2)};
    return ExpandMultiplication(e1, e2_1) / e2_2;
  }
  return e1 * e2;
}

Expression ExpandMultiplication(const Expression& e1, const Expression& e2,
                                const Expression& e3) {
  return ExpandMultiplication(ExpandMultiplication(e1, e2), e3);
}

// Helper function expanding pow(base, n). It assumes that base is expanded.
Expression ExpandPow(const Expression& base, const int n) {
  // Precondition: base is already expanded.
  DRAKE_ASSERT(base.EqualTo(base.Expand()));
  DRAKE_ASSERT(n >= 1);
  if (n == 1) {
    return base;
  }
  const Expression pow_half{ExpandPow(base, n / 2)};
  if (n % 2 == 1) {
    // pow(base, n) = base * pow(base, n / 2) * pow(base, n / 2)
    return ExpandMultiplication(base, pow_half, pow_half);
  }
  // pow(base, n) = pow(base, n / 2) * pow(base, n / 2)
  return ExpandMultiplication(pow_half, pow_half);
}

// Helper function expanding pow(base, exponent). It assumes that both of base
// and exponent are already expanded.
Expression ExpandPow(const Expression& base, const Expression& exponent) {
  // Precondition: base and exponent are already expanded.
  DRAKE_ASSERT(base.EqualTo(base.Expand()));
  DRAKE_ASSERT(exponent.EqualTo(exponent.Expand()));
  if (is_multiplication(base)) {
    //   pow(c * ∏ᵢ pow(e₁ᵢ, e₂ᵢ), exponent)
    // = pow(c, exponent) * ∏ᵢ pow(e₁ᵢ, e₂ᵢ * exponent)
    const double c{get_constant_in_multiplication(base)};
    auto map = get_base_to_exponent_map_in_multiplication(base);
    for (pair<const Expression, Expression>& p : map) {
      p.second = p.second * exponent;
    }
    return pow(c, exponent) * ExpressionMulFactory{1.0, map}.GetExpression();
  }

  // Expand if
  //     1) base is an addition expression and
  //     2) exponent is a positive integer.
  if (!is_addition(base) || !is_constant(exponent)) {
    return pow(base, exponent);
  }
  const double e{get_constant_value(exponent)};
  if (e <= 0 || !is_integer(e)) {
    return pow(base, exponent);
  }
  const int n{static_cast<int>(e)};
  return ExpandPow(base, n);
}
}  // anonymous namespace

ExpressionCell::ExpressionCell(const ExpressionKind k, const bool is_poly,
                               const bool is_expanded)
    : kind_{k}, is_polynomial_{is_poly}, is_expanded_{is_expanded} {}

UnaryExpressionCell::UnaryExpressionCell(const ExpressionKind k, Expression e,
                                         const bool is_poly,
                                         const bool is_expanded)
    : ExpressionCell{k, is_poly, is_expanded}, e_{std::move(e)} {}

void UnaryExpressionCell::HashAppendDetail(DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  hash_append(*hasher, e_);
}

Variables UnaryExpressionCell::GetVariables() const {
  return e_.GetVariables();
}

bool UnaryExpressionCell::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const auto& unary_e = static_cast<const UnaryExpressionCell&>(e);
  return e_.EqualTo(unary_e.e_);
}

bool UnaryExpressionCell::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const auto& unary_e = static_cast<const UnaryExpressionCell&>(e);
  return e_.Less(unary_e.e_);
}

double UnaryExpressionCell::Evaluate(const Environment& env) const {
  const double v{e_.Evaluate(env)};
  return DoEvaluate(v);
}

BinaryExpressionCell::BinaryExpressionCell(const ExpressionKind k,
                                           Expression e1, Expression e2,
                                           const bool is_poly,
                                           const bool is_expanded)
    : ExpressionCell{k, is_poly, is_expanded},
      e1_{std::move(e1)},
      e2_{std::move(e2)} {}

void BinaryExpressionCell::HashAppendDetail(DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  hash_append(*hasher, e1_);
  hash_append(*hasher, e2_);
}

Variables BinaryExpressionCell::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  ret.insert(e2_.GetVariables());
  return ret;
}

bool BinaryExpressionCell::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const auto& binary_e = static_cast<const BinaryExpressionCell&>(e);
  return e1_.EqualTo(binary_e.e1_) && e2_.EqualTo(binary_e.e2_);
}

bool BinaryExpressionCell::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const auto& binary_e = static_cast<const BinaryExpressionCell&>(e);
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

ExpressionVar::ExpressionVar(Variable v)
    : ExpressionCell{ExpressionKind::Var, true, true}, var_{std::move(v)} {
  // Dummy symbolic variable (ID = 0) should not be used in constructing
  // symbolic expressions.
  DRAKE_DEMAND(!var_.is_dummy());
  // Boolean symbolic variable should not be used in constructing symbolic
  // expressions.
  DRAKE_DEMAND(var_.get_type() != Variable::Type::BOOLEAN);
}

void ExpressionVar::HashAppendDetail(DelegatingHasher* hasher) const {
  DRAKE_ASSERT(hasher != nullptr);
  using drake::hash_append;
  hash_append(*hasher, var_);
}

Variables ExpressionVar::GetVariables() const { return {get_variable()}; }

bool ExpressionVar::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  return var_.equal_to(static_cast<const ExpressionVar&>(e).var_);
}

bool ExpressionVar::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  // Note the below is using the overloaded operator< between ExpressionVar
  // which is based on variable IDs.
  return var_.less(static_cast<const ExpressionVar&>(e).var_);
}

double ExpressionVar::Evaluate(const Environment& env) const {
  Environment::const_iterator const it{env.find(var_)};
  if (it != env.cend()) {
    DRAKE_ASSERT(!std::isnan(it->second));
    return it->second;
  }
  ostringstream oss;
  oss << "The following environment does not have an entry for the "
         "variable "
      << var_ << endl;
  oss << env << endl;
  throw runtime_error{oss.str()};
}

Expression ExpressionVar::Expand() const { return Expression{var_}; }

Expression ExpressionVar::Substitute(const Substitution& s) const {
  const Substitution::const_iterator it{s.find(var_)};
  if (it != s.end()) {
    return it->second;
  }
  return Expression{var_};
}

Expression ExpressionVar::Differentiate(const Variable& x) const {
  if (x.equal_to(var_)) {
    return Expression::One();
  }
  return Expression::Zero();
}

ostream& ExpressionVar::Display(ostream& os) const { return os << var_; }

ExpressionConstant::ExpressionConstant(const double v)
    : ExpressionCell{ExpressionKind::Constant, true, true}, v_{v} {
  DRAKE_ASSERT(!std::isnan(v));
}

void ExpressionConstant::HashAppendDetail(DelegatingHasher* hasher) const {
  using drake::hash_append;
  hash_append(*hasher, v_);
}

Variables ExpressionConstant::GetVariables() const { return Variables{}; }

bool ExpressionConstant::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  return v_ == static_cast<const ExpressionConstant&>(e).v_;
}

bool ExpressionConstant::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  return v_ < static_cast<const ExpressionConstant&>(e).v_;
}

double ExpressionConstant::Evaluate(const Environment&) const {
  DRAKE_DEMAND(!std::isnan(v_));
  return v_;
}

Expression ExpressionConstant::Expand() const { return Expression{v_}; }

Expression ExpressionConstant::Substitute(const Substitution&) const {
  DRAKE_DEMAND(!std::isnan(v_));
  return Expression{v_};
}

Expression ExpressionConstant::Differentiate(const Variable&) const {
  return Expression::Zero();
}

ostream& ExpressionConstant::Display(ostream& os) const { return os << v_; }

ExpressionNaN::ExpressionNaN()
    : ExpressionCell{ExpressionKind::NaN, false, false} {}

void ExpressionNaN::HashAppendDetail(DelegatingHasher*) const {}

Variables ExpressionNaN::GetVariables() const { return Variables{}; }

bool ExpressionNaN::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  return true;
}

bool ExpressionNaN::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  return false;
}

double ExpressionNaN::Evaluate(const Environment&) const {
  throw runtime_error("NaN is detected during Symbolic computation.");
}

Expression ExpressionNaN::Expand() const {
  throw runtime_error("NaN is detected during expansion.");
}

Expression ExpressionNaN::Substitute(const Substitution&) const {
  throw runtime_error("NaN is detected during substitution.");
}

Expression ExpressionNaN::Differentiate(const Variable&) const {
  throw runtime_error("NaN is detected during differentiation.");
}

ostream& ExpressionNaN::Display(ostream& os) const { return os << "NaN"; }

ExpressionAdd::ExpressionAdd(const double constant,
                             const map<Expression, double>& expr_to_coeff_map)
    : ExpressionCell{ExpressionKind::Add,
                     determine_polynomial(expr_to_coeff_map), false},
      constant_(constant),
      expr_to_coeff_map_(expr_to_coeff_map) {
  DRAKE_ASSERT(!expr_to_coeff_map_.empty());
}

void ExpressionAdd::HashAppendDetail(DelegatingHasher* hasher) const {
  using drake::hash_append;
  hash_append(*hasher, constant_);
  hash_append(*hasher, expr_to_coeff_map_);
}

Variables ExpressionAdd::GetVariables() const {
  Variables ret{};
  for (const auto& p : expr_to_coeff_map_) {
    ret.insert(p.first.GetVariables());
  }
  return ret;
}

bool ExpressionAdd::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const ExpressionAdd& add_e{static_cast<const ExpressionAdd&>(e)};
  // Compare constant.
  if (constant_ != add_e.constant_) {
    return false;
  }
  return equal(expr_to_coeff_map_.cbegin(), expr_to_coeff_map_.cend(),
               add_e.expr_to_coeff_map_.cbegin(),
               add_e.expr_to_coeff_map_.cend(),
               [](const pair<const Expression, double>& p1,
                  const pair<const Expression, double>& p2) {
                 return p1.first.EqualTo(p2.first) && p1.second == p2.second;
               });
}

bool ExpressionAdd::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const ExpressionAdd& add_e{static_cast<const ExpressionAdd&>(e)};
  // Compare the constants.
  if (constant_ < add_e.constant_) {
    return true;
  }
  if (add_e.constant_ < constant_) {
    return false;
  }
  // Compare the two maps.
  return lexicographical_compare(
      expr_to_coeff_map_.cbegin(), expr_to_coeff_map_.cend(),
      add_e.expr_to_coeff_map_.cbegin(), add_e.expr_to_coeff_map_.cend(),
      [](const pair<const Expression, double>& p1,
         const pair<const Expression, double>& p2) {
        const Expression& term1{p1.first};
        const Expression& term2{p2.first};
        if (term1.Less(term2)) {
          return true;
        }
        if (term2.Less(term1)) {
          return false;
        }
        const double coeff1{p1.second};
        const double coeff2{p2.second};
        return coeff1 < coeff2;
      });
}

double ExpressionAdd::Evaluate(const Environment& env) const {
  return accumulate(
      expr_to_coeff_map_.begin(), expr_to_coeff_map_.end(), constant_,
      [&env](const double init, const pair<const Expression, double>& p) {
        return init + p.first.Evaluate(env) * p.second;
      });
}

Expression ExpressionAdd::Expand() const {
  //   (c0 + c1 * e_1 + ... + c_n * e_n).Expand()
  // =  c0 + c1 * e_1.Expand() + ... + c_n * e_n.Expand()
  ExpressionAddFactory fac{constant_, {}};
  for (const pair<const Expression, double>& p : expr_to_coeff_map_) {
    const Expression& e_i{p.first};
    const double c_i{p.second};
    fac.AddExpression(
        ExpandMultiplication(e_i.is_expanded() ? e_i : e_i.Expand(), c_i));
  }
  return fac.GetExpression();
}

Expression ExpressionAdd::Substitute(const Substitution& s) const {
  return accumulate(
      expr_to_coeff_map_.begin(), expr_to_coeff_map_.end(),
      Expression{constant_},
      [&s](const Expression& init, const pair<const Expression, double>& p) {
        return init + p.first.Substitute(s) * p.second;
      });
}

Expression ExpressionAdd::Differentiate(const Variable& x) const {
  //   ∂/∂x (c_0 + c_1 * f_1 + ... + c_n * f_n)
  // = (∂/∂x c_0) + (∂/∂x c_1 * f_1) + ... + (∂/∂x c_n * f_n)
  // =  0.0       + c_1 * (∂/∂x f_1) + ... + c_n * (∂/∂x f_n)
  ExpressionAddFactory fac;
  for (const pair<const Expression, double>& p : expr_to_coeff_map_) {
    fac.AddExpression(p.second * p.first.Differentiate(x));
  }
  return fac.GetExpression();
}

ostream& ExpressionAdd::Display(ostream& os) const {
  DRAKE_ASSERT(!expr_to_coeff_map_.empty());
  bool print_plus{false};
  os << "(";
  if (constant_ != 0.0) {
    os << constant_;
    print_plus = true;
  }
  for (const auto& p : expr_to_coeff_map_) {
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
    const double constant, map<Expression, double> expr_to_coeff_map)
    : constant_{constant}, expr_to_coeff_map_{std::move(expr_to_coeff_map)} {}

ExpressionAddFactory::ExpressionAddFactory(
    const ExpressionAdd& add)
    : ExpressionAddFactory{add.get_constant(), add.get_expr_to_coeff_map()} {}

void ExpressionAddFactory::AddExpression(const Expression& e) {
  if (is_constant(e)) {
    const double v{get_constant_value(e)};
    return AddConstant(v);
  }
  if (is_addition(e)) {
    // Flattening
    return Add(to_addition(e));
  }
  if (is_multiplication(e)) {
    const double constant{get_constant_in_multiplication(e)};
    DRAKE_ASSERT(constant != 0.0);
    if (constant != 1.0) {
      // Instead of adding (1.0 * (constant * b1^t1 ... bn^tn)),
      // add (constant, 1.0 * b1^t1 ... bn^tn).
      return AddTerm(constant,
                     ExpressionMulFactory(
                         1.0, get_base_to_exponent_map_in_multiplication(e))
                         .GetExpression());
    }
  }
  return AddTerm(1.0, e);
}

void ExpressionAddFactory::Add(const ExpressionAdd& add) {
  AddConstant(add.get_constant());
  AddMap(add.get_expr_to_coeff_map());
}

ExpressionAddFactory& ExpressionAddFactory::operator=(
    const ExpressionAdd& add) {
  constant_ = add.get_constant();
  expr_to_coeff_map_ = add.get_expr_to_coeff_map();
  return *this;
}

ExpressionAddFactory& ExpressionAddFactory::Negate() {
  constant_ = -constant_;
  for (auto& p : expr_to_coeff_map_) {
    p.second = -p.second;
  }
  return *this;
}

Expression ExpressionAddFactory::GetExpression() const {
  if (expr_to_coeff_map_.empty()) {
    return Expression{constant_};
  }
  if (constant_ == 0.0 && expr_to_coeff_map_.size() == 1U) {
    // 0.0 + c1 * t1 -> c1 * t1
    const auto it(expr_to_coeff_map_.cbegin());
    return it->first * it->second;
  }
  return Expression{make_shared<ExpressionAdd>(constant_, expr_to_coeff_map_)};
}

void ExpressionAddFactory::AddConstant(const double constant) {
  constant_ += constant;
}

void ExpressionAddFactory::AddTerm(const double coeff, const Expression& term) {
  DRAKE_ASSERT(!is_constant(term));
  DRAKE_ASSERT(coeff != 0.0);

  const auto it(expr_to_coeff_map_.find(term));
  if (it != expr_to_coeff_map_.end()) {
    // Case1: term is already in the map
    double& this_coeff{it->second};
    this_coeff += coeff;
    if (this_coeff == 0.0) {
      // If the coefficient becomes zero, remove the entry.
      // TODO(soonho-tri): The following operation is not sound since it cancels
      // `term` which might contain 0/0 problems.
      expr_to_coeff_map_.erase(it);
    }
  } else {
    // Case2: term is not found in expr_to_coeff_map_.
    // Add the entry (term, coeff).
    expr_to_coeff_map_.emplace(term, coeff);
  }
}

void ExpressionAddFactory::AddMap(
    const map<Expression, double>& expr_to_coeff_map) {
  for (const auto& p : expr_to_coeff_map) {
    AddTerm(p.second, p.first);
  }
}

ExpressionMul::ExpressionMul(
    const double constant,
    const map<Expression, Expression>& base_to_exponent_map)
    : ExpressionCell{ExpressionKind::Mul,
                     determine_polynomial(base_to_exponent_map), false},
      constant_(constant),
      base_to_exponent_map_(base_to_exponent_map) {
  DRAKE_ASSERT(!base_to_exponent_map_.empty());
}

void ExpressionMul::HashAppendDetail(DelegatingHasher* hasher) const {
  using drake::hash_append;
  hash_append(*hasher, constant_);
  hash_append(*hasher, base_to_exponent_map_);
}

Variables ExpressionMul::GetVariables() const {
  Variables ret{};
  for (const auto& p : base_to_exponent_map_) {
    ret.insert(p.first.GetVariables());
    ret.insert(p.second.GetVariables());
  }
  return ret;
}

bool ExpressionMul::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const ExpressionMul& mul_e{static_cast<const ExpressionMul&>(e)};
  // Compare constant.
  if (constant_ != mul_e.constant_) {
    return false;
  }
  // Check each (term, coeff) pairs in two maps.
  return equal(
      base_to_exponent_map_.cbegin(), base_to_exponent_map_.cend(),
      mul_e.base_to_exponent_map_.cbegin(), mul_e.base_to_exponent_map_.cend(),
      [](const pair<const Expression, Expression>& p1,
         const pair<const Expression, Expression>& p2) {
        return p1.first.EqualTo(p2.first) && p1.second.EqualTo(p2.second);
      });
}

bool ExpressionMul::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const ExpressionMul& mul_e{static_cast<const ExpressionMul&>(e)};
  // Compare the constants.
  if (constant_ < mul_e.constant_) {
    return true;
  }
  if (mul_e.constant_ < constant_) {
    return false;
  }
  // Compare the two maps.
  return lexicographical_compare(
      base_to_exponent_map_.cbegin(), base_to_exponent_map_.cend(),
      mul_e.base_to_exponent_map_.cbegin(), mul_e.base_to_exponent_map_.cend(),
      [](const pair<const Expression, Expression>& p1,
         const pair<const Expression, Expression>& p2) {
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
      base_to_exponent_map_.begin(), base_to_exponent_map_.end(), constant_,
      [&env](const double init, const pair<const Expression, Expression>& p) {
        return init * std::pow(p.first.Evaluate(env), p.second.Evaluate(env));
      });
}

Expression ExpressionMul::Expand() const {
  //   (c * ∏ᵢ pow(bᵢ, eᵢ)).Expand()
  // = c * ExpandMultiplication(∏ ExpandPow(bᵢ.Expand(), eᵢ.Expand()))
  return accumulate(base_to_exponent_map_.begin(), base_to_exponent_map_.end(),
                    Expression{constant_},
                    [](const Expression& init,
                       const pair<const Expression, Expression>& p) {
                      const Expression& b_i{p.first};
                      const Expression& e_i{p.second};
                      return ExpandMultiplication(
                          init,
                          ExpandPow(b_i.is_expanded() ? b_i : b_i.Expand(),
                                    e_i.is_expanded() ? e_i : e_i.Expand()));
                    });
}

Expression ExpressionMul::Substitute(const Substitution& s) const {
  return accumulate(base_to_exponent_map_.begin(), base_to_exponent_map_.end(),
                    Expression{constant_},
                    [&s](const Expression& init,
                         const pair<const Expression, Expression>& p) {
                      return init *
                             pow(p.first.Substitute(s), p.second.Substitute(s));
                    });
}

// Computes ∂/∂x pow(f, g).
Expression DifferentiatePow(const Expression& f, const Expression& g,
                            const Variable& x) {
  if (is_constant(g)) {
    const Expression& n{g};  // alias n = g
    // Special case where exponent is a constant:
    //     ∂/∂x pow(f, n) = n * pow(f, n - 1) * ∂/∂x f
    return n * pow(f, n - 1) * f.Differentiate(x);
  }
  if (is_constant(f)) {
    const Expression& n{f};  // alias n = f
    // Special case where base is a constant:
    //     ∂/∂x pow(n, g) = log(n) * pow(n, g) * ∂/∂x g
    return log(n) * pow(n, g) * g.Differentiate(x);
  }
  // General case:
  //    ∂/∂x pow(f, g)
  // = ∂/∂f pow(f, g) * ∂/∂x f + ∂/∂g pow(f, g) * ∂/∂x g
  // = g * pow(f, g - 1) * ∂/∂x f + log(f) * pow(f, g) * ∂/∂x g
  // = pow(f, g - 1) * (g * ∂/∂x f + log(f) * f * ∂/∂x g)
  return pow(f, g - 1) *
         (g * f.Differentiate(x) + log(f) * f * g.Differentiate(x));
}

Expression ExpressionMul::Differentiate(const Variable& x) const {
  // ∂/∂x (c   * f₁^g₁  * f₂^g₂        * ... * fₙ^gₙ
  //= c * [expr * (∂/∂x f₁^g₁) / f₁^g₁ +
  //       expr * (∂/∂x f₂^g₂) / f₂^g₂ +
  //                      ...          +
  //       expr * (∂/∂x fₙ^gₙ) / fₙ^gₙ]
  // = c * expr * (∑ᵢ (∂/∂x fᵢ^gᵢ) / fᵢ^gᵢ)
  // where expr = (f₁^g₁ * f₂^g₂ * ... * fₙn^gₙ).
  //
  // We distribute (c * expr) into the summation. This possibly cancels the
  // division, "/ fᵢ^gᵢ", and results in a simpler formula.
  //
  // = ∑ᵢ (c * (∂/∂x fᵢ^gᵢ) / fᵢ^gᵢ * expr)

  // This factory will form the expression that we will return.
  ExpressionAddFactory add_fac;
  for (const pair<const Expression, Expression>& term : base_to_exponent_map_) {
    const Expression& base{term.first};
    const Expression& exponent{term.second};
    // This factory will form (c * (∂/∂x fᵢ^gᵢ) / fᵢ^gᵢ * expr).
    ExpressionMulFactory mul_fac{constant_, base_to_exponent_map_};
    mul_fac.AddExpression(DifferentiatePow(base, exponent, x));
    mul_fac.AddExpression(pow(base, -exponent));

    add_fac.AddExpression(mul_fac.GetExpression());
  }
  return add_fac.GetExpression();
}

ostream& ExpressionMul::Display(ostream& os) const {
  DRAKE_ASSERT(!base_to_exponent_map_.empty());
  bool print_mul{false};
  os << "(";
  if (constant_ != 1.0) {
    os << constant_;
    print_mul = true;
  }
  for (const auto& p : base_to_exponent_map_) {
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
  if (is_one(exponent)) {
    os << base;
  } else {
    os << "pow(" << base << ", " << exponent << ")";
  }
  return os;
}

ExpressionMulFactory::ExpressionMulFactory(
    const double constant, map<Expression, Expression> base_to_exponent_map)
    : constant_{constant},
      base_to_exponent_map_{std::move(base_to_exponent_map)} {}

ExpressionMulFactory::ExpressionMulFactory(const ExpressionMul& mul)
    : ExpressionMulFactory{mul.get_constant(),
                           mul.get_base_to_exponent_map()} {}

void ExpressionMulFactory::AddExpression(const Expression& e) {
  if (constant_ == 0.0) {
    return;  // Do nothing if it already represented 0.
  }
  if (is_zero(e)) {
    // X * 0 => 0. So clear the constant and the map.
    return SetZero();
  }
  if (is_constant(e)) {
    return AddConstant(get_constant_value(e));
  }
  if (is_multiplication(e)) {
    // Flattening
    return Add(to_multiplication(e));
  }
  // Add e^1
  return AddTerm(e, Expression{1.0});
}

void ExpressionMulFactory::Add(const ExpressionMul& mul) {
  if (constant_ == 0.0) {
    return;  // Do nothing if it already represented 0.
  }
  AddConstant(mul.get_constant());
  AddMap(mul.get_base_to_exponent_map());
}

void ExpressionMulFactory::SetZero() {
  constant_ = 0.0;
  base_to_exponent_map_.clear();
}

ExpressionMulFactory& ExpressionMulFactory::operator=(
    const ExpressionMul& mul) {
  constant_ = mul.get_constant();
  base_to_exponent_map_ = mul.get_base_to_exponent_map();
  return *this;
}

ExpressionMulFactory& ExpressionMulFactory::Negate() {
  constant_ = -constant_;
  return *this;
}

Expression ExpressionMulFactory::GetExpression() const {
  if (base_to_exponent_map_.empty()) {
    return Expression{constant_};
  }
  if (constant_ == 1.0 && base_to_exponent_map_.size() == 1U) {
    // 1.0 * c1^t1 -> c1^t1
    const auto it(base_to_exponent_map_.cbegin());
    return pow(it->first, it->second);
  }
  return Expression{
      make_shared<ExpressionMul>(constant_, base_to_exponent_map_)};
}

void ExpressionMulFactory::AddConstant(const double constant) {
  if (constant == 0.0) {
    return SetZero();
  }
  constant_ *= constant;
}

void ExpressionMulFactory::AddTerm(const Expression& base,
                                   const Expression& exponent) {
  // The following assertion holds because of
  // ExpressionMulFactory::AddExpression.
  DRAKE_ASSERT(!(is_constant(base) && is_constant(exponent)));
  if (is_pow(base)) {
    // If (base, exponent) = (pow(e1, e2), exponent)), then add (e1, e2 *
    // exponent)
    // Example: (x^2)^3 => x^(2 * 3)
    return AddTerm(get_first_argument(base),
                   get_second_argument(base) * exponent);
  }

  const auto it(base_to_exponent_map_.find(base));
  if (it != base_to_exponent_map_.end()) {
    // base is already in map.
    // (= b1^e1 * ... * (base^this_exponent) * ... * en^bn).
    // Update it to be (... * (base^(this_exponent + exponent)) * ...)
    // Example: x^3 * x^2 => x^5
    Expression& this_exponent = it->second;
    this_exponent += exponent;
    if (is_zero(this_exponent)) {
      // If it ends up with base^0 (= 1.0) then remove this entry from the map.
      // TODO(soonho-tri): The following operation is not sound since it can
      // cancels `base` which might include 0/0 problems.
      base_to_exponent_map_.erase(it);
    }
  } else {
    // Product is not found in base_to_exponent_map_. Add the entry (base,
    // exponent).
    base_to_exponent_map_.emplace(base, exponent);
  }
}

void ExpressionMulFactory::AddMap(
    const map<Expression, Expression>& base_to_exponent_map) {
  for (const auto& p : base_to_exponent_map) {
    AddTerm(p.first, p.second);
  }
}

ExpressionDiv::ExpressionDiv(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Div, e1, e2,
                           e1.is_polynomial() && is_constant(e2), false} {}

namespace {
// Helper class to implement ExpressionDiv::Expand. Given a symbolic expression
// `e` and a constant `n`, it pushes the division in `e / n` inside for the
// following cases:
//
// Case Addition      :      (c₀ + ∑ᵢ (cᵢ * eᵢ)) / n
//                        => c₀/n + ∑ᵢ (cᵢ / n * eᵢ)
//
// Case Multiplication:      (c₀ * ∏ᵢ (bᵢ * eᵢ)) / n
//                        => c₀ / n * ∏ᵢ (bᵢ * eᵢ)
//
// Case Division      :      (e₁ / m) / n
//                        => Recursively simplify e₁ / (n * m)
//
//                           (e₁ / e₂) / n
//                        =  (e₁ / n) / e₂
//                        => Recursively simplify (e₁ / n) and divide it by e₂
//
// Other cases        :      e / n
//                        => (1/n) * e
//
// Note that we use VisitExpression instead of VisitPolynomial because we want
// to handle cases such as `(6xy / z) / 3` where (6xy / z) is not a polynomial
// but it's desirable to simplify the expression into `2xy / z`.
class DivExpandVisitor {
 public:
  [[nodiscard]] Expression Simplify(const Expression& e, const double n) const {
    return VisitExpression<Expression>(this, e, n);
  }

 private:
  [[nodiscard]] Expression VisitAddition(const Expression& e,
                                         const double n) const {
    // e =  (c₀ + ∑ᵢ (cᵢ * eᵢ)) / n
    //   => c₀/n + ∑ᵢ (cᵢ / n * eᵢ)
    const double constant{get_constant_in_addition(e)};
    ExpressionAddFactory factory(constant / n, {});
    for (const pair<const Expression, double>& p :
         get_expr_to_coeff_map_in_addition(e)) {
      factory.AddExpression(p.second / n * p.first);
    }
    return factory.GetExpression();
  }
  [[nodiscard]] Expression VisitMultiplication(const Expression& e,
                                               const double n) const {
    // e =  (c₀ * ∏ᵢ (bᵢ * eᵢ)) / n
    //   => c₀ / n * ∏ᵢ (bᵢ * eᵢ)
    return ExpressionMulFactory{get_constant_in_multiplication(e) / n,
                                get_base_to_exponent_map_in_multiplication(e)}
        .GetExpression();
  }
  [[nodiscard]] Expression VisitDivision(const Expression& e,
                                         const double n) const {
    const Expression& e1{get_first_argument(e)};
    const Expression& e2{get_second_argument(e)};
    if (is_constant(e2)) {
      // e =  (e₁ / m) / n
      //   => Simplify `e₁ / (n * m)`
      const double m{get_constant_value(e2)};
      return Simplify(e1, m * n);
    } else {
      // e =  (e₁ / e₂) / n
      //   => (e₁ / n) / e₂
      return Simplify(e1, n) / e2;
    }
  }
  [[nodiscard]] Expression VisitVariable(const Expression& e,
                                         const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitConstant(const Expression& e,
                                         const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitLog(const Expression& e, const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitPow(const Expression& e, const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitAbs(const Expression& e, const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitExp(const Expression& e, const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitSqrt(const Expression& e,
                                     const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitSin(const Expression& e, const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitCos(const Expression& e, const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitTan(const Expression& e, const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitAsin(const Expression& e,
                                     const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitAcos(const Expression& e,
                                     const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitAtan(const Expression& e,
                                     const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitAtan2(const Expression& e,
                                      const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitSinh(const Expression& e,
                                     const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitCosh(const Expression& e,
                                     const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitTanh(const Expression& e,
                                     const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitMin(const Expression& e, const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitMax(const Expression& e, const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitCeil(const Expression& e,
                                     const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitFloor(const Expression& e,
                                      const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitIfThenElse(const Expression& e,
                                           const double n) const {
    return (1.0 / n) * e;
  }
  [[nodiscard]] Expression VisitUninterpretedFunction(const Expression& e,
                                                      const double n) const {
    return (1.0 / n) * e;
  }

  // Makes VisitExpression a friend of this class so that VisitExpression can
  // use its private methods.
  friend Expression drake::symbolic::VisitExpression<Expression>(
      const DivExpandVisitor*, const Expression&, const double&);
};
}  // namespace

Expression ExpressionDiv::Expand() const {
  const Expression& first{get_first_argument()};
  const Expression& second{get_second_argument()};
  const Expression e1{first.is_expanded() ? first : first.Expand()};
  const Expression e2{second.is_expanded() ? second : second.Expand()};
  if (is_constant(e2)) {
    // Simplifies the 'division by a constant' case, using DivExpandVisitor
    // defined above.
    return DivExpandVisitor{}.Simplify(e1, get_constant_value(e2));
  } else {
    return (e1 / e2);
  }
}

Expression ExpressionDiv::Substitute(const Substitution& s) const {
  return get_first_argument().Substitute(s) /
         get_second_argument().Substitute(s);
}

Expression ExpressionDiv::Differentiate(const Variable& x) const {
  // ∂/∂x (f / g) = (∂/∂x f * g - f * ∂/∂x g) / g^2
  const Expression& f{get_first_argument()};
  const Expression& g{get_second_argument()};
  return (f.Differentiate(x) * g - f * g.Differentiate(x)) / pow(g, 2.0);
}

ostream& ExpressionDiv::Display(ostream& os) const {
  return os << "(" << get_first_argument() << " / " << get_second_argument()
            << ")";
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
    : UnaryExpressionCell{ExpressionKind::Log, e, false, e.is_expanded()} {}

void ExpressionLog::check_domain(const double v) {
  if (!(v >= 0)) {
    ostringstream oss;
    oss << "log(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

Expression ExpressionLog::Expand() const {
  const Expression& arg{get_argument()};
  return log(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionLog::Substitute(const Substitution& s) const {
  return log(get_argument().Substitute(s));
}

Expression ExpressionLog::Differentiate(const Variable& x) const {
  // ∂/∂x log(f) = (∂/∂x f) / f
  const Expression& f{get_argument()};
  return f.Differentiate(x) / f;
}

ostream& ExpressionLog::Display(ostream& os) const {
  return os << "log(" << get_argument() << ")";
}

double ExpressionLog::DoEvaluate(const double v) const {
  check_domain(v);
  return std::log(v);
}

ExpressionAbs::ExpressionAbs(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Abs, e, false, e.is_expanded()} {}

Expression ExpressionAbs::Expand() const {
  const Expression& arg{get_argument()};
  return abs(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionAbs::Substitute(const Substitution& s) const {
  return abs(get_argument().Substitute(s));
}

Expression ExpressionAbs::Differentiate(const Variable& x) const {
  if (GetVariables().include(x)) {
    const Expression& arg{get_argument()};
    const Expression deriv = arg.Differentiate(x);
    return if_then_else(arg < 0, -deriv,
                        if_then_else(arg == 0, Expression::NaN(), deriv));
  } else {
    return Expression::Zero();
  }
}

ostream& ExpressionAbs::Display(ostream& os) const {
  return os << "abs(" << get_argument() << ")";
}

double ExpressionAbs::DoEvaluate(const double v) const { return std::fabs(v); }

ExpressionExp::ExpressionExp(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Exp, e, false, e.is_expanded()} {}

Expression ExpressionExp::Expand() const {
  const Expression& arg{get_argument()};
  return exp(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionExp::Substitute(const Substitution& s) const {
  return exp(get_argument().Substitute(s));
}

Expression ExpressionExp::Differentiate(const Variable& x) const {
  // ∂/∂x exp(f) = exp(f) * (∂/∂x f)
  const Expression& f{get_argument()};
  return exp(f) * f.Differentiate(x);
}

ostream& ExpressionExp::Display(ostream& os) const {
  return os << "exp(" << get_argument() << ")";
}

double ExpressionExp::DoEvaluate(const double v) const { return std::exp(v); }

ExpressionSqrt::ExpressionSqrt(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Sqrt, e, false, e.is_expanded()} {}

void ExpressionSqrt::check_domain(const double v) {
  if (!(v >= 0)) {
    ostringstream oss;
    oss << "sqrt(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

Expression ExpressionSqrt::Expand() const {
  const Expression& arg{get_argument()};
  return sqrt(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionSqrt::Substitute(const Substitution& s) const {
  return sqrt(get_argument().Substitute(s));
}

Expression ExpressionSqrt::Differentiate(const Variable& x) const {
  // ∂/∂x (sqrt(f)) = 1 / (2 * sqrt(f)) * (∂/∂x f)
  const Expression& f{get_argument()};
  return 1 / (2 * sqrt(f)) * f.Differentiate(x);
}

ostream& ExpressionSqrt::Display(ostream& os) const {
  return os << "sqrt(" << get_argument() << ")";
}

double ExpressionSqrt::DoEvaluate(const double v) const {
  check_domain(v);
  return std::sqrt(v);
}

ExpressionPow::ExpressionPow(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Pow, e1, e2,
                           determine_polynomial(e1, e2), false} {}

void ExpressionPow::check_domain(const double v1, const double v2) {
  if (std::isfinite(v1) && (v1 < 0.0) && std::isfinite(v2) && !is_integer(v2)) {
    ostringstream oss;
    oss << "pow(" << v1 << ", " << v2
        << ") : numerical argument out of domain. " << v1
        << " is finite negative and " << v2 << " is finite non-integer."
        << endl;
    throw domain_error(oss.str());
  }
}

Expression ExpressionPow::Expand() const {
  const Expression& e1{get_first_argument()};
  const Expression& e2{get_second_argument()};
  return ExpandPow(e1.is_expanded() ? e1 : e1.Expand(),
                   e2.is_expanded() ? e2 : e2.Expand());
}

Expression ExpressionPow::Substitute(const Substitution& s) const {
  return pow(get_first_argument().Substitute(s),
             get_second_argument().Substitute(s));
}

Expression ExpressionPow::Differentiate(const Variable& x) const {
  return DifferentiatePow(get_first_argument(), get_second_argument(), x);
}

ostream& ExpressionPow::Display(ostream& os) const {
  return os << "pow(" << get_first_argument() << ", " << get_second_argument()
            << ")";
}

double ExpressionPow::DoEvaluate(const double v1, const double v2) const {
  check_domain(v1, v2);
  return std::pow(v1, v2);
}

ExpressionSin::ExpressionSin(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Sin, e, false, e.is_expanded()} {}

Expression ExpressionSin::Expand() const {
  const Expression& arg{get_argument()};
  return sin(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionSin::Substitute(const Substitution& s) const {
  return sin(get_argument().Substitute(s));
}

Expression ExpressionSin::Differentiate(const Variable& x) const {
  // ∂/∂x (sin f) = (cos f) * (∂/∂x f)
  const Expression& f{get_argument()};
  return cos(f) * f.Differentiate(x);
}

ostream& ExpressionSin::Display(ostream& os) const {
  return os << "sin(" << get_argument() << ")";
}

double ExpressionSin::DoEvaluate(const double v) const { return std::sin(v); }

ExpressionCos::ExpressionCos(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Cos, e, false, e.is_expanded()} {}

Expression ExpressionCos::Expand() const {
  const Expression& arg{get_argument()};
  return cos(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionCos::Substitute(const Substitution& s) const {
  return cos(get_argument().Substitute(s));
}

Expression ExpressionCos::Differentiate(const Variable& x) const {
  // ∂/∂x (cos f) = - (sin f) * (∂/∂x f)
  const Expression& f{get_argument()};
  return -sin(f) * f.Differentiate(x);
}

ostream& ExpressionCos::Display(ostream& os) const {
  return os << "cos(" << get_argument() << ")";
}

double ExpressionCos::DoEvaluate(const double v) const { return std::cos(v); }

ExpressionTan::ExpressionTan(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Tan, e, false, e.is_expanded()} {}

Expression ExpressionTan::Expand() const {
  const Expression& arg{get_argument()};
  return tan(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionTan::Substitute(const Substitution& s) const {
  return tan(get_argument().Substitute(s));
}

Expression ExpressionTan::Differentiate(const Variable& x) const {
  // ∂/∂x (tan f) = (1 / (cos f)^2) * (∂/∂x f)
  const Expression& f{get_argument()};
  return (1 / pow(cos(f), 2)) * f.Differentiate(x);
}

ostream& ExpressionTan::Display(ostream& os) const {
  return os << "tan(" << get_argument() << ")";
}

double ExpressionTan::DoEvaluate(const double v) const { return std::tan(v); }

ExpressionAsin::ExpressionAsin(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Asin, e, false, e.is_expanded()} {}

void ExpressionAsin::check_domain(const double v) {
  if (!((v >= -1.0) && (v <= 1.0))) {
    ostringstream oss;
    oss << "asin(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

Expression ExpressionAsin::Expand() const {
  const Expression& arg{get_argument()};
  return asin(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionAsin::Substitute(const Substitution& s) const {
  return asin(get_argument().Substitute(s));
}

Expression ExpressionAsin::Differentiate(const Variable& x) const {
  // ∂/∂x (asin f) = (1 / sqrt(1 - f^2)) (∂/∂x f)
  const Expression& f{get_argument()};
  return (1 / sqrt(1 - pow(f, 2))) * f.Differentiate(x);
}

ostream& ExpressionAsin::Display(ostream& os) const {
  return os << "asin(" << get_argument() << ")";
}

double ExpressionAsin::DoEvaluate(const double v) const {
  check_domain(v);
  return std::asin(v);
}

ExpressionAcos::ExpressionAcos(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Acos, e, false, e.is_expanded()} {}

void ExpressionAcos::check_domain(const double v) {
  if (!((v >= -1.0) && (v <= 1.0))) {
    ostringstream oss;
    oss << "acos(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

Expression ExpressionAcos::Expand() const {
  const Expression& arg{get_argument()};
  return acos(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionAcos::Substitute(const Substitution& s) const {
  return acos(get_argument().Substitute(s));
}

Expression ExpressionAcos::Differentiate(const Variable& x) const {
  // ∂/∂x (acos f) = - 1 / sqrt(1 - f^2) * (∂/∂x f)
  const Expression& f{get_argument()};
  return -1 / sqrt(1 - pow(f, 2)) * f.Differentiate(x);
}

ostream& ExpressionAcos::Display(ostream& os) const {
  return os << "acos(" << get_argument() << ")";
}

double ExpressionAcos::DoEvaluate(const double v) const {
  check_domain(v);
  return std::acos(v);
}

ExpressionAtan::ExpressionAtan(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Atan, e, false, e.is_expanded()} {}

Expression ExpressionAtan::Expand() const {
  const Expression& arg{get_argument()};
  return atan(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionAtan::Substitute(const Substitution& s) const {
  return atan(get_argument().Substitute(s));
}

Expression ExpressionAtan::Differentiate(const Variable& x) const {
  // ∂/∂x (atan f) = (1 / (1 + f^2)) * ∂/∂x f
  const Expression& f{get_argument()};
  return (1 / (1 + pow(f, 2))) * f.Differentiate(x);
}

ostream& ExpressionAtan::Display(ostream& os) const {
  return os << "atan(" << get_argument() << ")";
}

double ExpressionAtan::DoEvaluate(const double v) const { return std::atan(v); }

ExpressionAtan2::ExpressionAtan2(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Atan2, e1, e2, false,
                           e1.is_expanded() && e2.is_expanded()} {}

Expression ExpressionAtan2::Expand() const {
  const Expression& e1{get_first_argument()};
  const Expression& e2{get_second_argument()};
  return atan2(e1.is_expanded() ? e1 : e1.Expand(),
               e2.is_expanded() ? e2 : e2.Expand());
}

Expression ExpressionAtan2::Substitute(const Substitution& s) const {
  return atan2(get_first_argument().Substitute(s),
               get_second_argument().Substitute(s));
}

Expression ExpressionAtan2::Differentiate(const Variable& x) const {
  // ∂/∂x (atan2(f,g)) = (g * (∂/∂x f) - f * (∂/∂x g)) / (f^2 + g^2)
  const Expression& f{get_first_argument()};
  const Expression& g{get_second_argument()};
  return (g * f.Differentiate(x) - f * g.Differentiate(x)) /
         (pow(f, 2) + pow(g, 2));
}

ostream& ExpressionAtan2::Display(ostream& os) const {
  return os << "atan2(" << get_first_argument() << ", " << get_second_argument()
            << ")";
}

double ExpressionAtan2::DoEvaluate(const double v1, const double v2) const {
  return std::atan2(v1, v2);
}

ExpressionSinh::ExpressionSinh(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Sinh, e, false, e.is_expanded()} {}

Expression ExpressionSinh::Expand() const {
  const Expression& arg{get_argument()};
  return sinh(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionSinh::Substitute(const Substitution& s) const {
  return sinh(get_argument().Substitute(s));
}

Expression ExpressionSinh::Differentiate(const Variable& x) const {
  // ∂/∂x (sinh f) = cosh(f) * (∂/∂x f)
  const Expression& f{get_argument()};
  return cosh(f) * f.Differentiate(x);
}

ostream& ExpressionSinh::Display(ostream& os) const {
  return os << "sinh(" << get_argument() << ")";
}

double ExpressionSinh::DoEvaluate(const double v) const { return std::sinh(v); }

ExpressionCosh::ExpressionCosh(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Cosh, e, false, e.is_expanded()} {}

Expression ExpressionCosh::Expand() const {
  const Expression& arg{get_argument()};
  return cosh(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionCosh::Substitute(const Substitution& s) const {
  return cosh(get_argument().Substitute(s));
}

Expression ExpressionCosh::Differentiate(const Variable& x) const {
  // ∂/∂x (cosh f) = sinh(f) * (∂/∂x f)
  const Expression& f{get_argument()};
  return sinh(f) * f.Differentiate(x);
}

ostream& ExpressionCosh::Display(ostream& os) const {
  return os << "cosh(" << get_argument() << ")";
}

double ExpressionCosh::DoEvaluate(const double v) const { return std::cosh(v); }

ExpressionTanh::ExpressionTanh(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Tanh, e, false, e.is_expanded()} {}

Expression ExpressionTanh::Expand() const {
  const Expression& arg{get_argument()};
  return tanh(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionTanh::Substitute(const Substitution& s) const {
  return tanh(get_argument().Substitute(s));
}

Expression ExpressionTanh::Differentiate(const Variable& x) const {
  // ∂/∂x (tanh f) = 1 / (cosh^2(f)) * (∂/∂x f)
  const Expression& f{get_argument()};
  return 1 / pow(cosh(f), 2) * f.Differentiate(x);
}

ostream& ExpressionTanh::Display(ostream& os) const {
  return os << "tanh(" << get_argument() << ")";
}

double ExpressionTanh::DoEvaluate(const double v) const { return std::tanh(v); }

ExpressionMin::ExpressionMin(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Min, e1, e2, false,
                           e1.is_expanded() && e2.is_expanded()} {}

Expression ExpressionMin::Expand() const {
  const Expression& e1{get_first_argument()};
  const Expression& e2{get_second_argument()};
  return min(e1.is_expanded() ? e1 : e1.Expand(),
             e2.is_expanded() ? e2 : e2.Expand());
}

Expression ExpressionMin::Substitute(const Substitution& s) const {
  return min(get_first_argument().Substitute(s),
             get_second_argument().Substitute(s));
}

Expression ExpressionMin::Differentiate(const Variable& x) const {
  if (GetVariables().include(x)) {
    const Expression& e1{get_first_argument()};
    const Expression& e2{get_second_argument()};
    return if_then_else(e1 < e2, e1.Differentiate(x),
                        if_then_else(
                            e1 == e2, Expression::NaN(),
                            e2.Differentiate(x)));
  } else {
    return Expression::Zero();
  }
}

ostream& ExpressionMin::Display(ostream& os) const {
  return os << "min(" << get_first_argument() << ", " << get_second_argument()
            << ")";
}

double ExpressionMin::DoEvaluate(const double v1, const double v2) const {
  return std::min(v1, v2);
}

ExpressionMax::ExpressionMax(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Max, e1, e2, false,
                           e1.is_expanded() && e2.is_expanded()} {}

Expression ExpressionMax::Expand() const {
  const Expression& e1{get_first_argument()};
  const Expression& e2{get_second_argument()};
  return max(e1.is_expanded() ? e1 : e1.Expand(),
             e2.is_expanded() ? e2 : e2.Expand());
}

Expression ExpressionMax::Substitute(const Substitution& s) const {
  return max(get_first_argument().Substitute(s),
             get_second_argument().Substitute(s));
}

Expression ExpressionMax::Differentiate(const Variable& x) const {
  if (GetVariables().include(x)) {
    const Expression& e1{get_first_argument()};
    const Expression& e2{get_second_argument()};
    return if_then_else(e1 > e2, e1.Differentiate(x),
                        if_then_else(e1 == e2, Expression::NaN(),
                                     e2.Differentiate(x)));
  } else {
    return Expression::Zero();
  }
}

ostream& ExpressionMax::Display(ostream& os) const {
  return os << "max(" << get_first_argument() << ", " << get_second_argument()
            << ")";
}

double ExpressionMax::DoEvaluate(const double v1, const double v2) const {
  return std::max(v1, v2);
}

ExpressionCeiling::ExpressionCeiling(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Ceil, e, false, e.is_expanded()} {}

Expression ExpressionCeiling::Expand() const {
  const Expression& arg{get_argument()};
  return ceil(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionCeiling::Substitute(const Substitution& s) const {
  return ceil(get_argument().Substitute(s));
}

Expression ExpressionCeiling::Differentiate(const Variable& x) const {
  if (GetVariables().include(x)) {
    const Expression& arg{get_argument()};
    // FYI:  'ceil(x) == floor(x)` is the same as `x % 1 == 0`.
    return if_then_else(ceil(arg) == floor(arg),
                        Expression::NaN(),
                        Expression::Zero());
  } else {
    return Expression::Zero();
  }
}

ostream& ExpressionCeiling::Display(ostream& os) const {
  return os << "ceil(" << get_argument() << ")";
}

double ExpressionCeiling::DoEvaluate(const double v) const {
  return std::ceil(v);
}

ExpressionFloor::ExpressionFloor(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Floor, e, false, e.is_expanded()} {}

Expression ExpressionFloor::Expand() const {
  const Expression& arg{get_argument()};
  return floor(arg.is_expanded() ? arg : arg.Expand());
}

Expression ExpressionFloor::Substitute(const Substitution& s) const {
  return floor(get_argument().Substitute(s));
}

Expression ExpressionFloor::Differentiate(const Variable& x) const {
  if (GetVariables().include(x)) {
    const Expression& arg{get_argument()};
    // FYI:  'ceil(x) == floor(x)` is the same as `x % 1 == 0`.
    return if_then_else(ceil(arg) == floor(arg),
                        Expression::NaN(),
                        Expression::Zero());
  } else {
    return Expression::Zero();
  }
}

ostream& ExpressionFloor::Display(ostream& os) const {
  return os << "floor(" << get_argument() << ")";
}

double ExpressionFloor::DoEvaluate(const double v) const {
  return std::floor(v);
}

// ExpressionIfThenElse
// --------------------
ExpressionIfThenElse::ExpressionIfThenElse(Formula f_cond, Expression e_then,
                                           Expression e_else)
    : ExpressionCell{ExpressionKind::IfThenElse, false, false},
      f_cond_{std::move(f_cond)},
      e_then_{std::move(e_then)},
      e_else_{std::move(e_else)} {}

void ExpressionIfThenElse::HashAppendDetail(DelegatingHasher* hasher) const {
  using drake::hash_append;
  hash_append(*hasher, f_cond_);
  hash_append(*hasher, e_then_);
  hash_append(*hasher, e_else_);
}

Variables ExpressionIfThenElse::GetVariables() const {
  Variables ret{f_cond_.GetFreeVariables()};
  ret.insert(e_then_.GetVariables());
  ret.insert(e_else_.GetVariables());
  return ret;
}

bool ExpressionIfThenElse::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const ExpressionIfThenElse& ite_e{
      static_cast<const ExpressionIfThenElse&>(e)};
  return f_cond_.EqualTo(ite_e.f_cond_) && e_then_.EqualTo(ite_e.e_then_) &&
         e_else_.EqualTo(ite_e.e_else_);
}

bool ExpressionIfThenElse::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
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
  }
  return e_else_.Evaluate(env);
}

Expression ExpressionIfThenElse::Expand() const {
  // TODO(soonho): use the following line when Formula::Expand() is implemented.
  // return if_then_else(f_cond_.Expand(), e_then_.Expand(), e_else_.Expand());
  throw runtime_error("Not yet implemented.");
}

Expression ExpressionIfThenElse::Substitute(const Substitution& s) const {
  return if_then_else(f_cond_.Substitute(s), e_then_.Substitute(s),
                      e_else_.Substitute(s));
}

Expression ExpressionIfThenElse::Differentiate(const Variable& x) const {
  if (GetVariables().include(x)) {
    if (is_relational(f_cond_)) {
      // In relational formulae, the discontinuity is at lhs == rhs.
      // TODO(ggould-tri) The logic of where/whether to find discontinuities
      // in a `Formula` belongs in that class, not here.  That could also
      // handle eg the degenerate case of constant formulae.
      // Refer to #8648 for additional information.
      return if_then_else(
          get_lhs_expression(f_cond_) == get_rhs_expression(f_cond_),
          Expression::NaN(),
          if_then_else(f_cond_,
                       e_then_.Differentiate(x),
                       e_else_.Differentiate(x)));
    } else {
      // Because we cannot write an expression for whether the condition is
      // discontinuous at a given environment, we blanket disallow
      // differentiation where the condition contains the differentiand.  We
      // hope that users can generally avoid this in practice, eg by using min
      // and max instead.
      ostringstream oss;
      Display(oss) << " is not differentiable with respect to " << x << ".";
      throw runtime_error(oss.str());
    }
  } else {
    return Expression::Zero();
  }
}

ostream& ExpressionIfThenElse::Display(ostream& os) const {
  return os << "(if " << f_cond_ << " then " << e_then_ << " else " << e_else_
            << ")";
}

// ExpressionUninterpretedFunction
// --------------------
ExpressionUninterpretedFunction::ExpressionUninterpretedFunction(
    string name, vector<Expression> arguments)
    : ExpressionCell{ExpressionKind::UninterpretedFunction, false,
                     all_of(arguments.begin(), arguments.end(),
                            [](const Expression& arg) {
                              return arg.is_expanded();
                            })},
      name_{std::move(name)},
      arguments_{std::move(arguments)} {}

void ExpressionUninterpretedFunction::HashAppendDetail(
    DelegatingHasher* hasher) const {
  using drake::hash_append;
  hash_append(*hasher, name_);
  hash_append_range(*hasher, arguments_.begin(), arguments_.end());
}

Variables ExpressionUninterpretedFunction::GetVariables() const {
  Variables ret;
  for (const Expression& arg : arguments_) {
    ret += arg.GetVariables();
  }
  return ret;
}

bool ExpressionUninterpretedFunction::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const ExpressionUninterpretedFunction& uf_e{
      static_cast<const ExpressionUninterpretedFunction&>(e)};
  return name_ == uf_e.name_ &&
         equal(arguments_.begin(), arguments_.end(), uf_e.arguments_.begin(),
               uf_e.arguments_.end(),
               [](const Expression& e1, const Expression& e2) {
                 return e1.EqualTo(e2);
               });
}

bool ExpressionUninterpretedFunction::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const ExpressionUninterpretedFunction& uf_e{
      static_cast<const ExpressionUninterpretedFunction&>(e)};
  if (name_ < uf_e.name_) {
    return true;
  }
  if (uf_e.name_ < name_) {
    return false;
  }
  return lexicographical_compare(
      arguments_.begin(), arguments_.end(), uf_e.arguments_.begin(),
      uf_e.arguments_.end(),
      [](const Expression& e1, const Expression& e2) { return e1.Less(e2); });
}

double ExpressionUninterpretedFunction::Evaluate(const Environment&) const {
  throw runtime_error("Uninterpreted-function expression cannot be evaluated.");
}

Expression ExpressionUninterpretedFunction::Expand() const {
  vector<Expression> new_arguments;
  new_arguments.reserve(arguments_.size());
  for (const Expression& arg : arguments_) {
    new_arguments.push_back(arg.is_expanded() ? arg : arg.Expand());
  }
  return uninterpreted_function(name_, std::move(new_arguments));
}

Expression ExpressionUninterpretedFunction::Substitute(
    const Substitution& s) const {
  vector<Expression> new_arguments;
  new_arguments.reserve(arguments_.size());
  for (const Expression& arg : arguments_) {
    new_arguments.push_back(arg.Substitute(s));
  }
  return uninterpreted_function(name_, std::move(new_arguments));
}

Expression ExpressionUninterpretedFunction::Differentiate(
    const Variable& x) const {
  if (GetVariables().include(x)) {
    // This uninterpreted function does have `x` as an argument, but we don't
    // have sufficient information to differentiate it with respect to `x`.
    ostringstream oss;
    oss << "Uninterpreted-function expression ";
    Display(oss);
    oss << " is not differentiable with respect to " << x << ".";
    throw runtime_error(oss.str());
  } else {
    // `x` is free in this uninterpreted function.
    return Expression::Zero();
  }
}

ostream& ExpressionUninterpretedFunction::Display(ostream& os) const {
  os << name_ << "(";
  if (!arguments_.empty()) {
    auto it = arguments_.begin();
    os << *(it++);
    for (; it != arguments_.end(); ++it) {
      os << ", " << *it;
    }
  }
  return os << ")";
}

bool is_constant(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Constant;
}
bool is_variable(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Var;
}
bool is_addition(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Add;
}
bool is_multiplication(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Mul;
}
bool is_division(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Div;
}
bool is_log(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Log;
}
bool is_abs(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Abs;
}
bool is_exp(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Exp;
}
bool is_sqrt(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Sqrt;
}
bool is_pow(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Pow;
}
bool is_sin(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Sin;
}
bool is_cos(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Cos;
}
bool is_tan(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Tan;
}
bool is_asin(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Asin;
}
bool is_acos(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Acos;
}
bool is_atan(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Atan;
}
bool is_atan2(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Atan2;
}
bool is_sinh(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Sinh;
}
bool is_cosh(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Cosh;
}
bool is_tanh(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Tanh;
}
bool is_min(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Min;
}
bool is_max(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Max;
}
bool is_ceil(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Ceil;
}
bool is_floor(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::Floor;
}
bool is_if_then_else(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::IfThenElse;
}
bool is_uninterpreted_function(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::UninterpretedFunction;
}

const ExpressionConstant& to_constant(const Expression& e) {
  DRAKE_ASSERT(is_constant(e));
  return static_cast<const ExpressionConstant&>(e.cell());
}

ExpressionConstant& to_constant(Expression* const e) {
  DRAKE_ASSERT(e && is_constant(*e));
  return static_cast<ExpressionConstant&>(e->mutable_cell());
}

const ExpressionVar& to_variable(const Expression& e) {
  DRAKE_ASSERT(is_variable(e));
  return static_cast<const ExpressionVar&>(e.cell());
}

ExpressionVar& to_variable(Expression* const e) {
  DRAKE_ASSERT(e && is_variable(*e));
  return static_cast<ExpressionVar&>(e->mutable_cell());
}

bool is_unary(const ExpressionCell& cell) {
  return (is_log(cell) || is_abs(cell) || is_exp(cell) || is_sqrt(cell) ||
          is_sin(cell) || is_cos(cell) || is_tan(cell) || is_asin(cell) ||
          is_acos(cell) || is_atan(cell) || is_sinh(cell) || is_cosh(cell) ||
          is_tanh(cell) || is_ceil(cell) || is_floor(cell));
}

const UnaryExpressionCell& to_unary(const Expression& e) {
  DRAKE_ASSERT(is_unary(e.cell()));
  return static_cast<const UnaryExpressionCell&>(e.cell());
}

UnaryExpressionCell& to_unary(Expression* const e) {
  DRAKE_ASSERT(e && is_unary(e->cell()));
  return static_cast<UnaryExpressionCell&>(e->mutable_cell());
}

bool is_binary(const ExpressionCell& cell) {
  return (is_division(cell) || is_pow(cell) || is_atan2(cell) ||
          is_min(cell) || is_max(cell));
}

const BinaryExpressionCell& to_binary(const Expression& e) {
  DRAKE_ASSERT(is_binary(e.cell()));
  return static_cast<const BinaryExpressionCell&>(e.cell());
}

BinaryExpressionCell& to_binary(Expression* const e) {
  DRAKE_ASSERT(e && is_binary(e->cell()));
  return static_cast<BinaryExpressionCell&>(e->mutable_cell());
}

const ExpressionAdd& to_addition(const Expression& e) {
  DRAKE_ASSERT(is_addition(e));
  return static_cast<const ExpressionAdd&>(e.cell());
}

ExpressionAdd& to_addition(Expression* const e) {
  DRAKE_ASSERT(e && is_addition(*e));
  return static_cast<ExpressionAdd&>(e->mutable_cell());
}

const ExpressionMul& to_multiplication(const Expression& e) {
  DRAKE_ASSERT(is_multiplication(e));
  return static_cast<const ExpressionMul&>(e.cell());
}

ExpressionMul& to_multiplication(Expression* const e) {
  DRAKE_ASSERT(e && is_multiplication(*e));
  return static_cast<ExpressionMul&>(e->mutable_cell());
}

const ExpressionDiv& to_division(const Expression& e) {
  DRAKE_ASSERT(is_division(e));
  return static_cast<const ExpressionDiv&>(e.cell());
}

ExpressionDiv& to_division(Expression* const e) {
  DRAKE_ASSERT(e && is_division(*e));
  return static_cast<ExpressionDiv&>(e->mutable_cell());
}

const ExpressionLog& to_log(const Expression& e) {
  DRAKE_ASSERT(is_log(e));
  return static_cast<const ExpressionLog&>(e.cell());
}

ExpressionLog& to_log(Expression* const e) {
  DRAKE_ASSERT(e && is_log(*e));
  return static_cast<ExpressionLog&>(e->mutable_cell());
}

const ExpressionAbs& to_abs(const Expression& e) {
  DRAKE_ASSERT(is_abs(e));
  return static_cast<const ExpressionAbs&>(e.cell());
}

ExpressionAbs& to_abs(Expression* const e) {
  DRAKE_ASSERT(e && is_abs(*e));
  return static_cast<ExpressionAbs&>(e->mutable_cell());
}

const ExpressionExp& to_exp(const Expression& e) {
  DRAKE_ASSERT(is_exp(e));
  return static_cast<const ExpressionExp&>(e.cell());
}

ExpressionExp& to_exp(Expression* const e) {
  DRAKE_ASSERT(e && is_exp(*e));
  return static_cast<ExpressionExp&>(e->mutable_cell());
}

const ExpressionSqrt& to_sqrt(const Expression& e) {
  DRAKE_ASSERT(is_sqrt(e));
  return static_cast<const ExpressionSqrt&>(e.cell());
}

ExpressionSqrt& to_sqrt(Expression* const e) {
  DRAKE_ASSERT(e && is_sqrt(*e));
  return static_cast<ExpressionSqrt&>(e->mutable_cell());
}

const ExpressionPow& to_pow(const Expression& e) {
  DRAKE_ASSERT(is_pow(e));
  return static_cast<const ExpressionPow&>(e.cell());
}

ExpressionPow& to_pow(Expression* const e) {
  DRAKE_ASSERT(e && is_pow(*e));
  return static_cast<ExpressionPow&>(e->mutable_cell());
}

const ExpressionSin& to_sin(const Expression& e) {
  DRAKE_ASSERT(is_sin(e));
  return static_cast<const ExpressionSin&>(e.cell());
}

ExpressionSin& to_sin(Expression* const e) {
  DRAKE_ASSERT(e && is_sin(*e));
  return static_cast<ExpressionSin&>(e->mutable_cell());
}

const ExpressionCos& to_cos(const Expression& e) {
  DRAKE_ASSERT(is_cos(e));
  return static_cast<const ExpressionCos&>(e.cell());
}

ExpressionCos& to_cos(Expression* const e) {
  DRAKE_ASSERT(e && is_cos(*e));
  return static_cast<ExpressionCos&>(e->mutable_cell());
}

const ExpressionTan& to_tan(const Expression& e) {
  DRAKE_ASSERT(is_tan(e));
  return static_cast<const ExpressionTan&>(e.cell());
}

ExpressionTan& to_tan(Expression* const e) {
  DRAKE_ASSERT(e && is_tan(*e));
  return static_cast<ExpressionTan&>(e->mutable_cell());
}

const ExpressionAsin& to_asin(const Expression& e) {
  DRAKE_ASSERT(is_asin(e));
  return static_cast<const ExpressionAsin&>(e.cell());
}

ExpressionAsin& to_asin(Expression* const e) {
  DRAKE_ASSERT(e && is_asin(*e));
  return static_cast<ExpressionAsin&>(e->mutable_cell());
}

const ExpressionAcos& to_acos(const Expression& e) {
  DRAKE_ASSERT(is_acos(e));
  return static_cast<const ExpressionAcos&>(e.cell());
}

ExpressionAcos& to_acos(Expression* const e) {
  DRAKE_ASSERT(e && is_acos(*e));
  return static_cast<ExpressionAcos&>(e->mutable_cell());
}

const ExpressionAtan& to_atan(const Expression& e) {
  DRAKE_ASSERT(is_atan(e));
  return static_cast<const ExpressionAtan&>(e.cell());
}

ExpressionAtan& to_atan(Expression* const e) {
  DRAKE_ASSERT(e && is_atan(*e));
  return static_cast<ExpressionAtan&>(e->mutable_cell());
}

const ExpressionAtan2& to_atan2(const Expression& e) {
  DRAKE_ASSERT(is_atan2(e));
  return static_cast<const ExpressionAtan2&>(e.cell());
}

ExpressionAtan2& to_atan2(Expression* const e) {
  DRAKE_ASSERT(e && is_atan2(*e));
  return static_cast<ExpressionAtan2&>(e->mutable_cell());
}

const ExpressionSinh& to_sinh(const Expression& e) {
  DRAKE_ASSERT(is_sinh(e));
  return static_cast<const ExpressionSinh&>(e.cell());
}

ExpressionSinh& to_sinh(Expression* const e) {
  DRAKE_ASSERT(e && is_sinh(*e));
  return static_cast<ExpressionSinh&>(e->mutable_cell());
}

const ExpressionCosh& to_cosh(const Expression& e) {
  DRAKE_ASSERT(is_cosh(e));
  return static_cast<const ExpressionCosh&>(e.cell());
}

ExpressionCosh& to_cosh(Expression* const e) {
  DRAKE_ASSERT(e && is_cosh(*e));
  return static_cast<ExpressionCosh&>(e->mutable_cell());
}

const ExpressionTanh& to_tanh(const Expression& e) {
  DRAKE_ASSERT(is_tanh(e));
  return static_cast<const ExpressionTanh&>(e.cell());
}

ExpressionTanh& to_tanh(Expression* const e) {
  DRAKE_ASSERT(e && is_tanh(*e));
  return static_cast<ExpressionTanh&>(e->mutable_cell());
}

const ExpressionMin& to_min(const Expression& e) {
  DRAKE_ASSERT(is_min(e));
  return static_cast<const ExpressionMin&>(e.cell());
}

ExpressionMin& to_min(Expression* const e) {
  DRAKE_ASSERT(e && is_min(*e));
  return static_cast<ExpressionMin&>(e->mutable_cell());
}

const ExpressionMax& to_max(const Expression& e) {
  DRAKE_ASSERT(is_max(e));
  return static_cast<const ExpressionMax&>(e.cell());
}

ExpressionMax& to_max(Expression* const e) {
  DRAKE_ASSERT(e && is_max(*e));
  return static_cast<ExpressionMax&>(e->mutable_cell());
}

const ExpressionCeiling& to_ceil(const Expression& e) {
  DRAKE_ASSERT(is_ceil(e));
  return static_cast<const ExpressionCeiling&>(e.cell());
}

ExpressionCeiling& to_ceil(Expression* const e) {
  DRAKE_ASSERT(e && is_ceil(*e));
  return static_cast<ExpressionCeiling&>(e->mutable_cell());
}

const ExpressionFloor& to_floor(const Expression& e) {
  DRAKE_ASSERT(is_floor(e));
  return static_cast<const ExpressionFloor&>(e.cell());
}

ExpressionFloor& to_floor(Expression* const e) {
  DRAKE_ASSERT(e && is_floor(*e));
  return static_cast<ExpressionFloor&>(e->mutable_cell());
}

const ExpressionIfThenElse& to_if_then_else(const Expression& e) {
  DRAKE_ASSERT(is_if_then_else(e));
  return static_cast<const ExpressionIfThenElse&>(e.cell());
}

ExpressionIfThenElse& to_if_then_else(Expression* const e) {
  DRAKE_ASSERT(e && is_if_then_else(*e));
  return static_cast<ExpressionIfThenElse&>(e->mutable_cell());
}

const ExpressionUninterpretedFunction& to_uninterpreted_function(
    const Expression& e) {
  DRAKE_ASSERT(is_uninterpreted_function(e));
  return static_cast<const ExpressionUninterpretedFunction&>(e.cell());
}

ExpressionUninterpretedFunction& to_uninterpreted_function(
    Expression* const e) {
  DRAKE_ASSERT(e && is_uninterpreted_function(*e));
  return static_cast<ExpressionUninterpretedFunction&>(e->mutable_cell());
}

}  // namespace symbolic
}  // namespace drake
