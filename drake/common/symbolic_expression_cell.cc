#include "drake/common/symbolic_expression_cell.h"

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
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_compat.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::accumulate;
using std::all_of;
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

namespace {
bool is_integer(const double v) {
  // v should be in [int_min, int_max].
  if (!((numeric_limits<int>::lowest() <= v) &&
        (v <= numeric_limits<int>::max()))) {
    return false;
  }

  double intpart;  // dummy variable
  return modf(v, &intpart) == 0.0;
}

bool is_non_negative_integer(const double v) {
  return (v >= 0) && is_integer(v);
}

// Determines if the summation represented by term_to_coeff_map is
// polynomial-convertible or not. This function is used in the
// constructor of ExpressionAdd.
bool determine_polynomial(
    const std::map<Expression, double>& term_to_coeff_map) {
  return all_of(term_to_coeff_map.begin(), term_to_coeff_map.end(),
                [](const pair<Expression, double>& p) {
                  return p.first.is_polynomial();
                });
}

// Determines if the product represented by term_to_coeff_map is
// polynomial-convertible or not. This function is used in the
// constructor of ExpressionMul.
static bool determine_polynomial(
    const std::map<Expression, Expression>& base_to_exponent_map) {
  return all_of(base_to_exponent_map.begin(), base_to_exponent_map.end(),
                [](const pair<Expression, Expression>& p) {
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
  // base ^ exponent is polynomial-convertible if the followings hold:
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
    return accumulate(
        m1.begin(), m1.end(), ExpandMultiplication(c0, e2),
        [&e2](const Expression& init, const pair<Expression, double>& p) {
          return init + ExpandMultiplication(p.second, p.first, e2);
        });
  } else if (is_addition(e2)) {
    //   e1 * (c0 + c1 * e_{2,1} + ... + c_n * e_{2, n})
    // = e1 * c0 + e1 * c1 * e_{2,1} + ... + e1 * c_n * e_{2,n}
    const double c0{get_constant_in_addition(e2)};
    const map<Expression, double>& m1{get_expr_to_coeff_map_in_addition(e2)};
    return accumulate(
        m1.begin(), m1.end(), ExpandMultiplication(e1, c0),
        [&e1](const Expression& init, const pair<Expression, double>& p) {
          return init + ExpandMultiplication(e1, p.second, p.first);
        });
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
  } else {
    // pow(base, n) = pow(base, n / 2) * pow(base, n / 2)
    return ExpandMultiplication(pow_half, pow_half);
  }
}

// Helper function expanding pow(base, exponent). It assumes that both of base
// and exponent are already expanded.
Expression ExpandPow(const Expression& base, const Expression& exponent) {
  // Precondition: base and exponent are already expanded.
  DRAKE_ASSERT(base.EqualTo(base.Expand()));
  DRAKE_ASSERT(exponent.EqualTo(exponent.Expand()));
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

ExpressionCell::ExpressionCell(const ExpressionKind k, const size_t hash,
                               const bool is_poly)
    : kind_{k},
      hash_{hash_combine(static_cast<size_t>(kind_), hash)},
      is_polynomial_{is_poly} {}

UnaryExpressionCell::UnaryExpressionCell(const ExpressionKind k,
                                         const Expression& e,
                                         const bool is_poly)
    : ExpressionCell{k, e.get_hash(), is_poly}, e_{e} {}

Variables UnaryExpressionCell::GetVariables() const {
  return e_.GetVariables();
}

bool UnaryExpressionCell::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const UnaryExpressionCell& unary_e =
      static_cast<const UnaryExpressionCell&>(e);
  return e_.EqualTo(unary_e.e_);
}

bool UnaryExpressionCell::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const UnaryExpressionCell& unary_e =
      static_cast<const UnaryExpressionCell&>(e);
  return e_.Less(unary_e.e_);
}

double UnaryExpressionCell::Evaluate(const Environment& env) const {
  const double v{e_.Evaluate(env)};
  return DoEvaluate(v);
}

BinaryExpressionCell::BinaryExpressionCell(const ExpressionKind k,
                                           const Expression& e1,
                                           const Expression& e2,
                                           const bool is_poly)
    : ExpressionCell{k, hash_combine(e1.get_hash(), e2), is_poly},
      e1_{e1},
      e2_{e2} {}

Variables BinaryExpressionCell::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  ret.insert(e2_.GetVariables());
  return ret;
}

bool BinaryExpressionCell::EqualTo(const ExpressionCell& e) const {
  // Expression::EqualTo guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const BinaryExpressionCell& binary_e =
      static_cast<const BinaryExpressionCell&>(e);
  return e1_.EqualTo(binary_e.e1_) && e2_.EqualTo(binary_e.e2_);
}

bool BinaryExpressionCell::Less(const ExpressionCell& e) const {
  // Expression::Less guarantees the following assertion.
  DRAKE_ASSERT(get_kind() == e.get_kind());
  const BinaryExpressionCell& binary_e =
      static_cast<const BinaryExpressionCell&>(e);
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
    : ExpressionCell{ExpressionKind::Var, hash_value<Variable>{}(v), true},
      var_{v} {
  // Dummy symbolic variable (ID = 0) should not be used in constructing
  // symbolic expressions.
  DRAKE_DEMAND(!var_.is_dummy());
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

Polynomial<double> ExpressionVar::ToPolynomial() const {
  return Polynomial<double>(1.0, var_.get_id());
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

Expression ExpressionVar::Expand() const { return Expression{var_}; }

Expression ExpressionVar::Substitute(const Substitution& s) const {
  const Substitution::const_iterator it{s.find(var_)};
  if (it != s.end()) {
    return it->second;
  } else {
    return Expression{var_};
  }
}

Expression ExpressionVar::Differentiate(const Variable& x) const {
  if (x.equal_to(var_)) {
    return Expression::One();
  } else {
    return Expression::Zero();
  }
}

ostream& ExpressionVar::Display(ostream& os) const { return os << var_; }

ExpressionConstant::ExpressionConstant(const double v)
    : ExpressionCell{ExpressionKind::Constant, hash<double>{}(v), true}, v_{v} {
  DRAKE_ASSERT(!std::isnan(v));
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

Polynomial<double> ExpressionConstant::ToPolynomial() const {
  return Polynomial<double>(v_);
}

double ExpressionConstant::Evaluate(const Environment& env) const {
  DRAKE_DEMAND(!std::isnan(v_));
  return v_;
}

Expression ExpressionConstant::Expand() const { return Expression{v_}; }

Expression ExpressionConstant::Substitute(const Substitution& s) const {
  DRAKE_DEMAND(!std::isnan(v_));
  return Expression{v_};
}

Expression ExpressionConstant::Differentiate(const Variable& x) const {
  return Expression::Zero();
}

ostream& ExpressionConstant::Display(ostream& os) const {
  ostringstream oss;
  oss << setprecision(numeric_limits<double>::max_digits10) << v_;
  return os << oss.str();
}

ExpressionNaN::ExpressionNaN()
    : ExpressionCell{ExpressionKind::NaN, 41, false} {
  // ExpressionCell constructor calls hash_combine(ExpressionKind::NaN, 41) to
  // compute the hash of ExpressionNaN. Here 41 does not have any special
  // meaning.
}

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

Polynomial<double> ExpressionNaN::ToPolynomial() const {
  throw runtime_error("NaN is detected while converting to Polynomial.");
}

double ExpressionNaN::Evaluate(const Environment& env) const {
  throw runtime_error("NaN is detected during Symbolic computation.");
}

Expression ExpressionNaN::Expand() const {
  throw runtime_error("NaN is detected during expansion.");
}

Expression ExpressionNaN::Substitute(const Substitution& s) const {
  throw runtime_error("NaN is detected during substitution.");
}

Expression ExpressionNaN::Differentiate(const Variable& x) const {
  throw runtime_error("NaN is detected during differentiation.");
}

ostream& ExpressionNaN::Display(ostream& os) const { return os << "NaN"; }

ExpressionAdd::ExpressionAdd(const double constant,
                             const map<Expression, double>& expr_to_coeff_map)
    : ExpressionCell{ExpressionKind::Add,
                     hash_combine(hash<double>{}(constant), expr_to_coeff_map),
                     determine_polynomial(expr_to_coeff_map)},
      constant_(constant),
      expr_to_coeff_map_(expr_to_coeff_map) {
  DRAKE_ASSERT(!expr_to_coeff_map_.empty());
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
               [](const pair<Expression, double>& p1,
                  const pair<Expression, double>& p2) {
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
      [](const pair<Expression, double>& p1,
         const pair<Expression, double>& p2) {
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

Polynomial<double> ExpressionAdd::ToPolynomial() const {
  DRAKE_ASSERT(is_polynomial());
  return accumulate(expr_to_coeff_map_.begin(), expr_to_coeff_map_.end(),
                    Polynomial<double>(constant_),
                    [](const Polynomial<double>& polynomial,
                       const pair<Expression, double>& p) {
                      return polynomial + p.first.ToPolynomial() * p.second;
                    });
}

double ExpressionAdd::Evaluate(const Environment& env) const {
  return accumulate(
      expr_to_coeff_map_.begin(), expr_to_coeff_map_.end(), constant_,
      [&env](const double init, const pair<Expression, double>& p) {
        return init + p.first.Evaluate(env) * p.second;
      });
}

Expression ExpressionAdd::Expand() const {
  //   (c0 + c1 * e_1 + ... + c_n * e_n).Expand()
  // =  c0 + c1 * e_1.Expand() + ... + c_n * e_n.Expand()
  return accumulate(
      expr_to_coeff_map_.begin(), expr_to_coeff_map_.end(),
      Expression{constant_},
      [](const Expression& init, const pair<Expression, double>& p) {
        return init + ExpandMultiplication(p.first.Expand(), p.second);
      });
}

Expression ExpressionAdd::Substitute(const Substitution& s) const {
  return accumulate(
      expr_to_coeff_map_.begin(), expr_to_coeff_map_.end(),
      Expression{constant_},
      [&s](const Expression& init, const pair<Expression, double>& p) {
        return init + p.first.Substitute(s) * p.second;
      });
}

Expression ExpressionAdd::Differentiate(const Variable& x) const {
  //   ∂/∂x (c_0 + c_1 * f_1 + ... + c_n * f_n)
  // = (∂/∂x c_0) + (∂/∂x c_1 * f_1) + ... + (∂/∂x c_n * f_n)
  // =  0.0       + c_1 * (∂/∂x f_1) + ... + c_n * (∂/∂x f_n)
  return accumulate(
      expr_to_coeff_map_.begin(), expr_to_coeff_map_.end(), Expression::Zero(),
      [&x](const Expression& init, const pair<Expression, double>& p) {
        return init + p.second * p.first.Differentiate(x);
      });
}

ostream& ExpressionAdd::Display(ostream& os) const {
  DRAKE_ASSERT(!expr_to_coeff_map_.empty());
  bool print_plus{false};
  os << "(";
  if (constant_ != 0.0) {
    os << constant_;
    print_plus = true;
  }
  for (auto& p : expr_to_coeff_map_) {
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
    const double constant, const map<Expression, double>& expr_to_coeff_map)
    : constant_{constant}, expr_to_coeff_map_{expr_to_coeff_map} {}

ExpressionAddFactory::ExpressionAddFactory(
    const shared_ptr<const ExpressionAdd> ptr)
    : ExpressionAddFactory{ptr->get_constant(), ptr->get_expr_to_coeff_map()} {}

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

void ExpressionAddFactory::Add(const shared_ptr<const ExpressionAdd> ptr) {
  AddConstant(ptr->get_constant());
  AddMap(ptr->get_expr_to_coeff_map());
}

ExpressionAddFactory& ExpressionAddFactory::operator=(
    const shared_ptr<ExpressionAdd> ptr) {
  constant_ = ptr->get_constant();
  expr_to_coeff_map_ = ptr->get_expr_to_coeff_map();
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
  if (constant_ == 0.0 && expr_to_coeff_map_.size() == 1u) {
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
    const map<Expression, double> expr_to_coeff_map) {
  for (const auto& p : expr_to_coeff_map) {
    AddTerm(p.second, p.first);
  }
}

ExpressionMul::ExpressionMul(
    const double constant,
    const map<Expression, Expression>& base_to_exponent_map)
    : ExpressionCell{ExpressionKind::Mul, hash_combine(hash<double>{}(constant),
                                                       base_to_exponent_map),
                     determine_polynomial(base_to_exponent_map)},
      constant_(constant),
      base_to_exponent_map_(base_to_exponent_map) {
  DRAKE_ASSERT(!base_to_exponent_map_.empty());
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
      [](const pair<Expression, Expression>& p1,
         const pair<Expression, Expression>& p2) {
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

Polynomial<double> ExpressionMul::ToPolynomial() const {
  DRAKE_ASSERT(is_polynomial());
  return accumulate(
      base_to_exponent_map_.begin(), base_to_exponent_map_.end(),
      Polynomial<double>{constant_}, [](const Polynomial<double>& polynomial,
                                        const pair<Expression, Expression>& p) {
        const Expression& base{p.first};
        const Expression& exponent{p.second};
        DRAKE_ASSERT(base.is_polynomial());
        DRAKE_ASSERT(is_constant(exponent));
        return polynomial * pow(base.ToPolynomial(),
                                static_cast<int>(get_constant_value(exponent)));
      });
}

double ExpressionMul::Evaluate(const Environment& env) const {
  return accumulate(
      base_to_exponent_map_.begin(), base_to_exponent_map_.end(), constant_,
      [&env](const double init, const pair<Expression, Expression>& p) {
        return init * std::pow(p.first.Evaluate(env), p.second.Evaluate(env));
      });
}

Expression ExpressionMul::Expand() const {
  //   (c * ∏ᵢ pow(bᵢ, eᵢ)).Expand()
  // = c * ExpandMultiplication(∏ ExpandPow(bᵢ.Expand(), eᵢ.Expand()))
  return accumulate(
      base_to_exponent_map_.begin(), base_to_exponent_map_.end(),
      Expression{constant_},
      [](const Expression& init, const pair<Expression, Expression>& p) {
        return ExpandMultiplication(
            init, ExpandPow(p.first.Expand(), p.second.Expand()));
      });
}

Expression ExpressionMul::Substitute(const Substitution& s) const {
  return accumulate(
      base_to_exponent_map_.begin(), base_to_exponent_map_.end(),
      Expression{constant_},
      [&s](const Expression& init, const pair<Expression, Expression>& p) {
        return init * pow(p.first.Substitute(s), p.second.Substitute(s));
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
  } else if (is_constant(f)) {
    const Expression& n{f};  // alias n = f
    // Special case where base is a constant:
    //     ∂/∂x pow(n, g) = log(n) * pow(n, g) * ∂/∂x g
    return log(n) * pow(n, g) * g.Differentiate(x);
  } else {
    // General case:
    //    ∂/∂x pow(f, g)
    // = ∂/∂f pow(f, g) * ∂/∂x f + ∂/∂g pow(f, g) * ∂/∂x g
    // = g * pow(f, g - 1) * ∂/∂x f + log(f) * pow(f, g) * ∂/∂x g
    // = pow(f, g - 1) * (g * ∂/∂x f + log(f) * f * ∂/∂x g)
    return pow(f, g - 1) *
           (g * f.Differentiate(x) + log(f) * f * g.Differentiate(x));
  }
}

Expression ExpressionMul::Differentiate(const Variable& x) const {
  // ∂/∂x (c   * f_1^g_1  * f_2^g_2        * ... * f_n^g_n
  //= c * [expr * (∂/∂x f_1^g_1) / f_1^g_1 +
  //       expr * (∂/∂x f_2^g_2) / f_2^g_2 +
  //                      ...              +
  //       expr * (∂/∂x f_n^g_n) / f_n^g_n]
  //
  // where expr = (f_1^g_1 * f_2^g_2 * ... * f_n^g_n).
  const map<Expression, Expression>& m{base_to_exponent_map_};
  Expression ret{Expression::Zero()};
  const Expression expr{
      ExpressionMulFactory{1.0, base_to_exponent_map_}.GetExpression()};
  for (const auto& term : m) {
    const Expression& base{term.first};
    const Expression& exponent{term.second};
    ret += expr * DifferentiatePow(base, exponent, x) * pow(base, -exponent);
  }
  return constant_ * ret;
}

ostream& ExpressionMul::Display(ostream& os) const {
  DRAKE_ASSERT(!base_to_exponent_map_.empty());
  bool print_mul{false};
  os << "(";
  if (constant_ != 1.0) {
    os << constant_;
    print_mul = true;
  }
  for (auto& p : base_to_exponent_map_) {
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
    const double constant,
    const map<Expression, Expression>& base_to_exponent_map)
    : constant_{constant}, base_to_exponent_map_{base_to_exponent_map} {}

ExpressionMulFactory::ExpressionMulFactory(
    const shared_ptr<const ExpressionMul> ptr)
    : ExpressionMulFactory{ptr->get_constant(),
                           ptr->get_base_to_exponent_map()} {}

void ExpressionMulFactory::AddExpression(const Expression& e) {
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

void ExpressionMulFactory::Add(const shared_ptr<const ExpressionMul> ptr) {
  AddConstant(ptr->get_constant());
  AddMap(ptr->get_base_to_exponent_map());
}

ExpressionMulFactory& ExpressionMulFactory::operator=(
    const shared_ptr<ExpressionMul> ptr) {
  constant_ = ptr->get_constant();
  base_to_exponent_map_ = ptr->get_base_to_exponent_map();
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
  if (constant_ == 1.0 && base_to_exponent_map_.size() == 1u) {
    // 1.0 * c1^t1 -> c1^t1
    const auto it(base_to_exponent_map_.cbegin());
    return pow(it->first, it->second);
  }
  return Expression{
      make_shared<ExpressionMul>(constant_, base_to_exponent_map_)};
}

void ExpressionMulFactory::AddConstant(const double constant) {
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
    const map<Expression, Expression> base_to_exponent_map) {
  for (const auto& p : base_to_exponent_map) {
    AddTerm(p.first, p.second);
  }
}

ExpressionDiv::ExpressionDiv(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Div, e1, e2,
                           e1.is_polynomial() && is_constant(e2)} {}

Polynomial<double> ExpressionDiv::ToPolynomial() const {
  DRAKE_ASSERT(is_polynomial());
  DRAKE_ASSERT(is_constant(get_second_argument()));
  return get_first_argument().ToPolynomial() /
         get_constant_value(get_second_argument());
}

Expression ExpressionDiv::Expand() const {
  return get_first_argument().Expand() / get_second_argument().Expand();
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
    : UnaryExpressionCell{ExpressionKind::Log, e, false} {}

void ExpressionLog::check_domain(const double v) {
  if (!(v >= 0)) {
    ostringstream oss;
    oss << "log(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

Polynomial<double> ExpressionLog::ToPolynomial() const {
  throw runtime_error("Log expression is not polynomial-convertible.");
}

Expression ExpressionLog::Expand() const {
  return log(get_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Abs, e, false} {}

Polynomial<double> ExpressionAbs::ToPolynomial() const {
  throw runtime_error("Abs expression is not polynomial-convertible.");
}

Expression ExpressionAbs::Expand() const {
  return abs(get_argument().Expand());
}

Expression ExpressionAbs::Substitute(const Substitution& s) const {
  return abs(get_argument().Substitute(s));
}

Expression ExpressionAbs::Differentiate(const Variable& x) const {
  if (GetVariables().include(x)) {
    ostringstream oss;
    Display(oss) << "is not differentiable with respect to " << x << ".";
    throw runtime_error(oss.str());
  } else {
    return Expression::Zero();
  }
}

ostream& ExpressionAbs::Display(ostream& os) const {
  return os << "abs(" << get_argument() << ")";
}

double ExpressionAbs::DoEvaluate(const double v) const { return std::fabs(v); }

ExpressionExp::ExpressionExp(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Exp, e, false} {}

Polynomial<double> ExpressionExp::ToPolynomial() const {
  throw runtime_error("Exp expression is not polynomial-convertible.");
}

Expression ExpressionExp::Expand() const {
  return exp(get_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Sqrt, e, false} {}

void ExpressionSqrt::check_domain(const double v) {
  if (!(v >= 0)) {
    ostringstream oss;
    oss << "sqrt(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

Polynomial<double> ExpressionSqrt::ToPolynomial() const {
  throw runtime_error("Sqrt expression is not polynomial-convertible.");
}

Expression ExpressionSqrt::Expand() const {
  return sqrt(get_argument().Expand());
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
                           determine_polynomial(e1, e2)} {}

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

Polynomial<double> ExpressionPow::ToPolynomial() const {
  DRAKE_ASSERT(is_polynomial());
  const int exponent{
      static_cast<int>(get_constant_value(get_second_argument()))};
  return pow(get_first_argument().ToPolynomial(), exponent);
}

Expression ExpressionPow::Expand() const {
  return ExpandPow(get_first_argument().Expand(),
                   get_second_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Sin, e, false} {}

Polynomial<double> ExpressionSin::ToPolynomial() const {
  throw runtime_error("Sin expression is not polynomial-convertible.");
}

Expression ExpressionSin::Expand() const {
  return sin(get_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Cos, e, false} {}

Polynomial<double> ExpressionCos::ToPolynomial() const {
  throw runtime_error("Cos expression is not polynomial-convertible.");
}

Expression ExpressionCos::Expand() const {
  return cos(get_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Tan, e, false} {}

Polynomial<double> ExpressionTan::ToPolynomial() const {
  throw runtime_error("Tan expression is not polynomial-convertible.");
}

Expression ExpressionTan::Expand() const {
  return tan(get_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Asin, e, false} {}

void ExpressionAsin::check_domain(const double v) {
  if (!((v >= -1.0) && (v <= 1.0))) {
    ostringstream oss;
    oss << "asin(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

Polynomial<double> ExpressionAsin::ToPolynomial() const {
  throw runtime_error("Asin expression is not polynomial-convertible.");
}

Expression ExpressionAsin::Expand() const {
  return asin(get_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Acos, e, false} {}

void ExpressionAcos::check_domain(const double v) {
  if (!((v >= -1.0) && (v <= 1.0))) {
    ostringstream oss;
    oss << "acos(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

Polynomial<double> ExpressionAcos::ToPolynomial() const {
  throw runtime_error("Acos expression is not polynomial-convertible.");
}

Expression ExpressionAcos::Expand() const {
  return acos(get_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Atan, e, false} {}

Polynomial<double> ExpressionAtan::ToPolynomial() const {
  throw runtime_error("Atan expression is not polynomial-convertible.");
}

Expression ExpressionAtan::Expand() const {
  return atan(get_argument().Expand());
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
    : BinaryExpressionCell{ExpressionKind::Atan2, e1, e2, false} {}

Polynomial<double> ExpressionAtan2::ToPolynomial() const {
  throw runtime_error("Atan2 expression is not polynomial-convertible.");
}

Expression ExpressionAtan2::Expand() const {
  return atan2(get_first_argument().Expand(), get_second_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Sinh, e, false} {}

Polynomial<double> ExpressionSinh::ToPolynomial() const {
  throw runtime_error("Sinh expression is not polynomial-convertible.");
}

Expression ExpressionSinh::Expand() const {
  return sinh(get_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Cosh, e, false} {}

Polynomial<double> ExpressionCosh::ToPolynomial() const {
  throw runtime_error("Cosh expression is not polynomial-convertible.");
}

Expression ExpressionCosh::Expand() const {
  return cosh(get_argument().Expand());
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
    : UnaryExpressionCell{ExpressionKind::Tanh, e, false} {}

Polynomial<double> ExpressionTanh::ToPolynomial() const {
  throw runtime_error("Tanh expression is not polynomial-convertible.");
}

Expression ExpressionTanh::Expand() const {
  return tanh(get_argument().Expand());
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
    : BinaryExpressionCell{ExpressionKind::Min, e1, e2, false} {}

Polynomial<double> ExpressionMin::ToPolynomial() const {
  throw runtime_error("Min expression is not polynomial-convertible.");
}

Expression ExpressionMin::Expand() const {
  return min(get_first_argument().Expand(), get_second_argument().Expand());
}

Expression ExpressionMin::Substitute(const Substitution& s) const {
  return min(get_first_argument().Substitute(s),
             get_second_argument().Substitute(s));
}

Expression ExpressionMin::Differentiate(const Variable& x) const {
  if (GetVariables().include(x)) {
    ostringstream oss;
    Display(oss) << "is not differentiable with respect to " << x << ".";
    throw runtime_error(oss.str());
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
    : BinaryExpressionCell{ExpressionKind::Max, e1, e2, false} {}

Polynomial<double> ExpressionMax::ToPolynomial() const {
  throw runtime_error("Max expression is not polynomial-convertible.");
}

Expression ExpressionMax::Expand() const {
  return max(get_first_argument().Expand(), get_second_argument().Expand());
}

Expression ExpressionMax::Substitute(const Substitution& s) const {
  return max(get_first_argument().Substitute(s),
             get_second_argument().Substitute(s));
}

Expression ExpressionMax::Differentiate(const Variable& x) const {
  if (GetVariables().include(x)) {
    ostringstream oss;
    Display(oss) << "is not differentiable with respect to " << x << ".";
    throw runtime_error(oss.str());
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

// ExpressionIfThenElse
// --------------------
ExpressionIfThenElse::ExpressionIfThenElse(const Formula& f_cond,
                                           const Expression& e_then,
                                           const Expression& e_else)
    : ExpressionCell{ExpressionKind::IfThenElse,
                     hash_combine(hash_value<Formula>{}(f_cond), e_then,
                                  e_else),
                     false},
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

Polynomial<double> ExpressionIfThenElse::ToPolynomial() const {
  throw runtime_error("IfThenElse expression is not polynomial-convertible.");
}

double ExpressionIfThenElse::Evaluate(const Environment& env) const {
  if (f_cond_.Evaluate(env)) {
    return e_then_.Evaluate(env);
  } else {
    return e_else_.Evaluate(env);
  }
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
    ostringstream oss;
    Display(oss) << "is not differentiable with respect to " << x << ".";
    throw runtime_error(oss.str());
  } else {
    return Expression::Zero();
  }
}

ostream& ExpressionIfThenElse::Display(ostream& os) const {
  return os << "(if " << f_cond_ << " then " << e_then_ << " else " << e_else_
            << ")";
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
bool is_if_then_else(const ExpressionCell& c) {
  return c.get_kind() == ExpressionKind::IfThenElse;
}

shared_ptr<ExpressionConstant> to_constant(
    const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_constant(*expr_ptr));
  return static_pointer_cast<ExpressionConstant>(expr_ptr);
}
shared_ptr<ExpressionConstant> to_constant(const Expression& e) {
  return to_constant(e.ptr_);
}

shared_ptr<ExpressionVar> to_variable(
    const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_variable(*expr_ptr));
  return static_pointer_cast<ExpressionVar>(expr_ptr);
}
shared_ptr<ExpressionVar> to_variable(const Expression& e) {
  return to_variable(e.ptr_);
}

shared_ptr<UnaryExpressionCell> to_unary(
    const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_log(*expr_ptr) || is_abs(*expr_ptr) || is_exp(*expr_ptr) ||
               is_sqrt(*expr_ptr) || is_sin(*expr_ptr) || is_cos(*expr_ptr) ||
               is_tan(*expr_ptr) || is_asin(*expr_ptr) || is_acos(*expr_ptr) ||
               is_atan(*expr_ptr) || is_sinh(*expr_ptr) || is_cosh(*expr_ptr) ||
               is_tanh(*expr_ptr));
  return static_pointer_cast<UnaryExpressionCell>(expr_ptr);
}
shared_ptr<UnaryExpressionCell> to_unary(const Expression& e) {
  return to_unary(e.ptr_);
}

shared_ptr<BinaryExpressionCell> to_binary(
    const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_division(*expr_ptr) || is_pow(*expr_ptr) ||
               is_atan2(*expr_ptr) || is_min(*expr_ptr) || is_max(*expr_ptr));
  return static_pointer_cast<BinaryExpressionCell>(expr_ptr);
}
shared_ptr<BinaryExpressionCell> to_binary(const Expression& e) {
  return to_binary(e.ptr_);
}

shared_ptr<ExpressionAdd> to_addition(
    const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_addition(*expr_ptr));
  return static_pointer_cast<ExpressionAdd>(expr_ptr);
}
shared_ptr<ExpressionAdd> to_addition(const Expression& e) {
  return to_addition(e.ptr_);
}

shared_ptr<ExpressionMul> to_multiplication(
    const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_multiplication(*expr_ptr));
  return static_pointer_cast<ExpressionMul>(expr_ptr);
}
shared_ptr<ExpressionMul> to_multiplication(const Expression& e) {
  return to_multiplication(e.ptr_);
}

shared_ptr<ExpressionDiv> to_division(
    const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_division(*expr_ptr));
  return static_pointer_cast<ExpressionDiv>(expr_ptr);
}
shared_ptr<ExpressionDiv> to_division(const Expression& e) {
  return to_division(e.ptr_);
}

shared_ptr<ExpressionLog> to_log(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_log(*expr_ptr));
  return static_pointer_cast<ExpressionLog>(expr_ptr);
}
shared_ptr<ExpressionLog> to_log(const Expression& e) { return to_log(e.ptr_); }

shared_ptr<ExpressionAbs> to_abs(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_abs(*expr_ptr));
  return static_pointer_cast<ExpressionAbs>(expr_ptr);
}
shared_ptr<ExpressionAbs> to_abs(const Expression& e) { return to_abs(e.ptr_); }

shared_ptr<ExpressionExp> to_exp(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_exp(*expr_ptr));
  return static_pointer_cast<ExpressionExp>(expr_ptr);
}
shared_ptr<ExpressionExp> to_exp(const Expression& e) { return to_exp(e.ptr_); }

shared_ptr<ExpressionSqrt> to_sqrt(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_sqrt(*expr_ptr));
  return static_pointer_cast<ExpressionSqrt>(expr_ptr);
}
shared_ptr<ExpressionSqrt> to_sqrt(const Expression& e) {
  return to_sqrt(e.ptr_);
}
shared_ptr<ExpressionPow> to_pow(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_pow(*expr_ptr));
  return static_pointer_cast<ExpressionPow>(expr_ptr);
}
shared_ptr<ExpressionPow> to_pow(const Expression& e) { return to_pow(e.ptr_); }

shared_ptr<ExpressionSin> to_sin(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_sin(*expr_ptr));
  return static_pointer_cast<ExpressionSin>(expr_ptr);
}
shared_ptr<ExpressionSin> to_sin(const Expression& e) { return to_sin(e.ptr_); }

shared_ptr<ExpressionCos> to_cos(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_cos(*expr_ptr));
  return static_pointer_cast<ExpressionCos>(expr_ptr);
}
shared_ptr<ExpressionCos> to_cos(const Expression& e) { return to_cos(e.ptr_); }

shared_ptr<ExpressionTan> to_tan(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_tan(*expr_ptr));
  return static_pointer_cast<ExpressionTan>(expr_ptr);
}
shared_ptr<ExpressionTan> to_tan(const Expression& e) { return to_tan(e.ptr_); }

shared_ptr<ExpressionAsin> to_asin(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_asin(*expr_ptr));
  return static_pointer_cast<ExpressionAsin>(expr_ptr);
}
shared_ptr<ExpressionAsin> to_asin(const Expression& e) {
  return to_asin(e.ptr_);
}

shared_ptr<ExpressionAcos> to_acos(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_acos(*expr_ptr));
  return static_pointer_cast<ExpressionAcos>(expr_ptr);
}
shared_ptr<ExpressionAcos> to_acos(const Expression& e) {
  return to_acos(e.ptr_);
}

shared_ptr<ExpressionAtan> to_atan(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_atan(*expr_ptr));
  return static_pointer_cast<ExpressionAtan>(expr_ptr);
}
shared_ptr<ExpressionAtan> to_atan(const Expression& e) {
  return to_atan(e.ptr_);
}

shared_ptr<ExpressionAtan2> to_atan2(
    const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_atan2(*expr_ptr));
  return static_pointer_cast<ExpressionAtan2>(expr_ptr);
}
shared_ptr<ExpressionAtan2> to_atan2(const Expression& e) {
  return to_atan2(e.ptr_);
}

shared_ptr<ExpressionSinh> to_sinh(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_sinh(*expr_ptr));
  return static_pointer_cast<ExpressionSinh>(expr_ptr);
}
shared_ptr<ExpressionSinh> to_sinh(const Expression& e) {
  return to_sinh(e.ptr_);
}

shared_ptr<ExpressionCosh> to_cosh(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_cosh(*expr_ptr));
  return static_pointer_cast<ExpressionCosh>(expr_ptr);
}
shared_ptr<ExpressionCosh> to_cosh(const Expression& e) {
  return to_cosh(e.ptr_);
}

shared_ptr<ExpressionTanh> to_tanh(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_tanh(*expr_ptr));
  return static_pointer_cast<ExpressionTanh>(expr_ptr);
}
shared_ptr<ExpressionTanh> to_tanh(const Expression& e) {
  return to_tanh(e.ptr_);
}

shared_ptr<ExpressionMin> to_min(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_min(*expr_ptr));
  return static_pointer_cast<ExpressionMin>(expr_ptr);
}
shared_ptr<ExpressionMin> to_min(const Expression& e) { return to_min(e.ptr_); }

shared_ptr<ExpressionMax> to_max(const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_max(*expr_ptr));
  return static_pointer_cast<ExpressionMax>(expr_ptr);
}
shared_ptr<ExpressionMax> to_max(const Expression& e) { return to_max(e.ptr_); }

shared_ptr<ExpressionIfThenElse> to_if_then_else(
    const shared_ptr<ExpressionCell> expr_ptr) {
  DRAKE_ASSERT(is_if_then_else(*expr_ptr));
  return static_pointer_cast<ExpressionIfThenElse>(expr_ptr);
}
shared_ptr<ExpressionIfThenElse> to_if_then_else(const Expression& e) {
  return to_if_then_else(e.ptr_);
}

}  // namespace symbolic
}  // namespace drake
