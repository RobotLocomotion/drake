#include "drake/common/symbolic_expression.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression_cell.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::make_shared;
using std::map;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::shared_ptr;
using std::string;
using std::vector;

bool operator<(ExpressionKind k1, ExpressionKind k2) {
  return static_cast<int>(k1) < static_cast<int>(k2);
}

namespace {
// This function is used in Expression(const double d) constructor. It turns out
// a ternary expression "std::isnan(d) ? make_shared<ExpressionNaN>() :
// make_shared<ExpressionConstant>()" does not work due to C++'s type-system.
// It throws "Incompatible operand types when using ternary conditional
// operator" error. Related S&O entry:
// http://stackoverflow.com/questions/29842095/incompatible-operand-types-when-using-ternary-conditional-operator.
shared_ptr<ExpressionCell> make_cell(const double d) {
  if (std::isnan(d)) {
    return make_shared<ExpressionNaN>();
  }
  return make_shared<ExpressionConstant>(d);
}

// Negates an addition expression.
// - (E_1 + ... + E_n) => (-E_1 + ... + -E_n)
Expression NegateAddition(const Expression& e) {
  DRAKE_ASSERT(is_addition(e));
  return ExpressionAddFactory{to_addition(e)}.Negate().GetExpression();
}

// Negates a multiplication expression.
// - (c0 * E_1 * ... * E_n) => (-c0 * E_1 * ... * E_n)
Expression NegateMultiplication(const Expression& e) {
  DRAKE_ASSERT(is_multiplication(e));
  return ExpressionMulFactory{to_multiplication(e)}.Negate().GetExpression();
}
}  // namespace

Expression::Expression(const Variable& var)
    : ptr_{make_shared<ExpressionVar>(var)} {}
Expression::Expression(const double d) : ptr_{make_cell(d)} {}
Expression::Expression(shared_ptr<ExpressionCell> ptr) : ptr_{std::move(ptr)} {}

ExpressionKind Expression::get_kind() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->get_kind();
}
size_t Expression::get_hash() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->get_hash();
}

Expression Expression::Zero() {
  static const never_destroyed<Expression> zero{0.0};
  return zero.access();
}

Expression Expression::One() {
  static const never_destroyed<Expression> one{1.0};
  return one.access();
}

Expression Expression::Pi() {
  static const never_destroyed<Expression> pi{M_PI};
  return pi.access();
}

Expression Expression::E() {
  static const never_destroyed<Expression> e{M_E};
  return e.access();
}

Expression Expression::NaN() {
  static const never_destroyed<Expression> nan{
      Expression{make_shared<ExpressionNaN>()}};
  return nan.access();
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

bool Expression::is_polynomial() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->is_polynomial();
}

Polynomial<double> Expression::ToPolynomial() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->ToPolynomial();
}

double Expression::Evaluate(const Environment& env) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->Evaluate(env);
}

Expression Expression::EvaluatePartial(const Environment& env) const {
  if (env.empty()) {
    return *this;
  }
  Substitution subst;
  for (const pair<Variable, double>& p : env) {
    subst.emplace(p.first, p.second);
  }
  return Substitute(subst);
}

Expression Expression::Expand() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->Expand();
}

Expression Expression::Substitute(const Variable& var,
                                  const Expression& e) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->Substitute({{var, e}});
}

Expression Expression::Substitute(const Substitution& s) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  if (!s.empty()) {
    return ptr_->Substitute(s);
  }
  return *this;
}

Expression Expression::Differentiate(const Variable& x) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->Differentiate(x);
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
  if (is_zero(lhs)) {
    lhs = rhs;
    return lhs;
  }
  // Simplification: x + 0 => x
  if (is_zero(rhs)) {
    return lhs;
  }
  // Simplification: Expression(c1) + Expression(c2) => Expression(c1 + c2)
  if (is_constant(lhs) && is_constant(rhs)) {
    lhs = get_constant_value(lhs) + get_constant_value(rhs);
    return lhs;
  }
  // Simplification: flattening. To build a new expression, we use
  // ExpressionAddFactory which holds intermediate terms and does
  // simplifications internally.
  ExpressionAddFactory add_factory{};
  if (is_addition(lhs)) {
    // 1. (e_1 + ... + e_n) + rhs
    add_factory = to_addition(lhs);
    // Note: AddExpression method takes care of the special case where `rhs` is
    // of ExpressionAdd.
    add_factory.AddExpression(rhs);
  } else {
    if (is_addition(rhs)) {
      // 2. lhs + (e_1 + ... + e_n)
      add_factory = to_addition(rhs);
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
  if (is_zero(rhs)) {
    return lhs;
  }
  // Simplification: Expression(c1) - Expression(c2) => Expression(c1 - c2)
  if (is_constant(lhs) && is_constant(rhs)) {
    lhs = get_constant_value(lhs) - get_constant_value(rhs);
    return lhs;
  }
  // x - y => x + (-y)
  lhs += -rhs;
  return lhs;
}

Expression operator-(const Expression& e) {
  // Simplification: constant folding
  if (is_constant(e)) {
    return Expression{-get_constant_value(e)};
  }
  // Simplification: push '-' inside over '+'.
  // -(E_1 + ... + E_n) => (-E_1 + ... + -E_n)
  if (is_addition(e)) {
    return NegateAddition(e);
  }
  // Simplification: push '-' inside over '*'.
  // -(c0 * E_1 * ... * E_n) => (-c0 * E_1 * ... * E_n)
  if (is_multiplication(e)) {
    return NegateMultiplication(e);
  }
  return -1 * e;
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
  if (is_one(lhs)) {
    lhs = rhs;
    return lhs;
  }
  // Simplification: x * 1 => x
  if (is_one(rhs)) {
    return lhs;
  }
  // Simplification: (E1 / E2) * (E3 / E4) => (E1 * E3) / (E2 * E4)
  if (is_division(lhs) && is_division(rhs)) {
    lhs = (get_first_argument(lhs) * get_first_argument(rhs)) /
          (get_second_argument(lhs) * get_second_argument(rhs));
    return lhs;
  }
  // Simplification: lhs * (c / E) => (c * lhs) / E
  if (is_division(rhs) && is_constant(get_first_argument(rhs))) {
    lhs = (get_first_argument(rhs) * lhs) / get_second_argument(rhs);
    return lhs;
  }
  // Simplification: (c / E) * rhs => (c * rhs) / E
  if (is_division(lhs) && is_constant(get_first_argument(lhs))) {
    lhs = (get_first_argument(lhs) * rhs) / get_second_argument(lhs);
    return lhs;
  }
  if (is_neg_one(lhs)) {
    if (is_addition(rhs)) {
      // Simplification: push '-' inside over '+'.
      // -1 * (E_1 + ... + E_n) => (-E_1 + ... + -E_n)
      lhs = NegateAddition(rhs);
      return lhs;
    }
    if (is_multiplication(rhs)) {
      // Simplification: push '-' inside over '*'.
      // -1 * (c0 * E_1 * ... * E_n) => (-c0 * E_1 * ... * E_n)
      lhs = NegateMultiplication(rhs);
      return lhs;
    }
  }

  if (is_neg_one(rhs)) {
    if (is_addition(lhs)) {
      // Simplification: push '-' inside over '+'.
      // (E_1 + ... + E_n) * -1 => (-E_1 + ... + -E_n)
      lhs = NegateAddition(lhs);
      return lhs;
    }
    if (is_multiplication(lhs)) {
      // Simplification: push '-' inside over '*'.
      // (c0 * E_1 * ... * E_n) * -1 => (-c0 * E_1 * ... * E_n)
      lhs = NegateMultiplication(lhs);
      return lhs;
    }
  }

  // Simplification: 0 * E => 0
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might cause 0/0 during evaluation.
  if (is_zero(lhs)) {
    return lhs;
  }
  // Simplification: E * 0 => 0
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might cause 0/0 during evaluation.
  if (is_zero(rhs)) {
    lhs = Expression::Zero();
    return lhs;
  }
  // Pow-related simplifications.
  if (is_pow(lhs)) {
    const Expression& e1{get_first_argument(lhs)};
    if (is_pow(rhs)) {
      const Expression& e3{get_first_argument(rhs)};
      if (e1.EqualTo(e3)) {
        // Simplification: pow(e1, e2) * pow(e1, e4) => pow(e1, e2 + e4)
        // TODO(soonho-tri): This simplification is not sound. For example, x^4
        // * x^(-3) => x. The original expression `x^4 * x^(-3)` is evaluated to
        // `nan` when x = 0 while the simplified expression `x` is evaluated to
        // 0.
        const Expression& e2{get_second_argument(lhs)};
        const Expression& e4{get_second_argument(rhs)};
        lhs = pow(e1, e2 + e4);
        return lhs;
      }
    }
    if (e1.EqualTo(rhs)) {
      // Simplification: pow(e1, e2) * e1 => pow(e1, e2 + 1)
      // TODO(soonho-tri): This simplification is not sound.
      const Expression& e2{get_second_argument(lhs)};
      lhs = pow(e1, e2 + 1);
      return lhs;
    }
  } else {
    if (is_pow(rhs)) {
      const Expression& e1{get_first_argument(rhs)};
      if (e1.EqualTo(lhs)) {
        // Simplification: (lhs * rhs == e1 * pow(e1, e2)) => pow(e1, 1 + e2)
        // TODO(soonho-tri): This simplification is not sound.
        const Expression& e2{get_second_argument(rhs)};
        lhs = pow(e1, 1 + e2);
        return lhs;
      }
    }
  }
  if (is_constant(lhs) && is_constant(rhs)) {
    // Simplification: Expression(c1) * Expression(c2) => Expression(c1 * c2)
    lhs = Expression{get_constant_value(lhs) * get_constant_value(rhs)};
    return lhs;
  }
  // Simplification: flattening
  ExpressionMulFactory mul_factory{};
  if (is_multiplication(lhs)) {
    // (e_1 * ... * e_n) * rhs
    mul_factory = to_multiplication(lhs);
    // Note: AddExpression method takes care of the special case where `rhs` is
    // of ExpressionMul.
    mul_factory.AddExpression(rhs);
  } else {
    if (is_multiplication(rhs)) {
      // e_1 * (e_2 * ... * e_n) -> (e_2 * ... * e_n * e_1)
      //
      // Note that we do not preserve the original ordering because * is
      // associative.
      mul_factory = to_multiplication(rhs);
      mul_factory.AddExpression(lhs);
    } else {
      // Simplification: x * x => x^2 (=pow(x,2))
      if (lhs.EqualTo(rhs)) {
        lhs = pow(lhs, 2.0);
        return lhs;
      }
      // nothing to flatten
      mul_factory.AddExpression(lhs);
      mul_factory.AddExpression(rhs);
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
  if (is_one(rhs)) {
    return lhs;
  }
  // Simplification: Expression(c1) / Expression(c2) => Expression(c1 / c2)
  if (is_constant(lhs) && is_constant(rhs)) {
    const double v1{get_constant_value(lhs)};
    const double v2{get_constant_value(rhs)};
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

ostream& operator<<(ostream& os, const Expression& e) {
  DRAKE_ASSERT(e.ptr_ != nullptr);
  return e.ptr_->Display(os);
}

Expression log(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    const double v{get_constant_value(e)};
    ExpressionLog::check_domain(v);
    return Expression{std::log(v)};
  }
  return Expression{make_shared<ExpressionLog>(e)};
}

Expression abs(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::fabs(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionAbs>(e)};
}

Expression exp(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::exp(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionExp>(e)};
}

Expression sqrt(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    const double v{get_constant_value(e)};
    ExpressionSqrt::check_domain(v);
    return Expression{std::sqrt(v)};
  }
  // Simplification: sqrt(pow(x, 2)) => abs(x)
  if (is_pow(e)) {
    if (is_two(get_second_argument(e))) {
      return abs(get_first_argument(e));
    }
  }
  return Expression{make_shared<ExpressionSqrt>(e)};
}

Expression pow(const Expression& e1, const Expression& e2) {
  // Simplification
  if (is_constant(e2)) {
    const double v2{get_constant_value(e2)};
    if (is_constant(e1)) {
      // Constant folding
      const double v1{get_constant_value(e1)};
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
  if (is_pow(e1)) {
    // pow(base, exponent) ^ e2 => pow(base, exponent * e2)
    const Expression& base{get_first_argument(e1)};
    const Expression& exponent{get_second_argument(e1)};
    return Expression{make_shared<ExpressionPow>(base, exponent * e2)};
  }
  return Expression{make_shared<ExpressionPow>(e1, e2)};
}

Expression sin(const Expression& e) {
  // simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::sin(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionSin>(e)};
}

Expression cos(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::cos(get_constant_value(e))};
  }

  return Expression{make_shared<ExpressionCos>(e)};
}

Expression tan(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::tan(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionTan>(e)};
}

Expression asin(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    const double v{get_constant_value(e)};
    ExpressionAsin::check_domain(v);
    return Expression{std::asin(v)};
  }
  return Expression{make_shared<ExpressionAsin>(e)};
}

Expression acos(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    const double v{get_constant_value(e)};
    ExpressionAcos::check_domain(v);
    return Expression{std::acos(v)};
  }
  return Expression{make_shared<ExpressionAcos>(e)};
}

Expression atan(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::atan(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionAtan>(e)};
}

Expression atan2(const Expression& e1, const Expression& e2) {
  // Simplification: constant folding.
  if (is_constant(e1) && is_constant(e2)) {
    return Expression{
        std::atan2(get_constant_value(e1), get_constant_value(e2))};
  }
  return Expression{make_shared<ExpressionAtan2>(e1, e2)};
}

Expression sinh(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::sinh(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionSinh>(e)};
}

Expression cosh(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::cosh(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionCosh>(e)};
}

Expression tanh(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::tanh(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionTanh>(e)};
}

Expression min(const Expression& e1, const Expression& e2) {
  // simplification: min(x, x) => x
  if (e1.EqualTo(e2)) {
    return e1;
  }
  // Simplification: constant folding.
  if (is_constant(e1) && is_constant(e2)) {
    return Expression{std::min(get_constant_value(e1), get_constant_value(e2))};
  }
  return Expression{make_shared<ExpressionMin>(e1, e2)};
}

Expression max(const Expression& e1, const Expression& e2) {
  // Simplification: max(x, x) => x
  if (e1.EqualTo(e2)) {
    return e1;
  }
  // Simplification: constant folding
  if (is_constant(e1) && is_constant(e2)) {
    return Expression{std::max(get_constant_value(e1), get_constant_value(e2))};
  }
  return Expression{make_shared<ExpressionMax>(e1, e2)};
}

Expression ceil(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::ceil(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionCeiling>(e)};
}

Expression floor(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::floor(get_constant_value(e))};
  }
  return Expression{make_shared<ExpressionFloor>(e)};
}

Expression if_then_else(const Formula& f_cond, const Expression& e_then,
                        const Expression& e_else) {
  // simplification:: if(true, e1, e2) => e1
  if (f_cond.EqualTo(Formula::True())) {
    return e_then;
  }
  // simplification:: if(false, e1, e2) => e2
  if (f_cond.EqualTo(Formula::False())) {
    return e_else;
  }
  return Expression{make_shared<ExpressionIfThenElse>(f_cond, e_then, e_else)};
}

Expression uninterpreted_function(const string& name, const Variables& vars) {
  return Expression{make_shared<ExpressionUninterpretedFunction>(name, vars)};
}

bool is_constant(const Expression& e) { return is_constant(*e.ptr_); }
bool is_constant(const Expression& e, const double v) {
  return is_constant(e) && (to_constant(e)->get_value() == v);
}
bool is_zero(const Expression& e) { return is_constant(e, 0.0); }
bool is_one(const Expression& e) { return is_constant(e, 1.0); }
bool is_neg_one(const Expression& e) { return is_constant(e, -1.0); }
bool is_two(const Expression& e) { return is_constant(e, 2.0); }
bool is_nan(const Expression& e) { return e.get_kind() == ExpressionKind::NaN; }
bool is_variable(const Expression& e) { return is_variable(*e.ptr_); }
bool is_addition(const Expression& e) { return is_addition(*e.ptr_); }
bool is_multiplication(const Expression& e) {
  return is_multiplication(*e.ptr_);
}
bool is_division(const Expression& e) { return is_division(*e.ptr_); }
bool is_log(const Expression& e) { return is_log(*e.ptr_); }
bool is_abs(const Expression& e) { return is_abs(*e.ptr_); }
bool is_exp(const Expression& e) { return is_exp(*e.ptr_); }
bool is_sqrt(const Expression& e) { return is_sqrt(*e.ptr_); }
bool is_pow(const Expression& e) { return is_pow(*e.ptr_); }
bool is_sin(const Expression& e) { return is_sin(*e.ptr_); }
bool is_cos(const Expression& e) { return is_cos(*e.ptr_); }
bool is_tan(const Expression& e) { return is_tan(*e.ptr_); }
bool is_asin(const Expression& e) { return is_asin(*e.ptr_); }
bool is_acos(const Expression& e) { return is_acos(*e.ptr_); }
bool is_atan(const Expression& e) { return is_atan(*e.ptr_); }
bool is_atan2(const Expression& e) { return is_atan2(*e.ptr_); }
bool is_sinh(const Expression& e) { return is_sinh(*e.ptr_); }
bool is_cosh(const Expression& e) { return is_cosh(*e.ptr_); }
bool is_tanh(const Expression& e) { return is_tanh(*e.ptr_); }
bool is_min(const Expression& e) { return is_min(*e.ptr_); }
bool is_max(const Expression& e) { return is_max(*e.ptr_); }
bool is_ceil(const Expression& e) { return is_ceil(*e.ptr_); }
bool is_floor(const Expression& e) { return is_floor(*e.ptr_); }
bool is_if_then_else(const Expression& e) { return is_if_then_else(*e.ptr_); }
bool is_uninterpreted_function(const Expression& e) {
  return is_uninterpreted_function(*e.ptr_);
}

double get_constant_value(const Expression& e) {
  return to_constant(e)->get_value();
}
const Variable& get_variable(const Expression& e) {
  return to_variable(e)->get_variable();
}
const Expression& get_argument(const Expression& e) {
  return to_unary(e)->get_argument();
}
const Expression& get_first_argument(const Expression& e) {
  return to_binary(e)->get_first_argument();
}
const Expression& get_second_argument(const Expression& e) {
  return to_binary(e)->get_second_argument();
}
double get_constant_in_addition(const Expression& e) {
  return to_addition(e)->get_constant();
}
const map<Expression, double>& get_expr_to_coeff_map_in_addition(
    const Expression& e) {
  return to_addition(e)->get_expr_to_coeff_map();
}
double get_constant_in_multiplication(const Expression& e) {
  return to_multiplication(e)->get_constant();
}
const map<Expression, Expression>& get_base_to_exponent_map_in_multiplication(
    const Expression& e) {
  return to_multiplication(e)->get_base_to_exponent_map();
}

const string& get_uninterpreted_function_name(const Expression& e) {
  return to_uninterpreted_function(e)->get_name();
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator+=(Expression& lhs, const Variable& rhs) {
  return lhs += Expression{rhs};
}
Expression operator+(const Variable& lhs, const Variable& rhs) {
  return Expression{lhs} + Expression{rhs};
}
Expression operator+(Expression lhs, const Variable& rhs) { return lhs += rhs; }
Expression operator+(const Variable& lhs, Expression rhs) { return rhs += lhs; }

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator-=(Expression& lhs, const Variable& rhs) {
  return lhs -= Expression{rhs};
}
Expression operator-(const Variable& lhs, const Variable& rhs) {
  return Expression{lhs} - Expression{rhs};
}
Expression operator-(Expression lhs, const Variable& rhs) { return lhs -= rhs; }
Expression operator-(const Variable& lhs, const Expression& rhs) {
  return Expression(lhs) - rhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator*=(Expression& lhs, const Variable& rhs) {
  return lhs *= Expression{rhs};
}
Expression operator*(const Variable& lhs, const Variable& rhs) {
  return Expression{lhs} * Expression{rhs};
}
Expression operator*(Expression lhs, const Variable& rhs) { return lhs *= rhs; }
Expression operator*(const Variable& lhs, Expression rhs) { return rhs *= lhs; }

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator/=(Expression& lhs, const Variable& rhs) {
  return lhs /= Expression{rhs};
}
Expression operator/(const Variable& lhs, const Variable& rhs) {
  return Expression{lhs} / Expression{rhs};
}
Expression operator/(Expression lhs, const Variable& rhs) { return lhs /= rhs; }
Expression operator/(const Variable& lhs, const Expression& rhs) {
  return Expression(lhs) / rhs;
}

Expression operator+(const Variable& var) { return Expression{var}; }
Expression operator-(const Variable& var) { return -Expression{var}; }

MatrixX<Expression> Jacobian(const Eigen::Ref<const VectorX<Expression>>& f,
                             const vector<Variable>& vars) {
  DRAKE_DEMAND(!vars.empty());
  const Eigen::Ref<const VectorX<Expression>>::Index n{f.size()};
  const size_t m{vars.size()};
  MatrixX<Expression> J(n, m);
  for (int i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      J(i, j) = f[i].Differentiate(vars[j]);
    }
  }
  return J;
}

MatrixX<Expression> Jacobian(const Eigen::Ref<const VectorX<Expression>>& f,
                             const Eigen::Ref<const VectorX<Variable>>& vars) {
  return Jacobian(f, vector<Variable>(vars.data(), vars.data() + vars.size()));
}
}  // namespace symbolic
}  // namespace drake
