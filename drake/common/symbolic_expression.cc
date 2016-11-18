#include "drake/common/symbolic_expression.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression_cell.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::make_shared;
using std::ostream;
using std::ostringstream;
using std::runtime_error;
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

void Expression::check_nan(const double v) {
  if (std::isnan(v)) {
    throw runtime_error("NaN is detected during Symbolic computation.");
  }
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

Expression cond(const Expression& e) { return e; }

}  // namespace symbolic
}  // namespace drake
