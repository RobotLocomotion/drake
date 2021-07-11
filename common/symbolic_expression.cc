// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ios>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Core>
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"
#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

namespace drake {
namespace symbolic {

using std::logic_error;
using std::make_shared;
using std::map;
using std::numeric_limits;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::shared_ptr;
using std::streamsize;
using std::string;
using std::vector;

bool operator<(ExpressionKind k1, ExpressionKind k2) {
  return static_cast<int>(k1) < static_cast<int>(k2);
}

namespace {
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

shared_ptr<ExpressionCell> Expression::make_cell(const double d) {
  if (d == 0.0) {
    // The objects created by `Expression(0.0)` share the unique
    // `ExpressionConstant` object created in `Expression::Zero()`.
    //
    // See https://github.com/RobotLocomotion/drake/issues/12453 for details.
    //
    // Typically we should never access a ptr_ directly, but here we know it
    // poses no risk because ExpressionConstant has no non-const member fields
    // of any consequence (only "is_expanded_" is non-const, and it can never
    // be reset to false).
    return Expression::Zero().ptr_;
  }
  if (std::isnan(d)) {
    return make_shared<ExpressionNaN>();
  }
  return make_shared<ExpressionConstant>(d);
}

Expression::Expression(const Variable& var)
    : Expression{make_shared<ExpressionVar>(var)} {}

Expression::Expression(const double d) : Expression{make_cell(d)} {}

Expression::Expression(std::shared_ptr<ExpressionCell> ptr)
    : ptr_{std::move(ptr)} {
  DRAKE_ASSERT(ptr_ != nullptr);
}

ExpressionKind Expression::get_kind() const {
  return cell().get_kind();
}

void Expression::HashAppend(DelegatingHasher* hasher) const {
  using drake::hash_append;
  hash_append(*hasher, get_kind());
  cell().HashAppendDetail(hasher);
}

ExpressionCell& Expression::mutable_cell() {
  DRAKE_ASSERT(ptr_ != nullptr);
  DRAKE_DEMAND(ptr_.use_count() == 1);
  return *ptr_;
}

Expression Expression::Zero() {
  static const never_destroyed<Expression> zero{
      Expression{make_shared<ExpressionConstant>(0.0)}};
  return zero.access();
}

Expression Expression::One() {
  static const never_destroyed<Expression> one{
      Expression{make_shared<ExpressionConstant>(1.0)}};
  return one.access();
}

Expression Expression::Pi() {
  static const never_destroyed<Expression> pi{
      Expression{make_shared<ExpressionConstant>(M_PI)}};
  return pi.access();
}

Expression Expression::E() {
  static const never_destroyed<Expression> e{
      Expression{make_shared<ExpressionConstant>(M_E)}};
  return e.access();
}

Expression Expression::NaN() {
  static const never_destroyed<Expression> nan{
      Expression{make_shared<ExpressionNaN>()}};
  return nan.access();
}

Variables Expression::GetVariables() const {
  return cell().GetVariables();
}

bool Expression::EqualTo(const Expression& e) const {
  const auto& this_cell = cell();
  const auto& other_cell = e.cell();
  if (&this_cell == &other_cell) {
    return true;
  }
  if (get_kind() != e.get_kind()) {
    return false;
  }
  // Check structural equality.
  return cell().EqualTo(e.cell());
}

bool Expression::Less(const Expression& e) const {
  const auto& this_cell = cell();
  const auto& other_cell = e.cell();
  if (&this_cell == &other_cell) {
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
  return this_cell.Less(other_cell);
}

bool Expression::is_polynomial() const {
  return cell().is_polynomial();
}

bool Expression::is_expanded() const {
  return cell().is_expanded();
}

double Expression::Evaluate(const Environment& env,
                            RandomGenerator* const random_generator) const {
  if (random_generator == nullptr) {
    return cell().Evaluate(env);
  } else {
    return cell().Evaluate(
        PopulateRandomVariables(env, GetVariables(), random_generator));
  }
}

double Expression::Evaluate(RandomGenerator* const random_generator) const {
  return Evaluate(Environment{}, random_generator);
}

Eigen::SparseMatrix<double> Evaluate(
    const Eigen::Ref<const Eigen::SparseMatrix<Expression>>& m,
    const Environment& env) {
  return m.unaryExpr([&env](const Expression& e) { return e.Evaluate(env); });
}

Expression Expression::EvaluatePartial(const Environment& env) const {
  if (env.empty()) {
    return *this;
  }
  Substitution subst;
  for (const pair<const Variable, double>& p : env) {
    subst.emplace(p.first, p.second);
  }
  return Substitute(subst);
}

Expression Expression::Expand() const {
  if (cell().is_expanded()) {
    // If it is already expanded, return the current expression without calling
    // Expand() on the cell.
    return *this;
  }

  Expression result = cell().Expand();
  if (!result.is_expanded()) {
    result.mutable_cell().set_expanded();
  }
  return result;
}

Expression Expression::Substitute(const Variable& var,
                                  const Expression& e) const {
  return cell().Substitute({{var, e}});
}

Expression Expression::Substitute(const Substitution& s) const {
  if (!s.empty()) {
    return cell().Substitute(s);
  }
  return *this;
}

Expression Expression::Differentiate(const Variable& x) const {
  return cell().Differentiate(x);
}

RowVectorX<Expression> Expression::Jacobian(
    const Eigen::Ref<const VectorX<Variable>>& vars) const {
  RowVectorX<Expression> J(vars.size());
  for (VectorX<Variable>::Index i = 0; i < vars.size(); ++i) {
    J(i) = Differentiate(vars(i));
  }
  return J;
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

Expression operator+(const Expression& e) { return e; }

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
  // Declare as non-const to allow move.
  Expression copy(*this);
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
  lhs = Expression{make_shared<ExpressionDiv>(lhs, rhs)};
  return lhs;
}

namespace {
// Changes the precision of `os` to be the `new_precision` and saves the
// original precision so that it can be reverted when an instance of this class
// is destructed. It is used in `operator<<` of symbolic expression.
class PrecisionGuard {
 public:
  PrecisionGuard(ostream* const os, const streamsize& new_precision)
      : os_{os}, original_precision_{os->precision()} {
    os_->precision(new_precision);
  }
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrecisionGuard)
  ~PrecisionGuard() { os_->precision(original_precision_); }

 private:
  ostream* const os_;
  const streamsize original_precision_;
};
}  // namespace

ostream& operator<<(ostream& os, const Expression& e) {
  const PrecisionGuard precision_guard{&os,
                                       numeric_limits<double>::max_digits10};
  return e.cell().Display(os);
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

Expression uninterpreted_function(string name, vector<Expression> arguments) {
  return Expression{make_shared<ExpressionUninterpretedFunction>(
      std::move(name), std::move(arguments))};
}

bool is_constant(const Expression& e) { return is_constant(e.cell()); }
bool is_constant(const Expression& e, const double v) {
  return is_constant(e) && (to_constant(e).get_value() == v);
}
bool is_zero(const Expression& e) { return is_constant(e, 0.0); }
bool is_one(const Expression& e) { return is_constant(e, 1.0); }
bool is_neg_one(const Expression& e) { return is_constant(e, -1.0); }
bool is_two(const Expression& e) { return is_constant(e, 2.0); }
bool is_nan(const Expression& e) { return e.get_kind() == ExpressionKind::NaN; }
bool is_variable(const Expression& e) { return is_variable(e.cell()); }
bool is_addition(const Expression& e) { return is_addition(e.cell()); }
bool is_multiplication(const Expression& e) {
  return is_multiplication(e.cell());
}
bool is_division(const Expression& e) { return is_division(e.cell()); }
bool is_log(const Expression& e) { return is_log(e.cell()); }
bool is_abs(const Expression& e) { return is_abs(e.cell()); }
bool is_exp(const Expression& e) { return is_exp(e.cell()); }
bool is_sqrt(const Expression& e) { return is_sqrt(e.cell()); }
bool is_pow(const Expression& e) { return is_pow(e.cell()); }
bool is_sin(const Expression& e) { return is_sin(e.cell()); }
bool is_cos(const Expression& e) { return is_cos(e.cell()); }
bool is_tan(const Expression& e) { return is_tan(e.cell()); }
bool is_asin(const Expression& e) { return is_asin(e.cell()); }
bool is_acos(const Expression& e) { return is_acos(e.cell()); }
bool is_atan(const Expression& e) { return is_atan(e.cell()); }
bool is_atan2(const Expression& e) { return is_atan2(e.cell()); }
bool is_sinh(const Expression& e) { return is_sinh(e.cell()); }
bool is_cosh(const Expression& e) { return is_cosh(e.cell()); }
bool is_tanh(const Expression& e) { return is_tanh(e.cell()); }
bool is_min(const Expression& e) { return is_min(e.cell()); }
bool is_max(const Expression& e) { return is_max(e.cell()); }
bool is_ceil(const Expression& e) { return is_ceil(e.cell()); }
bool is_floor(const Expression& e) { return is_floor(e.cell()); }
bool is_if_then_else(const Expression& e) { return is_if_then_else(e.cell()); }
bool is_uninterpreted_function(const Expression& e) {
  return is_uninterpreted_function(e.cell());
}

double get_constant_value(const Expression& e) {
  return to_constant(e).get_value();
}
const Variable& get_variable(const Expression& e) {
  return to_variable(e).get_variable();
}
const Expression& get_argument(const Expression& e) {
  return to_unary(e).get_argument();
}
const Expression& get_first_argument(const Expression& e) {
  return to_binary(e).get_first_argument();
}
const Expression& get_second_argument(const Expression& e) {
  return to_binary(e).get_second_argument();
}
double get_constant_in_addition(const Expression& e) {
  return to_addition(e).get_constant();
}
const map<Expression, double>& get_expr_to_coeff_map_in_addition(
    const Expression& e) {
  return to_addition(e).get_expr_to_coeff_map();
}
double get_constant_in_multiplication(const Expression& e) {
  return to_multiplication(e).get_constant();
}
const map<Expression, Expression>& get_base_to_exponent_map_in_multiplication(
    const Expression& e) {
  return to_multiplication(e).get_base_to_exponent_map();
}

const string& get_uninterpreted_function_name(const Expression& e) {
  return to_uninterpreted_function(e).get_name();
}

const vector<Expression>& get_uninterpreted_function_arguments(
    const Expression& e) {
  return to_uninterpreted_function(e).get_arguments();
}

const Formula& get_conditional_formula(const Expression& e) {
  return to_if_then_else(e).get_conditional_formula();
}

const Expression& get_then_expression(const Expression& e) {
  return to_if_then_else(e).get_then_expression();
}

const Expression& get_else_expression(const Expression& e) {
  return to_if_then_else(e).get_else_expression();
}

Expression operator+(const Variable& var) { return Expression{var}; }
Expression operator-(const Variable& var) { return -Expression{var}; }

VectorX<Variable> GetVariableVector(
    const Eigen::Ref<const VectorX<Expression>>& evec) {
  VectorX<Variable> vec(evec.size());
  for (int i = 0; i < evec.size(); i++) {
    const Expression e_i{evec(i)};
    if (is_variable(e_i)) {
      vec(i) = get_variable(e_i);
    } else {
      throw logic_error(fmt::format("{} is not a variable.", e_i));
    }
  }
  return vec;
}

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

namespace {

// Helper class for IsAffine functions below where an instance of this class
// is passed to Eigen::MatrixBase::visit() function.
class IsAffineVisitor {
 public:
  IsAffineVisitor() = default;
  explicit IsAffineVisitor(const Variables& variables)
      : variables_{&variables} {}

  // Called for the first coefficient. Needed for Eigen::MatrixBase::visit()
  // function.
  void init(const Expression& e, const Eigen::Index i, const Eigen::Index j) {
    (*this)(e, i, j);
  }

  // Called for all other coefficients. Needed for Eigen::MatrixBase::visit()
  // function.
  void operator()(const Expression& e, const Eigen::Index, const Eigen::Index) {
    // Note that `IsNotAffine` is only called when we have not found a
    // non-affine element yet.
    found_non_affine_element_ = found_non_affine_element_ || IsNotAffine(e);
  }

  [[nodiscard]] bool result() const { return !found_non_affine_element_; }

 private:
  // Returns true if `e` is *not* affine in variables_ (if exists) or all
  // variables in `e`.
  [[nodiscard]] bool IsNotAffine(const Expression& e) const {
    if (!e.is_polynomial()) {
      return true;
    }
    const Polynomial p{(variables_ != nullptr) ? Polynomial{e, *variables_}
                                               : Polynomial{e}};
    return p.TotalDegree() > 1;
  }

  bool found_non_affine_element_{false};
  const Variables* const variables_{nullptr};
};

}  // namespace

bool IsAffine(const Eigen::Ref<const MatrixX<Expression>>& m,
              const Variables& vars) {
  if (m.size() == 0) {
    return true;
  }
  IsAffineVisitor visitor{vars};
  m.visit(visitor);
  return visitor.result();
}

bool IsAffine(const Eigen::Ref<const MatrixX<Expression>>& m) {
  if (m.size() == 0) {
    return true;
  }
  IsAffineVisitor visitor;
  m.visit(visitor);
  return visitor.result();
}

namespace {
// Helper functions for TaylorExpand.
//
// We use the multi-index notation. Please read
// https://en.wikipedia.org/wiki/Multi-index_notation for more information.

// α = (a₁, ..., aₙ) where αᵢ ∈ Z.
using MultiIndex = vector<int>;

// Generates multi-indices of order `order` whose size is `num_vars` and append
// to `vec`. It generates the indices by increasing the elements of the given
// `base`. It only changes the i-th dimension which is greater than or equal to
// `start_dim` to avoid duplicates.
void DoEnumerateMultiIndex(const int order, const int num_vars,
                           const int start_dim, const MultiIndex& base,
                           vector<MultiIndex>* const vec) {
  DRAKE_ASSERT(order > 0);
  DRAKE_ASSERT(start_dim >= 0);
  DRAKE_ASSERT(base.size() == static_cast<size_t>(num_vars));
  if (order == 0) {
    return;
  }
  if (order == 1) {
    for (int i = start_dim; i < num_vars; ++i) {
      MultiIndex alpha = base;
      ++alpha[i];
      vec->push_back(std::move(alpha));
    }
    return;
  } else {
    for (int i = start_dim; i < num_vars; ++i) {
      MultiIndex alpha = base;
      ++alpha[i];
      DoEnumerateMultiIndex(order - 1, num_vars, i, alpha, vec);
    }
  }
}

// Returns the set of multi-indices of order `order` whose size is `num-vars`.
vector<MultiIndex> EnumerateMultiIndex(const int order, const int num_vars) {
  DRAKE_ASSERT(order > 0);
  DRAKE_ASSERT(num_vars >= 1);
  vector<MultiIndex> vec;
  MultiIndex base(num_vars, 0);  // base = (0, ..., 0)
  DoEnumerateMultiIndex(order, num_vars, 0, base, &vec);
  return vec;
}

// Computes the factorial of n.
int Factorial(const int n) {
  DRAKE_ASSERT(n >= 0);
  int f = 1;
  for (int i = 2; i <= n; ++i) {
    f *= i;
  }
  return f;
}

// Given a multi index α = (α₁, ..., αₙ), returns α₁! * ... * αₙ!.
int FactorialProduct(const MultiIndex& alpha) {
  int ret = 1;
  for (const int i : alpha) {
    ret *= Factorial(i);
  }
  return ret;
}

// Computes ∂fᵅ(a) = ∂ᵅ¹...∂ᵅⁿf(a).
Expression Derivative(Expression f, const MultiIndex& alpha,
                      const Environment& a) {
  int i = 0;
  for (const pair<const Variable, double>& p : a) {
    const Variable& v = p.first;
    for (int j = 0; j < alpha[i]; ++j) {
      f = f.Differentiate(v);
    }
    ++i;
  }
  return f.EvaluatePartial(a);
}

// Given terms = [e₁, ..., eₙ] and alpha = (α₁, ..., αₙ), returns
// pow(e₁,α₁) * ... * pow(eₙ,αₙ)
Expression Exp(const vector<Expression>& terms, const MultiIndex& alpha) {
  DRAKE_ASSERT(terms.size() == alpha.size());
  ExpressionMulFactory factory;
  for (size_t i = 0; i < terms.size(); ++i) {
    factory.AddExpression(pow(terms[i], alpha[i]));
  }
  return factory.GetExpression();
}

// Computes ∑_{|α| = order} ∂fᵅ(a) / α! * (x - a)ᵅ.
void DoTaylorExpand(const Expression& f, const Environment& a,
                    const vector<Expression>& terms, const int order,
                    const int num_vars, ExpressionAddFactory* const factory) {
  DRAKE_ASSERT(order > 0);
  DRAKE_ASSERT(terms.size() == static_cast<size_t>(num_vars));
  const vector<MultiIndex> multi_indices{EnumerateMultiIndex(order, num_vars)};
  for (const MultiIndex& alpha : multi_indices) {
    factory->AddExpression(Derivative(f, alpha, a) * Exp(terms, alpha) /
                           FactorialProduct(alpha));
  }
}
}  // namespace

Expression TaylorExpand(const Expression& f, const Environment& a,
                        const int order) {
  // The implementation uses the formulation:
  //      Taylor(f, a, order) = ∑_{|α| ≤ order} ∂fᵅ(a) / α! * (x - a)ᵅ.
  DRAKE_DEMAND(order >= 1);
  ExpressionAddFactory factory;
  factory.AddExpression(f.EvaluatePartial(a));
  const int num_vars = a.size();
  if (num_vars == 0) {
    return f;
  }
  vector<Expression> terms;  // (x - a)
  for (const pair<const Variable, double>& p : a) {
    const Variable& var = p.first;
    const double v = p.second;
    terms.push_back(var - v);
  }
  for (int i = 1; i <= order; ++i) {
    DoTaylorExpand(f, a, terms, i, num_vars, &factory);
  }
  return factory.GetExpression();
}

namespace {
// Visitor used in the implementation of the GetDistinctVariables function
// defined below.
struct GetDistinctVariablesVisitor {
  // called for the first coefficient
  void init(const Expression& value, Eigen::Index, Eigen::Index) {
    variables += value.GetVariables();
  }
  // called for all other coefficients
  void operator()(const Expression& value, Eigen::Index, Eigen::Index) {
    variables += value.GetVariables();
  }
  Variables variables;
};
}  // namespace

Variables GetDistinctVariables(const Eigen::Ref<const MatrixX<Expression>>& v) {
  GetDistinctVariablesVisitor visitor;
  v.visit(visitor);
  return visitor.variables;
}

}  // namespace symbolic

double ExtractDoubleOrThrow(const symbolic::Expression& e) {
  if (is_nan(e)) {
    // If this was a literal NaN provided by the user or a dummy_value<T>, then
    // it is sound to promote it as the "extracted value" during scalar
    // conversion.  (In contrast, if an expression tree includes a NaN term,
    // then it's still desirable to throw an exception and we should NOT return
    // NaN in that case.)
    return std::numeric_limits<double>::quiet_NaN();
  }
  return e.Evaluate();
}

}  // namespace drake
