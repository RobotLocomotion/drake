/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include): Our header file is included by all.h.
#include "drake/common/symbolic/expression/all.h"
/* clang-format on */

#include <algorithm>
#include <functional>
#include <ios>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/container/inlined_vector.h"
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#define DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER
#include "drake/common/symbolic/expression/expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER

namespace drake {
namespace symbolic {

using std::logic_error;
using std::make_unique;
using std::map;
using std::numeric_limits;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::streamsize;
using std::string;
using std::unique_ptr;
using std::vector;

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

Expression::Expression(const Variable& var)
    : Expression{make_unique<ExpressionVar>(var)} {}

// Constructs this taking ownership of the given call.
Expression::Expression(std::unique_ptr<ExpressionCell> cell) {
  boxed_.SetSharedCell(cell.release());
}

// Implements the Expression(double) constructor when the constant is NaN.
// Note that this uses the ExpressionNaN cell type to denote NaN, not a
// constant NaN value in an ExpressionKind::Constant.
void Expression::ConstructExpressionCellNaN() {
  *this = NaN();
}

ExpressionCell& Expression::mutable_cell() {
  ExpressionCell& result = const_cast<ExpressionCell&>(cell());
  // This is an invariant of Drake's symbolic library. If this ever trips,
  // that indicates an implementation defect within Drake; please file a bug.
  DRAKE_DEMAND(result.use_count() == 1);
  return result;
}

void Expression::HashAppend(DelegatingHasher* hasher) const {
  using drake::hash_append;
  if (is_constant(*this)) {
    hash_append(*hasher, ExpressionKind::Constant);
    hash_append(*hasher, get_constant_value(*this));
  } else {
    hash_append(*hasher, get_kind());
    cell().HashAppendDetail(hasher);
  }
}

Expression Expression::NaN() {
  // This global cell is initialized upon first use and never destroyed.
  static const ExpressionNaN* const flyweight = []() {
    ExpressionNaN* initial_value = new ExpressionNaN;
    ++initial_value->use_count();
    return initial_value;
  }();
  Expression result;
  result.boxed_.SetSharedCell(flyweight);
  return result;
}

Variables Expression::GetVariables() const {
  if (is_constant(*this)) {
    return {};
  }
  return cell().GetVariables();
}

bool Expression::EqualTo(const Expression& e) const {
  if (boxed_.trivially_equals(e.boxed_)) {
    return true;
  }
  const ExpressionKind k1{get_kind()};
  const ExpressionKind k2{e.get_kind()};
  if (k1 != k2) {
    return false;
  }
  if (k1 == ExpressionKind::Constant) {
    return get_constant_value(*this) == get_constant_value(e);
  }
  // Check structural equality.
  return cell().EqualTo(e.cell());
}

bool Expression::Less(const Expression& e) const {
  if (boxed_.trivially_equals(e.boxed_)) {
    return false;
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
  if (k1 == ExpressionKind::Constant) {
    return get_constant_value(*this) < get_constant_value(e);
  }
  return cell().Less(e.cell());
}

bool Expression::is_polynomial() const {
  if (is_constant(*this)) {
    return true;
  }
  return cell().is_polynomial();
}

bool Expression::is_expanded() const {
  if (is_constant(*this)) {
    return true;
  }
  return cell().is_expanded();
}

double Expression::Evaluate(const Environment& env,
                            RandomGenerator* const random_generator) const {
  if (is_constant(*this)) {
    return get_constant_value(*this);
  }
  if (random_generator == nullptr) {
    return cell().Evaluate(env);
  } else {
    return cell().Evaluate(
        PopulateRandomVariables(env, GetVariables(), random_generator));
  }
}

double Expression::Evaluate(RandomGenerator* const random_generator) const {
  if (is_constant(*this)) {
    return get_constant_value(*this);
  }
  return Evaluate(Environment{}, random_generator);
}

Eigen::SparseMatrix<double> Evaluate(
    const Eigen::Ref<const Eigen::SparseMatrix<Expression>>& m,
    const Environment& env) {
  return m.unaryExpr([&env](const Expression& e) {
    return e.Evaluate(env);
  });
}

Expression Expression::EvaluatePartial(const Environment& env) const {
  if (is_constant(*this) || env.empty()) {
    return *this;
  }
  return cell().EvaluatePartial(env);
}

Expression Expression::Expand() const {
  if (is_constant(*this)) {
    return *this;
  }
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
  if (is_constant(*this)) {
    return *this;
  }
  return cell().Substitute({{var, e}});
}

Expression Expression::Substitute(const Substitution& s) const {
  if (is_constant(*this) || s.empty()) {
    return *this;
  }
  return cell().Substitute(s);
}

Expression Expression::Differentiate(const Variable& x) const {
  if (is_constant(*this)) {
    return 0.0;
  }
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

void Expression::AddImpl(const Expression& rhs) {
  Expression& lhs = *this;
  // Simplification: 0 + x => x
  if (is_zero(lhs)) {
    lhs = rhs;
    return;
  }
  // Simplification: x + 0 => x
  if (is_zero(rhs)) {
    return;
  }
  // If both terms were constants, our header file `operator+=` function would
  // have already tried to add them, but got a speculative_result of NaN. If
  // we reach here, that means the floating-point result really was NaN.
  if (is_constant(lhs) && is_constant(rhs)) {
    ConstructExpressionCellNaN();
    return;
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
  lhs = std::move(add_factory).GetExpression();
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

Expression operator+(const Expression& e) {
  return e;
}

void Expression::SubImpl(const Expression& rhs) {
  Expression& lhs = *this;
  // Simplification: E - E => 0
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might cause 0/0 during evaluation.
  if (lhs.EqualTo(rhs)) {
    lhs = Expression::Zero();
    return;
  }
  // Simplification: x - 0 => x
  if (is_zero(rhs)) {
    return;
  }
  // If both terms were constants, our header file `operator-=` function would
  // have already tried to subtract them, but got a speculative_result of NaN.
  // If we reach here, that means the floating-point result really was NaN.
  // However, that should only happen with inf-inf or (-inf)-(-inf), which
  // we've handled already.
  DRAKE_ASSERT(!(is_constant(lhs) && is_constant(rhs)));
  // x - y => x + (-y)
  lhs += -rhs;
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

void Expression::MulImpl(const Expression& rhs) {
  Expression& lhs = *this;
  // Simplification: 1 * x => x
  if (is_one(lhs)) {
    lhs = rhs;
    return;
  }
  // Simplification: x * 1 => x
  if (is_one(rhs)) {
    return;
  }
  // Simplification: (E1 / E2) * (E3 / E4) => (E1 * E3) / (E2 * E4)
  if (is_division(lhs) && is_division(rhs)) {
    lhs = (get_first_argument(lhs) * get_first_argument(rhs)) /
          (get_second_argument(lhs) * get_second_argument(rhs));
    return;
  }
  // Simplification: lhs * (c / E) => (c * lhs) / E
  if (is_division(rhs) && is_constant(get_first_argument(rhs))) {
    lhs = (get_first_argument(rhs) * lhs) / get_second_argument(rhs);
    return;
  }
  // Simplification: (c / E) * rhs => (c * rhs) / E
  if (is_division(lhs) && is_constant(get_first_argument(lhs))) {
    lhs = (get_first_argument(lhs) * rhs) / get_second_argument(lhs);
    return;
  }
  if (is_neg_one(lhs)) {
    if (is_addition(rhs)) {
      // Simplification: push '-' inside over '+'.
      // -1 * (E_1 + ... + E_n) => (-E_1 + ... + -E_n)
      lhs = NegateAddition(rhs);
      return;
    }
    if (is_multiplication(rhs)) {
      // Simplification: push '-' inside over '*'.
      // -1 * (c0 * E_1 * ... * E_n) => (-c0 * E_1 * ... * E_n)
      lhs = NegateMultiplication(rhs);
      return;
    }
  }

  if (is_neg_one(rhs)) {
    if (is_addition(lhs)) {
      // Simplification: push '-' inside over '+'.
      // (E_1 + ... + E_n) * -1 => (-E_1 + ... + -E_n)
      lhs = NegateAddition(lhs);
      return;
    }
    if (is_multiplication(lhs)) {
      // Simplification: push '-' inside over '*'.
      // (c0 * E_1 * ... * E_n) * -1 => (-c0 * E_1 * ... * E_n)
      lhs = NegateMultiplication(lhs);
      return;
    }
  }

  // Simplification: 0 * E => 0
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might cause 0/0 during evaluation.
  if (is_zero(lhs)) {
    return;
  }
  // Simplification: E * 0 => 0
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might cause 0/0 during evaluation.
  if (is_zero(rhs)) {
    lhs = Expression::Zero();
    return;
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
        return;
      }
    }
    if (e1.EqualTo(rhs)) {
      // Simplification: pow(e1, e2) * e1 => pow(e1, e2 + 1)
      // TODO(soonho-tri): This simplification is not sound.
      const Expression& e2{get_second_argument(lhs)};
      lhs = pow(e1, e2 + 1);
      return;
    }
  } else {
    if (is_pow(rhs)) {
      const Expression& e1{get_first_argument(rhs)};
      if (e1.EqualTo(lhs)) {
        // Simplification: (lhs * rhs == e1 * pow(e1, e2)) => pow(e1, 1 + e2)
        // TODO(soonho-tri): This simplification is not sound.
        const Expression& e2{get_second_argument(rhs)};
        lhs = pow(e1, 1 + e2);
        return;
      }
    }
  }
  // If both terms were constants, our header file `operator*=` function would
  // have already tried to multiply them, but got a speculative_result of NaN.
  // If we reach here, that means the floating-point result really was NaN.
  // However, that should only happen with 0*inf or inf*0, which we've handled
  // already.
  DRAKE_ASSERT(!(is_constant(lhs) && is_constant(rhs)));
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
        return;
      }
      // nothing to flatten
      mul_factory.AddExpression(lhs);
      mul_factory.AddExpression(rhs);
    }
  }
  lhs = std::move(mul_factory).GetExpression();
}

void Expression::DivImpl(const Expression& rhs) {
  Expression& lhs = *this;
  // Simplification: x / 1 => x
  if (is_one(rhs)) {
    return;
  }
  // If both terms were constants, our header file `operator/=` function would
  // have already tried to divide them, but if it wasn't able to commit to a
  // result that means that either the divisor was zero or else the result
  // really is NaN.
  if (is_constant(lhs) && is_constant(rhs)) {
    if (is_zero(rhs)) {
      ostringstream oss{};
      oss << "Division by zero: " << lhs << "/0";
      throw runtime_error(oss.str());
    }
    ConstructExpressionCellNaN();
    return;
  }
  // Simplification: E / E => 1
  // TODO(soonho-tri): This simplification is not sound since it cancels `E`
  // which might contain 0/0 problems.
  if (lhs.EqualTo(rhs)) {
    lhs = Expression::One();
    return;
  }
  lhs = Expression{make_unique<ExpressionDiv>(lhs, rhs)};
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrecisionGuard);
  ~PrecisionGuard() { os_->precision(original_precision_); }

 private:
  ostream* const os_;
  const streamsize original_precision_;
};
}  // namespace

ostream& operator<<(ostream& os, const Expression& e) {
  const PrecisionGuard precision_guard{&os,
                                       numeric_limits<double>::max_digits10};
  if (is_constant(e)) {
    os << get_constant_value(e);
  } else {
    e.cell().Display(os);
  }
  return os;
}

Expression log(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    const double v{get_constant_value(e)};
    ExpressionLog::check_domain(v);
    return Expression{std::log(v)};
  }
  return Expression{make_unique<ExpressionLog>(e)};
}

Expression abs(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::fabs(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionAbs>(e)};
}

Expression exp(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::exp(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionExp>(e)};
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
  return Expression{make_unique<ExpressionSqrt>(e)};
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
    return Expression{make_unique<ExpressionPow>(base, exponent * e2)};
  }
  return Expression{make_unique<ExpressionPow>(e1, e2)};
}

Expression sin(const Expression& e) {
  // simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::sin(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionSin>(e)};
}

Expression cos(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::cos(get_constant_value(e))};
  }

  return Expression{make_unique<ExpressionCos>(e)};
}

Expression tan(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::tan(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionTan>(e)};
}

Expression asin(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    const double v{get_constant_value(e)};
    ExpressionAsin::check_domain(v);
    return Expression{std::asin(v)};
  }
  return Expression{make_unique<ExpressionAsin>(e)};
}

Expression acos(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    const double v{get_constant_value(e)};
    ExpressionAcos::check_domain(v);
    return Expression{std::acos(v)};
  }
  return Expression{make_unique<ExpressionAcos>(e)};
}

Expression atan(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::atan(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionAtan>(e)};
}

Expression atan2(const Expression& e1, const Expression& e2) {
  // Simplification: constant folding.
  if (is_constant(e1) && is_constant(e2)) {
    return Expression{
        std::atan2(get_constant_value(e1), get_constant_value(e2))};
  }
  return Expression{make_unique<ExpressionAtan2>(e1, e2)};
}

Expression sinh(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::sinh(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionSinh>(e)};
}

Expression cosh(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::cosh(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionCosh>(e)};
}

Expression tanh(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::tanh(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionTanh>(e)};
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
  return Expression{make_unique<ExpressionMin>(e1, e2)};
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
  return Expression{make_unique<ExpressionMax>(e1, e2)};
}

Expression clamp(const Expression& v, const Expression& lo,
                 const Expression& hi) {
  DRAKE_ASSERT(!is_constant(lo) || !is_constant(hi) || (lo <= hi));
  return min(hi, max(lo, v));
}

Expression ceil(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::ceil(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionCeiling>(e)};
}

Expression floor(const Expression& e) {
  // Simplification: constant folding.
  if (is_constant(e)) {
    return Expression{std::floor(get_constant_value(e))};
  }
  return Expression{make_unique<ExpressionFloor>(e)};
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
  return Expression{make_unique<ExpressionIfThenElse>(f_cond, e_then, e_else)};
}

Expression uninterpreted_function(string name, vector<Expression> arguments) {
  return Expression{make_unique<ExpressionUninterpretedFunction>(
      std::move(name), std::move(arguments))};
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

Expression operator+(const Variable& var) {
  return Expression{var};
}
Expression operator-(const Variable& var) {
  return -Expression{var};
}

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
  return std::move(factory).GetExpression();
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
  return std::move(factory).GetExpression();
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
}  // namespace symbolic
}  // namespace drake

namespace Eigen {
namespace internal {
// Using Matrix::visit requires providing functor_traits.
template <>
struct functor_traits<drake::symbolic::GetDistinctVariablesVisitor> {
  static constexpr int Cost = 10;
  [[maybe_unused]] static constexpr bool LinearAccess = false;
  [[maybe_unused]] static constexpr bool PacketAccess = false;
};
}  // namespace internal
}  // namespace Eigen

namespace drake {
namespace symbolic {

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

namespace symbolic {
namespace internal {

template <bool reverse>
void Gemm<reverse>::CalcDV(const MatrixRef<double>& D,
                           const MatrixRef<Variable>& V,
                           EigenPtr<MatrixX<Expression>> result) {
  // These checks are guaranteed by our header file functions that call us.
  DRAKE_ASSERT(result != nullptr);
  if constexpr (!reverse) {
    // Calculating D * V.
    DRAKE_ASSERT(result->rows() == D.rows());
    DRAKE_ASSERT(result->cols() == V.cols());
    DRAKE_ASSERT(D.cols() == V.rows());
  } else {
    // Calculating V * D.
    DRAKE_ASSERT(result->rows() == V.rows());
    DRAKE_ASSERT(result->cols() == D.cols());
    DRAKE_ASSERT(V.cols() == D.rows());
  }

  // Our temporary vectors will use the stack for this number of items or fewer.
  constexpr size_t kSmallSize = 8;

  // In the below, we'll use index `i` to refer to a column of the V matrix
  // (or row, when in reverse mode) and index `j` to refer to a row of the
  // D matrix (or column, when in reverse mode).
  const int max_i = !reverse ? V.cols() : V.rows();
  const int max_j = !reverse ? D.rows() : D.cols();

  // We'll use index `k` to index over the inner dot product.
  const int max_k = !reverse ? D.cols() : V.cols();

  // The four containers below are reused across the outer loop. Each one is
  // either cleared or fully overwritten at the start of each outer loop.
  //
  // We'll use `t` to index over a sorted and unique vector of the variables in
  // each inner product. When there are no duplicates, `t`'s range will be the
  // same as `k` but in a different order. With duplicates, `t` will be shorter.
  //
  // Each inner product is calculated as Σ inner_coeffs(t) * inner_var(t). In
  // other words, we directly compute the canonical form of ExpressionAdd for
  // each inner product, without going through the ExpressionAddFactory.
  absl::InlinedVector<double, kSmallSize> inner_coeffs;
  inner_coeffs.reserve(max_k);
  absl::InlinedVector<Variable, kSmallSize> inner_vars;
  inner_vars.reserve(max_k);
  absl::InlinedVector<Expression, kSmallSize> inner_vars_as_expr;
  inner_vars_as_expr.reserve(max_k);
  // The unique_ids is used to efficiently de-duplicate vars.
  absl::flat_hash_set<Variable::Id> unique_ids;
  unique_ids.reserve(max_k);
  // The var_index[k] refers to the index in inner_vars[] for that variable.
  // We have inner_vars[var_index[k]] == V(k, j) during the j'th row loop,
  // or V(j, k) when in reverse mode.
  absl::InlinedVector<int, kSmallSize> var_index;
  var_index.resize(max_k);

  // The outer loop over `i` steps through each column (or row) of V, so that we
  // can share the variable de-duplication and sorting needed by the for-j loop.
  for (int i = 0; i < max_i; ++i) {
    // Convert each Variable to an Expression, consolidating duplicates.
    // Sort the variables so that map insertion later will be linear time.
    inner_vars.clear();
    unique_ids.clear();
    for (int k = 0; k < max_k; ++k) {
      const Variable& var = !reverse ? V(k, i) : V(i, k);
      const Variable::Id var_id = var.get_id();
      const bool inserted = unique_ids.insert(var_id).second;
      if (inserted) {
        inner_vars.emplace_back(var);
      }
    }
    std::sort(inner_vars.begin(), inner_vars.end(), std::less<Variable>{});

    // Compute the var_index that projects this V block into inner_vars.
    for (int k = 0; k < max_k; ++k) {
      const Variable& var = !reverse ? V(k, i) : V(i, k);
      auto iter = std::lower_bound(inner_vars.begin(), inner_vars.end(), var,
                                   std::less<Variable>{});
      DRAKE_ASSERT(iter != inner_vars.end());
      DRAKE_ASSERT(iter->get_id() == var.get_id());
      const int index = std::distance(inner_vars.begin(), iter);
      var_index[k] = index;
    }

    // Convert the vars to expression cells. We want these to all be shared for
    // faster comparisons downtream after the Gemm.
    inner_vars_as_expr.clear();
    for (size_t t = 0; t < inner_vars.size(); ++t) {
      inner_vars_as_expr.push_back(inner_vars[t]);
    }

    // Now that the V block is precomputed in a useful format, we can quickly
    // loop through each block of double coeffs in turn.
    for (int j = 0; j < max_j; ++j) {
      // Prepare storage for the summed-up coeffs.
      inner_coeffs.clear();
      inner_coeffs.resize(inner_vars.size(), 0.0);

      // Sum up D(j, k) * V(k, i) or in reverse mode V(j, i) * D(k, j).
      for (int k = 0; k < max_k; ++k) {
        inner_coeffs[var_index[k]] += !reverse ? D(j, k) : D(k, j);
      }

      // Convert the sum to the ExpressionAdd representation, skipping zeros,
      // but for now in vector-pair form instead of map form.
      using TermsMap = map<Expression, double>;
      using TermsVec = absl::InlinedVector<TermsMap::value_type, kSmallSize>;
      TermsVec terms_vec;
      for (size_t t = 0; t < inner_coeffs.size(); ++t) {
        const double coeff = inner_coeffs[t];
        if (coeff == 0.0) {
          continue;
        }
        terms_vec.emplace_back(inner_vars_as_expr[t], coeff);
      }

      // Convert the ExpressionAdd representation to an Expression cell.
      Expression inner_product;
      if (!terms_vec.empty()) {
        if ((terms_vec.size() == 1) && (terms_vec.front().second == 1.0)) {
          // Special case for `1.0 * v`, use a Var cell (not an Add cell).
          inner_product = std::move(terms_vec.front().first);
        } else {
          // This is carefully crafted to be a linear time insertion because the
          // vector is already sorted, and to avoid unnecessary copying because
          // both collections have the same value_type.
          TermsMap terms_map(std::make_move_iterator(terms_vec.begin()),
                             std::make_move_iterator(terms_vec.end()));
          auto cell = make_unique<ExpressionAdd>(0.0, std::move(terms_map));
          cell->set_expanded();
          inner_product = Expression{std::move(cell)};
        }
      }
      if constexpr (!reverse) {
        (*result)(j, i) = std::move(inner_product);
      } else {
        (*result)(i, j) = std::move(inner_product);
      }
    }
  }
}

namespace {

template <typename T1, typename T2>
Expression GenericGevv(const Eigen::Ref<const VectorX<T1>, 0, StrideX>& a,
                       const Eigen::Ref<const VectorX<T2>, 0, StrideX>& b) {
  DRAKE_ASSERT(a.size() == b.size());
  ExpressionAddFactory fac;
  for (int k = 0; k < a.size(); ++k) {
    fac.AddExpression(a[k] * b[k]);
  }
  return std::move(fac).GetExpression();
}

template <typename T1, typename T2>
void GenericGemm(const Eigen::Ref<const MatrixX<T1>, 0, StrideX>& left,
                 const Eigen::Ref<const MatrixX<T2>, 0, StrideX>& right,
                 EigenPtr<MatrixX<Expression>> result) {
  // These checks are guaranteed by our header file functions that call us.
  DRAKE_ASSERT(result != nullptr);
  DRAKE_ASSERT(result->rows() == left.rows());
  DRAKE_ASSERT(result->cols() == right.cols());
  DRAKE_ASSERT(left.cols() == right.rows());

  // Delegate to Gevv.
  for (int i = 0; i < result->rows(); ++i) {
    for (int j = 0; j < result->cols(); ++j) {
      (*result)(i, j) = GenericGevv<T1, T2>(left.row(i), right.col(j));
    }
  }
}

}  // namespace

template <bool reverse>
void Gemm<reverse>::CalcVV(const MatrixRef<Variable>& A,
                           const MatrixRef<Variable>& B,
                           EigenPtr<MatrixX<Expression>> result) {
  // We convert Variable => Expression up front, so the ExpressionVar cells get
  // reused during the computation instead of creating lots of duplicates.
  // TODO(jwnimmer-tri) If A or B contain duplicate variables (e.g., symmetric),
  // it's possible that interning the duplicates would improve performance of
  // subsequent operations.
  CalcEE(A.template cast<Expression>(), B.template cast<Expression>(), result);
}

template <bool reverse>
void Gemm<reverse>::CalcDE(const MatrixRef<double>& D,
                           const MatrixRef<Expression>& E,
                           EigenPtr<MatrixX<Expression>> result) {
  if constexpr (!reverse) {
    GenericGemm<double, Expression>(D, E, result);
  } else {
    GenericGemm<Expression, double>(E, D, result);
  }
}

template <bool reverse>
void Gemm<reverse>::CalcVE(const MatrixRef<Variable>& V,
                           const MatrixRef<Expression>& E,
                           EigenPtr<MatrixX<Expression>> result) {
  // We convert Variable => Expression up front, so the ExpressionVar cells get
  // reused during the computation instead of creating lots of duplicates.
  // TODO(jwnimmer-tri) If V contains duplicate variables (e.g., symmetric),
  // it's possible that interning the duplicates would improve performance of
  // subsequent operations.
  CalcEE(V.template cast<Expression>(), E, result);
}

template <bool reverse>
void Gemm<reverse>::CalcEE(const MatrixRef<Expression>& A,
                           const MatrixRef<Expression>& B,
                           EigenPtr<MatrixX<Expression>> result) {
  if constexpr (!reverse) {
    GenericGemm<Expression, Expression>(A, B, result);
  } else {
    GenericGemm<Expression, Expression>(B, A, result);
  }
}

template struct Gemm<false>;
template struct Gemm<true>;

}  // namespace internal
}  // namespace symbolic
}  // namespace drake
