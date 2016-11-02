#include "drake/common/symbolic_expression.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/hash_combine.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::domain_error;
using std::endl;
using std::hash;
using std::invalid_argument;
using std::make_shared;
using std::ostream;
using std::ostringstream;
using std::prev;
using std::runtime_error;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;
using std::setprecision;
using std::numeric_limits;

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

Expression operator+(const double lhs, const Expression& rhs) {
  // Uses () to avoid a conflict between cpplint and clang-format.
  return (Expression{lhs}) + rhs;
}

Expression operator+(Expression lhs, const double rhs) {
  lhs += rhs;
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator+=(Expression& lhs, const Expression& rhs) {
  // simplification #1 : 0 + x => x
  if (lhs.EqualTo(Expression::Zero())) {
    lhs = rhs;
    return lhs;
  }
  // simplification #2 : x + 0 => x
  if (rhs.EqualTo(Expression::Zero())) {
    return lhs;
  }
  // simplification #3 : Expression(c1) + Expression(c2) => Expression(c1 + c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    const double v1{
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value()};
    const double v2{
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value()};
    lhs = Expression{v1 + v2};
    return lhs;
  }
  lhs.ptr_ = make_shared<ExpressionAdd>(lhs, rhs);
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator+=(Expression& lhs, const double rhs) {
  lhs += Expression{rhs};
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

Expression operator-(const double lhs, const Expression& rhs) {
  Expression ret{lhs};
  ret -= rhs;
  return ret;
}

Expression operator-(Expression lhs, const double rhs) {
  lhs -= Expression{rhs};
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator-=(Expression& lhs, const Expression& rhs) {
  // simplification #1 : E - E => 0
  if (lhs.EqualTo(rhs)) {
    lhs = Expression::Zero();
    return lhs;
  }
  // simplification #2 : x - 0 => x
  if (rhs.EqualTo(Expression::Zero())) {
    return lhs;
  }
  // simplification #2 : Expression(c1) - Expression(c2) => Expression(c1 - c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    const double v1 =
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value();
    const double v2 =
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value();
    lhs = Expression{v1 - v2};
    return lhs;
  }
  lhs.ptr_ = make_shared<ExpressionSub>(lhs, rhs);
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator-=(Expression& lhs, const double rhs) {
  lhs -= Expression{rhs};
  return lhs;
}

Expression operator-(Expression e) {
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{-v};
  } else {
    return Expression{make_shared<ExpressionNeg>(e)};
  }
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

Expression operator*(const double lhs, const Expression& rhs) {
  // Uses () to avoid a conflict between cpplint and clang-format.
  return (Expression{lhs}) * rhs;
}

Expression operator*(Expression lhs, const double rhs) {
  lhs *= Expression{rhs};
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator*=(Expression& lhs, const Expression& rhs) {
  // simplification #1 : 1 * x => x
  if (lhs.EqualTo(Expression::One())) {
    lhs = rhs;
    return lhs;
  }
  // simplification #2 : x * 1 => x
  if (rhs.EqualTo(Expression::One())) {
    return lhs;
  }
  // simplification #3 : -1 * x => -x
  if (lhs.EqualTo(-Expression::One())) {
    lhs = -rhs;
    return lhs;
  }
  // simplification #4 : x * -1 => -x
  if (rhs.EqualTo(-Expression::One())) {
    lhs = -lhs;
    return lhs;
  }
  // simplification #5 : 0 * x => 0
  if (lhs.EqualTo(Expression::Zero())) {
    return lhs;
  }
  // simplification #6 : x * 0 => 0
  if (rhs.EqualTo(Expression::Zero())) {
    lhs = Expression::Zero();
    return lhs;
  }
  // simplification #7 : Expression(c1) * Expression(c2) => Expression(c1 * c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    const double v1 =
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value();
    const double v2 =
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value();
    lhs = Expression{v1 * v2};
    return lhs;
  }
  lhs.ptr_ = make_shared<ExpressionMul>(lhs, rhs);
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator*=(Expression& lhs, const double rhs) {
  lhs *= Expression{rhs};
  return lhs;
}

Expression operator/(Expression lhs, const Expression& rhs) {
  lhs /= rhs;
  return lhs;
}

Expression operator/(const double lhs, const Expression& rhs) {
  Expression ret{lhs};
  ret /= rhs;
  return ret;
}

Expression operator/(Expression lhs, const double rhs) {
  lhs /= Expression{rhs};
  return lhs;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Expression& operator/=(Expression& lhs, const Expression& rhs) {
  // simplification #1 : x / 1 => x
  if (rhs.EqualTo(Expression::One())) {
    return lhs;
  }
  // simplification #2 : Expression(c1) / Expression(c2) => Expression(c1 / c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    const double v1 =
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value();
    const double v2 =
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value();
    if (v2 == 0.0) {
      ostringstream oss;
      oss << "Division by zero: " << v1 << "/" << v2;
      throw runtime_error(oss.str());
    }
    lhs = Expression{v1 / v2};
    return lhs;
  }
  // simplification #3 : E / E => 1
  if (lhs.EqualTo(rhs)) {
    lhs = Expression::One();
    return lhs;
  }
  lhs.ptr_ = make_shared<ExpressionDiv>(lhs, rhs);
  return lhs;
}

Expression& operator/=(Expression& lhs, const double rhs) {
  lhs /= Expression{rhs};
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
  const UnaryExpressionCell& unary_e =
      static_cast<const UnaryExpressionCell&>(e);
  return e_.EqualTo(unary_e.e_);
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
  const BinaryExpressionCell& binary_e =
      static_cast<const BinaryExpressionCell&>(e);
  return e1_.EqualTo(binary_e.e1_) && e2_.EqualTo(binary_e.e2_);
}

double BinaryExpressionCell::Evaluate(const Environment& env) const {
  const double v1{e1_.Evaluate(env)};
  const double v2{e2_.Evaluate(env)};
  return DoEvaluate(v1, v2);
}

ExpressionVar::ExpressionVar(const Variable& v)
    : ExpressionCell{ExpressionKind::Var, hash<Variable>{}(v)}, var_{v} {}

Variables ExpressionVar::GetVariables() const { return {get_variable()}; }

bool ExpressionVar::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Var) {
    return false;
  }
  return var_ == static_cast<const ExpressionVar&>(e).var_;
}

double ExpressionVar::Evaluate(const Environment& env) const {
  Environment::const_iterator const it{env.find(var_)};
  if (it != env.cend()) {
    DRAKE_ASSERT(!std::isnan(it->second));
    return it->second;
  } else {
    ostringstream oss;
    oss << "The following environment does not have an entry for the variable "
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

ExpressionAdd::ExpressionAdd(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Add, e1, e2} {}

ostream& ExpressionAdd::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " + " << get_2nd_expression() << ")";
  return os;
}

double ExpressionAdd::DoEvaluate(const double v1, const double v2) const {
  return v1 + v2;
}

ExpressionSub::ExpressionSub(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Sub, e1, e2} {}

ostream& ExpressionSub::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " - " << get_2nd_expression() << ")";
  return os;
}

double ExpressionSub::DoEvaluate(const double v1, const double v2) const {
  return v1 - v2;
}

ExpressionMul::ExpressionMul(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Mul, e1, e2} {}

ostream& ExpressionMul::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " * " << get_2nd_expression() << ")";
  return os;
}

double ExpressionMul::DoEvaluate(const double v1, const double v2) const {
  return v1 * v2;
}

ExpressionDiv::ExpressionDiv(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Div, e1, e2} {}

ostream& ExpressionDiv::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " / " << get_2nd_expression() << ")";
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
  os << "pow(" << get_1st_expression() << ", " << get_2nd_expression() << ")";
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
  os << "atan2(" << get_1st_expression() << ", " << get_2nd_expression() << ")";
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
  os << "min(" << get_1st_expression() << ", " << get_2nd_expression() << ")";
  return os;
}

double ExpressionMin::DoEvaluate(const double v1, const double v2) const {
  return std::min(v1, v2);
}

ExpressionMax::ExpressionMax(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Max, e1, e2} {}

ostream& ExpressionMax::Display(ostream& os) const {
  os << "max(" << get_1st_expression() << ", " << get_2nd_expression() << ")";
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
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionLog::check_domain(v);
    return Expression{std::log(v)};
  }
  return Expression{make_shared<ExpressionLog>(e)};
}

Expression abs(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::fabs(v)};
  }
  return Expression{make_shared<ExpressionAbs>(e)};
}

Expression exp(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::exp(v)};
  }
  return Expression{make_shared<ExpressionExp>(e)};
}

Expression sqrt(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionSqrt::check_domain(v);
    return Expression{std::sqrt(v)};
  }
  return Expression{make_shared<ExpressionSqrt>(e)};
}

Expression pow(const Expression& e1, const Expression& e2) {
  // simplification
  if (e1.get_kind() == ExpressionKind::Constant &&
      e2.get_kind() == ExpressionKind::Constant) {
    const double v1 =
        static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value();
    const double v2 =
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value();
    ExpressionPow::check_domain(v1, v2);
    return Expression{std::pow(v1, v2)};
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
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::sin(v)};
  }
  return Expression{make_shared<ExpressionSin>(e)};
}

Expression cos(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::cos(v)};
  }

  return Expression{make_shared<ExpressionCos>(e)};
}

Expression tan(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::tan(v)};
  }
  return Expression{make_shared<ExpressionTan>(e)};
}

Expression asin(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionAsin::check_domain(v);
    return Expression{std::asin(v)};
  }
  return Expression{make_shared<ExpressionAsin>(e)};
}

Expression acos(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionAcos::check_domain(v);
    return Expression{std::acos(v)};
  }
  return Expression{make_shared<ExpressionAcos>(e)};
}

Expression atan(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::atan(v)};
  }
  return Expression{make_shared<ExpressionAtan>(e)};
}

Expression atan2(const Expression& e1, const Expression& e2) {
  // simplification
  if (e1.get_kind() == ExpressionKind::Constant &&
      e2.get_kind() == ExpressionKind::Constant) {
    const double v1 =
        static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value();
    const double v2 =
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value();
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
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::sinh(v)};
  }
  return Expression{make_shared<ExpressionSinh>(e)};
}

Expression cosh(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::cosh(v)};
  }
  return Expression{make_shared<ExpressionCosh>(e)};
}

Expression tanh(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    const double v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::tanh(v)};
  }
  return Expression{make_shared<ExpressionTanh>(e)};
}

Expression min(const Expression& e1, const Expression& e2) {
  // simplification #1: min(x, x) -> x
  if (e1.EqualTo(e2)) {
    return e1;
  }
  // simplification #2: constant folding
  if (e1.get_kind() == ExpressionKind::Constant &&
      e2.get_kind() == ExpressionKind::Constant) {
    const double v1 =
        static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value();
    const double v2 =
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value();
    return Expression{std::min(v1, v2)};
  }
  return Expression{make_shared<ExpressionMin>(e1, e2)};
}

Expression min(const Expression& e1, const double v2) {
  return min(e1, Expression{v2});
}

Expression min(const double v1, const Expression& e2) {
  return min(Expression{v1}, e2);
}

Expression max(const Expression& e1, const Expression& e2) {
  // simplification #1: max(x, x) -> x
  if (e1.EqualTo(e2)) {
    return e1;
  }
  // simplification #2: constant folding
  if (e1.get_kind() == ExpressionKind::Constant &&
      e2.get_kind() == ExpressionKind::Constant) {
    const double v1 =
        static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value();
    const double v2 =
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value();
    return Expression{std::max(v1, v2)};
  }
  return Expression{make_shared<ExpressionMax>(e1, e2)};
}

Expression max(const Expression& e1, const double v2) {
  return max(e1, Expression{v2});
}

Expression max(const double v1, const Expression& e2) {
  return max(Expression{v1}, e2);
}

}  // namespace symbolic
}  // namespace drake
