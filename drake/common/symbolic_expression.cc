#include "drake/common/symbolic_expression.h"

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
    double const v1 =
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value();
    double const v2 =
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value();
    lhs = Expression{v1 - v2};
    return lhs;
  }
  lhs.ptr_ = make_shared<ExpressionSub>(lhs, rhs);
  return lhs;
}

Expression& operator-=(Expression& lhs, double const rhs) {
  lhs -= Expression{rhs};
  return lhs;
}

Expression operator-(Expression e) {
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

Expression operator*(double const lhs, const Expression& rhs) {
  // Uses () to avoid a conflict between cpplint and clang-format.
  return (Expression{lhs}) * rhs;
}

Expression operator*(Expression lhs, double const rhs) {
  lhs *= Expression{rhs};
  return lhs;
}

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
  // simplification #3 : 0 * x => 0
  if (lhs.EqualTo(Expression::Zero())) {
    return lhs;
  }
  // simplification #4 : x * 0 => 0
  if (rhs.EqualTo(Expression::Zero())) {
    lhs = Expression::Zero();
    return lhs;
  }
  // simplification #5 : Expression(c1) * Expression(c2) => Expression(c1 * c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    double const v1 =
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value();
    double const v2 =
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value();
    lhs = Expression{v1 * v2};
    return lhs;
  }
  lhs.ptr_ = make_shared<ExpressionMul>(lhs, rhs);
  return lhs;
}

Expression& operator*=(Expression& lhs, double const rhs) {
  lhs *= Expression{rhs};
  return lhs;
}

Expression operator/(Expression lhs, const Expression& rhs) {
  lhs /= rhs;
  return lhs;
}

Expression operator/(double const lhs, const Expression& rhs) {
  Expression ret{lhs};
  ret /= rhs;
  return ret;
}

Expression operator/(Expression lhs, double const rhs) {
  lhs /= Expression{rhs};
  return lhs;
}

Expression& operator/=(Expression& lhs, const Expression& rhs) {
  // simplification #1 : x / 1 => x
  if (rhs.EqualTo(Expression::One())) {
    return lhs;
  }
  // simplification #2 : Expression(c1) / Expression(c2) => Expression(c1 / c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    double const v1 =
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value();
    double const v2 =
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

Expression& operator/=(Expression& lhs, double const rhs) {
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
                                         const size_t hash, const Expression& e)
    : ExpressionCell{k, hash}, e_{e} {}

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

BinaryExpressionCell::BinaryExpressionCell(const ExpressionKind k,
                                           const size_t hash,
                                           const Expression& e1,
                                           const Expression& e2)
    : ExpressionCell{k, hash}, e1_{e1}, e2_{e2} {}

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

ExpressionConstant::ExpressionConstant(double const v)
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
    : UnaryExpressionCell{ExpressionKind::Neg, e.get_hash(), e} {}

double ExpressionNeg::Evaluate(const Environment& env) const {
  return -get_expression().Evaluate(env);
}
ostream& ExpressionNeg::Display(ostream& os) const {
  os << "-(" << get_expression() << ")";
  return os;
}

ExpressionAdd::ExpressionAdd(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Add,
                           hash_combine(e1.get_hash(), e2.get_hash()), e1, e2} {
}

double ExpressionAdd::Evaluate(const Environment& env) const {
  return get_1st_expression().Evaluate(env) +
         get_2nd_expression().Evaluate(env);
}

ostream& ExpressionAdd::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " + " << get_2nd_expression() << ")";
  return os;
}

ExpressionSub::ExpressionSub(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Sub,
                           hash_combine(e1.get_hash(), e2.get_hash()), e1, e2} {
}

double ExpressionSub::Evaluate(const Environment& env) const {
  return get_1st_expression().Evaluate(env) -
         get_2nd_expression().Evaluate(env);
}

ostream& ExpressionSub::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " - " << get_2nd_expression() << ")";
  return os;
}

ExpressionMul::ExpressionMul(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Mul,
                           hash_combine(e1.get_hash(), e2.get_hash()), e1, e2} {
}

double ExpressionMul::Evaluate(const Environment& env) const {
  return get_1st_expression().Evaluate(env) *
         get_2nd_expression().Evaluate(env);
}

ostream& ExpressionMul::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " * " << get_2nd_expression() << ")";
  return os;
}

ExpressionDiv::ExpressionDiv(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Div,
                           hash_combine(e1.get_hash(), e2.get_hash()), e1, e2} {
}

double ExpressionDiv::Evaluate(const Environment& env) const {
  double const rhs{get_2nd_expression().Evaluate(env)};
  if (rhs == 0.0) {
    ostringstream oss;
    oss << "Division by zero: ";
    this->Display(oss) << endl;
    oss << "under the following environment:" << endl << env << endl;
    throw runtime_error(oss.str());
  }
  return get_1st_expression().Evaluate(env) / rhs;
}

ostream& ExpressionDiv::Display(ostream& os) const {
  os << "(" << get_1st_expression() << " / " << get_2nd_expression() << ")";
  return os;
}

ExpressionLog::ExpressionLog(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Log, e.get_hash(), e} {}

void ExpressionLog::check_domain(double const v) {
  if (!(v >= 0)) {
    ostringstream oss;
    oss << "log(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

double ExpressionLog::Evaluate(const Environment& env) const {
  double const eval_res{get_expression().Evaluate(env)};
  check_domain(eval_res);
  return std::log(eval_res);
}

ostream& ExpressionLog::Display(ostream& os) const {
  os << "log(" << get_expression() << ")";
  return os;
}

ExpressionAbs::ExpressionAbs(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Abs, e.get_hash(), e} {}

double ExpressionAbs::Evaluate(const Environment& env) const {
  double const eval_res{get_expression().Evaluate(env)};
  return std::fabs(eval_res);
}

ostream& ExpressionAbs::Display(ostream& os) const {
  os << "abs(" << get_expression() << ")";
  return os;
}

ExpressionExp::ExpressionExp(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Exp, e.get_hash(), e} {}

double ExpressionExp::Evaluate(const Environment& env) const {
  return std::exp(get_expression().Evaluate(env));
}

ostream& ExpressionExp::Display(ostream& os) const {
  os << "exp(" << get_expression() << ")";
  return os;
}

ExpressionSqrt::ExpressionSqrt(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Sqrt, e.get_hash(), e} {}

void ExpressionSqrt::check_domain(double const v) {
  if (!(v >= 0)) {
    ostringstream oss;
    oss << "sqrt(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

double ExpressionSqrt::Evaluate(const Environment& env) const {
  double const eval_res{get_expression().Evaluate(env)};
  check_domain(eval_res);
  return std::sqrt(eval_res);
}

ostream& ExpressionSqrt::Display(ostream& os) const {
  os << "sqrt(" << get_expression() << ")";
  return os;
}

ExpressionPow::ExpressionPow(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Pow,
                           hash_combine(e1.get_hash(), e2.get_hash()), e1, e2} {
}

static bool is_int(double const v) {
  double intpart;  // dummy variable
  return modf(v, &intpart) == 0.0;
}

void ExpressionPow::check_domain(double const v1, double const v2) {
  if (std::isfinite(v1) && (v1 < 0.0) && std::isfinite(v2) && !is_int(v2)) {
    ostringstream oss;
    oss << "pow(" << v1 << ", " << v2
        << ") : numerical argument out of domain. " << v1
        << " is finite negative and " << v2 << " is finite non-integer."
        << endl;
    throw domain_error(oss.str());
  }
}

double ExpressionPow::Evaluate(const Environment& env) const {
  double const v1{get_1st_expression().Evaluate(env)};
  double const v2{get_2nd_expression().Evaluate(env)};
  check_domain(v1, v2);
  return std::pow(v1, v2);
}

ostream& ExpressionPow::Display(ostream& os) const {
  os << "pow(" << get_1st_expression() << ", " << get_2nd_expression() << ")";
  return os;
}

ExpressionSin::ExpressionSin(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Sin, e.get_hash(), e} {}

double ExpressionSin::Evaluate(const Environment& env) const {
  return std::sin(get_expression().Evaluate(env));
}

ostream& ExpressionSin::Display(ostream& os) const {
  os << "sin(" << get_expression() << ")";
  return os;
}

ExpressionCos::ExpressionCos(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Cos, e.get_hash(), e} {}

double ExpressionCos::Evaluate(const Environment& env) const {
  return std::cos(get_expression().Evaluate(env));
}

ostream& ExpressionCos::Display(ostream& os) const {
  os << "cos(" << get_expression() << ")";
  return os;
}

ExpressionTan::ExpressionTan(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Tan, e.get_hash(), e} {}

double ExpressionTan::Evaluate(const Environment& env) const {
  return std::tan(get_expression().Evaluate(env));
}

ostream& ExpressionTan::Display(ostream& os) const {
  os << "tan(" << get_expression() << ")";
  return os;
}

ExpressionAsin::ExpressionAsin(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Asin, e.get_hash(), e} {}

void ExpressionAsin::check_domain(double const v) {
  if (!((v >= -1.0) && (v <= 1.0))) {
    ostringstream oss;
    oss << "asin(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

double ExpressionAsin::Evaluate(const Environment& env) const {
  double const eval_res{get_expression().Evaluate(env)};
  check_domain(eval_res);
  return std::asin(eval_res);
}

ostream& ExpressionAsin::Display(ostream& os) const {
  os << "asin(" << get_expression() << ")";
  return os;
}

ExpressionAcos::ExpressionAcos(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Acos, e.get_hash(), e} {}

void ExpressionAcos::check_domain(double const v) {
  if (!((v >= -1.0) && (v <= 1.0))) {
    ostringstream oss;
    oss << "acos(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

double ExpressionAcos::Evaluate(const Environment& env) const {
  double const eval_res{get_expression().Evaluate(env)};
  check_domain(eval_res);
  return std::acos(eval_res);
}

ostream& ExpressionAcos::Display(ostream& os) const {
  os << "acos(" << get_expression() << ")";
  return os;
}

ExpressionAtan::ExpressionAtan(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Atan, e.get_hash(), e} {}

double ExpressionAtan::Evaluate(const Environment& env) const {
  return std::atan(get_expression().Evaluate(env));
}

ostream& ExpressionAtan::Display(ostream& os) const {
  os << "atan(" << get_expression() << ")";
  return os;
}

ExpressionAtan2::ExpressionAtan2(const Expression& e1, const Expression& e2)
    : BinaryExpressionCell{ExpressionKind::Atan2,
                           hash_combine(e1.get_hash(), e2.get_hash()), e1, e2} {
}

double ExpressionAtan2::Evaluate(const Environment& env) const {
  return std::atan2(get_1st_expression().Evaluate(env),
                    get_2nd_expression().Evaluate(env));
}

ostream& ExpressionAtan2::Display(ostream& os) const {
  os << "atan2(" << get_1st_expression() << ", " << get_2nd_expression() << ")";
  return os;
}

ExpressionSinh::ExpressionSinh(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Sinh, e.get_hash(), e} {}

double ExpressionSinh::Evaluate(const Environment& env) const {
  return std::sinh(get_expression().Evaluate(env));
}

ostream& ExpressionSinh::Display(ostream& os) const {
  os << "sinh(" << get_expression() << ")";
  return os;
}
ExpressionCosh::ExpressionCosh(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Cosh, e.get_hash(), e} {}

double ExpressionCosh::Evaluate(const Environment& env) const {
  return std::cosh(get_expression().Evaluate(env));
}
ostream& ExpressionCosh::Display(ostream& os) const {
  os << "cosh(" << get_expression() << ")";
  return os;
}

ExpressionTanh::ExpressionTanh(const Expression& e)
    : UnaryExpressionCell{ExpressionKind::Tanh, e.get_hash(), e} {}

double ExpressionTanh::Evaluate(const Environment& env) const {
  return std::tanh(get_expression().Evaluate(env));
}

ostream& ExpressionTanh::Display(ostream& os) const {
  os << "tanh(" << get_expression() << ")";
  return os;
}

ostream& operator<<(ostream& os, const Expression& e) {
  DRAKE_ASSERT(e.ptr_ != nullptr);
  return e.ptr_->Display(os);
}

Expression log(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionLog::check_domain(v);
    return Expression{std::log(v)};
  }
  return Expression{make_shared<ExpressionLog>(e)};
}

Expression abs(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::fabs(v)};
  }
  return Expression{make_shared<ExpressionAbs>(e)};
}

Expression exp(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::exp(v)};
  }
  return Expression{make_shared<ExpressionExp>(e)};
}

Expression sqrt(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
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
    double const v1 =
        static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value();
    double const v2 =
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value();
    ExpressionPow::check_domain(v1, v2);
    return Expression{std::pow(v1, v2)};
  }
  return Expression{make_shared<ExpressionPow>(e1, e2)};
}

Expression pow(const Expression& e1, double const v2) {
  return pow(e1, Expression{v2});
}

Expression pow(double const v1, const Expression& e2) {
  return pow(Expression{v1}, e2);
}

Expression sin(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::sin(v)};
  }
  return Expression{make_shared<ExpressionSin>(e)};
}

Expression cos(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::cos(v)};
  }

  return Expression{make_shared<ExpressionCos>(e)};
}

Expression tan(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::tan(v)};
  }
  return Expression{make_shared<ExpressionTan>(e)};
}

Expression asin(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionAsin::check_domain(v);
    return Expression{std::asin(v)};
  }
  return Expression{make_shared<ExpressionAsin>(e)};
}

Expression acos(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionAcos::check_domain(v);
    return Expression{std::acos(v)};
  }
  return Expression{make_shared<ExpressionAcos>(e)};
}

Expression atan(const Expression& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::atan(v)};
  }
  return Expression{make_shared<ExpressionAtan>(e)};
}

Expression atan2(const Expression& e1, const Expression& e2) {
  // simplification
  if (e1.get_kind() == ExpressionKind::Constant &&
      e2.get_kind() == ExpressionKind::Constant) {
    double const v1 =
        static_pointer_cast<ExpressionConstant>(e1.ptr_)->get_value();
    double const v2 =
        static_pointer_cast<ExpressionConstant>(e2.ptr_)->get_value();
    return Expression{std::atan2(v1, v2)};
  }
  return Expression{make_shared<ExpressionAtan2>(e1, e2)};
}

Expression atan2(const Expression& e1, double const v2) {
  return atan2(e1, Expression{v2});
}

Expression atan2(double const v1, const Expression& e2) {
  return atan2(Expression{v1}, e2);
}

Expression sinh(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::sinh(v)};
  }
  return Expression{make_shared<ExpressionSinh>(e)};
}

Expression cosh(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::cosh(v)};
  }
  return Expression{make_shared<ExpressionCosh>(e)};
}

Expression tanh(const Expression& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::tanh(v)};
  }
  return Expression{make_shared<ExpressionTanh>(e)};
}
}  // namespace symbolic
}  // namespace drake
