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
Expression::Expression(shared_ptr<ExpressionCell> const ptr) : ptr_{ptr} {}

ExpressionKind Expression::get_kind() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->get_kind();
}
size_t Expression::get_hash() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->get_hash();
}

Expression Expression::Zero() {
  static Expression const zero{0.0};
  return zero;
}

Expression Expression::One() {
  static Expression const one{1.0};
  return one;
}

Expression Expression::Pi() {
  static Expression const pi{M_PI};
  return pi;
}

Expression Expression::E() {
  static Expression const e{M_E};
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
  return ptr_->Evaluate(env);
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
  *this += Expression{1.0};
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
  if (rhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value() == 0.0) {
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
  *this -= Expression{1.0};
  return *this;
}

Expression Expression::operator--(int) {
  Expression const copy(*this);
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
  if (rhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value() == 1.0) {
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
    : ExpressionCell{ExpressionKind::Constant, hash<double>{}(v)}, v_{v} {}

Variables ExpressionConstant::GetVariables() const { return Variables{}; }

bool ExpressionConstant::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Constant) {
    return false;
  }
  return v_ == static_cast<const ExpressionConstant&>(e).v_;
}

double ExpressionConstant::Evaluate(const Environment& env) const { return v_; }

ostream& ExpressionConstant::Display(ostream& os) const {
  ostringstream oss;
  oss << setprecision(numeric_limits<double>::max_digits10) << v_;
  os << oss.str();
  return os;
}

ExpressionNeg::ExpressionNeg(const Expression& e)
    : ExpressionCell{ExpressionKind::Neg, e.get_hash()}, e_{e} {}

Variables ExpressionNeg::GetVariables() const { return e_.GetVariables(); }

bool ExpressionNeg::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Neg) {
    return false;
  }
  const ExpressionNeg& e_neg{static_cast<const ExpressionNeg&>(e)};
  return e_.EqualTo(e_neg.e_);
}

double ExpressionNeg::Evaluate(const Environment& env) const {
  return -e_.Evaluate(env);
}
ostream& ExpressionNeg::Display(ostream& os) const {
  os << "-(" << e_ << ")";
  return os;
}

ExpressionAdd::ExpressionAdd(const Expression& e1, const Expression& e2)
    : ExpressionCell{ExpressionKind::Add,
                     hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

Variables ExpressionAdd::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool ExpressionAdd::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Add) {
    return false;
  }
  const ExpressionAdd& e_add{static_cast<const ExpressionAdd&>(e)};
  return e1_.EqualTo(e_add.e1_) && e2_.EqualTo(e_add.e2_);
}

double ExpressionAdd::Evaluate(const Environment& env) const {
  return e1_.Evaluate(env) + e2_.Evaluate(env);
}

ostream& ExpressionAdd::Display(ostream& os) const {
  os << "(" << e1_ << " + " << e2_ << ")";
  return os;
}

ExpressionSub::ExpressionSub(const Expression& e1, const Expression& e2)
    : ExpressionCell{ExpressionKind::Sub,
                     hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

Variables ExpressionSub::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool ExpressionSub::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Sub) {
    return false;
  }
  const ExpressionSub& e_sub{static_cast<const ExpressionSub&>(e)};
  return e1_.EqualTo(e_sub.e1_) && e2_.EqualTo(e_sub.e2_);
}

double ExpressionSub::Evaluate(const Environment& env) const {
  return e1_.Evaluate(env) - e2_.Evaluate(env);
}

ostream& ExpressionSub::Display(ostream& os) const {
  os << "(" << e1_ << " - " << e2_ << ")";
  return os;
}

ExpressionMul::ExpressionMul(const Expression& e1, const Expression& e2)
    : ExpressionCell{ExpressionKind::Mul,
                     hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

Variables ExpressionMul::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool ExpressionMul::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Mul) {
    return false;
  }
  const ExpressionMul& e_mul{static_cast<const ExpressionMul&>(e)};
  return e1_.EqualTo(e_mul.e1_) && e2_.EqualTo(e_mul.e2_);
}

double ExpressionMul::Evaluate(const Environment& env) const {
  return e1_.Evaluate(env) * e2_.Evaluate(env);
}

ostream& ExpressionMul::Display(ostream& os) const {
  os << "(" << e1_ << " * " << e2_ << ")";
  return os;
}

ExpressionDiv::ExpressionDiv(const Expression& e1, const Expression& e2)
    : ExpressionCell{ExpressionKind::Div,
                     hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

Variables ExpressionDiv::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool ExpressionDiv::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Div) {
    return false;
  }
  const ExpressionDiv& e_div{static_cast<const ExpressionDiv&>(e)};
  return e1_.EqualTo(e_div.e1_) && e2_.EqualTo(e_div.e2_);
}

double ExpressionDiv::Evaluate(const Environment& env) const {
  double const rhs{e2_.Evaluate(env)};
  if (rhs == 0.0) {
    ostringstream oss;
    oss << "Division by zero: ";
    this->Display(oss) << endl;
    oss << "under the following environment:" << endl << env << endl;
    throw runtime_error(oss.str());
  }
  return e1_.Evaluate(env) / rhs;
}

ostream& ExpressionDiv::Display(ostream& os) const {
  os << "(" << e1_ << " / " << e2_ << ")";
  return os;
}

ExpressionLog::ExpressionLog(const Expression& e)
    : ExpressionCell{ExpressionKind::Log, e.get_hash()}, e_{e} {}

void ExpressionLog::check_domain(double const v) {
  if (v < 0) {
    ostringstream oss;
    oss << "log(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

Variables ExpressionLog::GetVariables() const { return e_.GetVariables(); }

bool ExpressionLog::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Log) {
    return false;
  }
  const ExpressionLog& e_log{static_cast<const ExpressionLog&>(e)};
  return e_.EqualTo(e_log.e_);
}

double ExpressionLog::Evaluate(const Environment& env) const {
  double const eval_res{e_.Evaluate(env)};
  check_domain(eval_res);
  return std::log(eval_res);
}

ostream& ExpressionLog::Display(ostream& os) const {
  os << "log(" << e_ << ")";
  return os;
}

ExpressionAbs::ExpressionAbs(const Expression& e)
    : ExpressionCell{ExpressionKind::Abs, e.get_hash()}, e_{e} {}

Variables ExpressionAbs::GetVariables() const { return e_.GetVariables(); }

bool ExpressionAbs::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Abs) {
    return false;
  }
  const ExpressionAbs& e_abs{static_cast<const ExpressionAbs&>(e)};
  return e_.EqualTo(e_abs.e_);
}

double ExpressionAbs::Evaluate(const Environment& env) const {
  double const eval_res{e_.Evaluate(env)};
  return std::fabs(eval_res);
}

ostream& ExpressionAbs::Display(ostream& os) const {
  os << "abs(" << e_ << ")";
  return os;
}

ExpressionExp::ExpressionExp(const Expression& e)
    : ExpressionCell{ExpressionKind::Exp, e.get_hash()}, e_{e} {}

Variables ExpressionExp::GetVariables() const { return e_.GetVariables(); }

bool ExpressionExp::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Exp) {
    return false;
  }
  const ExpressionExp& e_exp{static_cast<const ExpressionExp&>(e)};
  return e_.EqualTo(e_exp.e_);
}

double ExpressionExp::Evaluate(const Environment& env) const {
  return std::exp(e_.Evaluate(env));
}

ostream& ExpressionExp::Display(ostream& os) const {
  os << "exp(" << e_ << ")";
  return os;
}

ExpressionSqrt::ExpressionSqrt(const Expression& e)
    : ExpressionCell{ExpressionKind::Sqrt, e.get_hash()}, e_{e} {}

void ExpressionSqrt::check_domain(double const v) {
  if (v < 0) {
    ostringstream oss;
    oss << "sqrt(" << v << ") : numerical argument out of domain. " << v
        << " is not in [0, +oo)" << endl;
    throw domain_error(oss.str());
  }
}

Variables ExpressionSqrt::GetVariables() const { return e_.GetVariables(); }

bool ExpressionSqrt::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Sqrt) {
    return false;
  }
  const ExpressionSqrt& e_sqrt{static_cast<const ExpressionSqrt&>(e)};
  return e_.EqualTo(e_sqrt.e_);
}

double ExpressionSqrt::Evaluate(const Environment& env) const {
  double const eval_res{e_.Evaluate(env)};
  check_domain(eval_res);
  return std::sqrt(eval_res);
}

ostream& ExpressionSqrt::Display(ostream& os) const {
  os << "sqrt(" << e_ << ")";
  return os;
}

ExpressionPow::ExpressionPow(const Expression& e1, const Expression& e2)
    : ExpressionCell{ExpressionKind::Pow,
                     hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

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

Variables ExpressionPow::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool ExpressionPow::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Pow) {
    return false;
  }
  const ExpressionPow& e_pow{static_cast<const ExpressionPow&>(e)};
  return e1_.EqualTo(e_pow.e1_) && e2_.EqualTo(e_pow.e2_);
}

double ExpressionPow::Evaluate(const Environment& env) const {
  double const v1{e1_.Evaluate(env)};
  double const v2{e2_.Evaluate(env)};
  check_domain(v1, v2);
  return std::pow(v1, v2);
}

ostream& ExpressionPow::Display(ostream& os) const {
  os << "pow(" << e1_ << ", " << e2_ << ")";
  return os;
}

ExpressionSin::ExpressionSin(const Expression& e)
    : ExpressionCell{ExpressionKind::Sin, e.get_hash()}, e_{e} {}

Variables ExpressionSin::GetVariables() const { return e_.GetVariables(); }

bool ExpressionSin::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Sin) {
    return false;
  }
  const ExpressionSin& e_sin{static_cast<const ExpressionSin&>(e)};
  return e_.EqualTo(e_sin.e_);
}

double ExpressionSin::Evaluate(const Environment& env) const {
  return std::sin(e_.Evaluate(env));
}

ostream& ExpressionSin::Display(ostream& os) const {
  os << "sin(" << e_ << ")";
  return os;
}

ExpressionCos::ExpressionCos(const Expression& e)
    : ExpressionCell{ExpressionKind::Cos, e.get_hash()}, e_{e} {}

Variables ExpressionCos::GetVariables() const { return e_.GetVariables(); }

bool ExpressionCos::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Cos) {
    return false;
  }
  const ExpressionCos& e_cos{static_cast<const ExpressionCos&>(e)};
  return e_.EqualTo(e_cos.e_);
}

double ExpressionCos::Evaluate(const Environment& env) const {
  return std::cos(e_.Evaluate(env));
}

ostream& ExpressionCos::Display(ostream& os) const {
  os << "cos(" << e_ << ")";
  return os;
}

ExpressionTan::ExpressionTan(const Expression& e)
    : ExpressionCell{ExpressionKind::Tan, e.get_hash()}, e_{e} {}

Variables ExpressionTan::GetVariables() const { return e_.GetVariables(); }

bool ExpressionTan::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Tan) {
    return false;
  }
  const ExpressionTan& e_tan{static_cast<const ExpressionTan&>(e)};
  return e_.EqualTo(e_tan.e_);
}

double ExpressionTan::Evaluate(const Environment& env) const {
  return std::tan(e_.Evaluate(env));
}

ostream& ExpressionTan::Display(ostream& os) const {
  os << "tan(" << e_ << ")";
  return os;
}

ExpressionAsin::ExpressionAsin(const Expression& e)
    : ExpressionCell{ExpressionKind::Asin, e.get_hash()}, e_{e} {}

void ExpressionAsin::check_domain(double const v) {
  if (v < -1.0 || v > 1.0) {
    ostringstream oss;
    oss << "asin(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

Variables ExpressionAsin::GetVariables() const { return e_.GetVariables(); }

bool ExpressionAsin::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Asin) {
    return false;
  }
  const ExpressionAsin& e_asin{static_cast<const ExpressionAsin&>(e)};
  return e_.EqualTo(e_asin.e_);
}

double ExpressionAsin::Evaluate(const Environment& env) const {
  double const eval_res{e_.Evaluate(env)};
  check_domain(eval_res);
  return std::asin(eval_res);
}

ostream& ExpressionAsin::Display(ostream& os) const {
  os << "asin(" << e_ << ")";
  return os;
}

ExpressionAcos::ExpressionAcos(const Expression& e)
    : ExpressionCell{ExpressionKind::Acos, e.get_hash()}, e_{e} {}

void ExpressionAcos::check_domain(double const v) {
  if (v < -1.0 || v > 1.0) {
    ostringstream oss;
    oss << "acos(" << v << ") : numerical argument out of domain. " << v
        << " is not in [-1.0, +1.0]" << endl;
    throw domain_error(oss.str());
  }
}

Variables ExpressionAcos::GetVariables() const { return e_.GetVariables(); }

bool ExpressionAcos::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Acos) {
    return false;
  }
  const ExpressionAcos& e_acos{static_cast<const ExpressionAcos&>(e)};
  return e_.EqualTo(e_acos.e_);
}

double ExpressionAcos::Evaluate(const Environment& env) const {
  double const eval_res{e_.Evaluate(env)};
  check_domain(eval_res);
  return std::acos(eval_res);
}

ostream& ExpressionAcos::Display(ostream& os) const {
  os << "acos(" << e_ << ")";
  return os;
}

ExpressionAtan::ExpressionAtan(const Expression& e)
    : ExpressionCell{ExpressionKind::Atan, e.get_hash()}, e_{e} {}

Variables ExpressionAtan::GetVariables() const { return e_.GetVariables(); }

bool ExpressionAtan::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Atan) {
    return false;
  }
  const ExpressionAtan& e_atan{static_cast<const ExpressionAtan&>(e)};
  return e_.EqualTo(e_atan.e_);
}

double ExpressionAtan::Evaluate(const Environment& env) const {
  return std::atan(e_.Evaluate(env));
}

ostream& ExpressionAtan::Display(ostream& os) const {
  os << "atan(" << e_ << ")";
  return os;
}

ExpressionAtan2::ExpressionAtan2(const Expression& e1, const Expression& e2)
    : ExpressionCell{ExpressionKind::Atan2,
                     hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

Variables ExpressionAtan2::GetVariables() const {
  Variables ret{e1_.GetVariables()};
  Variables const res_from_e2{e2_.GetVariables()};
  ret.insert(res_from_e2.begin(), res_from_e2.end());
  return ret;
}

bool ExpressionAtan2::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Atan2) {
    return false;
  }
  ExpressionAtan2 const& e_atan2{static_cast<ExpressionAtan2 const&>(e)};
  return e1_.EqualTo(e_atan2.e1_) && e2_.EqualTo(e_atan2.e2_);
}

double ExpressionAtan2::Evaluate(const Environment& env) const {
  return std::atan2(e1_.Evaluate(env), e2_.Evaluate(env));
}

ostream& ExpressionAtan2::Display(ostream& os) const {
  os << "atan2(" << e1_ << ", " << e2_ << ")";
  return os;
}

ExpressionSinh::ExpressionSinh(const Expression& e)
    : ExpressionCell{ExpressionKind::Sinh, e.get_hash()}, e_{e} {}

Variables ExpressionSinh::GetVariables() const { return e_.GetVariables(); }

bool ExpressionSinh::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Sinh) {
    return false;
  }
  const ExpressionSinh& e_sinh{static_cast<const ExpressionSinh&>(e)};
  return e_.EqualTo(e_sinh.e_);
}

double ExpressionSinh::Evaluate(const Environment& env) const {
  return std::sinh(e_.Evaluate(env));
}

ostream& ExpressionSinh::Display(ostream& os) const {
  os << "sinh(" << e_ << ")";
  return os;
}
ExpressionCosh::ExpressionCosh(const Expression& e)
    : ExpressionCell{ExpressionKind::Cosh, e.get_hash()}, e_{e} {}

Variables ExpressionCosh::GetVariables() const { return e_.GetVariables(); }

bool ExpressionCosh::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Cosh) {
    return false;
  }
  const ExpressionCosh& e_cosh{static_cast<const ExpressionCosh&>(e)};
  return e_.EqualTo(e_cosh.e_);
}

double ExpressionCosh::Evaluate(const Environment& env) const {
  return std::cosh(e_.Evaluate(env));
}
ostream& ExpressionCosh::Display(ostream& os) const {
  os << "cosh(" << e_ << ")";
  return os;
}

ExpressionTanh::ExpressionTanh(const Expression& e)
    : ExpressionCell{ExpressionKind::Tanh, e.get_hash()}, e_{e} {}

Variables ExpressionTanh::GetVariables() const { return e_.GetVariables(); }

bool ExpressionTanh::EqualTo(const ExpressionCell& e) const {
  if (e.get_kind() != ExpressionKind::Tanh) {
    return false;
  }
  const ExpressionTanh& e_tanh{static_cast<const ExpressionTanh&>(e)};
  return e_.EqualTo(e_tanh.e_);
}

double ExpressionTanh::Evaluate(const Environment& env) const {
  return std::tanh(e_.Evaluate(env));
}

ostream& ExpressionTanh::Display(ostream& os) const {
  os << "tanh(" << e_ << ")";
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
