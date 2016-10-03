#include "drake/common/symbolic_expression.h"

#include <cmath>
#include <functional>
#include <iostream>
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
using std::runtime_error;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;
using std::prev;

Expression::Expression(Variable const& name)
    : ptr_{make_shared<ExpressionVar>(name)} {}
Expression::Expression(double const d)
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

bool Expression::EqualTo(Expression const& e) const {
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

double Expression::Evaluate(Environment const& env) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->Evaluate(env);
}

Expression operator+(Expression lhs, Expression const& rhs) {
  lhs += rhs;
  return lhs;
}

Expression operator+(double const lhs, Expression const& rhs) {
  return Expression{lhs} + rhs;
}

Expression operator+(Expression lhs, double const rhs) {
  lhs += rhs;
  return lhs;
}

Expression& operator+=(Expression& lhs, Expression const& rhs) {
  // simplification #1 : 0 + x = x
  if (lhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value() == 0.0) {
    lhs = rhs;
    return lhs;
  }
  // simplification #2 : x + 0 = x
  if (rhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value() == 0.0) {
    return lhs;
  }
  // simplification #3 : Expression(c1) + Expression(c2) = Expression(c1 + c2)
  if (lhs.get_kind() == ExpressionKind::Constant &&
      rhs.get_kind() == ExpressionKind::Constant) {
    double const v1 =
        static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value();
    double const v2 =
        static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value();
    lhs = Expression{v1 + v2};
    return lhs;
  }
  lhs.ptr_ = make_shared<ExpressionAdd>(lhs, rhs);
  return lhs;
}

Expression& operator+=(Expression& lhs, double const rhs) {
  lhs += Expression{rhs};
  return lhs;
}

Expression& Expression::operator++() {
  *this += Expression{1.0};
  return *this;
}

/// Postfix increment
Expression Expression::operator++(int) {
  Expression copy(*this);
  ++*this;
  return copy;
}

Expression operator-(Expression lhs, Expression const& rhs) {
  lhs -= rhs;
  return lhs;
}

Expression operator-(double const lhs, Expression const& rhs) {
  Expression ret = Expression{lhs};
  ret -= rhs;
  return ret;
}

Expression operator-(Expression lhs, double const rhs) {
  lhs -= Expression{rhs};
  return lhs;
}

Expression& operator-=(Expression& lhs, Expression const& rhs) {
  // simplification #1 : E - E = 0
  if (lhs.EqualTo(rhs)) {
    lhs = Expression::Zero();
    return lhs;
  }
  // simplification #2 : x - 0 = x
  if (rhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value() == 0.0) {
    return lhs;
  }
  // simplification #2 : Expression(c1) - Expression(c2) = Expression(c1 - c2)
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

/// Postfix increment
Expression Expression::operator--(int) {
  Expression const copy(*this);
  --*this;
  return copy;
}

Expression operator*(Expression lhs, Expression const& rhs) {
  lhs *= rhs;
  return lhs;
}

Expression operator*(double const lhs, Expression const& rhs) {
  return Expression{lhs} * rhs;
}

Expression operator*(Expression lhs, double const rhs) {
  lhs *= Expression{rhs};
  return lhs;
}

Expression& operator*=(Expression& lhs, Expression const& rhs) {
  // simplification #1 : 1 * x = x
  if (lhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value() == 1.0) {
    lhs = rhs;
    return lhs;
  }
  // simplification #2 : x * 1 = x
  if (rhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value() == 1.0) {
    return lhs;
  }
  // simplification #3 : 0 * x = 0
  if (lhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(lhs.ptr_)->get_value() == 0.0) {
    return lhs;
  }
  // simplification #4 : x * 0 = 0
  if (rhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value() == 0.0) {
    lhs = Expression{0.0};
    return lhs;
  }
  // simplification #5 : Expression(c1) * Expression(c2) = Expression(c1 * c2)
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

Expression operator/(Expression lhs, Expression const& rhs) {
  lhs /= rhs;
  return lhs;
}

Expression operator/(double const lhs, Expression const& rhs) {
  Expression ret = Expression{lhs};
  ret /= rhs;
  return ret;
}

Expression operator/(Expression lhs, double const rhs) {
  lhs /= Expression{rhs};
  return lhs;
}

Expression& operator/=(Expression& lhs, Expression const& rhs) {
  // simplification #1 : x / 1 = x
  if (rhs.get_kind() == ExpressionKind::Constant &&
      static_pointer_cast<ExpressionConstant>(rhs.ptr_)->get_value() == 1.0) {
    return lhs;
  }
  // simplification #2 : Expression(c1) / Expression(c2) = Expression(c1 / c2)
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
  // simplification #3 : E / E = 1
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

ExpressionVar::ExpressionVar(Variable const& v)
    : ExpressionCell{ExpressionKind::Var, hash<Variable>{}(v)}, var_{v} {}

Variables ExpressionVar::GetVariables() const { return {get_variable()}; }

bool ExpressionVar::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Var) {
    return false;
  }
  return var_ == static_cast<ExpressionVar const&>(e).var_;
}

double ExpressionVar::Evaluate(Environment const& env) const {
  Environment::const_iterator const it = env.find(var_);
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

bool ExpressionConstant::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Constant) {
    return false;
  }
  return v_ == static_cast<ExpressionConstant const&>(e).v_;
}

double ExpressionConstant::Evaluate(Environment const& env) const { return v_; }

ostream& ExpressionConstant::Display(ostream& os) const {
  os << v_;
  return os;
}

ExpressionNeg::ExpressionNeg(Expression const& e)
    : ExpressionCell{ExpressionKind::Neg, e.get_hash()}, e_{e} {}

Variables ExpressionNeg::GetVariables() const { return e_.GetVariables(); }

bool ExpressionNeg::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Neg) {
    return false;
  }
  ExpressionNeg const& e_neg = static_cast<ExpressionNeg const&>(e);
  return e_.EqualTo(e_neg.e_);
}

double ExpressionNeg::Evaluate(Environment const& env) const {
  return -e_.Evaluate(env);
}
ostream& ExpressionNeg::Display(ostream& os) const {
  os << "-(" << e_ << ")";
  return os;
}

ExpressionAdd::ExpressionAdd(Expression const& e1, Expression const& e2)
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

bool ExpressionAdd::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Add) {
    return false;
  }
  ExpressionAdd const& e_add = static_cast<ExpressionAdd const&>(e);
  return e1_.EqualTo(e_add.e1_) && e2_.EqualTo(e_add.e2_);
}

double ExpressionAdd::Evaluate(Environment const& env) const {
  return e1_.Evaluate(env) + e2_.Evaluate(env);
}

ostream& ExpressionAdd::Display(ostream& os) const {
  os << "(" << e1_ << " + " << e2_ << ")";
  return os;
}

ExpressionSub::ExpressionSub(Expression const& e1, Expression const& e2)
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

bool ExpressionSub::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Sub) {
    return false;
  }
  ExpressionSub const& e_sub = static_cast<ExpressionSub const&>(e);
  return e1_.EqualTo(e_sub.e1_) && e2_.EqualTo(e_sub.e2_);
}

double ExpressionSub::Evaluate(Environment const& env) const {
  return e1_.Evaluate(env) - e2_.Evaluate(env);
}

ostream& ExpressionSub::Display(ostream& os) const {
  os << "(" << e1_ << " - " << e2_ << ")";
  return os;
}

ExpressionMul::ExpressionMul(Expression const& e1, Expression const& e2)
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

bool ExpressionMul::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Mul) {
    return false;
  }
  ExpressionMul const& e_mul = static_cast<ExpressionMul const&>(e);
  return e1_.EqualTo(e_mul.e1_) && e2_.EqualTo(e_mul.e2_);
}

double ExpressionMul::Evaluate(Environment const& env) const {
  return e1_.Evaluate(env) * e2_.Evaluate(env);
}

ostream& ExpressionMul::Display(ostream& os) const {
  os << "(" << e1_ << " * " << e2_ << ")";
  return os;
}

ExpressionDiv::ExpressionDiv(Expression const& e1, Expression const& e2)
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

bool ExpressionDiv::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Div) {
    return false;
  }
  ExpressionDiv const& e_div = static_cast<ExpressionDiv const&>(e);
  return e1_.EqualTo(e_div.e1_) && e2_.EqualTo(e_div.e2_);
}

double ExpressionDiv::Evaluate(Environment const& env) const {
  double const rhs = e2_.Evaluate(env);
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

ExpressionLog::ExpressionLog(Expression const& e)
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

bool ExpressionLog::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Log) {
    return false;
  }
  ExpressionLog const& e_log = static_cast<ExpressionLog const&>(e);
  return e_.EqualTo(e_log.e_);
}

double ExpressionLog::Evaluate(Environment const& env) const {
  double const eval_res = e_.Evaluate(env);
  check_domain(eval_res);
  return std::log(eval_res);
}

ostream& ExpressionLog::Display(ostream& os) const {
  os << "log(" << e_ << ")";
  return os;
}

ExpressionAbs::ExpressionAbs(Expression const& e)
    : ExpressionCell{ExpressionKind::Abs, e.get_hash()}, e_{e} {}

Variables ExpressionAbs::GetVariables() const { return e_.GetVariables(); }

bool ExpressionAbs::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Abs) {
    return false;
  }
  ExpressionAbs const& e_abs = static_cast<ExpressionAbs const&>(e);
  return e_.EqualTo(e_abs.e_);
}

double ExpressionAbs::Evaluate(Environment const& env) const {
  double const eval_res = e_.Evaluate(env);
  return std::fabs(eval_res);
}

ostream& ExpressionAbs::Display(ostream& os) const {
  os << "abs(" << e_ << ")";
  return os;
}

ExpressionExp::ExpressionExp(Expression const& e)
    : ExpressionCell{ExpressionKind::Exp, e.get_hash()}, e_{e} {}

Variables ExpressionExp::GetVariables() const { return e_.GetVariables(); }

bool ExpressionExp::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Exp) {
    return false;
  }
  ExpressionExp const& e_exp = static_cast<ExpressionExp const&>(e);
  return e_.EqualTo(e_exp.e_);
}

double ExpressionExp::Evaluate(Environment const& env) const {
  return std::exp(e_.Evaluate(env));
}

ostream& ExpressionExp::Display(ostream& os) const {
  os << "exp(" << e_ << ")";
  return os;
}

ExpressionSqrt::ExpressionSqrt(Expression const& e)
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

bool ExpressionSqrt::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Sqrt) {
    return false;
  }
  ExpressionSqrt const& e_sqrt = static_cast<ExpressionSqrt const&>(e);
  return e_.EqualTo(e_sqrt.e_);
}

double ExpressionSqrt::Evaluate(Environment const& env) const {
  double const eval_res = e_.Evaluate(env);
  check_domain(eval_res);
  return std::sqrt(eval_res);
}

ostream& ExpressionSqrt::Display(ostream& os) const {
  os << "sqrt(" << e_ << ")";
  return os;
}

ExpressionPow::ExpressionPow(Expression const& e1, Expression const& e2)
    : ExpressionCell{ExpressionKind::Pow,
                     hash_combine(e1.get_hash(), e2.get_hash())},
      e1_{e1},
      e2_{e2} {}

static bool is_int(double const v) {
  static double intpart;  // dummy variable
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

bool ExpressionPow::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Pow) {
    return false;
  }
  ExpressionPow const& e_pow = static_cast<ExpressionPow const&>(e);
  return e1_.EqualTo(e_pow.e1_) && e2_.EqualTo(e_pow.e2_);
}

double ExpressionPow::Evaluate(Environment const& env) const {
  double const v1 = e1_.Evaluate(env);
  double const v2 = e2_.Evaluate(env);
  check_domain(v1, v2);
  return std::pow(v1, v2);
}

ostream& ExpressionPow::Display(ostream& os) const {
  os << "pow(" << e1_ << ", " << e2_ << ")";
  return os;
}

ExpressionSin::ExpressionSin(Expression const& e)
    : ExpressionCell{ExpressionKind::Sin, e.get_hash()}, e_{e} {}

Variables ExpressionSin::GetVariables() const { return e_.GetVariables(); }

bool ExpressionSin::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Sin) {
    return false;
  }
  ExpressionSin const& e_sin = static_cast<ExpressionSin const&>(e);
  return e_.EqualTo(e_sin.e_);
}

double ExpressionSin::Evaluate(Environment const& env) const {
  return std::sin(e_.Evaluate(env));
}

ostream& ExpressionSin::Display(ostream& os) const {
  os << "sin(" << e_ << ")";
  return os;
}

ExpressionCos::ExpressionCos(Expression const& e)
    : ExpressionCell{ExpressionKind::Cos, e.get_hash()}, e_{e} {}

Variables ExpressionCos::GetVariables() const { return e_.GetVariables(); }

bool ExpressionCos::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Cos) {
    return false;
  }
  ExpressionCos const& e_cos = static_cast<ExpressionCos const&>(e);
  return e_.EqualTo(e_cos.e_);
}

double ExpressionCos::Evaluate(Environment const& env) const {
  return std::cos(e_.Evaluate(env));
}

ostream& ExpressionCos::Display(ostream& os) const {
  os << "cos(" << e_ << ")";
  return os;
}

ExpressionTan::ExpressionTan(Expression const& e)
    : ExpressionCell{ExpressionKind::Tan, e.get_hash()}, e_{e} {}

Variables ExpressionTan::GetVariables() const { return e_.GetVariables(); }

bool ExpressionTan::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Tan) {
    return false;
  }
  ExpressionTan const& e_tan = static_cast<ExpressionTan const&>(e);
  return e_.EqualTo(e_tan.e_);
}

double ExpressionTan::Evaluate(Environment const& env) const {
  return std::tan(e_.Evaluate(env));
}

ostream& ExpressionTan::Display(ostream& os) const {
  os << "tan(" << e_ << ")";
  return os;
}

ExpressionAsin::ExpressionAsin(Expression const& e)
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

bool ExpressionAsin::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Asin) {
    return false;
  }
  ExpressionAsin const& e_asin = static_cast<ExpressionAsin const&>(e);
  return e_.EqualTo(e_asin.e_);
}

double ExpressionAsin::Evaluate(Environment const& env) const {
  double const eval_res = e_.Evaluate(env);
  check_domain(eval_res);
  return std::asin(eval_res);
}

ostream& ExpressionAsin::Display(ostream& os) const {
  os << "asin(" << e_ << ")";
  return os;
}

ExpressionAcos::ExpressionAcos(Expression const& e)
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

bool ExpressionAcos::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Acos) {
    return false;
  }
  ExpressionAcos const& e_acos = static_cast<ExpressionAcos const&>(e);
  return e_.EqualTo(e_acos.e_);
}

double ExpressionAcos::Evaluate(Environment const& env) const {
  double const eval_res = e_.Evaluate(env);
  check_domain(eval_res);
  return std::acos(eval_res);
}

ostream& ExpressionAcos::Display(ostream& os) const {
  os << "acos(" << e_ << ")";
  return os;
}

ExpressionAtan::ExpressionAtan(Expression const& e)
    : ExpressionCell{ExpressionKind::Atan, e.get_hash()}, e_{e} {}

Variables ExpressionAtan::GetVariables() const { return e_.GetVariables(); }

bool ExpressionAtan::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Atan) {
    return false;
  }
  ExpressionAtan const& e_atan = static_cast<ExpressionAtan const&>(e);
  return e_.EqualTo(e_atan.e_);
}

double ExpressionAtan::Evaluate(Environment const& env) const {
  return std::atan(e_.Evaluate(env));
}

ostream& ExpressionAtan::Display(ostream& os) const {
  os << "atan(" << e_ << ")";
  return os;
}

ExpressionAtan2::ExpressionAtan2(Expression const& e1, Expression const& e2)
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

bool ExpressionAtan2::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Atan2) {
    return false;
  }
  ExpressionAtan2 const& e_atan2 = static_cast<ExpressionAtan2 const&>(e);
  return e1_.EqualTo(e_atan2.e1_) && e2_.EqualTo(e_atan2.e2_);
}

double ExpressionAtan2::Evaluate(Environment const& env) const {
  return std::atan2(e1_.Evaluate(env), e2_.Evaluate(env));
}

ostream& ExpressionAtan2::Display(ostream& os) const {
  os << "atan2(" << e1_ << ", " << e2_ << ")";
  return os;
}

ExpressionSinh::ExpressionSinh(Expression const& e)
    : ExpressionCell{ExpressionKind::Sinh, e.get_hash()}, e_{e} {}

Variables ExpressionSinh::GetVariables() const { return e_.GetVariables(); }

bool ExpressionSinh::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Sinh) {
    return false;
  }
  ExpressionSinh const& e_sinh = static_cast<ExpressionSinh const&>(e);
  return e_.EqualTo(e_sinh.e_);
}

double ExpressionSinh::Evaluate(Environment const& env) const {
  return std::sinh(e_.Evaluate(env));
}

ostream& ExpressionSinh::Display(ostream& os) const {
  os << "sinh(" << e_ << ")";
  return os;
}
ExpressionCosh::ExpressionCosh(Expression const& e)
    : ExpressionCell{ExpressionKind::Cosh, e.get_hash()}, e_{e} {}

Variables ExpressionCosh::GetVariables() const { return e_.GetVariables(); }

bool ExpressionCosh::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Cosh) {
    return false;
  }
  ExpressionCosh const& e_cosh = static_cast<ExpressionCosh const&>(e);
  return e_.EqualTo(e_cosh.e_);
}

double ExpressionCosh::Evaluate(Environment const& env) const {
  return std::cosh(e_.Evaluate(env));
}
ostream& ExpressionCosh::Display(ostream& os) const {
  os << "cosh(" << e_ << ")";
  return os;
}

ExpressionTanh::ExpressionTanh(Expression const& e)
    : ExpressionCell{ExpressionKind::Tanh, e.get_hash()}, e_{e} {}

Variables ExpressionTanh::GetVariables() const { return e_.GetVariables(); }

bool ExpressionTanh::EqualTo(ExpressionCell const& e) const {
  if (e.get_kind() != ExpressionKind::Tanh) {
    return false;
  }
  ExpressionTanh const& e_tanh = static_cast<ExpressionTanh const&>(e);
  return e_.EqualTo(e_tanh.e_);
}

double ExpressionTanh::Evaluate(Environment const& env) const {
  return std::tanh(e_.Evaluate(env));
}

ostream& ExpressionTanh::Display(ostream& os) const {
  os << "tanh(" << e_ << ")";
  return os;
}

ostream& operator<<(ostream& os, Expression const& e) {
  DRAKE_ASSERT(e.ptr_ != nullptr);
  return e.ptr_->Display(os);
}

Expression log(Expression const& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionLog::check_domain(v);
    return Expression{std::log(v)};
  }
  return Expression{make_shared<ExpressionLog>(e)};
}

Expression abs(Expression const& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::fabs(v)};
  }
  return Expression{make_shared<ExpressionAbs>(e)};
}

Expression exp(Expression const& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::exp(v)};
  }
  return Expression{make_shared<ExpressionExp>(e)};
}

Expression sqrt(Expression const& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionSqrt::check_domain(v);
    return Expression{std::sqrt(v)};
  }
  return Expression{make_shared<ExpressionSqrt>(e)};
}

Expression pow(Expression const& e1, Expression const& e2) {
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

Expression pow(Expression const& e1, double const v2) {
  return pow(e1, Expression{v2});
}

Expression pow(double const v1, Expression const& e2) {
  return pow(Expression{v1}, e2);
}

Expression sin(Expression const& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::sin(v)};
  }
  return Expression{make_shared<ExpressionSin>(e)};
}

Expression cos(Expression const& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::cos(v)};
  }

  return Expression{make_shared<ExpressionCos>(e)};
}

Expression tan(Expression const& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::tan(v)};
  }
  return Expression{make_shared<ExpressionTan>(e)};
}

Expression asin(Expression const& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionAsin::check_domain(v);
    return Expression{std::asin(v)};
  }
  return Expression{make_shared<ExpressionAsin>(e)};
}

Expression acos(Expression const& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    ExpressionAcos::check_domain(v);
    return Expression{std::acos(v)};
  }
  return Expression{make_shared<ExpressionAcos>(e)};
}

Expression atan(Expression const& e) {  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::atan(v)};
  }
  return Expression{make_shared<ExpressionAtan>(e)};
}

Expression atan2(Expression const& e1, Expression const& e2) {
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

Expression atan2(Expression const& e1, double const v2) {
  return atan2(e1, Expression{v2});
}

Expression atan2(double const v1, Expression const& e2) {
  return atan2(Expression{v1}, e2);
}

Expression sinh(Expression const& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::sinh(v)};
  }
  return Expression{make_shared<ExpressionSinh>(e)};
}

Expression cosh(Expression const& e) {
  // simplification
  if (e.get_kind() == ExpressionKind::Constant) {
    double const v =
        static_pointer_cast<ExpressionConstant>(e.ptr_)->get_value();
    return Expression{std::cosh(v)};
  }
  return Expression{make_shared<ExpressionCosh>(e)};
}

Expression tanh(Expression const& e) {
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

namespace std {
string to_string(drake::symbolic::Expression const& e) {
  ostringstream oss;
  oss << e;
  return oss.str();
}
}  // namespace std
