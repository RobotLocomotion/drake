#pragma once

#include <cstddef>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <eigen3/Eigen/Core>

#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"
#include "drake/drakeCommon_export.h"

namespace drake {

namespace symbolic {

enum class ExpressionKind {
  Var,
  Constant,
  Neg,    // unary minus
  Add,    // addition (+)
  Sub,    // subtraction (-)
  Mul,    // multiplication (*)
  Div,    // division (/)
  Log,    // logarithms
  Abs,    // absolute value function
  Exp,    // exponentiation
  Sqrt,   // square root
  Pow,    // power function
  Sin,    // sine
  Cos,    // cosine
  Tan,    // tangent
  Asin,   // arcsine
  Acos,   // arccosine
  Atan,   // arctangent
  Atan2,  // arctangent2 (atan2(y,x) = atan(y/x))
  Sinh,   // hyperbolic sine
  Cosh,   // hyperbolic cosine
  Tanh,   // hyperbolic tangent
  // TODO(soonho): add Integral
};

class ExpressionCell;

/** \brief Represent a symbolic form of an expression.

Its syntax tree is as follows:

\verbatim
    E := Var | Constant | - E | E + E | E - E | E * E | E / E | log(E) | abs(E)
       | exp(E) | sqrt(E) | pow(E, E) | sin(E) | cos(E) | tan(E) | asin(E)
       | acos(E) | atan(E) | atan2(E, E) | sinh(E) | cosh(E) | tanh(E)
\endverbatim

In the implementation, Expression is a simple wrapper including a shared pointer
to
ExpressionCell class which is a super-class of different kinds of symbolic
expressions
(i.e. ExpressionAdd, ExpressionSub, ExpressionLog, ExpressionSin). Note that it
includes a shared
pointer, not a unique pointer, to allow sharing sub-expressions.

\note The sharing of sub-expressions is not yet implemented.

The following simple simplifications using identity and unity are implemented:
\verbatim
    E + 0          ->  E
    0 + E          ->  E
    E - 0          ->  E
    E - E          ->  0
    E * 1          ->  E
    1 * E          ->  E
    E * 0          ->  0
    0 * E          ->  0
    E / 1          ->  E
    E / E          ->  1
\endverbatim

Constant folding is implemented:
\verbatim
    E(c1) + E(c2)  ->  E(c1 + c2)    // c1, c2 are constants
    E(c1) - E(c2)  ->  E(c1 - c2)
    E(c1) * E(c2)  ->  E(c1 * c2)
    E(c1) / E(c2)  ->  E(c1 / c2)
    f(E(c))        ->  E(f(c))       // c is a constant, f is a math function
\endverbatim

For the math functions which are only defined over restricted domain (namely,
log, sqrt, pow, asin, acos), we check the domain of argument(s), and throw
std::domain_error exception if a function is not well-defined for a given
argument(s).

Relational operators over expressions (==, !=, <, >, <=, >=) return
symbolic::Formula instead of bool. Those operations are declared in
symbolic_formula.h file. To check structural equality between two expressions, a
separate function, Expression::EqualTo is provided.

symbolic::Expression may also be used as the scalar type of Eigen types.
*/
class DRAKECOMMON_EXPORT Expression {
 private:
  std::shared_ptr<ExpressionCell> ptr_;

 public:
  /** default constructor */
  Expression() = default;

  /** Move-construct a set from an rvalue. */
  Expression(Expression&& e) = default;

  /** Copy-construct a set from an lvalue. */
  Expression(Expression const& e) = default;

  /** Move-assign a set from an rvalue. */
  Expression& operator=(Expression&& e) = default;

  /** Copy-assign a set from an lvalue. */
  Expression& operator=(Expression const& e) = default;

  /** Construct a constant. */
  explicit Expression(double const d);
  /** Construct a variable. */
  explicit Expression(Variable const& name);
  explicit Expression(std::shared_ptr<ExpressionCell> const ptr);
  ExpressionKind get_kind() const;
  size_t get_hash() const;
  /** Collect variables in expression. */
  Variables GetVariables() const;

  /** Check structural equality*/
  bool EqualTo(Expression const& e) const;
  /** Evaluate under a given environment (by default, an empty environment)*/
  double Evaluate(Environment const& env = Environment{}) const;

  /** return zero*/
  static Expression Zero();
  /** return one*/
  static Expression One();
  /** return Pi, the ratio of a circle’s circumference to its diameter*/
  static Expression Pi();
  /** return e, the base of natural logarithms*/
  static Expression E();

  friend DRAKECOMMON_EXPORT Expression operator+(Expression lhs,
                                                 Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression operator+(double const lhs,
                                                 Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression operator+(Expression lhs,
                                                 double const rhs);
  friend DRAKECOMMON_EXPORT Expression& operator+=(Expression& lhs,
                                                   Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression& operator+=(Expression& lhs,
                                                   double const rhs);

  /** Prefix increment*/
  Expression& operator++();
  /** Postfix increment*/
  Expression operator++(int);

  friend DRAKECOMMON_EXPORT Expression operator-(Expression lhs,
                                                 Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression operator-(double const lhs,
                                                 Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression operator-(Expression lhs,
                                                 double const rhs);
  friend DRAKECOMMON_EXPORT Expression& operator-=(Expression& lhs,
                                                   Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression& operator-=(Expression& lhs,
                                                   double const rhs);

  /** unary minus*/
  friend DRAKECOMMON_EXPORT Expression operator-(Expression e);
  /** Prefix decrement*/
  Expression& operator--();
  /** Postfix decrement*/
  Expression operator--(int);

  friend DRAKECOMMON_EXPORT Expression operator*(Expression lhs,
                                                 Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression operator*(double const lhs,
                                                 Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression operator*(Expression lhs,
                                                 double const rhs);
  friend DRAKECOMMON_EXPORT Expression& operator*=(Expression& lhs,
                                                   Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression& operator*=(Expression& lhs,
                                                   double const rhs);

  friend DRAKECOMMON_EXPORT Expression operator/(Expression lhs,
                                                 Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression operator/(double const lhs,
                                                 Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression operator/(Expression lhs,
                                                 double const rhs);
  friend DRAKECOMMON_EXPORT Expression& operator/=(Expression& lhs,
                                                   Expression const& rhs);
  friend DRAKECOMMON_EXPORT Expression& operator/=(Expression& lhs,
                                                   double const rhs);

  friend DRAKECOMMON_EXPORT Expression log(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression abs(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression exp(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression sqrt(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression pow(Expression const& e1,
                                           Expression const& e2);
  friend DRAKECOMMON_EXPORT Expression pow(double const v1,
                                           Expression const& e2);
  friend DRAKECOMMON_EXPORT Expression pow(Expression const& e1,
                                           double const v2);
  friend DRAKECOMMON_EXPORT Expression sin(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression cos(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression tan(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression asin(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression acos(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression atan(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression atan2(Expression const& e1,
                                             Expression const& e2);
  friend DRAKECOMMON_EXPORT Expression atan2(double const v1,
                                             Expression const& e2);
  friend DRAKECOMMON_EXPORT Expression atan2(Expression const& e1,
                                             double const v2);
  friend DRAKECOMMON_EXPORT Expression sinh(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression cosh(Expression const& e);
  friend DRAKECOMMON_EXPORT Expression tanh(Expression const& e);

  friend DRAKECOMMON_EXPORT std::ostream& operator<<(std::ostream& os,
                                                     Expression const& e);
  friend DRAKECOMMON_EXPORT void swap(Expression& a, Expression& b) {
    std::swap(a.ptr_, b.ptr_);
  }
};

/** \brief Abstract class which is the base of concrete symbolic-expression
 * classes.
 *
 * \note It provides virtual function, ExpressionCell::Display, because
 * operator<< is
 * not allowed to be a virtual function.
 */
class ExpressionCell {
 protected:
  ExpressionKind kind_;
  size_t hash_;

 public:
  ExpressionCell(ExpressionKind const k, size_t const hash);
  ExpressionKind get_kind() const { return kind_; }
  size_t get_hash() const { return hash_; }
  /** Collect variables in expression. */
  virtual Variables GetVariables() const = 0;
  /** Check structural equality*/
  virtual bool EqualTo(ExpressionCell const& c) const = 0;
  /** Evaluate under a given environment (by default, an empty environment). */
  virtual double Evaluate(Environment const& env) const = 0;
  virtual std::ostream& Display(std::ostream& os) const = 0;
};

/** \brief Symbolic expression representing a variable. */
class ExpressionVar : public ExpressionCell {
 private:
  Variable var_;

 public:
  explicit ExpressionVar(Variable const& v);
  Variable get_variable() const { return var_; }
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing a constant. */
class ExpressionConstant : public ExpressionCell {
 private:
  double v_;

 public:
  explicit ExpressionConstant(double const v);
  double get_value() const { return v_; }
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing unary minus. */
class ExpressionNeg : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionNeg(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing addition. */
class ExpressionAdd : public ExpressionCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  ExpressionAdd(Expression const& e1, Expression const& e2);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing subtraction. */
class ExpressionSub : public ExpressionCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  ExpressionSub(Expression const& e1, Expression const& e2);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing multiplication. */
class ExpressionMul : public ExpressionCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  ExpressionMul(Expression const& e1, Expression const& e2);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing division. */
class ExpressionDiv : public ExpressionCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  ExpressionDiv(Expression const& e1, Expression const& e2);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing logarithms. */
class ExpressionLog : public ExpressionCell {
 private:
  Expression e_;
  /// Throw std::domain_error if v ∉ [0, +oo)
  static void check_domain(double const v);

 public:
  explicit ExpressionLog(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression log(Expression const& e);
};

/** \brief Symbolic expression representing absolute value function. */
class ExpressionAbs : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionAbs(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression abs(Expression const& e);
};

/** \brief Symbolic expression representing exponentiation using the base of
 * natural logarithms. */
class ExpressionExp : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionExp(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing square-root. */
class ExpressionSqrt : public ExpressionCell {
 private:
  Expression e_;
  /// Throw std::domain_error if v ∉ [0, +oo)
  static void check_domain(double const v);

 public:
  explicit ExpressionSqrt(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression sqrt(Expression const& e);
};

/** \brief Symbolic expression representing power function. */
class ExpressionPow : public ExpressionCell {
 private:
  Expression e1_;
  Expression e2_;
  /// Throw std::domain_error if v1 is finite negative and v2 is finite
  /// non-integer.
  static void check_domain(double const v1, double const v2);

 public:
  explicit ExpressionPow(Expression const& e1, Expression const& e2);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression pow(Expression const& e1,
                                           Expression const& e2);
  friend DRAKECOMMON_EXPORT Expression pow(double const v1,
                                           Expression const& e2);
  friend DRAKECOMMON_EXPORT Expression pow(Expression const& e1,
                                           double const v2);
};

/** \brief Symbolic expression representing sine function. */
class ExpressionSin : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionSin(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing cosine function. */
class ExpressionCos : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionCos(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing tangent function. */
class ExpressionTan : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionTan(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing arcsine function. */
class ExpressionAsin : public ExpressionCell {
 private:
  Expression e_;
  /// Throw std::domain_error if v ∉ [-1.0, +1.0]
  static void check_domain(double const v);

 public:
  explicit ExpressionAsin(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression asin(Expression const& e);
};

/** \brief Symbolic expression representing arccosine function. */
class ExpressionAcos : public ExpressionCell {
 private:
  Expression e_;
  /// Throw std::domain_error if v ∉ [-1.0, +1.0]
  static void check_domain(double const v);

 public:
  explicit ExpressionAcos(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression acos(Expression const& e);
};

/** \brief Symbolic expression representing arctangent function. */
class ExpressionAtan : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionAtan(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing atan2 function (arctangent function
 * with two arguments). */
class ExpressionAtan2 : public ExpressionCell {
 private:
  Expression e1_;
  Expression e2_;

 public:
  explicit ExpressionAtan2(Expression const& e1, Expression const& e2);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing hyperbolic sine function. */
class ExpressionSinh : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionSinh(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing hyperbolic cosine function. */
class ExpressionCosh : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionCosh(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** \brief Symbolic expression representing hyperbolic tangent function. */
class ExpressionTanh : public ExpressionCell {
 private:
  Expression e_;

 public:
  explicit ExpressionTanh(Expression const& e);
  Variables GetVariables() const override;
  bool EqualTo(ExpressionCell const& e) const override;
  double Evaluate(Environment const& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

std::ostream& operator<<(std::ostream& os, Expression const& e);

}  // namespace drake
}  // namespace symbolic

/** Provide std::hash<drake::symbolic::Expression>. */
namespace std {
template <>
struct hash<drake::symbolic::Expression> {
  size_t operator()(drake::symbolic::Expression const& e) const {
    return e.get_hash();
  }
};

/** Provide std::equal_to<drake::symbolic::Expression>. */
template <>
struct equal_to<drake::symbolic::Expression> {
  bool operator()(drake::symbolic::Expression const& lhs,
                  drake::symbolic::Expression const& rhs) const {
    return lhs.EqualTo(rhs);
  }
};

/** Provide std::to_string for drake::symbolic::Expression. */
DRAKECOMMON_EXPORT std::string to_string(drake::symbolic::Expression const& e);
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
// Define Eigen traits needed for Matrix<drake::symbolic::Expression>.
namespace Eigen {
// Eigen scalar type traits for Matrix<drake::symbolic::Expression>.
template <>
struct NumTraits<drake::symbolic::Expression>
    : GenericNumTraits<drake::symbolic::Expression> {
  typedef drake::symbolic::Expression Real;
  typedef drake::symbolic::Expression NonInteger;
  typedef drake::symbolic::Expression Nested;
  static inline Real epsilon() { return drake::symbolic::Expression::Zero(); }
  static inline Real dummy_precision() {
    return drake::symbolic::Expression::Zero();
  }
  static inline int digits10() { return 0; }
  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 1,
    MulCost = 1
  };
};
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
