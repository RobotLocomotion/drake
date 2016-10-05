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

/** Represents a symbolic form of an expression.

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
  /** default constructor. */
  Expression() = default;

  /** Move-construct a set from an rvalue. */
  Expression(Expression&& e) = default;

  /** Copy-construct a set from an lvalue. */
  Expression(const Expression& e) = default;

  /** Move-assign a set from an rvalue. */
  Expression& operator=(Expression&& e) = default;

  /** Copy-assign a set from an lvalue. */
  Expression& operator=(const Expression& e) = default;

  /** Constructs a constant. */
  explicit Expression(const double d);
  /** Constructs a variable. */
  explicit Expression(const Variable& name);
  explicit Expression(const std::shared_ptr<ExpressionCell> ptr);
  ExpressionKind get_kind() const;
  size_t get_hash() const;
  /** Collects variables in expression. */
  Variables GetVariables() const;

  /** Checks structural equality. */
  bool EqualTo(const Expression& e) const;
  /** Evaluates under a given environment (by default, an empty environment). */
  double Evaluate(const Environment& env = Environment{}) const;

  /** Returns zero. */
  static Expression Zero();
  /** Returns one. */
  static Expression One();
  /** Returns Pi, the ratio of a circle’s circumference to its diameter. */
  static Expression Pi();
  /** Return e, the base of natural logarithms. */
  static Expression E();

  friend DRAKECOMMON_EXPORT Expression operator+(Expression lhs,
                                                 const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression operator+(const double lhs,
                                                 const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression operator+(Expression lhs,
                                                 const double rhs);
  friend DRAKECOMMON_EXPORT Expression& operator+=(Expression& lhs,
                                                   const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression& operator+=(Expression& lhs,
                                                   const double rhs);

  /** Provides prefix increment operator (i.e. ++x). */
  Expression& operator++();
  /** Provides postfix increment operator (i.e. x++). */
  Expression operator++(int);

  friend DRAKECOMMON_EXPORT Expression operator-(Expression lhs,
                                                 const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression operator-(const double lhs,
                                                 const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression operator-(Expression lhs,
                                                 const double rhs);
  friend DRAKECOMMON_EXPORT Expression& operator-=(Expression& lhs,
                                                   const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression& operator-=(Expression& lhs,
                                                   const double rhs);

  /** Provides unary minus operator. */
  friend DRAKECOMMON_EXPORT Expression operator-(Expression e);
  /** Provides prefix decrement operator (i.e. --x). */
  Expression& operator--();
  /** Provides postfix decrement operator (i.e. x--). */
  Expression operator--(int);

  friend DRAKECOMMON_EXPORT Expression operator*(Expression lhs,
                                                 const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression operator*(const double lhs,
                                                 const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression operator*(Expression lhs,
                                                 const double rhs);
  friend DRAKECOMMON_EXPORT Expression& operator*=(Expression& lhs,
                                                   const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression& operator*=(Expression& lhs,
                                                   const double rhs);

  friend DRAKECOMMON_EXPORT Expression operator/(Expression lhs,
                                                 const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression operator/(const double lhs,
                                                 const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression operator/(Expression lhs,
                                                 const double rhs);
  friend DRAKECOMMON_EXPORT Expression& operator/=(Expression& lhs,
                                                   const Expression& rhs);
  friend DRAKECOMMON_EXPORT Expression& operator/=(Expression& lhs,
                                                   const double rhs);

  friend DRAKECOMMON_EXPORT Expression log(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression abs(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression exp(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression sqrt(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression pow(const Expression& e1,
                                           const Expression& e2);
  friend DRAKECOMMON_EXPORT Expression pow(const double v1,
                                           const Expression& e2);
  friend DRAKECOMMON_EXPORT Expression pow(const Expression& e1,
                                           const double v2);
  friend DRAKECOMMON_EXPORT Expression sin(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression cos(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression tan(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression asin(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression acos(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression atan(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression atan2(const Expression& e1,
                                             const Expression& e2);
  friend DRAKECOMMON_EXPORT Expression atan2(const double v1,
                                             const Expression& e2);
  friend DRAKECOMMON_EXPORT Expression atan2(const Expression& e1,
                                             const double v2);
  friend DRAKECOMMON_EXPORT Expression sinh(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression cosh(const Expression& e);
  friend DRAKECOMMON_EXPORT Expression tanh(const Expression& e);

  friend DRAKECOMMON_EXPORT std::ostream& operator<<(std::ostream& os,
                                                     const Expression& e);
  friend DRAKECOMMON_EXPORT void swap(Expression& a, Expression& b) {
    std::swap(a.ptr_, b.ptr_);
  }
};

/** Represents an abstract class which is the base of concrete
 * symbolic-expression classes.
 *
 * \note It provides virtual function, ExpressionCell::Display,
 * because operator<< is not allowed to be a virtual function.
 */
class ExpressionCell {
 protected:
  const ExpressionKind kind_{};
  const size_t hash_{};

 public:
  ExpressionCell(const ExpressionKind k, size_t const hash);
  ExpressionKind get_kind() const { return kind_; }
  size_t get_hash() const { return hash_; }
  /** Collects variables in expression. */
  virtual Variables GetVariables() const = 0;
  /** Checks structural equality. */
  virtual bool EqualTo(const ExpressionCell& c) const = 0;
  /** Evaluates under a given environment (by default, an empty environment). */
  virtual double Evaluate(const Environment& env) const = 0;
  virtual std::ostream& Display(std::ostream& os) const = 0;
};

/** Symbolic expression representing a variable. */
class ExpressionVar : public ExpressionCell {
 private:
  const Variable var_;

 public:
  explicit ExpressionVar(const Variable& v);
  Variable get_variable() const { return var_; }
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing a constant. */
class ExpressionConstant : public ExpressionCell {
 private:
  const double v_{};

 public:
  explicit ExpressionConstant(const double v);
  double get_value() const { return v_; }
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing unary minus. */
class ExpressionNeg : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionNeg(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing addition. */
class ExpressionAdd : public ExpressionCell {
 private:
  const Expression e1_;
  const Expression e2_;

 public:
  ExpressionAdd(const Expression& e1, const Expression& e2);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing subtraction. */
class ExpressionSub : public ExpressionCell {
 private:
  const Expression e1_;
  const Expression e2_;

 public:
  ExpressionSub(const Expression& e1, const Expression& e2);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing multiplication. */
class ExpressionMul : public ExpressionCell {
 private:
  const Expression e1_;
  const Expression e2_;

 public:
  ExpressionMul(const Expression& e1, const Expression& e2);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing division. */
class ExpressionDiv : public ExpressionCell {
 private:
  const Expression e1_;
  const Expression e2_;

 public:
  ExpressionDiv(const Expression& e1, const Expression& e2);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing logarithms. */
class ExpressionLog : public ExpressionCell {
 private:
  const Expression e_;
  /// Throw std::domain_error if v ∉ [0, +oo)
  static void check_domain(const double v);

 public:
  explicit ExpressionLog(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression log(const Expression& e);
};

/** Symbolic expression representing absolute value function. */
class ExpressionAbs : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionAbs(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression abs(const Expression& e);
};

/** Symbolic expression representing exponentiation using the base of
 * natural logarithms. */
class ExpressionExp : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionExp(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing square-root. */
class ExpressionSqrt : public ExpressionCell {
 private:
  const Expression e_;
  /// Throw std::domain_error if v ∉ [0, +oo)
  static void check_domain(const double v);

 public:
  explicit ExpressionSqrt(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression sqrt(const Expression& e);
};

/** Symbolic expression representing power function. */
class ExpressionPow : public ExpressionCell {
 private:
  const Expression e1_;
  const Expression e2_;
  /// Throw std::domain_error if v1 is finite negative and v2 is finite
  /// non-integer.
  static void check_domain(const double v1, const double v2);

 public:
  explicit ExpressionPow(const Expression& e1, const Expression& e2);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression pow(const Expression& e1,
                                           const Expression& e2);
  friend DRAKECOMMON_EXPORT Expression pow(const double v1,
                                           const Expression& e2);
  friend DRAKECOMMON_EXPORT Expression pow(const Expression& e1,
                                           const double v2);
};

/** Symbolic expression representing sine function. */
class ExpressionSin : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionSin(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing cosine function. */
class ExpressionCos : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionCos(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing tangent function. */
class ExpressionTan : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionTan(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing arcsine function. */
class ExpressionAsin : public ExpressionCell {
 private:
  const Expression e_;
  /// Throw std::domain_error if v ∉ [-1.0, +1.0]
  static void check_domain(const double v);

 public:
  explicit ExpressionAsin(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression asin(const Expression& e);
};

/** Symbolic expression representing arccosine function. */
class ExpressionAcos : public ExpressionCell {
 private:
  const Expression e_;
  /// Throw std::domain_error if v ∉ [-1.0, +1.0]
  static void check_domain(const double v);

 public:
  explicit ExpressionAcos(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKECOMMON_EXPORT Expression acos(const Expression& e);
};

/** Symbolic expression representing arctangent function. */
class ExpressionAtan : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionAtan(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing atan2 function (arctangent function
 * with two arguments). */
class ExpressionAtan2 : public ExpressionCell {
 private:
  const Expression e1_;
  const Expression e2_;

 public:
  explicit ExpressionAtan2(const Expression& e1, const Expression& e2);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing hyperbolic sine function. */
class ExpressionSinh : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionSinh(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing hyperbolic cosine function. */
class ExpressionCosh : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionCosh(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing hyperbolic tangent function. */
class ExpressionTanh : public ExpressionCell {
 private:
  const Expression e_;

 public:
  explicit ExpressionTanh(const Expression& e);
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;
};

std::ostream& operator<<(std::ostream& os, const Expression& e);

}  // namespace drake
}  // namespace symbolic

/** Provides std::hash<drake::symbolic::Expression>. */
namespace std {
template <>
struct hash<drake::symbolic::Expression> {
  size_t operator()(const drake::symbolic::Expression& e) const {
    return e.get_hash();
  }
};

/** Provides std::equal_to<drake::symbolic::Expression>. */
template <>
struct equal_to<drake::symbolic::Expression> {
  bool operator()(const drake::symbolic::Expression& lhs,
                  const drake::symbolic::Expression& rhs) const {
    return lhs.EqualTo(rhs);
  }
};

/** Provides std::to_string for drake::symbolic::Expression. */
DRAKECOMMON_EXPORT std::string to_string(const drake::symbolic::Expression& e);
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
