#pragma once

#include <algorithm>  // for cpplint only
#include <cstddef>
#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {

namespace symbolic {

/** Kinds of symbolic expressions. */
enum class ExpressionKind {
  Var,       ///< variable
  Constant,  ///< constant (double)
  Neg,       ///< unary minus
  Add,       ///< addition (+)
  Sub,       ///< subtraction (-)
  Mul,       ///< multiplication (*)
  Div,       ///< division (/)
  Log,       ///< logarithms
  Abs,       ///< absolute value function
  Exp,       ///< exponentiation
  Sqrt,      ///< square root
  Pow,       ///< power function
  Sin,       ///< sine
  Cos,       ///< cosine
  Tan,       ///< tangent
  Asin,      ///< arcsine
  Acos,      ///< arccosine
  Atan,      ///< arctangent
  Atan2,     ///< arctangent2 (atan2(y,x) = atan(y/x))
  Sinh,      ///< hyperbolic sine
  Cosh,      ///< hyperbolic cosine
  Tanh,      ///< hyperbolic tangent
  Min,       ///< min
  Max,       ///< max
  // TODO(soonho): add Integral
};

class ExpressionCell;

/** Represents a symbolic form of an expression.

Its syntax tree is as follows:

\verbatim
    E := Var | Constant | - E | E + E | E - E | E * E | E / E | log(E) | abs(E)
       | exp(E) | sqrt(E) | pow(E, E) | sin(E) | cos(E) | tan(E) | asin(E)
       | acos(E) | atan(E) | atan2(E, E) | sinh(E) | cosh(E) | tanh(E)
       | min(E, E) | max(E, E)
\endverbatim

In the implementation, Expression is a simple wrapper including a shared pointer
to ExpressionCell class which is a super-class of different kinds of symbolic
expressions (i.e. ExpressionAdd, ExpressionSub, ExpressionLog,
ExpressionSin). Note that it includes a shared pointer, not a unique pointer, to
allow sharing sub-expressions.

\note The sharing of sub-expressions is not yet implemented.

The following simple simplifications using identity and unity are implemented:
\verbatim
    E + 0          ->  E
    0 + E          ->  E
    E - 0          ->  E
    E - E          ->  0
    E * 1          ->  E
    1 * E          ->  E
    E * -1         -> -E
   -1 * E          -> -E
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

For the math functions which are only defined over a restricted domain (namely,
log, sqrt, pow, asin, acos), we check the domain of argument(s), and throw
std::domain_error exception if a function is not well-defined for a given
argument(s).

Relational operators over expressions (==, !=, <, >, <=, >=) return
symbolic::Formula instead of bool. Those operations are declared in
symbolic_formula.h file. To check structural equality between two expressions a
separate function, Expression::EqualTo, is provided.

symbolic::Expression can be used as a scalar type of Eigen types.
*/
class DRAKE_EXPORT Expression {
 public:
  /** Default constructor. It constructs Zero(). */
  Expression() { *this = Zero(); }

  /** Move-construct a set from an rvalue. */
  Expression(Expression&& e) = default;

  /** Copy-construct a set from an lvalue. */
  Expression(const Expression& e) = default;

  /** Move-assign a set from an rvalue. */
  Expression& operator=(Expression&& e) = default;

  /** Copy-assign a set from an lvalue. */
  Expression& operator=(const Expression& e) = default;

  /** Constructs a constant. */
  explicit Expression(double d);
  /** Constructs a variable. */
  explicit Expression(const Variable& name);
  ExpressionKind get_kind() const;
  size_t get_hash() const;
  /** Collects variables in expression. */
  Variables GetVariables() const;

  /** Checks structural equality. */
  bool EqualTo(const Expression& e) const;
  /** Evaluates under a given environment (by default, an empty environment). */
  double Evaluate(const Environment& env = Environment{}) const;

  /** Returns string representation of Expression. */
  std::string to_string() const;

  /** Returns zero. */
  static Expression Zero();
  /** Returns one. */
  static Expression One();
  /** Returns Pi, the ratio of a circle’s circumference to its diameter. */
  static Expression Pi();
  /** Return e, the base of natural logarithms. */
  static Expression E();

  friend DRAKE_EXPORT Expression operator+(Expression lhs,
                                           const Expression& rhs);
  friend DRAKE_EXPORT Expression operator+(double lhs, const Expression& rhs);
  friend DRAKE_EXPORT Expression operator+(Expression lhs, double rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator+=(Expression& lhs,
                                             const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator+=(Expression& lhs, double rhs);

  /** Provides prefix increment operator (i.e. ++x). */
  Expression& operator++();
  /** Provides postfix increment operator (i.e. x++). */
  Expression operator++(int);

  friend DRAKE_EXPORT Expression operator-(Expression lhs,
                                           const Expression& rhs);
  friend DRAKE_EXPORT Expression operator-(double lhs, const Expression& rhs);
  friend DRAKE_EXPORT Expression operator-(Expression lhs, double rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator-=(Expression& lhs,
                                             const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator-=(Expression& lhs, double rhs);

  /** Provides unary minus operator. */
  friend DRAKE_EXPORT Expression operator-(Expression e);
  /** Provides prefix decrement operator (i.e. --x). */
  Expression& operator--();
  /** Provides postfix decrement operator (i.e. x--). */
  Expression operator--(int);

  friend DRAKE_EXPORT Expression operator*(Expression lhs,
                                           const Expression& rhs);
  friend DRAKE_EXPORT Expression operator*(double lhs, const Expression& rhs);
  friend DRAKE_EXPORT Expression operator*(Expression lhs, double rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator*=(Expression& lhs,
                                             const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator*=(Expression& lhs, double rhs);

  friend DRAKE_EXPORT Expression operator/(Expression lhs,
                                           const Expression& rhs);
  friend DRAKE_EXPORT Expression operator/(double lhs, const Expression& rhs);
  friend DRAKE_EXPORT Expression operator/(Expression lhs, double rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator/=(Expression& lhs,
                                             const Expression& rhs);
  // NOLINTNEXTLINE(runtime/references) per C++ standard signature.
  friend DRAKE_EXPORT Expression& operator/=(Expression& lhs, double rhs);

  friend DRAKE_EXPORT Expression log(const Expression& e);
  friend DRAKE_EXPORT Expression abs(const Expression& e);
  friend DRAKE_EXPORT Expression exp(const Expression& e);
  friend DRAKE_EXPORT Expression sqrt(const Expression& e);
  friend DRAKE_EXPORT Expression pow(const Expression& e1,
                                     const Expression& e2);
  friend DRAKE_EXPORT Expression pow(double v1, const Expression& e2);
  friend DRAKE_EXPORT Expression pow(const Expression& e1, double v2);
  friend DRAKE_EXPORT Expression sin(const Expression& e);
  friend DRAKE_EXPORT Expression cos(const Expression& e);
  friend DRAKE_EXPORT Expression tan(const Expression& e);
  friend DRAKE_EXPORT Expression asin(const Expression& e);
  friend DRAKE_EXPORT Expression acos(const Expression& e);
  friend DRAKE_EXPORT Expression atan(const Expression& e);
  friend DRAKE_EXPORT Expression atan2(const Expression& e1,
                                       const Expression& e2);
  friend DRAKE_EXPORT Expression atan2(double v1, const Expression& e2);
  friend DRAKE_EXPORT Expression atan2(const Expression& e1, double v2);
  friend DRAKE_EXPORT Expression sinh(const Expression& e);
  friend DRAKE_EXPORT Expression cosh(const Expression& e);
  friend DRAKE_EXPORT Expression tanh(const Expression& e);
  friend DRAKE_EXPORT Expression min(double v1, const Expression& e2);
  friend DRAKE_EXPORT Expression min(const Expression& e1, double v2);
  friend DRAKE_EXPORT Expression min(const Expression& e1,
                                     const Expression& e2);
  friend DRAKE_EXPORT Expression max(double v1, const Expression& e2);
  friend DRAKE_EXPORT Expression max(const Expression& e1, double v2);
  friend DRAKE_EXPORT Expression max(const Expression& e1,
                                     const Expression& e2);

  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream& os,
                                               const Expression& e);
  friend DRAKE_EXPORT void swap(Expression& a, Expression& b) {
    std::swap(a.ptr_, b.ptr_);
  }

  /** Checks whether \p v is NaN or not. If \p is NaN, it throws a std::runtime
   * exception. */
  static void check_nan(double v);

 private:
  explicit Expression(const std::shared_ptr<ExpressionCell> ptr);
  std::shared_ptr<ExpressionCell> ptr_;
};

/** Represents an abstract class which is the base of concrete
 * symbolic-expression classes.
 *
 * \note It provides virtual function, ExpressionCell::Display,
 * because operator<< is not allowed to be a virtual function.
 */
class ExpressionCell {
 public:
  ExpressionKind get_kind() const { return kind_; }
  size_t get_hash() const { return hash_; }
  /** Collects variables in expression. */
  virtual Variables GetVariables() const = 0;
  /** Checks structural equality. */
  virtual bool EqualTo(const ExpressionCell& c) const = 0;
  /** Evaluates under a given environment. */
  virtual double Evaluate(const Environment& env) const = 0;
  /** Output string representation of expression into output stream \p os. */
  virtual std::ostream& Display(std::ostream& os) const = 0;

 protected:
  /** Default constructor. */
  ExpressionCell() = default;
  /** Move-construct a set from an rvalue. */
  ExpressionCell(ExpressionCell&& e) = default;
  /** Copy-construct a set from an lvalue. */
  ExpressionCell(const ExpressionCell& e) = default;
  /** Move-assign (DELETED). */
  ExpressionCell& operator=(ExpressionCell&& e) = delete;
  /** Copy-assign (DELETED). */
  ExpressionCell& operator=(const ExpressionCell& e) = delete;
  /** Construct ExpressionCell of kind \p k with \p hash. */
  ExpressionCell(ExpressionKind k, size_t hash);

 private:
  const ExpressionKind kind_{};
  const size_t hash_{};
};

/** Represents the base class for unary expressions.  */
class UnaryExpressionCell : public ExpressionCell {
 public:
  /** Collects variables in expression. */
  Variables GetVariables() const override;
  /** Checks structural equality. */
  bool EqualTo(const ExpressionCell& c) const override;
  /** Evaluate expression under a given environment \p env. */
  double Evaluate(const Environment& env) const override;

 protected:
  /** Default constructor (DELETED). */
  UnaryExpressionCell() = delete;
  /** Move-construct a set from an rvalue. */
  UnaryExpressionCell(UnaryExpressionCell&& e) = default;
  /** Copy-construct a set from an lvalue. */
  UnaryExpressionCell(const UnaryExpressionCell& e) = default;
  /** Move-assign (DELETED). */
  UnaryExpressionCell& operator=(UnaryExpressionCell&& e) = delete;
  /** Copy-assign (DELETED). */
  UnaryExpressionCell& operator=(const UnaryExpressionCell& e) = delete;
  /** Constructs UnaryExpressionCell of kind \p k with \p hash and \p e. */
  UnaryExpressionCell(ExpressionKind k, const Expression& e);
  /** Returns the nested expression. */
  const Expression& get_expression() const { return e_; }
  /** Returns the evaluation result f(\p v ). */
  virtual double DoEvaluate(double v) const = 0;

 private:
  const Expression e_;
};

/** Represents the base class for binary expressions.  */
class BinaryExpressionCell : public ExpressionCell {
 public:
  /** Collects variables in expression. */
  Variables GetVariables() const override;
  /** Checks structural equality. */
  bool EqualTo(const ExpressionCell& c) const override;
  /** Evaluate expression under a given environment \p env. */
  double Evaluate(const Environment& env) const override;

 protected:
  /** Default constructor (DELETED). */
  BinaryExpressionCell() = delete;
  /** Move-construct a set from an rvalue. */
  BinaryExpressionCell(BinaryExpressionCell&& e) = default;
  /** Copy-construct a set from an lvalue. */
  BinaryExpressionCell(const BinaryExpressionCell& e) = default;
  /** Move-assign (DELETED). */
  BinaryExpressionCell& operator=(BinaryExpressionCell&& e) = delete;
  /** Copy-assign (DELETED). */
  BinaryExpressionCell& operator=(const BinaryExpressionCell& e) = delete;
  /** Constructs BinaryExpressionCell of kind \p k with \p hash, \p e1, \p e2.
   */
  BinaryExpressionCell(ExpressionKind k, const Expression& e1,
                       const Expression& e2);
  /** Returns the first expression. */
  const Expression& get_1st_expression() const { return e1_; }
  /** Returns the second expression. */
  const Expression& get_2nd_expression() const { return e2_; }
  /** Returns the evaluation result f(\p v1, \p v2 ). */
  virtual double DoEvaluate(double v1, double v2) const = 0;

 private:
  const Expression e1_;
  const Expression e2_;
};

/** Symbolic expression representing a variable. */
class ExpressionVar : public ExpressionCell {
 public:
  explicit ExpressionVar(const Variable& v);
  Variable get_variable() const { return var_; }
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Variable var_;
};

/** Symbolic expression representing a constant. */
class ExpressionConstant : public ExpressionCell {
 public:
  explicit ExpressionConstant(double v);
  double get_value() const { return v_; }
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const double v_{};
};

/** Symbolic expression representing unary minus. */
class ExpressionNeg : public UnaryExpressionCell {
 public:
  explicit ExpressionNeg(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing addition. */
class ExpressionAdd : public BinaryExpressionCell {
 public:
  ExpressionAdd(const Expression& e1, const Expression& e2);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing subtraction. */
class ExpressionSub : public BinaryExpressionCell {
 public:
  ExpressionSub(const Expression& e1, const Expression& e2);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing multiplication. */
class ExpressionMul : public BinaryExpressionCell {
 public:
  ExpressionMul(const Expression& e1, const Expression& e2);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing division. */
class ExpressionDiv : public BinaryExpressionCell {
 public:
  ExpressionDiv(const Expression& e1, const Expression& e2);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing logarithms. */
class ExpressionLog : public UnaryExpressionCell {
 public:
  explicit ExpressionLog(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKE_EXPORT Expression log(const Expression& e);

 private:
  /* Throws std::domain_error if v ∉ [0, +oo). */
  static void check_domain(double v);
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing absolute value function. */
class ExpressionAbs : public UnaryExpressionCell {
 public:
  explicit ExpressionAbs(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKE_EXPORT Expression abs(const Expression& e);

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing exponentiation using the base of
 * natural logarithms. */
class ExpressionExp : public UnaryExpressionCell {
 public:
  explicit ExpressionExp(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing square-root. */
class ExpressionSqrt : public UnaryExpressionCell {
 public:
  explicit ExpressionSqrt(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKE_EXPORT Expression sqrt(const Expression& e);

 private:
  /* Throws std::domain_error if v ∉ [0, +oo). */
  static void check_domain(double v);
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing power function. */
class ExpressionPow : public BinaryExpressionCell {
 public:
  explicit ExpressionPow(const Expression& e1, const Expression& e2);
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKE_EXPORT Expression pow(const Expression& e1,
                                     const Expression& e2);
  friend DRAKE_EXPORT Expression pow(double v1, const Expression& e2);
  friend DRAKE_EXPORT Expression pow(const Expression& e1, double v2);

 private:
  /* Throws std::domain_error if v1 is finite negative and v2 is finite
     non-integer. */
  static void check_domain(double v1, double v2);
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing sine function. */
class ExpressionSin : public UnaryExpressionCell {
 public:
  explicit ExpressionSin(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing cosine function. */
class ExpressionCos : public UnaryExpressionCell {
 public:
  explicit ExpressionCos(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing tangent function. */
class ExpressionTan : public UnaryExpressionCell {
 public:
  explicit ExpressionTan(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing arcsine function. */
class ExpressionAsin : public UnaryExpressionCell {
 public:
  explicit ExpressionAsin(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKE_EXPORT Expression asin(const Expression& e);

 private:
  /* Throws std::domain_error if v ∉ [-1.0, +1.0]. */
  static void check_domain(double v);
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing arccosine function. */
class ExpressionAcos : public UnaryExpressionCell {
 public:
  explicit ExpressionAcos(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

  friend DRAKE_EXPORT Expression acos(const Expression& e);

 private:
  /* Throws std::domain_error if v ∉ [-1.0, +1.0]. */
  static void check_domain(double v);
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing arctangent function. */
class ExpressionAtan : public UnaryExpressionCell {
 public:
  explicit ExpressionAtan(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing atan2 function (arctangent function with
 * two arguments). atan2(y, x) is defined as atan(y/x). */
class ExpressionAtan2 : public BinaryExpressionCell {
 public:
  explicit ExpressionAtan2(const Expression& e1, const Expression& e2);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing hyperbolic sine function. */
class ExpressionSinh : public UnaryExpressionCell {
 public:
  explicit ExpressionSinh(const Expression& e);
  double Evaluate(const Environment& env) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing hyperbolic cosine function. */
class ExpressionCosh : public UnaryExpressionCell {
 public:
  explicit ExpressionCosh(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing hyperbolic tangent function. */
class ExpressionTanh : public UnaryExpressionCell {
 public:
  explicit ExpressionTanh(const Expression& e);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing min function. */
class ExpressionMin : public BinaryExpressionCell {
 public:
  explicit ExpressionMin(const Expression& e1, const Expression& e2);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing max function. */
class ExpressionMax : public BinaryExpressionCell {
 public:
  explicit ExpressionMax(const Expression& e1, const Expression& e2);
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

std::ostream& operator<<(std::ostream& os, const Expression& e);

/** \relates Expression Return a copy of \p lhs updated to record component-wise
 * multiplication by a constant \p rhs.
 */
template <typename MatrixL>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value,
    typename MatrixL::PlainObject>::type
operator*(const MatrixL& lhs, double rhs) {
  return lhs * Expression{rhs};
}

/** \relates Expression
 * Return a copy of \p rhs updated to record component-wise multiplication by a
 * constant \p rhs.
 */
template <typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        std::is_same<typename MatrixR::Scalar, Expression>::value,
    typename MatrixR::PlainObject>::type
operator*(double lhs, const MatrixR& rhs) {
  return (Expression{lhs}) * rhs;
}

/** \relates Expression
 * Update \p lhs to record component-wise multiplication by a constant \p rhs.
 */
template <typename MatrixL>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value,
    MatrixL&>::type
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
operator*=(MatrixL& lhs, double rhs) {
  return lhs *= Expression{rhs};
}

/** \relates Expression
 * Return a copy of \p lhs updated to record component-wise division by a
 * constant \p rhs.
 */
template <typename MatrixL>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value,
    typename MatrixL::PlainObject>::type
operator/(const MatrixL& lhs, double rhs) {
  return lhs / Expression{rhs};
}

/** \relates Expression
 * Update \p lhs to record component-wise division by a constant \p rhs.
 */
template <typename MatrixL>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_same<typename MatrixL::Scalar, Expression>::value,
    MatrixL&>::type
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
operator/=(MatrixL& lhs, double rhs) {
  return lhs /= Expression{rhs};
}
}  // namespace symbolic

/** Specializes is_numeric to be false for symbolic::Expression type. */
template <>
struct DRAKE_EXPORT is_numeric<symbolic::Expression> {
  static constexpr bool value = false;
};
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Expression>. */
template <>
struct hash<drake::symbolic::Expression> {
  size_t operator()(const drake::symbolic::Expression& e) const {
    return e.get_hash();
  }
};

/* Provides std::equal_to<drake::symbolic::Expression>. */
template <>
struct equal_to<drake::symbolic::Expression> {
  bool operator()(const drake::symbolic::Expression& lhs,
                  const drake::symbolic::Expression& rhs) const {
    return lhs.EqualTo(rhs);
  }
};
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
  typedef drake::symbolic::Expression Literal;
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
  template <bool Vectorized>
  struct Div {
    enum { Cost = 1 };
  };
};

namespace internal {
// Eigen component-wise Matrix<Expression>::isConstant(Expression).
template <>
struct scalar_fuzzy_impl<drake::symbolic::Expression> {
  static inline bool isApprox(drake::symbolic::Expression x,
                              drake::symbolic::Expression y,
                              drake::symbolic::Expression) {
    return x.EqualTo(y);
  }
};
}  // namespace internal
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
