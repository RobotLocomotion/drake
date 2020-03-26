#pragma once
/// @file
///
/// Provides implementation-details of symbolic expressions.
///
/// It is strongly discouraged to include and use this header file outside of
/// drake/common/symbolic_* files. To include this file, you need to define
/// `DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER` before. Without it, you have
/// compile-time errors.
#ifndef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#warning Do not include this file unless you implement symbolic libraries.
#endif

#include <algorithm>  // for cpplint only
#include <cstddef>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/random.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

// Checks if @p v contains an integer value.
bool is_integer(double v);

// Checks if @p v contains a positive integer value.
bool is_positive_integer(double v);

// Checks if @p v contains a non-negative integer value.
bool is_non_negative_integer(double v);

/** Represents an abstract class which is the base of concrete
 * symbolic-expression classes.
 *
 * @note It provides virtual function, ExpressionCell::Display, because
 * operator<< is not allowed to be a virtual function.
 */
class ExpressionCell {
 public:
  /** Returns expression kind. */
  ExpressionKind get_kind() const { return kind_; }

  /** Sends all hash-relevant bytes for this ExpressionCell type into the given
   * hasher, per the @ref hash_append concept -- except for get_kind(), because
   * Expression already sends that.
   */
  virtual void HashAppendDetail(DelegatingHasher*) const = 0;

  /** Collects variables in expression. */
  virtual Variables GetVariables() const = 0;

  /** Checks structural equality. */
  virtual bool EqualTo(const ExpressionCell& c) const = 0;

  /** Provides lexicographical ordering between expressions. */
  virtual bool Less(const ExpressionCell& c) const = 0;

  /** Checks if this symbolic expression is convertible to Polynomial. */
  bool is_polynomial() const { return is_polynomial_; }

  /** Checks if this symbolic expression is already expanded. */
  bool is_expanded() const { return is_expanded_; }

  /** Sets this symbolic expression as already expanded. */
  void set_expanded() { is_expanded_ = true; }

  /** Evaluates under a given environment (by default, an empty environment).
   *  @throws std::runtime_error if NaN is detected during evaluation.
   */
  virtual double Evaluate(const Environment& env) const = 0;

  /** Expands out products and positive integer powers in expression.
   * @throws std::runtime_error if NaN is detected during expansion.
   */
  virtual Expression Expand() const = 0;

  /** Returns an Expression obtained by replacing all occurrences of the
   * variables in @p s in the current expression cell with the corresponding
   * expressions in @p s.
   * @throws std::runtime_error if NaN is detected during substitution.
   */
  virtual Expression Substitute(const Substitution& s) const = 0;

  /** Differentiates this symbolic expression with respect to the variable @p
   * var.
   * @throws std::runtime_error if it is not differentiable.
   */
  virtual Expression Differentiate(const Variable& x) const = 0;

  /** Outputs string representation of expression into output stream @p os. */
  virtual std::ostream& Display(std::ostream& os) const = 0;

 protected:
  /** Default constructor. */
  ExpressionCell() = default;
  /** Move-constructs an ExpressionCell from an rvalue. */
  ExpressionCell(ExpressionCell&& e) = default;
  /** Copy-constructs an ExpressionCell from an lvalue. */
  ExpressionCell(const ExpressionCell& e) = default;
  /** Move-assigns (DELETED). */
  ExpressionCell& operator=(ExpressionCell&& e) = delete;
  /** Copy-assigns (DELETED). */
  ExpressionCell& operator=(const ExpressionCell& e) = delete;
  /** Constructs ExpressionCell of kind @p k with @p is_poly and @p is_expanded.
   */
  ExpressionCell(ExpressionKind k, bool is_poly, bool is_expanded);
  /** Default destructor. */
  virtual ~ExpressionCell() = default;

 private:
  const ExpressionKind kind_{};
  const bool is_polynomial_{false};
  bool is_expanded_{false};
};

/** Represents the base class for unary expressions.  */
class UnaryExpressionCell : public ExpressionCell {
 public:
  void HashAppendDetail(DelegatingHasher*) const override;
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  bool Less(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  /** Returns the argument. */
  const Expression& get_argument() const { return e_; }

 protected:
  /** Default constructor (DELETED). */
  UnaryExpressionCell() = delete;
  /** Move-constructs from an rvalue. */
  UnaryExpressionCell(UnaryExpressionCell&& e) = default;
  /** Copy-constructs from an lvalue. */
  UnaryExpressionCell(const UnaryExpressionCell& e) = default;
  /** Move-assigns (DELETED). */
  UnaryExpressionCell& operator=(UnaryExpressionCell&& e) = delete;
  /** Copy-assigns (DELETED). */
  UnaryExpressionCell& operator=(const UnaryExpressionCell& e) = delete;
  /** Constructs UnaryExpressionCell of kind @p k with @p e, @p is_poly, and @p
   * is_expanded. */
  UnaryExpressionCell(ExpressionKind k, const Expression& e, bool is_poly,
                      bool is_expanded);
  /** Returns the evaluation result f(@p v ). */
  virtual double DoEvaluate(double v) const = 0;

 private:
  const Expression e_;
};

/** Represents the base class for binary expressions.
 */
class BinaryExpressionCell : public ExpressionCell {
 public:
  void HashAppendDetail(DelegatingHasher*) const override;
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  bool Less(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  /** Returns the first argument. */
  const Expression& get_first_argument() const { return e1_; }
  /** Returns the second argument. */
  const Expression& get_second_argument() const { return e2_; }

 protected:
  /** Default constructor (DELETED). */
  BinaryExpressionCell() = delete;
  /** Move-constructs from an rvalue. */
  BinaryExpressionCell(BinaryExpressionCell&& e) = default;
  /** Copy-constructs from an lvalue. */
  BinaryExpressionCell(const BinaryExpressionCell& e) = default;
  /** Move-assigns (DELETED). */
  BinaryExpressionCell& operator=(BinaryExpressionCell&& e) = delete;
  /** Copy-assigns (DELETED). */
  BinaryExpressionCell& operator=(const BinaryExpressionCell& e) = delete;
  /** Constructs BinaryExpressionCell of kind @p k with @p e1, @p e2,
   * @p is_poly, and @p is_expanded.
   */
  BinaryExpressionCell(ExpressionKind k, const Expression& e1,
                       const Expression& e2, bool is_poly, bool is_expanded);
  /** Returns the evaluation result f(@p v1, @p v2 ). */
  virtual double DoEvaluate(double v1, double v2) const = 0;

 private:
  const Expression e1_;
  const Expression e2_;
};

/** Symbolic expression representing a variable. */
class ExpressionVar : public ExpressionCell {
 public:
  /** Constructs an expression from @p var.
   * @pre @p var is neither a dummy nor a BOOLEAN variable.
   */
  void HashAppendDetail(DelegatingHasher*) const override;
  explicit ExpressionVar(const Variable& v);
  const Variable& get_variable() const { return var_; }
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  bool Less(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Variable var_;
};

/** Symbolic expression representing a constant. */
class ExpressionConstant : public ExpressionCell {
 public:
  explicit ExpressionConstant(double v);
  double get_value() const { return v_; }
  void HashAppendDetail(DelegatingHasher*) const override;
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  bool Less(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const double v_{};
};

/** Symbolic expression representing NaN (not-a-number). */
class ExpressionNaN : public ExpressionCell {
 public:
  ExpressionNaN();
  void HashAppendDetail(DelegatingHasher*) const override;
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  bool Less(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;
};

/** Symbolic expression representing an addition which is a sum of products.
 *
 * @f[
 *     c_0 + \sum c_i * e_i
 * @f]
 *
 *  where @f$ c_i @f$ is a constant and @f$ e_i @f$ is a symbolic expression.
 *
 * Internally this class maintains a member variable @c constant_ to represent
 * @f$ c_0 @f$ and another member variable @c expr_to_coeff_map_ to represent a
 * mapping from an expression @f$ e_i @f$ to its coefficient @f$ c_i @f$ of
 * double.
 */
class ExpressionAdd : public ExpressionCell {
 public:
  /** Constructs ExpressionAdd from @p constant_term and @p term_to_coeff_map.
   */
  ExpressionAdd(double constant,
                const std::map<Expression, double>& expr_to_coeff_map);
  void HashAppendDetail(DelegatingHasher*) const override;
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  bool Less(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;
  /** Returns the constant. */
  double get_constant() const { return constant_; }
  /** Returns map from an expression to its coefficient. */
  const std::map<Expression, double>& get_expr_to_coeff_map() const {
    return expr_to_coeff_map_;
  }

 private:
  std::ostream& DisplayTerm(std::ostream& os, bool print_plus, double coeff,
                            const Expression& term) const;

  const double constant_{};
  const std::map<Expression, double> expr_to_coeff_map_;
};

/** Factory class to help build ExpressionAdd expressions. */
class ExpressionAddFactory {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExpressionAddFactory)

  /** Default constructor. */
  ExpressionAddFactory() = default;

  /** Constructs ExpressionAddFactory with @p constant and @p
   * expr_to_coeff_map. */
  ExpressionAddFactory(double constant,
                       std::map<Expression, double> expr_to_coeff_map);

  /** Constructs ExpressionAddFactory from @p ptr. */
  explicit ExpressionAddFactory(
      const std::shared_ptr<const ExpressionAdd>& ptr);

  /** Adds @p e to this factory. */
  void AddExpression(const Expression& e);
  /** Adds ExpressionAdd pointed by @p ptr to this factory. */
  void Add(const std::shared_ptr<const ExpressionAdd>& ptr);
  /** Assigns a factory from a shared pointer to ExpressionAdd.  */
  ExpressionAddFactory& operator=(
      const std::shared_ptr<const ExpressionAdd>& ptr);

  /** Negates the expressions in factory.
   * If it represents c0 + c1 * t1 + ... + * cn * tn,
   * this method flips it into -c0 - c1 * t1 - ... - cn * tn.
   * @returns *this.
   */
  ExpressionAddFactory& Negate();
  /** Returns a symbolic expression. */
  Expression GetExpression() const;

 private:
  /* Adds constant to this factory.
   * Adding constant constant into an add factory representing
   *
   *     c0 + c1 * t1 + ... + cn * tn
   *
   * results in (c0 + constant) + c1 * t1 + ... + cn * tn.  */
  void AddConstant(double constant);
  /* Adds coeff * term to this factory.
   *
   * Adding (coeff * term) into an add factory representing
   *
   *     c0 + c1 * t1 + ... + cn * tn
   *
   * results in c0 + c1 * t1 + ... + (coeff * term) + ... + cn * tn. Note that
   * it also performs simplifications to merge the coefficients of common terms.
   */
  void AddTerm(double coeff, const Expression& term);
  /* Adds expr_to_coeff_map to this factory. It calls AddConstant and AddTerm
   * methods. */
  void AddMap(const std::map<Expression, double>& expr_to_coeff_map);

  double constant_{0.0};
  std::map<Expression, double> expr_to_coeff_map_;
};

/** Symbolic expression representing a multiplication of powers.
 *
 * @f[
 *     c_0 \cdot \prod b_i^{e_i}
 * @f]
 *
 * where @f$ c_0 @f$ is a constant and @f$ b_i @f$ and @f$ e_i @f$ are symbolic
 * expressions.
 *
 * Internally this class maintains a member variable @c constant_ representing
 * @f$ c_0 @f$ and another member variable @c base_to_exponent_map_ representing
 * a mapping from a base, @f$ b_i @f$ to its exponentiation @f$ e_i @f$.
 */
class ExpressionMul : public ExpressionCell {
 public:
  /** Constructs ExpressionMul from @p constant and @p base_to_exponent_map. */
  ExpressionMul(double constant,
                const std::map<Expression, Expression>& base_to_exponent_map);
  void HashAppendDetail(DelegatingHasher*) const override;
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  bool Less(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;
  /** Returns constant term. */
  double get_constant() const { return constant_; }
  /** Returns map from a term to its coefficient. */
  const std::map<Expression, Expression>& get_base_to_exponent_map() const {
    return base_to_exponent_map_;
  }

 private:
  std::ostream& DisplayTerm(std::ostream& os, bool print_mul,
                            const Expression& base,
                            const Expression& exponent) const;

  double constant_{};
  std::map<Expression, Expression> base_to_exponent_map_;
};

/** Factory class to help build ExpressionMul expressions. */
class ExpressionMulFactory {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExpressionMulFactory)

  /** Default constructor. It constructs. */
  ExpressionMulFactory() = default;

  /** Constructs ExpressionMulFactory with @p constant and @p
   * base_to_exponent_map. */
  ExpressionMulFactory(double constant,
                       std::map<Expression, Expression> base_to_exponent_map);

  /** Constructs ExpressionMulFactory from @p ptr. */
  explicit ExpressionMulFactory(
      const std::shared_ptr<const ExpressionMul>& ptr);

  /** Adds @p e to this factory. */
  void AddExpression(const Expression& e);
  /** Adds ExpressionMul pointed by @p ptr to this factory. */
  void Add(const std::shared_ptr<const ExpressionMul>& ptr);
  /** Assigns a factory from a shared pointer to ExpressionMul.  */
  ExpressionMulFactory& operator=(
      const std::shared_ptr<const ExpressionMul>& ptr);
  /** Negates the expressions in factory.
   * If it represents c0 * p1 * ... * pn,
   * this method flips it into -c0 * p1 * ... * pn.
   * @returns *this.
   */
  ExpressionMulFactory& Negate();
  /** Returns a symbolic expression. */
  Expression GetExpression() const;

 private:
  /* Adds constant to this factory.
     Adding constant into an mul factory representing

         c * b1 ^ e1 * ... * bn ^ en

     results in (constant * c) * b1 ^ e1 * ... * bn ^ en. */
  void AddConstant(double constant);
  /* Adds pow(base, exponent) to this factory.
     Adding pow(base, exponent) into an mul factory representing

         c * b1 ^ e1 * ... * bn ^ en

     results in c * b1 ^ e1 * ... * base^exponent * ... * bn ^ en. Note that
     it also performs simplifications to merge the exponents of common bases.
  */
  void AddTerm(const Expression& base, const Expression& exponent);
  /* Adds base_to_exponent_map to this factory. It calls AddConstant and AddTerm
   * methods. */
  void AddMap(const std::map<Expression, Expression>& base_to_exponent_map);

  /* Sets to represent a zero expression. */
  void SetZero();

  double constant_{1.0};
  std::map<Expression, Expression> base_to_exponent_map_;
};

/** Symbolic expression representing division. */
class ExpressionDiv : public BinaryExpressionCell {
 public:
  ExpressionDiv(const Expression& e1, const Expression& e2);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing logarithms. */
class ExpressionLog : public UnaryExpressionCell {
 public:
  explicit ExpressionLog(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression log(const Expression& e);

 private:
  /* Throws std::domain_error if v ∉ [0, +oo). */
  static void check_domain(double v);
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing absolute value function. */
class ExpressionAbs : public UnaryExpressionCell {
 public:
  explicit ExpressionAbs(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression abs(const Expression& e);

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing exponentiation using the base of
 * natural logarithms. */
class ExpressionExp : public UnaryExpressionCell {
 public:
  explicit ExpressionExp(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing square-root. */
class ExpressionSqrt : public UnaryExpressionCell {
 public:
  explicit ExpressionSqrt(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression sqrt(const Expression& e);

 private:
  /* Throws std::domain_error if v ∉ [0, +oo). */
  static void check_domain(double v);
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing power function. */
class ExpressionPow : public BinaryExpressionCell {
 public:
  ExpressionPow(const Expression& e1, const Expression& e2);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression pow(const Expression& e1, const Expression& e2);

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
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing cosine function. */
class ExpressionCos : public UnaryExpressionCell {
 public:
  explicit ExpressionCos(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing tangent function. */
class ExpressionTan : public UnaryExpressionCell {
 public:
  explicit ExpressionTan(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing arcsine function. */
class ExpressionAsin : public UnaryExpressionCell {
 public:
  explicit ExpressionAsin(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression asin(const Expression& e);

 private:
  /* Throws std::domain_error if v ∉ [-1.0, +1.0]. */
  static void check_domain(double v);
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing arccosine function. */
class ExpressionAcos : public UnaryExpressionCell {
 public:
  explicit ExpressionAcos(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression acos(const Expression& e);

 private:
  /* Throws std::domain_error if v ∉ [-1.0, +1.0]. */
  static void check_domain(double v);
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing arctangent function. */
class ExpressionAtan : public UnaryExpressionCell {
 public:
  explicit ExpressionAtan(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing atan2 function (arctangent function with
 * two arguments). atan2(y, x) is defined as atan(y/x). */
class ExpressionAtan2 : public BinaryExpressionCell {
 public:
  ExpressionAtan2(const Expression& e1, const Expression& e2);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing hyperbolic sine function. */
class ExpressionSinh : public UnaryExpressionCell {
 public:
  explicit ExpressionSinh(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing hyperbolic cosine function. */
class ExpressionCosh : public UnaryExpressionCell {
 public:
  explicit ExpressionCosh(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing hyperbolic tangent function. */
class ExpressionTanh : public UnaryExpressionCell {
 public:
  explicit ExpressionTanh(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing min function. */
class ExpressionMin : public BinaryExpressionCell {
 public:
  ExpressionMin(const Expression& e1, const Expression& e2);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing max function. */
class ExpressionMax : public BinaryExpressionCell {
 public:
  ExpressionMax(const Expression& e1, const Expression& e2);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing ceil function. */
class ExpressionCeiling : public UnaryExpressionCell {
 public:
  explicit ExpressionCeiling(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing floor function. */
class ExpressionFloor : public UnaryExpressionCell {
 public:
  explicit ExpressionFloor(const Expression& e);
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  double DoEvaluate(double v) const override;
};

/** Symbolic expression representing if-then-else expression.  */
class ExpressionIfThenElse : public ExpressionCell {
 public:
  /** Constructs if-then-else expression from @p f_cond, @p e_then, and @p
   * e_else. */
  ExpressionIfThenElse(const Formula& f_cond, const Expression& e_then,
                       const Expression& e_else);
  void HashAppendDetail(DelegatingHasher*) const override;
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  bool Less(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  /** Returns the conditional formula. */
  const Formula& get_conditional_formula() const { return f_cond_; }
  /** Returns the 'then' expression. */
  const Expression& get_then_expression() const { return e_then_; }
  /** Returns the 'else' expression. */
  const Expression& get_else_expression() const { return e_else_; }

 private:
  const Formula f_cond_;
  const Expression e_then_;
  const Expression e_else_;
};

/** Symbolic expression representing an uninterpreted function. */
class ExpressionUninterpretedFunction : public ExpressionCell {
 public:
  /** Constructs an uninterpreted-function expression from @p name and @p
   * arguments.
   */
  ExpressionUninterpretedFunction(std::string name,
                                  std::vector<Expression> arguments);
  void HashAppendDetail(DelegatingHasher*) const override;
  Variables GetVariables() const override;
  bool EqualTo(const ExpressionCell& e) const override;
  bool Less(const ExpressionCell& e) const override;
  double Evaluate(const Environment& env) const override;
  Expression Expand() const override;
  Expression Substitute(const Substitution& s) const override;
  Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  /** Returns the name of this expression. */
  const std::string& get_name() const { return name_; }

  /** Returns the arguments of this expression. */
  const std::vector<Expression>& get_arguments() const { return arguments_; }

 private:
  const std::string name_;
  const std::vector<Expression> arguments_;
};

/** Checks if @p c is a constant expression. */
bool is_constant(const ExpressionCell& c);
/** Checks if @p c is a variable expression. */
bool is_variable(const ExpressionCell& c);
/** Checks if @p c is an addition expression. */
bool is_addition(const ExpressionCell& c);
/** Checks if @p c is an multiplication expression. */
bool is_multiplication(const ExpressionCell& c);
/** Checks if @p c is a division expression. */
bool is_division(const ExpressionCell& c);
/** Checks if @p c is a log expression. */
bool is_log(const ExpressionCell& c);
/** Checks if @p c is an absolute-value-function expression. */
bool is_abs(const ExpressionCell& c);
/** Checks if @p c is an exp expression. */
bool is_exp(const ExpressionCell& c);
/** Checks if @p c is a square-root expression. */
bool is_sqrt(const ExpressionCell& c);
/** Checks if @p c is a power-function expression. */
bool is_pow(const ExpressionCell& c);
/** Checks if @p c is a sine expression. */
bool is_sin(const ExpressionCell& c);
/** Checks if @p c is a cosine expression. */
bool is_cos(const ExpressionCell& c);
/** Checks if @p c is a tangent expression. */
bool is_tan(const ExpressionCell& c);
/** Checks if @p c is an arcsine expression. */
bool is_asin(const ExpressionCell& c);
/** Checks if @p c is an arccosine expression. */
bool is_acos(const ExpressionCell& c);
/** Checks if @p c is an arctangent expression. */
bool is_atan(const ExpressionCell& c);
/** Checks if @p c is a arctangent2  expression. */
bool is_atan2(const ExpressionCell& c);
/** Checks if @p c is a hyperbolic-sine expression. */
bool is_sinh(const ExpressionCell& c);
/** Checks if @p c is a hyperbolic-cosine expression. */
bool is_cosh(const ExpressionCell& c);
/** Checks if @p c is a hyperbolic-tangent expression. */
bool is_tanh(const ExpressionCell& c);
/** Checks if @p c is a min expression. */
bool is_min(const ExpressionCell& c);
/** Checks if @p c is a max expression. */
bool is_max(const ExpressionCell& c);
/** Checks if @p c is a ceil expression. */
bool is_ceil(const ExpressionCell& c);
/** Checks if @p c is a floor expression. */
bool is_floor(const ExpressionCell& c);
/** Checks if @p c is an if-then-else expression. */
bool is_if_then_else(const ExpressionCell& c);
/** Checks if @p c is an uninterpreted-function expression. */
bool is_uninterpreted_function(const ExpressionCell& c);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionConstant>.
 *  @pre @p *expr_ptr is of @c ExpressionConstant.
 */
std::shared_ptr<ExpressionConstant> to_constant(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionConstant>.
 *  @pre @p *(e.ptr_) is of @c ExpressionConstant.
 */
std::shared_ptr<const ExpressionConstant> to_constant(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionConstant>.
 *  @pre @p *(e->ptr_) is of @c ExpressionConstant.
 */
std::shared_ptr<ExpressionConstant> to_constant(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionVar>.
 *  @pre @p *expr_ptr is of @c ExpressionVar.
 */
std::shared_ptr<ExpressionVar> to_variable(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionVar>.
 *  @pre @p *(e.ptr_) is of @c ExpressionVar.
 */
std::shared_ptr<const ExpressionVar> to_variable(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionVar>.
 *  @pre @p *(e->ptr_) is of @c ExpressionVar.
 */
std::shared_ptr<ExpressionVar> to_variable(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<UnaryExpressionCell>.
 *  @pre @c *expr_ptr is of @c UnaryExpressionCell.
 */
std::shared_ptr<UnaryExpressionCell> to_unary(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const UnaryExpressionCell>.
 *  @pre @c *(e.ptr_) is of @c UnaryExpressionCell.
 */
std::shared_ptr<const UnaryExpressionCell> to_unary(const Expression& e);

/** Casts @p e to @c shared_ptr<UnaryExpressionCell>.
 *  @pre @c *(e->ptr_) is of @c UnaryExpressionCell.
 */
std::shared_ptr<UnaryExpressionCell> to_unary(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<BinaryExpressionCell>.
 *  @pre @c *expr_ptr is of @c BinaryExpressionCell.
 */
std::shared_ptr<BinaryExpressionCell> to_binary(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const BinaryExpressionCell>.
 *  @pre @c *(e.ptr_) is of @c BinaryExpressionCell.
 */
std::shared_ptr<const BinaryExpressionCell> to_binary(const Expression& e);

/** Casts @p e to @c shared_ptr<BinaryExpressionCell>.
 *  @pre @c *(e->ptr_) is of @c BinaryExpressionCell.
 */
std::shared_ptr<BinaryExpressionCell> to_binary(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionAdd>.
 *  @pre @c *expr_ptr is of @c ExpressionAdd.
 */
std::shared_ptr<ExpressionAdd> to_addition(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionAdd>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAdd.
 */
std::shared_ptr<const ExpressionAdd> to_addition(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionAdd>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAdd.
 */
std::shared_ptr<ExpressionAdd> to_addition(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionMul>.
 *  @pre @c *expr_ptr is of @c ExpressionMul.
 */
std::shared_ptr<ExpressionMul> to_multiplication(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionMul>.
 *  @pre @c *(e.ptr_) is of @c ExpressionMul.
 */
std::shared_ptr<const ExpressionMul> to_multiplication(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionMul>.
 *  @pre @c *(e.ptr_) is of @c ExpressionMul.
 */
std::shared_ptr<ExpressionMul> to_multiplication(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionDiv>.
 *  @pre @c *expr_ptr is of @c ExpressionDiv.
 */
std::shared_ptr<ExpressionDiv> to_division(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionDiv>.
 *  @pre @c *(e.ptr_) is of @c ExpressionDiv.
 */
std::shared_ptr<const ExpressionDiv> to_division(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionDiv>.
 *  @pre @c *(e.ptr_) is of @c ExpressionDiv.
 */
std::shared_ptr<ExpressionDiv> to_division(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionLog>.
 *  @pre @c *expr_ptr is of @c ExpressionLog.
 */
std::shared_ptr<ExpressionLog> to_log(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionLog>.
 *  @pre @c *(e.ptr_) is of @c ExpressionLog.
 */
std::shared_ptr<const ExpressionLog> to_log(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionLog>.
 *  @pre @c *(e.ptr_) is of @c ExpressionLog.
 */
std::shared_ptr<ExpressionLog> to_log(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionExp>.
 *  @pre @c *expr_ptr is of @c ExpressionExp.
 */
std::shared_ptr<ExpressionExp> to_exp(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionExp>.
 *  @pre @c *(e.ptr_) is of @c ExpressionExp.
 */
std::shared_ptr<const ExpressionExp> to_exp(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionExp>.
 *  @pre @c *(e.ptr_) is of @c ExpressionExp.
 */
std::shared_ptr<ExpressionExp> to_exp(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionAbs>.
 *  @pre @c *expr_ptr is of @c ExpressionAbs.
 */
std::shared_ptr<ExpressionAbs> to_abs(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionAbs>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAbs.
 */
std::shared_ptr<const ExpressionAbs> to_abs(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionAbs>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAbs.
 */
std::shared_ptr<ExpressionAbs> to_abs(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionExp>.
 *  @pre @c *expr_ptr is of @c ExpressionExp.
 */
std::shared_ptr<ExpressionExp> to_exp(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionExp>.
 *  @pre @c *(e.ptr_) is of @c ExpressionExp.
 */
std::shared_ptr<const ExpressionExp> to_exp(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionExp>.
 *  @pre @c *(e.ptr_) is of @c ExpressionExp.
 */
std::shared_ptr<ExpressionExp> to_exp(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionSqrt>.
 *  @pre @c *expr_ptr is of @c ExpressionSqrt.
 */
std::shared_ptr<ExpressionSqrt> to_sqrt(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionSqrt>.
 *  @pre @c *(e.ptr_) is of @c ExpressionSqrt.
 */
std::shared_ptr<const ExpressionSqrt> to_sqrt(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionSqrt>.
 *  @pre @c *(e.ptr_) is of @c ExpressionSqrt.
 */
std::shared_ptr<ExpressionSqrt> to_sqrt(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionPow>.
 *  @pre @c *expr_ptr is of @c ExpressionPow.
 */
std::shared_ptr<ExpressionPow> to_pow(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionPow>.
 *  @pre @c *(e.ptr_) is of @c ExpressionPow.
 */
std::shared_ptr<const ExpressionPow> to_pow(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionPow>.
 *  @pre @c *(e.ptr_) is of @c ExpressionPow.
 */
std::shared_ptr<ExpressionPow> to_pow(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionSin>.
 *  @pre @c *expr_ptr is of @c ExpressionSin.
 */
std::shared_ptr<ExpressionSin> to_sin(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionSin>.
 *  @pre @c *(e.ptr_) is of @c ExpressionSin.
 */
std::shared_ptr<const ExpressionSin> to_sin(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionSin>.
 *  @pre @c *(e.ptr_) is of @c ExpressionSin.
 */
std::shared_ptr<ExpressionSin> to_sin(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionCos>.
 *  @pre @c *expr_ptr is of @c ExpressionCos.
 */
std::shared_ptr<ExpressionCos> to_cos(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionCos>.
 *  @pre @c *(e.ptr_) is of @c ExpressionCos.
 */
std::shared_ptr<const ExpressionCos> to_cos(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionCos>.
 *  @pre @c *(e.ptr_) is of @c ExpressionCos.
 */
std::shared_ptr<ExpressionCos> to_cos(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionTan>.
 *  @pre @c *expr_ptr is of @c ExpressionTan.
 */
std::shared_ptr<ExpressionTan> to_tan(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionTan>.
 *  @pre @c *(e.ptr_) is of @c ExpressionTan.
 */
std::shared_ptr<const ExpressionTan> to_tan(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionTan>.
 *  @pre @c *(e.ptr_) is of @c ExpressionTan.
 */
std::shared_ptr<ExpressionTan> to_tan(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionAsin>.
 *  @pre @c *expr_ptr is of @c ExpressionAsin.
 */
std::shared_ptr<ExpressionAsin> to_asin(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionAsin>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAsin.
 */
std::shared_ptr<const ExpressionAsin> to_asin(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionAsin>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAsin.
 */
std::shared_ptr<ExpressionAsin> to_asin(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionAcos>.
 *  @pre @c *expr_ptr is of @c ExpressionAcos.
 */
std::shared_ptr<ExpressionAcos> to_acos(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionAcos>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAcos.
 */
std::shared_ptr<const ExpressionAcos> to_acos(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionAcos>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAcos.
 */
std::shared_ptr<ExpressionAcos> to_acos(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionAtan>.
 *  @pre @c *expr_ptr is of @c ExpressionAtan.
 */
std::shared_ptr<ExpressionAtan> to_atan(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionAtan>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAtan.
 */
std::shared_ptr<const ExpressionAtan> to_atan(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionAtan>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAtan.
 */
std::shared_ptr<ExpressionAtan> to_atan(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionAtan2>.
 *  @pre @c *expr_ptr is of @c ExpressionAtan2.
 */
std::shared_ptr<ExpressionAtan2> to_atan2(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionAtan2>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAtan2.
 */
std::shared_ptr<const ExpressionAtan2> to_atan2(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionAtan2>.
 *  @pre @c *(e.ptr_) is of @c ExpressionAtan2.
 */
std::shared_ptr<ExpressionAtan2> to_atan2(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionSinh>.
 *  @pre @c *expr_ptr is of @c ExpressionSinh.
 */
std::shared_ptr<ExpressionSinh> to_sinh(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionSinh>.
 *  @pre @c *(e.ptr_) is of @c ExpressionSinh.
 */
std::shared_ptr<const ExpressionSinh> to_sinh(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionSinh>.
 *  @pre @c *(e.ptr_) is of @c ExpressionSinh.
 */
std::shared_ptr<ExpressionSinh> to_sinh(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionCosh>.
 *  @pre @c *expr_ptr is of @c ExpressionCosh.
 */
std::shared_ptr<ExpressionCosh> to_cosh(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionCosh>.
 *  @pre @c *(e.ptr_) is of @c ExpressionCosh.
 */
std::shared_ptr<const ExpressionCosh> to_cosh(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionCosh>.
 *  @pre @c *(e.ptr_) is of @c ExpressionCosh.
 */
std::shared_ptr<ExpressionCosh> to_cosh(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionTanh>.
 *  @pre @c *expr_ptr is of @c ExpressionTanh.
 */
std::shared_ptr<ExpressionTanh> to_tanh(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionTanh>.
 *  @pre @c *(e.ptr_) is of @c ExpressionTanh.
 */
std::shared_ptr<const ExpressionTanh> to_tanh(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionTanh>.
 *  @pre @c *(e.ptr_) is of @c ExpressionTanh.
 */
std::shared_ptr<ExpressionTanh> to_tanh(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionMin>.
 *  @pre @c *expr_ptr is of @c ExpressionMin.
 */
std::shared_ptr<ExpressionMin> to_min(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionMin>.
 *  @pre @c *(e.ptr_) is of @c ExpressionMin.
 */
std::shared_ptr<const ExpressionMin> to_min(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionMin>.
 *  @pre @c *(e.ptr_) is of @c ExpressionMin.
 */
std::shared_ptr<ExpressionMin> to_min(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionMax>.
 *  @pre @c *expr_ptr is of @c ExpressionMax.
 */
std::shared_ptr<ExpressionMax> to_max(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionMax>.
 *  @pre @c *(e.ptr_) is of @c ExpressionMax.
 */
std::shared_ptr<const ExpressionMax> to_max(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionMax>.
 *  @pre @c *(e.ptr_) is of @c ExpressionMax.
 */
std::shared_ptr<ExpressionMax> to_max(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionCeiling>.
 *  @pre @c *expr_ptr is of @c ExpressionCeiling.
 */
std::shared_ptr<ExpressionCeiling> to_ceil(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionCeiling>.
 *  @pre @c *(e.ptr_) is of @c ExpressionCeiling.
 */
std::shared_ptr<const ExpressionCeiling> to_ceil(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionCeiling>.
 *  @pre @c *(e.ptr_) is of @c ExpressionCeiling.
 */
std::shared_ptr<ExpressionCeiling> to_ceil(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionFloor>.
 *  @pre @c *expr_ptr is of @c ExpressionFloor.
 */
std::shared_ptr<ExpressionFloor> to_floor(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionFloor>.
 *  @pre @c *(e.ptr_) is of @c ExpressionFloor.
 */
std::shared_ptr<const ExpressionFloor> to_floor(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionFloor>.
 *  @pre @c *(e.ptr_) is of @c ExpressionFloor.
 */
std::shared_ptr<ExpressionFloor> to_floor(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionIfThenElse>.
 *  @pre @c *expr_ptr is of @c ExpressionIfThenElse.
 */
std::shared_ptr<ExpressionIfThenElse> to_if_then_else(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionIfThenElse>.
 *  @pre @c *(e.ptr_) is of @c ExpressionIfThenElse.
 */
std::shared_ptr<const ExpressionIfThenElse> to_if_then_else(
    const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionIfThenElse>.
 *  @pre @c *(e.ptr_) is of @c ExpressionIfThenElse.
 */
std::shared_ptr<ExpressionIfThenElse> to_if_then_else(Expression* e);

/** Casts @p expr_ptr to @c shared_ptr<ExpressionUninterpretedFunction>.
 *  @pre @c *expr_ptr is of @c ExpressionUninterpretedFunction.
 */
std::shared_ptr<ExpressionUninterpretedFunction> to_uninterpreted_function(
    const std::shared_ptr<ExpressionCell>& expr_ptr);

/** Casts @p e to @c shared_ptr<const ExpressionUninterpretedFunction>.
 *  @pre @c *(e.ptr_) is of @c ExpressionUninterpretedFunction.
 */
std::shared_ptr<const ExpressionUninterpretedFunction>
to_uninterpreted_function(const Expression& e);

/** Casts @p e to @c shared_ptr<ExpressionUninterpretedFunction>.
 *  @pre @c *(e.ptr_) is of @c ExpressionUninterpretedFunction.
 */
std::shared_ptr<ExpressionUninterpretedFunction> to_uninterpreted_function(
    Expression* e);

}  // namespace symbolic
}  // namespace drake
