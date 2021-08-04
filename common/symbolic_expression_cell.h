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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExpressionCell)

  /** Returns expression kind. */
  [[nodiscard]] ExpressionKind get_kind() const { return kind_; }

  /** Sends all hash-relevant bytes for this ExpressionCell type into the given
   * hasher, per the @ref hash_append concept -- except for get_kind(), because
   * Expression already sends that.
   */
  virtual void HashAppendDetail(DelegatingHasher*) const = 0;

  /** Collects variables in expression. */
  [[nodiscard]] virtual Variables GetVariables() const = 0;

  /** Checks structural equality. */
  [[nodiscard]] virtual bool EqualTo(const ExpressionCell& c) const = 0;

  /** Provides lexicographical ordering between expressions. */
  [[nodiscard]] virtual bool Less(const ExpressionCell& c) const = 0;

  /** Checks if this symbolic expression is convertible to Polynomial. */
  [[nodiscard]] bool is_polynomial() const { return is_polynomial_; }

  /** Checks if this symbolic expression is already expanded. */
  [[nodiscard]] bool is_expanded() const { return is_expanded_; }

  /** Sets this symbolic expression as already expanded. */
  void set_expanded() { is_expanded_ = true; }

  /** Evaluates under a given environment (by default, an empty environment).
   *  @throws std::exception if NaN is detected during evaluation.
   */
  [[nodiscard]] virtual double Evaluate(const Environment& env) const = 0;

  /** Expands out products and positive integer powers in expression.
   * @throws std::exception if NaN is detected during expansion.
   */
  [[nodiscard]] virtual Expression Expand() const = 0;

  /** Returns an Expression obtained by replacing all occurrences of the
   * variables in @p s in the current expression cell with the corresponding
   * expressions in @p s.
   * @throws std::exception if NaN is detected during substitution.
   */
  [[nodiscard]] virtual Expression Substitute(const Substitution& s) const = 0;

  /** Differentiates this symbolic expression with respect to the variable @p
   * var.
   * @throws std::exception if it is not differentiable.
   */
  [[nodiscard]] virtual Expression Differentiate(const Variable& x) const = 0;

  /** Outputs string representation of expression into output stream @p os. */
  virtual std::ostream& Display(std::ostream& os) const = 0;

 protected:
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
  [[nodiscard]] Variables GetVariables() const override;
  [[nodiscard]] bool EqualTo(const ExpressionCell& e) const override;
  [[nodiscard]] bool Less(const ExpressionCell& e) const override;
  [[nodiscard]] double Evaluate(const Environment& env) const override;
  /** Returns the argument. */
  [[nodiscard]] const Expression& get_argument() const { return e_; }

 protected:
  /** Constructs UnaryExpressionCell of kind @p k with @p e, @p is_poly, and @p
   * is_expanded. */
  UnaryExpressionCell(ExpressionKind k, Expression e, bool is_poly,
                      bool is_expanded);
  /** Returns the evaluation result f(@p v ). */
  [[nodiscard]] virtual double DoEvaluate(double v) const = 0;

 private:
  const Expression e_;
};

/** Represents the base class for binary expressions.
 */
class BinaryExpressionCell : public ExpressionCell {
 public:
  void HashAppendDetail(DelegatingHasher*) const override;
  [[nodiscard]] Variables GetVariables() const override;
  [[nodiscard]] bool EqualTo(const ExpressionCell& e) const override;
  [[nodiscard]] bool Less(const ExpressionCell& e) const override;
  [[nodiscard]] double Evaluate(const Environment& env) const override;
  /** Returns the first argument. */
  [[nodiscard]] const Expression& get_first_argument() const { return e1_; }
  /** Returns the second argument. */
  [[nodiscard]] const Expression& get_second_argument() const { return e2_; }

 protected:
  /** Constructs BinaryExpressionCell of kind @p k with @p e1, @p e2,
   * @p is_poly, and @p is_expanded.
   */
  BinaryExpressionCell(ExpressionKind k, Expression e1, Expression e2,
                       bool is_poly, bool is_expanded);
  /** Returns the evaluation result f(@p v1, @p v2 ). */
  [[nodiscard]] virtual double DoEvaluate(double v1, double v2) const = 0;

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
  explicit ExpressionVar(Variable v);
  [[nodiscard]] const Variable& get_variable() const { return var_; }
  [[nodiscard]] Variables GetVariables() const override;
  [[nodiscard]] bool EqualTo(const ExpressionCell& e) const override;
  [[nodiscard]] bool Less(const ExpressionCell& e) const override;
  [[nodiscard]] double Evaluate(const Environment& env) const override;
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const Variable var_;
};

/** Symbolic expression representing a constant. */
class ExpressionConstant : public ExpressionCell {
 public:
  explicit ExpressionConstant(double v);
  [[nodiscard]] double get_value() const { return v_; }
  void HashAppendDetail(DelegatingHasher*) const override;
  [[nodiscard]] Variables GetVariables() const override;
  [[nodiscard]] bool EqualTo(const ExpressionCell& e) const override;
  [[nodiscard]] bool Less(const ExpressionCell& e) const override;
  [[nodiscard]] double Evaluate(const Environment& env) const override;
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  const double v_{};
};

/** Symbolic expression representing NaN (not-a-number). */
class ExpressionNaN : public ExpressionCell {
 public:
  ExpressionNaN();
  void HashAppendDetail(DelegatingHasher*) const override;
  [[nodiscard]] Variables GetVariables() const override;
  [[nodiscard]] bool EqualTo(const ExpressionCell& e) const override;
  [[nodiscard]] bool Less(const ExpressionCell& e) const override;
  [[nodiscard]] double Evaluate(const Environment& env) const override;
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
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
  [[nodiscard]] Variables GetVariables() const override;
  [[nodiscard]] bool EqualTo(const ExpressionCell& e) const override;
  [[nodiscard]] bool Less(const ExpressionCell& e) const override;
  [[nodiscard]] double Evaluate(const Environment& env) const override;
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;
  /** Returns the constant. */
  [[nodiscard]] double get_constant() const { return constant_; }
  /** Returns map from an expression to its coefficient. */
  [[nodiscard]] const std::map<Expression, double>& get_expr_to_coeff_map()
      const {
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

  /** Constructs ExpressionAddFactory from @p add. */
  explicit ExpressionAddFactory(const ExpressionAdd& add);

  /** Adds @p e to this factory. */
  void AddExpression(const Expression& e);
  /** Adds ExpressionAdd pointed by @p ptr to this factory. */
  void Add(const ExpressionAdd& add);
  /** Assigns a factory from a an ExpressionAdd.  */
  ExpressionAddFactory& operator=(const ExpressionAdd& ptr);

  /** Negates the expressions in factory.
   * If it represents c0 + c1 * t1 + ... + * cn * tn,
   * this method flips it into -c0 - c1 * t1 - ... - cn * tn.
   * @returns *this.
   */
  ExpressionAddFactory& Negate();
  /** Returns a symbolic expression. */
  [[nodiscard]] Expression GetExpression() const;

 private:
  /* Adds a constant @p constant to this factory.
   * Adding constant into an add factory representing
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
  [[nodiscard]] Variables GetVariables() const override;
  [[nodiscard]] bool EqualTo(const ExpressionCell& e) const override;
  [[nodiscard]] bool Less(const ExpressionCell& e) const override;
  [[nodiscard]] double Evaluate(const Environment& env) const override;
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;
  /** Returns constant term. */
  [[nodiscard]] double get_constant() const { return constant_; }
  /** Returns map from a term to its coefficient. */
  [[nodiscard]] const std::map<Expression, Expression>&
  get_base_to_exponent_map() const {
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

  /** Default constructor. */
  ExpressionMulFactory() = default;

  /** Constructs ExpressionMulFactory with @p constant and @p
   * base_to_exponent_map. */
  ExpressionMulFactory(double constant,
                       std::map<Expression, Expression> base_to_exponent_map);

  /** Constructs ExpressionMulFactory from @p mul. */
  explicit ExpressionMulFactory(const ExpressionMul& mul);

  /** Adds @p e to this factory. */
  void AddExpression(const Expression& e);
  /** Adds ExpressionMul pointed by @p ptr to this factory. */
  void Add(const ExpressionMul& ptr);
  /** Assigns a factory from an ExpressionMul.  */
  ExpressionMulFactory& operator=(const ExpressionMul& ptr);
  /** Negates the expressions in factory.
   * If it represents c0 * p1 * ... * pn,
   * this method flips it into -c0 * p1 * ... * pn.
   * @returns *this.
   */
  ExpressionMulFactory& Negate();
  /** Returns a symbolic expression. */
  [[nodiscard]] Expression GetExpression() const;

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
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing logarithms. */
class ExpressionLog : public UnaryExpressionCell {
 public:
  explicit ExpressionLog(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression log(const Expression& e);

 private:
  /* Throws std::exception if v ∉ [0, +oo). */
  static void check_domain(double v);
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing absolute value function. */
class ExpressionAbs : public UnaryExpressionCell {
 public:
  explicit ExpressionAbs(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression abs(const Expression& e);

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing exponentiation using the base of
 * natural logarithms. */
class ExpressionExp : public UnaryExpressionCell {
 public:
  explicit ExpressionExp(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing square-root. */
class ExpressionSqrt : public UnaryExpressionCell {
 public:
  explicit ExpressionSqrt(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression sqrt(const Expression& e);

 private:
  /* Throws std::exception if v ∉ [0, +oo). */
  static void check_domain(double v);
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing power function. */
class ExpressionPow : public BinaryExpressionCell {
 public:
  ExpressionPow(const Expression& e1, const Expression& e2);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression pow(const Expression& e1, const Expression& e2);

 private:
  /* Throws std::exception if v1 is finite negative and v2 is finite
     non-integer. */
  static void check_domain(double v1, double v2);
  [[nodiscard]] double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing sine function. */
class ExpressionSin : public UnaryExpressionCell {
 public:
  explicit ExpressionSin(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing cosine function. */
class ExpressionCos : public UnaryExpressionCell {
 public:
  explicit ExpressionCos(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing tangent function. */
class ExpressionTan : public UnaryExpressionCell {
 public:
  explicit ExpressionTan(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing arcsine function. */
class ExpressionAsin : public UnaryExpressionCell {
 public:
  explicit ExpressionAsin(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression asin(const Expression& e);

 private:
  /* Throws std::exception if v ∉ [-1.0, +1.0]. */
  static void check_domain(double v);
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing arccosine function. */
class ExpressionAcos : public UnaryExpressionCell {
 public:
  explicit ExpressionAcos(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  friend Expression acos(const Expression& e);

 private:
  /* Throws std::exception if v ∉ [-1.0, +1.0]. */
  static void check_domain(double v);
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing arctangent function. */
class ExpressionAtan : public UnaryExpressionCell {
 public:
  explicit ExpressionAtan(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing atan2 function (arctangent function with
 * two arguments). atan2(y, x) is defined as atan(y/x). */
class ExpressionAtan2 : public BinaryExpressionCell {
 public:
  ExpressionAtan2(const Expression& e1, const Expression& e2);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing hyperbolic sine function. */
class ExpressionSinh : public UnaryExpressionCell {
 public:
  explicit ExpressionSinh(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing hyperbolic cosine function. */
class ExpressionCosh : public UnaryExpressionCell {
 public:
  explicit ExpressionCosh(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing hyperbolic tangent function. */
class ExpressionTanh : public UnaryExpressionCell {
 public:
  explicit ExpressionTanh(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing min function. */
class ExpressionMin : public BinaryExpressionCell {
 public:
  ExpressionMin(const Expression& e1, const Expression& e2);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing max function. */
class ExpressionMax : public BinaryExpressionCell {
 public:
  ExpressionMax(const Expression& e1, const Expression& e2);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v1, double v2) const override;
};

/** Symbolic expression representing ceil function. */
class ExpressionCeiling : public UnaryExpressionCell {
 public:
  explicit ExpressionCeiling(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing floor function. */
class ExpressionFloor : public UnaryExpressionCell {
 public:
  explicit ExpressionFloor(const Expression& e);
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

 private:
  [[nodiscard]] double DoEvaluate(double v) const override;
};

/** Symbolic expression representing if-then-else expression.  */
class ExpressionIfThenElse : public ExpressionCell {
 public:
  /** Constructs if-then-else expression from @p f_cond, @p e_then, and @p
   * e_else. */
  ExpressionIfThenElse(Formula f_cond, Expression e_then, Expression e_else);
  void HashAppendDetail(DelegatingHasher*) const override;
  [[nodiscard]] Variables GetVariables() const override;
  [[nodiscard]] bool EqualTo(const ExpressionCell& e) const override;
  [[nodiscard]] bool Less(const ExpressionCell& e) const override;
  [[nodiscard]] double Evaluate(const Environment& env) const override;
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  /** Returns the conditional formula. */
  [[nodiscard]] const Formula& get_conditional_formula() const {
    return f_cond_;
  }
  /** Returns the 'then' expression. */
  [[nodiscard]] const Expression& get_then_expression() const {
    return e_then_;
  }
  /** Returns the 'else' expression. */
  [[nodiscard]] const Expression& get_else_expression() const {
    return e_else_;
  }

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
  [[nodiscard]] Variables GetVariables() const override;
  [[nodiscard]] bool EqualTo(const ExpressionCell& e) const override;
  [[nodiscard]] bool Less(const ExpressionCell& e) const override;
  [[nodiscard]] double Evaluate(const Environment& env) const override;
  [[nodiscard]] Expression Expand() const override;
  [[nodiscard]] Expression Substitute(const Substitution& s) const override;
  [[nodiscard]] Expression Differentiate(const Variable& x) const override;
  std::ostream& Display(std::ostream& os) const override;

  /** Returns the name of this expression. */
  [[nodiscard]] const std::string& get_name() const { return name_; }

  /** Returns the arguments of this expression. */
  [[nodiscard]] const std::vector<Expression>& get_arguments() const {
    return arguments_;
  }

 private:
  const std::string name_;
  const std::vector<Expression> arguments_;
};

/** Checks if @p c is a constant expression. */
bool is_constant(const ExpressionCell& c);
/** Checks if @p c is a variable expression. */
bool is_variable(const ExpressionCell& c);
/** Checks if @p c is a unary expression. */
bool is_unary(const ExpressionCell& c);
/** Checks if @p c is a binary expression. */
bool is_binary(const ExpressionCell& c);
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

}  // namespace symbolic
}  // namespace drake
