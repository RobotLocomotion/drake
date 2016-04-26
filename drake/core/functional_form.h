#pragma once

#include "drake/drakeCore_export.h"

#include <cstddef>
#include <initializer_list>
#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

namespace drake {

/** \brief Represent an abstract form of a function of zero or more variables.
 *
 * We define a functional form to be a set of input variables along with an
 * abstract description of how the variables are combined to express the
 * output.  The form may be one of:
 *
 * - \anchor zero
 *   A \b zero value with no variables (<tt>0</tt>).
 * - \anchor constant
 *   A \b constant value with no variables (<tt>c</tt>).
 * - \anchor linear
 *   A \b linear combination of one or more variables (<tt>b'*x</tt>).
 * - \anchor affine
 *   An \b affine combination of one or more variables (<tt>b'*x + c</tt>).
 * - \anchor polynomial
 *   A \b polynomial combination of one or more variables
 *   (<tt>... + x'*A*x + b'*x + c</tt>).
 * - \anchor differentiable
 *   A \b differentiable (almost everywhere) combination of one or more
 *   variables (<tt>f(x)</tt> such that <tt>f'(x)</tt> exists for
 *   <i> almost all \c x in the domain of interest</i>).
 * - \anchor arbitrary
 *   An \b arbitrary combination of one or more variables (<tt>f(x)</tt>).
 * - \anchor undefined
 *   An \b undefined combination of zero or more variables
 *   (<tt>f(x)</tt> where \c f contains an undefined operation).
 *
 * Each variable is represented by an instance of the Variable class
 * which serves as a placeholder providing a distinguishing identifier.
 * We do not represent any details about how an individual variable appears
 * in the combination of one of the above forms, only that it participates.
 */
class DRAKECORE_EXPORT FunctionalForm {
 public:
  class Variable;
  class Variables;

  /** Construct an \ref undefined form with no variables. */
  FunctionalForm();

  /** Construct a \ref constant or \ref zero form with no variables.  */
  explicit FunctionalForm(double d);

  /** Return a \ref zero form with no variables. */
  static FunctionalForm Zero();

  /** Return a \ref constant form with no variables. */
  static FunctionalForm Constant();

  /** Return a \ref linear form of one or more variables. */
  static FunctionalForm Linear(Variables v);

  /** Return an \ref affine form of one or more variables. */
  static FunctionalForm Affine(Variables v);

  /** Return a \ref polynomial form of one or more variables. */
  static FunctionalForm Polynomial(Variables v);

  /** Return a \ref differentiable form of one or more variables. */
  static FunctionalForm Differentiable(Variables v);

  /** Return an \ref arbitrary form of one or more variables. */
  static FunctionalForm Arbitrary(Variables v);

  /** Return an \ref undefined form of zero or more variables. */
  static FunctionalForm Undefined(Variables v);

  /** Return true if the form is \ref zero. */
  bool IsZero() const;

  /** Return true if the form is \ref constant. */
  bool IsConstant() const;

  /** Return true if the form is \ref linear. */
  bool IsLinear() const;

  /** Return true if the form is \ref affine. */
  bool IsAffine() const;

  /** Return true if the form is \ref polynomial. */
  bool IsPolynomial() const;

  /** Return true if the form is \ref differentiable. */
  bool IsDifferentiable() const;

  /** Return true if the form is \ref arbitrary. */
  bool IsArbitrary() const;

  /** Return true if the form is \ref undefined. */
  bool IsUndefined() const;

  /** Return true if the given form combines the same variables
      in the same way as we do.  */
  bool Is(FunctionalForm const& f) const;

  /** Return the set of variables combined by this form.  */
  Variables GetVariables() const;

  /** Print a description of this form to the given stream.
   *
   * The format is one of:
   *
   * - "zero"
   * - "cons"
   * - "lin(x,...)"
   * - "aff(x,...)"
   * - "poly(x,...)"
   * - "diff(x,...)"
   * - "arb(x,...)"
   * - "undf(x,...)"
   *
   * where "x,..." represents a comma-separated list of the variables
   * combined by the form.
   */
  friend DRAKECORE_EXPORT std::ostream& operator<<(std::ostream& os,
                                                   FunctionalForm const& f);

  /** \brief Represent a set of Variable instances.
   *
   * The set contains a list of Variable instances ordered according
   * to the \ref VariableOrdering "Variable Ordering" without duplicates.
   * The contained set is immutable.
   */
  class DRAKECORE_EXPORT Variables {
   public:
    /** Construct an empty set.  */
    Variables() = default;

    /** Construct a set from an initializer list.  */
    Variables(std::initializer_list<Variable> init);

    /** Move-construct a set from an rvalue.  */
    Variables(Variables&&) = default;

    /** Copy-construct a set from an lvalue.  */
    Variables(Variables const&) = default;

    /** Move-assign a set from an rvalue.  */
    Variables& operator=(Variables&&) = default;

    /** Copy-assign a set from an lvalue.  */
    Variables& operator=(Variables const&) = default;

    /** Return the union of two sets.  */
    static Variables Union(Variables const& l, Variables const& r);

#if defined(DRAKE_DOXYGEN_CXX)
    /** Type used to iterate through the set.  The iterator dereferences
        to <tt>const Variable</tt> and supports bidirectional iteration but
        is otherwise of an unspecified type.  */
    typedef unspecified_bidirectional_const_iterator<Variable> const_iterator;
#else
    typedef std::vector<Variable>::const_iterator const_iterator;
#endif

    /** Return an iterator at the beginning of the set.  */
    const_iterator begin() const;

    /** Return an iterator at the end of the set.  */
    const_iterator end() const;

    /** Return \c true if the set is empty and \c false otherwise.  */
    bool empty() const;

    /** Return the size of the set.  */
    std::size_t size() const;

    /** Return \c true if \c lhs and \c rhs represent the same set.  */
    friend DRAKECORE_EXPORT bool operator==(Variables const& lhs,
                                            Variables const& rhs);

    /** Return \c false if \c lhs and \c rhs represent the same set.  */
    friend DRAKECORE_EXPORT bool operator!=(Variables const& lhs,
                                            Variables const& rhs);

   private:
    explicit Variables(std::vector<Variable>&& vars);
    std::shared_ptr<std::vector<Variable> const> vars_;
  };

 private:
  enum class Form;
  FunctionalForm(Form f, Variables&& v);

  Variables vars_;
  Form form_;

#if !defined(DRAKE_DOXYGEN_CXX)
  // Internal class for friendship-access in implementation file.
  class Internal;
  friend class Internal;
#endif  // !defined(DRAKE_DOXYGEN_CXX)
};

/** \brief Represent a variable in a FunctionalForm.
 *
 * A FunctionalForm is defined with respect to zero or more variables that an
 * expression combines in some form.  Instances of Variable serve as
 * placeholders identifying distinct variables in a FunctionalForm.
 * Each Variable has a distinguishing identifier of one of these types:
 *
 * - \b nil: No identifier.
 * - \b index: Identified by a non-negative numeric index (type \c std::size_t).
 * - \b named: Identified by a non-empty string name (type \c std::string).
 *
 * Instances are considered equivalent if they have the same identification
 * type and value.
 *
 * \anchor VariableOrdering
 *
 * We define a Variable Ordering first by type (in the above order) and then by
 * the natural order of values within each type.
 */
class
#if !defined(DRAKE_DOXYGEN_CXX)
    DRAKECORE_EXPORT
#endif  // !defined(DRAKE_DOXYGEN_CXX)
        FunctionalForm::Variable {
 public:
  /** Construct the nil variable.  */
  Variable();

  /** Construct a variable identified by a non-negative numeric \p index.  */
  Variable(std::size_t index);

  /** Construct a variable identified by a non-empty string \p name. */
  Variable(std::string name);

  /** Construct a variable identified by a non-empty string \p name
      (string literal).  */
  template <int N>
  Variable(char const (&name)[N]) : Variable(std::string(name)) {}

  /** Copy-construct a variable from an lvalue.  */
  Variable(Variable const& v);

  /** Move-construct a variable from an rvalue.  */
  Variable(Variable&& v) noexcept;

  /** Destroy a variable.  */
  ~Variable();

  /** Copy-assign a variable from an lvalue.  */
  Variable& operator=(Variable const& v);

  /** Move-assign a variable from an rvalue.  */
  Variable& operator=(Variable&& v) noexcept;

  /** Return true if this variable has no identifier.  */
  bool is_nil() const;

  /** Return true if this variable is identified by a numeric index.  */
  bool is_index() const;

  /** Return true if this variable is identified by a string name.  */
  bool is_named() const;

  /** If this variable is identified by a numeric index, return the index.
      Otherwise, return the largest index representable by \c std::size_t.  */
  std::size_t index() const;

  /** If this variable is identified by a string name, return the name.
      Otherwise, return the empty string.  */
  std::string const& name() const;

  /** Print the variable's identifier, if any, to the given stream.  */
  friend DRAKECORE_EXPORT std::ostream& operator<<(std::ostream& os,
                                                   Variable const& v);

  /** Return true if both variables have the same identifier type and value.  */
  friend DRAKECORE_EXPORT bool operator==(Variable const& lhs,
                                          Variable const& rhs);

  /** Return false if both variables have the same identifier type and value. */
  friend DRAKECORE_EXPORT bool operator!=(Variable const& lhs,
                                          Variable const& rhs);

  /** Return true if \p lhs comes before \p rhs in the \ref VariableOrdering
   * "Variable Ordering".  */
  friend DRAKECORE_EXPORT bool operator<(Variable const& lhs,
                                         Variable const& rhs);

  /** Return true if \p lhs does not come after \p rhs in the \ref
   * VariableOrdering "Variable Ordering".  */
  friend DRAKECORE_EXPORT bool operator<=(Variable const& lhs,
                                          Variable const& rhs);

  /** Return true if \p lhs comes after \p rhs in the \ref VariableOrdering
   * "Variable Ordering".  */
  friend DRAKECORE_EXPORT bool operator>(Variable const& lhs,
                                         Variable const& rhs);

  /** Return true if \p lhs does not come before \p rhs in the \ref
   * VariableOrdering "Variable Ordering".  */
  friend DRAKECORE_EXPORT bool operator>=(Variable const& lhs,
                                          Variable const& rhs);

 private:
  void Destruct() noexcept;

// Hide this from doxygen because it incorrectly documents these
// members as public.  They are not user-facing anyway.
#if !defined(DRAKE_DOXYGEN_CXX)
  union {
    std::size_t index_;
    std::string name_;
  };
#endif  // !defined(DRAKE_DOXYGEN_CXX)

  enum class Tag;
  Tag tag_;
};

}  // namespace drake
