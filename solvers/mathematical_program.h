#pragma once

#include <array>
#include <cstddef>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/create_constraint.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/function.h"
#include "drake/solvers/indeterminate.h"
#include "drake/solvers/program_attribute.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace solvers {

template <int...>
struct NewVariableNames {};
/**
 * The type of the names for the newly added variables.
 * @tparam Size If Size is a fixed non-negative integer, then the type of the
 * name is std::array<std::string, Size>. Otherwise the type is
 * std::vector<std::string>.
 */
template <int Size>
struct NewVariableNames<Size> {
  typedef std::array<std::string, Size> type;
};

template <>
struct NewVariableNames<Eigen::Dynamic> {
  typedef std::vector<std::string> type;
};

template <int Rows, int Cols>
struct NewVariableNames<Rows, Cols>
    : public NewVariableNames<MultiplyEigenSizes<Rows, Cols>::value> {};

template <int Rows>
struct NewSymmetricVariableNames
    : public NewVariableNames<Rows == Eigen::Dynamic ? Eigen::Dynamic
                                                     : Rows*(Rows + 1) / 2> {};

namespace internal {
/**
 * Return un-initialized new variable names.
 */
template <int Size>
typename std::enable_if<Size >= 0, typename NewVariableNames<Size>::type>::type
CreateNewVariableNames(int) {
  typename NewVariableNames<Size>::type names;
  return names;
}

/**
 * Return un-initialized new variable names.
 */
template <int Size>
typename std::enable_if<Size == Eigen::Dynamic,
                        typename NewVariableNames<Size>::type>::type
CreateNewVariableNames(int size) {
  typename NewVariableNames<Eigen::Dynamic>::type names(size);
  return names;
}
/**
 * Set the names of the newly added variables.
 * @param name The common name of all new variables.
 * @param rows The number of rows in the new variables.
 * @param cols The number of columns in the new variables.
 * @pre The size of @p names is @p rows * @p cols.
 */
template <typename Derived>
void SetVariableNames(const std::string& name, int rows, int cols,
                      Derived* names) {
  DRAKE_DEMAND(static_cast<int>(names->size()) == rows * cols);
  if (cols == 1) {
    for (int i = 0; i < rows; ++i) {
      (*names)[i] = name + "(" + std::to_string(i) + ")";
    }
  } else {
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        (*names)[j * rows + i] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      }
    }
  }
}

/**
 * Template condition to only catch when Constraints are inadvertently passed
 * as an argument. If the class is binding-compatible with a Constraint, then
 * this will provide a static assertion to enable easier debugging of which
 * type failed.
 * @tparam F The type to be tested.
 * @see http://stackoverflow.com/a/13366183/7829525
 */
template <typename F>
struct assert_if_is_constraint {
  static constexpr bool value = is_binding_compatible<F, Constraint>::value;
  // Use deferred evaluation
  static_assert(
      !value,
      "You cannot pass a Constraint to create a Cost object from a function. "
      "Please ensure you are passing a Cost.");
};
}  // namespace internal

/**
 * MathematicalProgram stores the decision variables, the constraints and costs
 * of an optimization problem. The user can solve the problem by calling
 * solvers::Solve() function, and obtain the results of the optimization.
 *
 * @ingroup solvers
 */
class MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MathematicalProgram)
  using VarType = symbolic::Variable::Type;

  /// The optimal cost is +∞ when the problem is globally infeasible.
  static constexpr double kGlobalInfeasibleCost =
      std::numeric_limits<double>::infinity();
  /// The optimal cost is -∞ when the problem is unbounded.
  static constexpr double kUnboundedCost =
      -std::numeric_limits<double>::infinity();

  MathematicalProgram();
  virtual ~MathematicalProgram();

  /** Clones an optimization program.
   * The clone will be functionally equivalent to the source program with the
   * same:
   *
   * - decision variables
   * - constraints
   * - costs
   * - solver settings
   * - initial guess
   *
   * However, the clone's x values will be initialized to NaN, and all internal
   * solvers will be freshly constructed.
   * @retval new_prog. The newly constructed mathematical program.
   */
  std::unique_ptr<MathematicalProgram> Clone() const;

  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @param rows  The number of rows in the new variables.
   * @param name The name of the newly added variables
   * @return The VectorDecisionVariable of size rows x 1, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables(2, "x");
   * @endcode
   * This adds a 2 x 1 vector containing decision variables into the program.
   * The names of the variables are "x(0)" and "x(1)".
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  VectorXDecisionVariable NewContinuousVariables(
      int rows, const std::string& name = "x") {
    return NewContinuousVariables<Eigen::Dynamic, 1>(rows, 1, name);
  }

  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam Rows The number of rows of the new variables, in the compile time.
   * @tparam Cols The number of columns of the new variables, in the compile
   * time.
   * @param rows The number of rows in the new variables. When Rows is not
   * Eigen::Dynamic, rows is ignored.
   * @param cols The number of columns in the new variables. When Cols is not
   * Eigen::Dynamic, cols is ignored.
   * @param name All variables will share the same name, but different index.
   * @return The MatrixDecisionVariable of size Rows x Cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables(2, 3, "X");
   * auto y = prog.NewContinuousVariables<2, 3>(2, 3, "X");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  template <int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
  MatrixDecisionVariable<Rows, Cols> NewContinuousVariables(
      int rows, int cols, const std::string& name = "X") {
    rows = Rows == Eigen::Dynamic ? rows : Rows;
    cols = Cols == Eigen::Dynamic ? cols : Cols;
    auto names =
        internal::CreateNewVariableNames<MultiplyEigenSizes<Rows, Cols>::value>(
            rows * cols);
    internal::SetVariableNames(name, rows, cols, &names);
    return NewVariables<Rows, Cols>(VarType::CONTINUOUS, names, rows, cols);
  }

  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam Rows  The number of rows in the new variables.
   * @tparam Cols  The number of columns in the new variables. The default is 1.
   * @param name All variables will share the same name, but different index.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables<2, 3>("X");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  template <int Rows, int Cols = 1>
  MatrixDecisionVariable<Rows, Cols> NewContinuousVariables(
      const std::string& name = "X") {
    return NewContinuousVariables<Rows, Cols>(Rows, Cols, name);
  }

  /**
   * Adds binary variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam Rows  The number of rows in the new variables.
   * @tparam Cols  The number of columns in the new variables.
   * @param rows The number of rows in the new variables.
   * @param cols The number of columns in the new variables.
   * @param name The commonly shared name of the new variables.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto b = prog.NewBinaryVariables(2, 3, "b");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  template <int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
  MatrixDecisionVariable<Rows, Cols> NewBinaryVariables(
      int rows, int cols, const std::string& name) {
    rows = Rows == Eigen::Dynamic ? rows : Rows;
    cols = Cols == Eigen::Dynamic ? cols : Cols;
    auto names =
        internal::CreateNewVariableNames<MultiplyEigenSizes<Rows, Cols>::value>(
            rows * cols);
    internal::SetVariableNames(name, rows, cols, &names);
    return NewVariables<Rows, Cols>(VarType::BINARY, names, rows, cols);
  }

  /**
   * Adds a matrix of binary variables into the optimization program.
   * @tparam Rows The number of rows in the newly added binary variables.
   * @tparam Cols  The number of columns in the new variables. The default is 1.
   * @param name Each newly added binary variable will share the same name. The
   * default name is "b".
   * @return A matrix containing the newly added variables.
   */
  template <int Rows, int Cols = 1>
  MatrixDecisionVariable<Rows, Cols> NewBinaryVariables(
      const std::string& name = "b") {
    return NewBinaryVariables<Rows, Cols>(Rows, Cols, name);
  }

  /**
   * Adds binary variables to this MathematicalProgram. The new variables are
   * viewed as a column vector, with size @p rows x 1.
   * @see NewBinaryVariables(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  VectorXDecisionVariable NewBinaryVariables(int rows,
                                             const std::string& name = "b") {
    return NewBinaryVariables<Eigen::Dynamic, 1>(rows, 1, name);
  }

  /**
   * Adds a runtime sized symmetric matrix as decision variables to
   * this MathematicalProgram.
   * The optimization will only use the stacked columns of the
   * lower triangular part of the symmetric matrix as decision variables.
   * @param rows The number of rows in the symmetric matrix.
   * @param name The name of the matrix. It is only used the for user to
   * understand the optimization program. The default name is "Symmetric", and
   * each variable will be named as
   * <pre>
   * Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
   * Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
   *            ...
   * Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)
   * </pre>
   * Notice that the (i,j)'th entry and (j,i)'th entry has the same name.
   * @return The newly added decision variables.
   */
  MatrixXDecisionVariable NewSymmetricContinuousVariables(
      int rows, const std::string& name = "Symmetric");

  /**
   * Adds a static sized symmetric matrix as decision variables to
   * this MathematicalProgram.
   * The optimization will only use the stacked columns of the
   * lower triangular part of the symmetric matrix as decision variables.
   * @tparam rows The number of rows in the symmetric matrix.
   * @param name The name of the matrix. It is only used the for user to
   * understand the optimization program. The default name is "Symmetric", and
   * each variable will be named as
   * <pre>
   * Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
   * Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
   *            ...
   * Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)
   * </pre>
   * Notice that the (i,j)'th entry and (j,i)'th entry has the same name.
   * @return The newly added decision variables.
   */
  template <int rows>
  MatrixDecisionVariable<rows, rows> NewSymmetricContinuousVariables(
      const std::string& name = "Symmetric") {
    typename NewSymmetricVariableNames<rows>::type names;
    int var_count = 0;
    for (int j = 0; j < static_cast<int>(rows); ++j) {
      for (int i = j; i < static_cast<int>(rows); ++i) {
        names[var_count] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
        ++var_count;
      }
    }
    return NewSymmetricVariables<rows>(VarType::CONTINUOUS, names);
  }

  /** Appends new variables to the end of the existing variables.
   * @param decision_variables The newly added decision_variables.
   * @pre `decision_variables` should not intersect with the existing variables
   * or indeterminates in the optimization program.
   * @pre Each entry in `decision_variables` should not be a dummy variable.
   * @throws std::runtime_error if the preconditions are not satisfied.
   */
  void AddDecisionVariables(
      const Eigen::Ref<const VectorXDecisionVariable>& decision_variables);

  /**
   * Returns a free polynomial in a monomial basis over @p indeterminates of a
   * given @p degree. It uses @p coeff_name to make new decision variables and
   * use them as coefficients. For example, `NewFreePolynomial({x₀, x₁}, 2)`
   * returns a₀x₁² + a₁x₀x₁ + a₂x₀² + a₃x₁ + a₄x₀ + a₅.
   */
  symbolic::Polynomial NewFreePolynomial(
      const symbolic::Variables& indeterminates, int degree,
      const std::string& coeff_name = "a");

  /**
   * Types of non-negative polynomial that can be found through conic
   * optimization. We currently support SOS, SDSOS and DSOS. For more
   * information about these polynomial types, please refer to
   * "DSOS and SDSOS Optimization: More Tractable
   * Alternatives to Sum of Squares and Semidefinite Optimization" by Amir Ali
   * Ahmadi and Anirudha Majumdar, with arXiv link
   * https://arxiv.org/abs/1706.02586
   */
  enum class NonnegativePolynomial {
    kSos,    ///< A sum-of-squares polynomial.
    kSdsos,  ///< A scaled-diagonally dominant sum-of-squares polynomial.
    kDsos,   ///< A diagonally dominant sum-of-squares polynomial.
  };

  /**
   * Returns a pair of nonnegative polynomial p = mᵀQm and the Grammian
   * matrix Q, where m is @p monomial_basis. Adds Q as decision variables to the
   * program. Depending on the type of the polynomial, we will impose different
   * constraint on Q.
   * - if type = kSos, we impose Q being positive semidefinite.
   * - if type = kSdsos, we impose Q being scaled diagonally dominant.
   * - if type = kDsos, we impose Q being positive diagonally dominant.
   * @param monomial_basis The monomial basis.
   * @param type The type of the nonnegative polynomial.
   * @return (p, Q) The polynomial p and the Grammian matrix Q. Q has been
   * added as decision variables to the program.
   */
  std::pair<symbolic::Polynomial, MatrixXDecisionVariable>
  NewNonnegativePolynomial(
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis,
      NonnegativePolynomial type);

  /**
   * Overloads NewNonnegativePolynomial(), except the Grammian matrix Q is an
   * input instead of an output.
   */
  symbolic::Polynomial NewNonnegativePolynomial(
      const Eigen::Ref<const MatrixX<symbolic::Variable>>& grammian,
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis,
      NonnegativePolynomial type);

  /**
   * Overloads NewNonnegativePolynomial(). Instead of passing the monomial
   * basis, we use a monomial basis that contains all monomials of @p
   * indeterminates of total order up to @p degree / 2, hence the returned
   * polynomial p contains all the monomials of @p indeterminates of total order
   * up to @p degree.
   * @param indeterminates All the indeterminates in the polynomial p.
   * @param degree The polynomial p will contain all the monomials up to order
   * @p degree.
   * @param type The type of the nonnegative polynomial.
   * @return (p, Q) The polynomial p and the Grammian matrix Q. Q has been
   * added as decision variables to the program.
   * @pre @p degree is a positive even number.
   */
  std::pair<symbolic::Polynomial, MatrixXDecisionVariable>
  NewNonnegativePolynomial(const symbolic::Variables& indeterminates,
                           int degree, NonnegativePolynomial type);

  /** Returns a pair of a SOS polynomial p = mᵀQm and the Grammian matrix Q,
   * where m is the @p monomial basis.
   * For example, `NewSosPolynomial(Vector2<Monomial>{x,y})` returns a
   * polynomial
   *   p = Q₍₀,₀₎x² + 2Q₍₁,₀₎xy + Q₍₁,₁₎y²
   * and Q.
   * @note Q is a symmetric monomial_basis.rows() x monomial_basis.rows()
   * matrix.
   */
  std::pair<symbolic::Polynomial, MatrixXDecisionVariable> NewSosPolynomial(
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis);

  /** Returns a pair of a SOS polynomial p = m(x)ᵀQm(x) of degree @p degree
   * and the Grammian matrix Q that should be PSD, where m(x) is the
   * result of calling `MonomialBasis(indeterminates, degree/2)`. For example,
   * `NewSosPolynomial({x}, 4)` returns a pair of a polynomial
   *   p = Q₍₀,₀₎x⁴ + 2Q₍₁,₀₎ x³ + (2Q₍₂,₀₎ + Q₍₁,₁₎)x² + 2Q₍₂,₁₎x + Q₍₂,₂₎
   * and Q.
   *
   * @throws std::runtime_error if @p degree is not a positive even integer.
   * @see MonomialBasis.
   */
  std::pair<symbolic::Polynomial, MatrixXDecisionVariable> NewSosPolynomial(
      const symbolic::Variables& indeterminates, int degree);

  /**
   * Creates a symbolic polynomial from the given expression `e`. It uses this
   * MathematicalProgram's `indeterminates()` in constructing the polynomial.
   *
   * This method helps a user create a polynomial with the right set of
   * indeterminates which are declared in this MathematicalProgram. We recommend
   * users to use this method over an explicit call to Polynomial constructors
   * to avoid a possible mismatch between this MathematicalProgram's
   * indeterminates and the user-specified indeterminates (or unspecified, which
   * then includes all symbolic variables in the expression `e`). Consider the
   * following example.
   *
   *   e = ax + bx + c
   *
   *   MP.indeterminates()     = {x}
   *   MP.decision_variables() = {a, b}
   *
   * - `MP.MakePolynomial(e)` create a polynomial, `(a + b)x + c`.  Here only
   *   `x` is an indeterminate of this polynomial.
   *
   * - In contrast, `symbolic::Polynomial(e)` returns `ax + bx + c` where all
   *   variables `{a, b, x}` are indeterminates. Note that this is problematic
   *   as its indeterminates, `{a, b, x}` and the MathematicalProgram's decision
   *   variables, `{a, b}` overlap.
   *
   * @note This function does not require that the decision variables in `e` is
   * a subset of the decision variables in MathematicalProgram.
   */
  [[nodiscard]] symbolic::Polynomial MakePolynomial(
      const symbolic::Expression& e) const;

  /**
   * Reparses the polynomial `p` using this MathematicalProgram's
   * indeterminates.
   */
  void Reparse(symbolic::Polynomial* p) const;

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing indeterminates.
   * @tparam rows  The number of rows in the new indeterminates.
   * @tparam cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * std::array<std::string, 6> names = {"x1", "x2", "x3", "x4", "x5", "x6"};
   * auto x = prog.NewIndeterminates<2, 3>(names);
   * @endcode
   * This adds a 2 x 3 matrix indeterminates into the program.
   *
   * The name of the indeterminates is only used for the user in order to ease
   * readability.
   *
   * @exclude_from_pydrake_mkdoc{Overloads that require explicit template
   * arguments (rows, cols) are not bound in pydrake.}
   */
  template <int rows, int cols>
  MatrixIndeterminate<rows, cols> NewIndeterminates(
      const std::array<std::string, rows * cols>& names) {
    MatrixIndeterminate<rows, cols> indeterminates_matrix;
    NewIndeterminates_impl(names, indeterminates_matrix);
    return indeterminates_matrix;
  }

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing indeterminates.
   * @tparam rows  The number of rows in the new indeterminates.
   * @tparam cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * std::array<std::string, 2> names = {"x1", "x2"};
   * auto x = prog.NewIndeterminates<2>(names);
   * @endcode
   * This adds a 2 vector indeterminates into the program.
   *
   * The name of the indeterminates is only used for the user in order to ease
   * readability.
   *
   * @exclude_from_pydrake_mkdoc{Overloads that require explicit template
   * arguments (rows) are not bound in pydrake.}
   */
  template <int rows>
  VectorIndeterminate<rows> NewIndeterminates(
      const std::array<std::string, rows>& names) {
    return NewIndeterminates<rows, 1>(names);
  }

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing indeterminates.
   * @tparam rows  The number of rows in the new indeterminates.
   * @tparam cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewIndeterminates<2, 3>("X");
   * @endcode
   * This adds a 2 x 3 matrix indeterminates into the program.
   *
   * The name of the indeterminates is only used for the user in order to ease
   * readability.
   *
   * @exclude_from_pydrake_mkdoc{Overloads that require explicit template
   * arguments (rows, cols) are not bound in pydrake.}
   */
  template <int rows, int cols>
  MatrixIndeterminate<rows, cols> NewIndeterminates(
      const std::string& name = "X") {
    std::array<std::string, rows * cols> names;
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        names[j * rows + i] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      }
    }
    return NewIndeterminates<rows, cols>(names);
  }

  /**
   * Adds indeterminates to the program.
   * The name for all newly added indeterminates are set to @p name. The default
   * name is "x"
   * @see NewIndeterminates(const std::array<std::string, rows>& names)
   *
   * @exclude_from_pydrake_mkdoc{Overloads that require explicit template
   * arguments (rows) are not bound in pydrake.}
   */
  template <int rows>
  VectorIndeterminate<rows> NewIndeterminates(const std::string& name = "x") {
    std::array<std::string, rows> names;
    int offset = (name.compare("x") == 0) ? num_vars() : 0;
    for (int i = 0; i < rows; ++i) {
      names[i] = name + "(" + std::to_string(offset + i) + ")";
    }
    return NewIndeterminates<rows>(names);
  }

  /**
   * Adds indeterminates to this MathematicalProgram.
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  VectorXIndeterminate NewIndeterminates(int rows,
                                         const std::vector<std::string>& names);

  /**
   * Adds indeterminates to this MathematicalProgram, with default name
   * "x".
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  VectorXIndeterminate NewIndeterminates(int rows,
                                         const std::string& name = "x");

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing vars.
   * @param rows  The number of rows in the new indeterminates.
   * @param cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewIndeterminates(2, 3, {"x1", "x2", "x3", "x4",
   * "x5", "x6"});
   * @endcode
   * This adds a 2 x 3 matrix indeterminates into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  MatrixXIndeterminate NewIndeterminates(int rows, int cols,
                                         const std::vector<std::string>& names);

  /**
   * Adds indeterminates to this MathematicalProgram, with default name
   * "X". The new variables are returned and viewed as a matrix, with size
   * @p rows x @p cols.
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  MatrixXIndeterminate NewIndeterminates(int rows, int cols,
                                         const std::string& name = "X");

  /** Adds indeterminates.
   * This method appends some indeterminates to the end of the program's old
   * indeterminates.
   * @param new_indeterminates The indeterminates to be appended to the
   * program's old indeterminates.
   * @pre `new_indeterminates` should not intersect with the program's old
   * indeterminates or decision variables.
   * @pre Each entry in new_indeterminates should not be dummy.
   * @pre Each entry in new_indeterminates should be of CONTINUOUS type.
   */
  void AddIndeterminates(
      const Eigen::Ref<const VectorXIndeterminate>& new_indeterminates);

  /**
   * Adds a callback method to visualize intermediate results of the
   * optimization.
   *
   * @note Just like other costs/constraints, not all solvers support callbacks.
   * Adding a callback here will force MathematicalProgram::Solve to select a
   * solver that support callbacks.  For instance, adding a visualization
   * callback to a quadratic programming problem may result in using a nonlinear
   * programming solver as the default solver.
   *
   * @param callback a std::function that accepts an Eigen::Vector of doubles
   * representing the bound decision variables.
   * @param vars the decision variables that should be passed to the callback.
   */
  Binding<VisualizationCallback> AddVisualizationCallback(
      const VisualizationCallback::CallbackFunction& callback,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a callback method to visualize intermediate results of the
   * optimization.
   *
   * @note Just like other costs/constraints, not all solvers support callbacks.
   * Adding a callback here will force MathematicalProgram::Solve to select a
   * solver that support callbacks.  For instance, adding a visualization
   * callback to a quadratic programming problem may result in using a nonlinear
   * programming solver as the default solver.
   *
   * @param callback a std::function that accepts an Eigen::Vector of doubles
   * representing the for the bound decision variables.
   * @param vars the decision variables that should be passed to the callback.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<VisualizationCallback> AddVisualizationCallback(
      const VisualizationCallback::CallbackFunction& callback,
      const VariableRefList& vars) {
    return AddVisualizationCallback(callback,
                                    ConcatenateVariableRefList((vars)));
  }

  /**
   * Adds a generic cost to the optimization program.
   */
  Binding<Cost> AddCost(const Binding<Cost>& binding);

  /**
   * Adds a cost type to the optimization program.
   * @param obj The added objective.
   * @param vars The decision variables on which the cost depend.
   */
  template <typename C>
  auto AddCost(const std::shared_ptr<C>& obj,
               const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    // Redirect to the appropriate type
    // Use auto to enable the overloading method to upcast if needed
    return AddCost(internal::CreateBinding(obj, vars));
  }

  /**
   * Adds a generic cost to the optimization program.
   * @param obj The added objective.
   * @param vars The decision variables on which the cost depend.
   */
  template <typename C>
  auto AddCost(const std::shared_ptr<C>& obj, const VariableRefList& vars) {
    return AddCost(obj, ConcatenateVariableRefList(vars));
  }

  /**
   * Convert an input of type @p F to a FunctionCost object.
   * @tparam F This class should have functions numInputs(), numOutputs and
   * eval(x, y).
   */
  template <typename F>
  static std::shared_ptr<Cost> MakeCost(F&& f) {
    return MakeFunctionCost(f);
  }

  /**
   * Adds a cost to the optimization program on a list of variables.
   * @tparam F it should define functions numInputs, numOutputs and eval. Check
   */
  template <typename F>
  typename std::enable_if<internal::is_cost_functor_candidate<F>::value,
                          Binding<Cost>>::type
  AddCost(F&& f, const VariableRefList& vars) {
    return AddCost(f, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost to the optimization program on an Eigen::Vector containing
   * decision variables.
   * @tparam F Type that defines functions numInputs, numOutputs and eval.
   */
  template <typename F>
  typename std::enable_if<internal::is_cost_functor_candidate<F>::value,
                          Binding<Cost>>::type
  AddCost(F&& f, const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    auto c = MakeFunctionCost(std::forward<F>(f));
    return AddCost(c, vars);
  }

  /**
   * Statically assert if a user inadvertently passes a
   * binding-compatible Constraint.
   * @tparam F The type to check.
   */
  template <typename F, typename Vars>
  typename std::enable_if<internal::assert_if_is_constraint<F>::value,
                          Binding<Cost>>::type
  AddCost(F&&, Vars&&) {
    throw std::runtime_error("This will assert at compile-time.");
  }

  /**
   * Adds a cost term of the form c'*x.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  Binding<LinearCost> AddCost(const Binding<LinearCost>& binding);

  /**
   * Adds a linear cost term of the form a'*x + b.
   * @param e A linear symbolic expression.
   * @pre e is a linear expression a'*x + b, where each entry of x is a decision
   * variable in the mathematical program.
   * @return The newly added linear constraint, together with the bound
   * variables.
   */
  Binding<LinearCost> AddLinearCost(const symbolic::Expression& e);

  /**
   * Adds a linear cost term of the form a'*x + b.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  Binding<LinearCost> AddLinearCost(const Eigen::Ref<const Eigen::VectorXd>& a,
                                    double b, const VariableRefList& vars) {
    return AddLinearCost(a, b, ConcatenateVariableRefList((vars)));
  }

  /**
   * Adds a linear cost term of the form a'*x + b.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  Binding<LinearCost> AddLinearCost(
      const Eigen::Ref<const Eigen::VectorXd>& a, double b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a linear cost term of the form a'*x.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  template <typename VarType>
  Binding<LinearCost> AddLinearCost(const Eigen::Ref<const Eigen::VectorXd>& a,
                                    const VarType& vars) {
    const double b = 0.;
    return AddLinearCost(a, b, vars);
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x.
   * Applied to subset of the variables and pushes onto
   * the quadratic cost data structure.
   */
  Binding<QuadraticCost> AddCost(const Binding<QuadraticCost>& binding);

  /**
   * Add a quadratic cost term of the form 0.5*x'*Q*x + b'*x + c.
   * Notice that in the optimization program, the constant term `c` in the cost
   * is ignored.
   * @param e A quadratic symbolic expression.
   * @throws std::runtime error if the expression is not quadratic.
   * @return The newly added cost together with the bound variables.
   */
  Binding<QuadraticCost> AddQuadraticCost(const symbolic::Expression& e);

  /**
   * Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   */
  Binding<QuadraticCost> AddQuadraticErrorCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& x_desired,
      const VariableRefList& vars) {
    return AddQuadraticErrorCost(Q, x_desired,
                                 ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   */
  Binding<QuadraticCost> AddQuadraticErrorCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& x_desired,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a cost term of the form | Ax - b |^2.
   */
  Binding<QuadraticCost> AddL2NormCost(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return AddL2NormCost(A, b, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost term of the form | Ax - b |^2.
   */
  Binding<QuadraticCost> AddL2NormCost(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddCost(MakeL2NormCost(A, b), vars);
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x.
   * Applied to subset of the variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return AddQuadraticCost(Q, b, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x + c
   * Applied to subset of the variables.
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b, double c,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x
   * Applied to subset of the variables.
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a cost term in the polynomial form.
   * @param e A symbolic expression in the polynomial form.
   * @return The newly created cost and the bound variables.
   */
  Binding<PolynomialCost> AddPolynomialCost(const symbolic::Expression& e);

  /**
   * Adds a cost in the symbolic form.
   * Note that the constant part of the cost is ignored. So if you set
   * `e = x + 2`, then only the cost on `x` is added, the constant term 2 is
   * ignored.
   * @param e The linear or quadratic expression of the cost.
   * @pre `e` is linear or `e` is quadratic. Otherwise throws a runtime error.
   * @return The newly created cost, together with the bound variables.
   */
  Binding<Cost> AddCost(const symbolic::Expression& e);

  /**
   * Adds the cost to maximize the log determinant of symmetric matrix X.
   * log(det(X)) is a concave function of X, so we can maximize it through
   * convex optimization. In order to do that, we introduce slack variables t,
   * and a lower triangular matrix Z, with the constraints
   * ⌈X         Z⌉ is positive semidifinite.
   * ⌊Zᵀ  diag(Z)⌋
   * log(Z(i, i)) >= t(i)
   * and we will minimize -∑ᵢt(i).
   * @param X A symmetric positive semidefinite matrix X, whose log(det(X)) will
   * be maximized.
   * @pre X is a symmetric matrix.
   * @note The constraint log(Z(i, i)) >= t(i) is imposed as an exponential cone
   * constraint. Please make sure your have a solver that supports exponential
   * cone constraint (currently SCS does).
   */
  void AddMaximizeLogDeterminantSymmetricMatrixCost(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * @anchor maximize_geometric_mean
   * @name    Maximize geometric mean
   * Adds the cost to maximize the geometric mean of z = Ax+b, i.e.
   * power(∏ᵢz(i), 1/n), where z ∈ ℝⁿ, z(i) >= 0. Mathematically, the cost we
   * add is -power(∏ᵢz(i), 1/r), where r = power(2, ceil(log₂n)), namely r is
   * the smallest power of 2 that is no smaller than the size of z. For example,
   * if z ∈ ℝ², then the added cost is -power(z(0)*z(1), 1/2) if z ∈ ℝ³, then
   * the added cost is -power(z(0)*z(1)*z(2), 1/4).
   *
   * In order to add this cost, we need to introduce a set of second-order cone
   * constraints. For example, to maximize power(z(0) * z(1), 1/2), we
   * introduce the slack variable w(0), together with the second order cone
   * constraint w(0)² ≤ z(0) * z(1), z(0) ≥ 0, z(1) ≥ 0, and we maximize w(0).
   *
   * To maximize power(z(0) * z(1) * z(2), 1/ 4), we introduce the slack
   * variable w(0), w(1), w(2), together with the second order cone constraints
   * <pre>
   * w(0)² ≤ z(0) * z(1), z(0) ≥ 0, z(1) ≥ 0
   * w(1)² ≤ z(2), z(2) ≥ 0
   * w(2)² ≤ w(0) * w(1), w(0) ≥ 0, w(1) ≥ 0
   * </pre>
   * and we maximize w(2).
   */
  //@{
  /**
   * An overloaded version of @ref maximize_geometric_mean.
   * @pre A.rows() == b.rows(), A.rows() >= 2.
   */
  void AddMaximizeGeometricMeanCost(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& x);

  /**
   * An overloaded version of @ref maximize_geometric_mean.
   * We add the cost to maximize the geometric mean of x, i.e., c*power(∏ᵢx(i),
   * 1/n).
   * @param c The positive coefficient of the geometric mean cost, @default
   * is 1.
   * @pre x.rows() >= 2.
   * @pre c > 0.
   */
  void AddMaximizeGeometricMeanCost(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& x, double c = 1.0);
  //@}

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   */
  Binding<Constraint> AddConstraint(const Binding<Constraint>& binding);

  /**
   * Adds one row of constraint lb <= e <= ub where @p e is a symbolic
   * expression.
   * @throws std::exception if
   * 1. <tt>lb <= e <= ub</tt> is a trivial constraint such as 1 <= 2 <= 3.
   * 2. <tt>lb <= e <= ub</tt> is unsatisfiable such as 1 <= -5 <= 3
   *
   * @param e A symbolic expression of the the decision variables.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   *
   * The resulting constraint may be a BoundingBoxConstraint, LinearConstraint,
   * LinearEqualityConstraint, or ExpressionConstraint, depending on the
   * arguments.  Constraints of the form x == 1 (which could be created as a
   * BoundingBoxConstraint or LinearEqualityConstraint) will be
   * constructed as a LinearEqualityConstraint.
   */
  Binding<Constraint> AddConstraint(const symbolic::Expression& e, double lb,
                                    double ub);

  /**
   * Adds constraints represented by symbolic expressions to the program. It
   * throws if <tt>lb <= v <= ub</tt> includes trivial/unsatisfiable
   * constraints.
   *
   * @overload Binding<Constraint> AddConstraint(const symbolic::Expression& e,
   *    double lb, double ub)
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<Constraint> AddConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub);

  /**
   * Add a constraint represented by a symbolic formula to the program. The
   * input formula @p f can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   * 4. A conjunction of relational formulas where each conjunct is
   *    a relational formula matched by 1, 2, or 3.
   *
   * Note that first two cases might return an object of
   * Binding<BoundingBoxConstraint>, Binding<LinearConstraint>, or
   * Binding<ExpressionConstraint>, depending
   * on @p f. Also the third case might return an object of
   * Binding<LinearEqualityConstraint> or Binding<ExpressionConstraint>.
   *
   * It throws an exception if
   *  1. @p f is not matched with one of the above patterns. Especially, strict
   *     inequalities (<, >) are not allowed.
   *  2. @p f is either a trivial constraint such as "1 <= 2" or an
   *     unsatisfiable constraint such as "2 <= 1".
   *  3. It is not possible to find numerical bounds of `e1` and `e2` where @p f
   *     = e1 ≃ e2. We allow `e1` and `e2` to be infinite but only if there are
   *     no other terms. For example, `x <= ∞` is allowed. However, `x - ∞ <= 0`
   *     is not allowed because `x ↦ ∞` introduces `nan` in the evaluation.
   */
  Binding<Constraint> AddConstraint(const symbolic::Formula& f);

  /**
   * Add a constraint represented by an Eigen::Array<symbolic::Formula>
   * to the program. A common use-case of this function is to add a constraint
   * with the element-wise comparison between two Eigen matrices,
   * using `A.array() <= B.array()`. See the following example.
   *
   * @code
   *   MathematicalProgram prog;
   *   Eigen::Matrix<double, 2, 2> A;
   *   auto x = prog.NewContinuousVariables(2, "x");
   *   Eigen::Vector2d b;
   *   ... // set up A and b
   *   prog.AddConstraint((A * x).array() <= b.array());
   * @endcode
   *
   * A formula in @p formulas can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   *
   * It throws an exception if AddConstraint(const symbolic::Formula& f)
   * throws an exception for f ∈ @p formulas.
   *
   * @overload Binding<Constraint> AddConstraint(const symbolic::Formula& f)
   *
   * @tparam Derived An Eigen Array type of Formula.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename Derived>
  typename std::enable_if<
      is_eigen_scalar_same<Derived, symbolic::Formula>::value,
      Binding<Constraint>>::type
  AddConstraint(const Eigen::ArrayBase<Derived>& formulas) {
    return AddConstraint(internal::ParseConstraint(formulas));
  }

  /**
   * Add a constraint represented by an Eigen::Matrix<symbolic::Formula>
   * to the program.
   *
   * A formula in @p formulas can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   *
   * It throws an exception if AddConstraint(const symbolic::Formula& f)
   * throws an exception for any f ∈ formulas.
   *
   * @tparam Derived An Eigen Matrix type of Formula.
   *
   * @pydrake_mkdoc_identifier{matrix_formula}
   */
  template <typename Derived>
  typename std::enable_if<
      is_eigen_scalar_same<Derived, symbolic::Formula>::value,
      Binding<Constraint>>::type
  AddConstraint(const Eigen::MatrixBase<Derived>& formulas) {
    return AddConstraint(formulas.array());
  }

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename C>
  auto AddConstraint(std::shared_ptr<C> con, const VariableRefList& vars) {
    return AddConstraint(con, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   * @pydrake_mkdoc_identifier{2args_con_vars}
   */
  template <typename C>
  auto AddConstraint(std::shared_ptr<C> con,
                     const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddConstraint(internal::CreateBinding(con, vars));
  }

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearConstraint> AddConstraint(
      const Binding<LinearConstraint>& binding);

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const VariableRefList& vars) {
    return AddLinearConstraint(A, lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds one row of linear constraint referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * lb <= a*vars <= ub
   * @param a A row vector.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   * @param vars The decision variables on which to impose the linear
   * constraint.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double lb, double ub,
      const VariableRefList& vars) {
    return AddLinearConstraint(a, lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds one row of linear constraint referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * lb <= a*vars <= ub
   * @param a A row vector.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   * @param vars The decision variables on which to impose the linear
   * constraint.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double lb, double ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddLinearConstraint(a, Vector1d(lb), Vector1d(ub), vars);
  }

  /**
   * Adds one row of linear constraint lb <= e <= ub where @p e is a symbolic
   * expression.
   * @throws std::exception if
   * 1. @p e is a non-linear expression.
   * 2. <tt>lb <= e <= ub</tt> is a trivial constraint such as 1 <= 2 <= 3.
   * 3. <tt>lb <= e <= ub</tt> is unsatisfiable such as 1 <= -5 <= 3
   *
   * @param e A linear symbolic expression in the form of <tt>c0 + c1 * v1 +
   * ... + cn * vn</tt> where @c c_i is a constant and @v_i is a variable.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   */
  Binding<LinearConstraint> AddLinearConstraint(const symbolic::Expression& e,
                                                double lb, double ub);

  /**
   * Adds linear constraints represented by symbolic expressions to the
   * program. It throws if @v includes a non-linear expression or <tt>lb <= v <=
   * ub</tt> includes trivial/unsatisfiable constraints.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub);

  /**
   * Add a linear constraint represented by a symbolic formula to the
   * program. The input formula @p f can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   * 4. A conjunction of relational formulas where each conjunct is
   *    a relational formula matched by 1, 2, or 3.
   *
   * Note that first two cases might return an object of
   * Binding<BoundingBoxConstraint> depending on @p f. Also the third case
   * returns an object of Binding<LinearEqualityConstraint>.
   *
   * It throws an exception if
   *  1. @p f is not matched with one of the above patterns. Especially, strict
   *     inequalities (<, >) are not allowed.
   *  2. @p f includes a non-linear expression.
   *  3. @p f is either a trivial constraint such as "1 <= 2" or an
   *     unsatisfiable constraint such as "2 <= 1".
   *  4. It is not possible to find numerical bounds of `e1` and `e2` where @p f
   *     = e1 ≃ e2. We allow `e1` and `e2` to be infinite but only if there are
   *     no other terms. For example, `x <= ∞` is allowed. However, `x - ∞ <= 0`
   *     is not allowed because `x ↦ ∞` introduces `nan` in the evaluation.
   */
  Binding<LinearConstraint> AddLinearConstraint(const symbolic::Formula& f);

  /**
   * Add a linear constraint represented by an Eigen::Array<symbolic::Formula>
   * to the program. A common use-case of this function is to add a linear
   * constraint with the element-wise comparison between two Eigen matrices,
   * using `A.array() <= B.array()`. See the following example.
   *
   * @code
   *   MathematicalProgram prog;
   *   Eigen::Matrix<double, 2, 2> A;
   *   auto x = prog.NewContinuousVariables(2, "x");
   *   Eigen::Vector2d b;
   *   ... // set up A and b
   *   prog.AddLinearConstraint((A * x).array() <= b.array());
   * @endcode
   *
   * A formula in @p formulas can be of the following forms:
   *
   *  1. e1 <= e2
   *  2. e1 >= e2
   *  3. e1 == e2
   *
   * It throws an exception if AddLinearConstraint(const symbolic::Formula& f)
   * throws an exception for f ∈ @p formulas.
   * @tparam Derived An Eigen Array type of Formula.
   */
  template <typename Derived>
  typename std::enable_if<
      is_eigen_scalar_same<Derived, symbolic::Formula>::value,
      Binding<LinearConstraint>>::type
  AddLinearConstraint(const Eigen::ArrayBase<Derived>& formulas) {
    Binding<Constraint> binding = internal::ParseConstraint(formulas);
    Constraint* constraint = binding.evaluator().get();
    if (dynamic_cast<LinearConstraint*>(constraint)) {
      return AddConstraint(
          internal::BindingDynamicCast<LinearConstraint>(binding));
    } else {
      std::stringstream oss;
      oss << "Formulas are non-linear.";
      throw std::runtime_error(
          "AddLinearConstraint called but formulas are non-linear");
    }
  }

  /**
   * Adds linear equality constraints referencing potentially a
   * subset of the decision variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearEqualityConstraint> AddConstraint(
      const Binding<LinearEqualityConstraint>& binding);

  /**
   * Adds one row of linear constraint e = b where @p e is a symbolic
   * expression.
   * @throws std::exception if
   * 1. @p e is a non-linear expression.
   * 2. @p e is a constant.
   *
   * @param e A linear symbolic expression in the form of <tt>c0 + c1 * x1 +
   * ... + cn * xn</tt> where @c c_i is a constant and @x_i is a variable.
   * @param b A scalar.
   * @return The newly added linear equality constraint, together with the
   * bound variable.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const symbolic::Expression& e, double b);

  /**
   * Adds a linear equality constraint represented by a symbolic formula to the
   * program. The input formula @p f is either an equality formula (`e1 == e2`)
   * or a conjunction of equality formulas.
   *
   * It throws an exception if
   *
   * 1. @p f is neither an equality formula nor a conjunction of equalities.
   * 2. @p f includes a non-linear expression.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const symbolic::Formula& f);

  /**
   * Adds linear equality constraints \f$ v = b \f$, where \p v(i) is a symbolic
   * linear expression.
   * @throws std::exception if
   * 1. @p v(i) is a non-linear expression.
   * 2. @p v(i) is a constant.
   *
   * @tparam DerivedV An Eigen Matrix type of Expression. A column vector.
   * @tparam DerivedB An Eigen Matrix type of double. A column vector.
   * @param v v(i) is a linear symbolic expression in the form of
   * <tt> c0 + c1 * x1 + ... + cn * xn </tt> where ci is a constant and @xi is
   * a variable.
   * @param b A vector of doubles.
   * @return The newly added linear equality constraint, together with the
   * bound variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename DerivedV, typename DerivedB>
  typename std::enable_if<
      is_eigen_vector_expression_double_pair<DerivedV, DerivedB>::value,
      Binding<LinearEqualityConstraint>>::type
  AddLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& v,
                              const Eigen::MatrixBase<DerivedB>& b) {
    return AddConstraint(internal::ParseLinearEqualityConstraint(v, b));
  }

  /**
   * Adds a linear equality constraint for a matrix of linear expression @p V,
   * such that V(i, j) = B(i, j). If V is a symmetric matrix, then the user
   * may only want to constrain the lower triangular part of V.
   * This function is meant to provide convenience to the user, it incurs
   * additional copy and memory allocation. For faster speed, add each column
   * of the matrix equality in a for loop.
   * @tparam DerivedV An Eigen Matrix type of Expression. The number of columns
   * at compile time should not be 1.
   * @tparam DerivedB An Eigen Matrix type of double.
   * @param V An Eigen Matrix of symbolic expressions. V(i, j) should be a
   * linear expression.
   * @param B An Eigen Matrix of doubles.
   * @param lower_triangle If true, then only the lower triangular part of @p V
   * is constrained, otherwise the whole matrix V is constrained. @default is
   * false.
   * @return The newly added linear equality constraint, together with the
   * bound variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename DerivedV, typename DerivedB>
  typename std::enable_if<
      is_eigen_nonvector_expression_double_pair<DerivedV, DerivedB>::value,
      Binding<LinearEqualityConstraint>>::type
  AddLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& V,
                              const Eigen::MatrixBase<DerivedB>& B,
                              bool lower_triangle = false) {
    return AddConstraint(
        internal::ParseLinearEqualityConstraint(V, B, lower_triangle));
  }

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints referencing potentially a subset of
   * the decision variables.
   *
   * Example: to add two equality constraints which only depend on two of the
   * elements of x, you could use
   * @code{.cc}
   *   auto x = prog.NewContinuousVariables(6,"myvar");
   *   Eigen::Matrix2d Aeq;
   *   Aeq << -1, 2,
   *           1, 1;
   *   Eigen::Vector2d beq(1, 3);
   *   prog.AddLinearEqualityConstraint(Aeq, beq, {x.segment<1>(2),
   *                                    x.segment<1>(5)});
   * @endcode
   * The code above imposes constraints
   * @f[-x(2) + 2x(5) = 1 @f]
   * @f[ x(2) +  x(5) = 3 @f]
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
      const Eigen::Ref<const Eigen::VectorXd>& beq,
      const VariableRefList& vars) {
    return AddLinearEqualityConstraint(Aeq, beq,
                                       ConcatenateVariableRefList(vars));
  }

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints referencing potentially a subset of
   * the decision variables.
   *
   * Example: to add two equality constraints which only depend on two of the
   * elements of x, you could use
   * @code{.cc}
   *   auto x = prog.NewContinuousVariables(6,"myvar");
   *   Eigen::Matrix2d Aeq;
   *   Aeq << -1, 2,
   *           1, 1;
   *   Eigen::Vector2d beq(1, 3);
   *   // Imposes constraint
   *   // -x(0) + 2x(1) = 1
   *   //  x(0) +  x(1) = 3
   *   prog.AddLinearEqualityConstraint(Aeq, beq, x.head<2>());
   * @endcode
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
      const Eigen::Ref<const Eigen::VectorXd>& beq,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds one row of linear equality constraint referencing potentially a subset
   * of decision variables.
   * @f[
   * ax = beq
   * @f]
   * @param a A row vector.
   * @param beq A scalar.
   * @param vars The decision variables on which the constraint is imposed.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double beq,
      const VariableRefList& vars) {
    return AddConstraint(std::make_shared<LinearEqualityConstraint>(a, beq),
                         ConcatenateVariableRefList(vars));
  }

  /**
   * Adds one row of linear equality constraint referencing potentially a subset
   * of decision variables.
   * @f[
   * ax = beq
   * @f]
   * @param a A row vector.
   * @param beq A scalar.
   * @param vars The decision variables on which the constraint is imposed.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double beq,
      const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddLinearEqualityConstraint(a, Vector1d(beq), vars);
  }

  /**
   * Adds bounding box constraints referencing potentially a subest of the
   * decision variables.
   * @param binding Binds a BoundingBoxConstraint with some decision variables,
   * such that
   * binding.evaluator()->lower_bound()(i) <= binding.variables()(i)
   *                   <= binding.evaluator().upper_bound()(i)
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<BoundingBoxConstraint> AddConstraint(
      const Binding<BoundingBoxConstraint>& binding);

  /** AddBoundingBoxConstraint
   *
   * Adds bounding box constraints referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * Example
   * \code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables<2>("x");
   * auto y = prog.NewContinuousVariables<1>("y");
   * Eigen::Vector3d lb(0, 1, 2);
   * Eigen::Vector3d ub(1, 2, 3);
   * // Imposes the constraint
   * // 0 ≤ x(0) ≤ 1
   * // 1 ≤ x(1) ≤ 2
   * // 2 ≤ y    ≤ 3
   * prog.AddBoundingBoxConstraint(lb, ub, {x, y});
   * \endcode
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const VariableRefList& vars) {
    return AddBoundingBoxConstraint(lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds bounding box constraints referencing potentially a subset of the
   * decision variables.
   * @param lb The lower bound.
   * @param ub The upper bound.
   * @param vars Will imposes constraint lb(i) <= vars(i) <= ub(i).
   * @return The newly constructed BoundingBoxConstraint.
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds bounds for a single variable.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param var The decision variable.
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const symbolic::Variable& var) {
    MatrixDecisionVariable<1, 1> var_matrix(var);
    return AddBoundingBoxConstraint(Vector1d(lb), Vector1d(ub), var_matrix);
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in the list.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const VariableRefList& vars) {
    return AddBoundingBoxConstraint(lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in @p vars.
   * @tparam Derived An Eigen Vector type with Variable as the scalar
   * type.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value &&
          Derived::ColsAtCompileTime == 1,
      Binding<BoundingBoxConstraint>>::type
  AddBoundingBoxConstraint(double lb, double ub,
                           const Eigen::MatrixBase<Derived>& vars) {
    const int kSize = Derived::RowsAtCompileTime;
    return AddBoundingBoxConstraint(
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), lb),
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), ub), vars);
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in @p vars.
   * @tparam Derived An Eigen::Matrix with Variable as the scalar
   * type. The matrix has unknown number of columns at compile time, or has
   * more than one column.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value &&
          Derived::ColsAtCompileTime != 1,
      Binding<BoundingBoxConstraint>>::type
  AddBoundingBoxConstraint(double lb, double ub,
                           const Eigen::MatrixBase<Derived>& vars) {
    const int kSize =
        Derived::RowsAtCompileTime != Eigen::Dynamic &&
                Derived::ColsAtCompileTime != Eigen::Dynamic
            ? Derived::RowsAtCompileTime * Derived::ColsAtCompileTime
            : Eigen::Dynamic;
    Eigen::Matrix<symbolic::Variable, kSize, 1> flat_vars(vars.size());
    for (int j = 0; j < vars.cols(); ++j) {
      for (int i = 0; i < vars.rows(); ++i) {
        flat_vars(j * vars.rows() + i) = vars(i, j);
      }
    }
    return AddBoundingBoxConstraint(
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), lb),
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), ub), flat_vars);
  }

  /**
   * Adds Lorentz cone constraint referencing potentially a subset
   * of the decision variables.
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)² + ... + z(n-1)²}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LorentzConeConstraint> AddConstraint(
      const Binding<LorentzConeConstraint>& binding);

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables.
   * @param v An Eigen::Vector of symbolic::Expression. Constraining that
   * \f[
   * v_0 \ge \sqrt{v_1^2 + ... + v_{n-1}^2}
   * \f]
   * @return The newly constructed Lorentz cone constraint with the bounded
   * variables.
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v);

  /**
   * Adds Lorentz cone constraint on the linear expression v1 and quadratic
   * expression v2, such that v1 >= sqrt(v2)
   * @param linear_expression The linear expression v1.
   * @param quadratic_expression  The quadratic expression v2.
   * @param tol The tolerance to determine if the matrix in v2 is positive
   * semidefinite or not. @see DecomposePositiveQuadraticForm for more
   * explanation. @default is 0.
   * @retval binding The newly added Lorentz cone constraint, together with the
   * bound variables.
   * @pre
   * 1. `v1` is a linear expression, in the form of c'*x + d.
   * 2. `v2` is a quadratic expression, in the form of
   *    <pre>
   *          x'*Q*x + b'x + a
   *    </pre>
   *    Also the quadratic expression has to be convex, namely Q is a
   *    positive semidefinite matrix, and the quadratic expression needs
   *    to be non-negative for any x.
   * @throws std::runtime_error if the preconditions are not satisfied.
   *
   * Notice this constraint is equivalent to the vector [z;y] is within a
   * Lorentz cone, where
   * <pre>
   *  z = v1
   *  y = R * x + d
   * </pre>
   * while (R, d) satisfies y'*y = x'*Q*x + b'*x + a
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const symbolic::Expression& linear_expression,
      const symbolic::Expression& quadratic_expression, double tol = 0);

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables (defined in the vars parameter).
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)² + ... + z(n-1)²}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   * @param A A @f$\mathbb{R}^{n\times m}@f$ matrix, whose number of columns
   * equals to the size of the decision variables.
   * @param b A @f$\mathbb{R}^n@f$ vector, whose number of rows equals to the
   * size of the decision variables.
   * @param vars The list of @f$ m @f$ decision variables.
   * @return The newly added Lorentz cone constraint.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return AddLorentzConeConstraint(A, b, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables (defined in the vars parameter).
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)² + ... + z(n-1)²}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   * @param A A @f$\mathbb{R}^{n\times m}@f$ matrix, whose number of columns
   * equals to the size of the decision variables.
   * @param b A @f$\mathbb{R}^n@f$ vector, whose number of rows equals to the
   * size of the decision variables.
   * @param vars The Eigen vector of @f$ m @f$ decision variables.
   * @return The newly added Lorentz cone constraint.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Imposes that a vector @f$ x\in\mathbb{R}^m @f$ lies in Lorentz cone. Namely
   * @f[
   * x_0 \ge \sqrt{x_1^2 + .. + x_{m-1}^2}
   * @f]
   * <!-->
   * x(0) >= sqrt(x(1)² + ... + x(m-1)²)
   * <-->
   * @param vars The stacked column of vars should lie within the Lorentz cone.
   * @return The newly added Lorentz cone constraint.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const VariableRefList& vars) {
    return AddLorentzConeConstraint(ConcatenateVariableRefList(vars));
  }

  /**
   * Imposes that a vector @f$ x\in\mathbb{R}^m @f$ lies in Lorentz cone. Namely
   * @f[
   * x_0 \ge \sqrt{x_1^2 + .. + x_{m-1}^2}
   * @f]
   * <!-->
   * x(0) >= sqrt(x(1)² + ... + x(m-1)²)
   * <-->
   * @param vars The stacked column of vars should lie within the Lorentz cone.
   * @return The newly added Lorentz cone constraint.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <int rows>
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::MatrixBase<VectorDecisionVariable<rows>>& vars) {
    Eigen::Matrix<double, rows, rows> A(vars.rows(), vars.rows());
    A.setIdentity();
    Eigen::Matrix<double, rows, 1> b(vars.rows());
    b.setZero();
    return AddLorentzConeConstraint(A, b, vars);
  }

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables. The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)² + ... + z(n-1)²
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<RotatedLorentzConeConstraint> AddConstraint(
      const Binding<RotatedLorentzConeConstraint>& binding);

  /**
   * Adds rotated Lorentz cone constraint on the linear expression v1, v2 and
   * quadratic expression u, such that v1 * v2 >= u, v1 >= 0, v2 >= 0
   * @param linear_expression1 The linear expression v1.
   * @param linear_expression2 The linear expression v2.
   * @param quadratic_expression The quadratic expression u.
   * @param tol The tolerance to determine if the matrix in v2 is positive
   * semidefinite or not. @see DecomposePositiveQuadraticForm for more
   * explanation. @default is 0.
   * @retval binding The newly added rotated Lorentz cone constraint, together
   * with the bound variables.
   * @pre
   * 1. `linear_expression1` is a linear (affine) expression, in the form of
   *    v1 = c1'*x + d1.
   * 2. `linear_expression2` is a linear (affine) expression, in the form of
   *    v2 = c2'*x + d2.
   * 2. `quadratic_expression` is a quadratic expression, in the form of
   *    <pre>
   *          u = x'*Q*x + b'x + a
   *    </pre>
   *    Also the quadratic expression has to be convex, namely Q is a
   *    positive semidefinite matrix, and the quadratic expression needs
   *    to be non-negative for any x.
   * @throws std::runtime_error if the preconditions are not satisfied.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const symbolic::Expression& linear_expression1,
      const symbolic::Expression& linear_expression2,
      const symbolic::Expression& quadratic_expression, double tol = 0);

  /**
   * Adds a constraint that a symbolic expression @param v is in the rotated
   * Lorentz cone, i.e.,
   * \f[
   * v_0v_1 \ge v_2^2 + ... + v_{n-1}^2\\
   * v_0 \ge 0, v_1 \ge 0
   * \f]
   * @param v A linear expression of variables, \f$ v = A x + b\f$, where \f$ A,
   * b \f$ are given matrices of the correct size, \f$ x \f$ is the vector of
   * decision variables.
   * @retval binding The newly added rotated Lorentz cone constraint, together
   * with the bound variables.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v);

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables, The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)² + ... + z(n-1)²
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   * where @f$ A\in\mathbb{R}^{n\times m}, b\in\mathbb{R}^n@f$ are given
   * matrices.
   * @param A A matrix whose number of columns equals to the size of the
   * decision variables.
   * @param b A vector whose number of rows equals to the size fo the decision
   * variables.
   * @param vars The decision variables on which the constraint is imposed.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return AddRotatedLorentzConeConstraint(A, b,
                                           ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables, The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)² + ... + z(n-1)²
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   * where @f$ A\in\mathbb{R}^{n\times m}, b\in\mathbb{R}^n@f$ are given
   * matrices.
   * @param A A matrix whose number of columns equals to the size of the
   * decision variables.
   * @param b A vector whose number of rows equals to the size fo the decision
   * variables.
   * @param vars The decision variables on which the constraint is imposed.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Impose that a vector @f$ x\in\mathbb{R}^m @f$ is in rotated Lorentz cone.
   * Namely
   * @f[
   * x_0 x_1 \ge x_2^2 + ... + x_{m-1}^2\\
   * x_0 \ge 0, x_1 \ge 0
   * @f]
   * <!-->
   * x(0)*x(1) >= x(2)^2 + ... x(m-1)^2
   * x(0) >= 0, x(1) >= 0
   * <-->
   * @param vars The stacked column of vars lies in the rotated Lorentz cone.
   * @return The newly added rotated Lorentz cone constraint.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const VariableRefList& vars) {
    return AddRotatedLorentzConeConstraint(ConcatenateVariableRefList(vars));
  }

  /**
   * Impose that a vector @f$ x\in\mathbb{R}^m @f$ is in rotated Lorentz cone.
   * Namely
   * @f[
   * x_0 x_1 \ge x_2^2 + ... + x_{m-1}^2\\
   * x_0 \ge 0, x_1 \ge 0
   * @f]
   * <!-->
   * x(0)*x(1) >= x(2)^2 + ... x(m-1)^2
   * x(0) >= 0, x(1) >= 0
   * <-->
   * @param vars The stacked column of vars lies in the rotated Lorentz cone.
   * @return The newly added rotated Lorentz cone constraint.
   */
  template <int rows>
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::MatrixBase<VectorDecisionVariable<rows>>& vars) {
    Eigen::Matrix<double, rows, rows> A(vars.rows(), vars.rows());
    A.setIdentity();
    Eigen::Matrix<double, rows, 1> b(vars.rows());
    b.setZero();
    return AddRotatedLorentzConeConstraint(A, b, vars);
  }

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearComplementarityConstraint> AddConstraint(
      const Binding<LinearComplementarityConstraint>& binding);

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  Binding<LinearComplementarityConstraint> AddLinearComplementarityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& M,
      const Eigen::Ref<const Eigen::VectorXd>& q, const VariableRefList& vars) {
    return AddLinearComplementarityConstraint(M, q,
                                              ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  Binding<LinearComplementarityConstraint> AddLinearComplementarityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& M,
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a polynomial constraint to the program referencing a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<Constraint> AddPolynomialConstraint(
      const VectorXPoly& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const VariableRefList& vars) {
    return AddPolynomialConstraint(polynomials, poly_vars, lb, ub,
                                   ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a polynomial constraint to the program referencing a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<Constraint> AddPolynomialConstraint(
      const VectorXPoly& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<PositiveSemidefiniteConstraint> AddConstraint(
      const Binding<PositiveSemidefiniteConstraint>& binding);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<PositiveSemidefiniteConstraint> AddConstraint(
      std::shared_ptr<PositiveSemidefiniteConstraint> con,
      const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   *
   * @throws std::runtime_error in Debug mode if @p symmetric_matrix_var is not
   * symmetric.
   * @param symmetric_matrix_var A symmetric MatrixDecisionVariable object.
   */
  Binding<PositiveSemidefiniteConstraint> AddPositiveSemidefiniteConstraint(
      const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix of symbolic
   * expressions @p e. We create a new symmetric matrix of variables M being
   * positive semidefinite, with the linear equality constraint e == M.
   * @tparam Derived An Eigen Matrix of symbolic expressions.
   * @param e Imposes constraint "e is positive semidefinite".
   * @pre{1. e is symmetric.
   *      2. e(i, j) is linear for all i, j
   *      }
   * @return The newly added positive semidefinite constraint, with the bound
   * variable M that are also newly added.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Expression>::value,
      Binding<PositiveSemidefiniteConstraint>>::type
  AddPositiveSemidefiniteConstraint(const Eigen::MatrixBase<Derived>& e) {
    DRAKE_DEMAND(e.rows() == e.cols());
    DRAKE_ASSERT(e == e.transpose());
    const int e_rows = Derived::RowsAtCompileTime;
    MatrixDecisionVariable<e_rows, e_rows> M{};
    if (e_rows == Eigen::Dynamic) {
      M = NewSymmetricContinuousVariables(e.rows());
    } else {
      M = NewSymmetricContinuousVariables<e_rows>();
    }
    // Adds the linear equality constraint that M = e.
    AddLinearEqualityConstraint(
        e - M, Eigen::Matrix<double, e_rows, e_rows>::Zero(e.rows(), e.rows()),
        true);
    const int M_flat_size =
        e_rows == Eigen::Dynamic ? Eigen::Dynamic : e_rows * e_rows;
    const Eigen::Map<Eigen::Matrix<symbolic::Variable, M_flat_size, 1>> M_flat(
        &M(0, 0), e.size());
    return AddPositiveSemidefiniteConstraint(M);
  }

  /**
   * Adds a linear matrix inequality constraint to the program.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearMatrixInequalityConstraint> AddConstraint(
      const Binding<LinearMatrixInequalityConstraint>& binding);

  /**
   * Adds a linear matrix inequality constraint to the program.
   */
  Binding<LinearMatrixInequalityConstraint> AddLinearMatrixInequalityConstraint(
      const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
      const VariableRefList& vars) {
    return AddLinearMatrixInequalityConstraint(
        F, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a linear matrix inequality constraint to the program.
   */
  Binding<LinearMatrixInequalityConstraint> AddLinearMatrixInequalityConstraint(
      const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds the constraint that a symmetric matrix is diagonally dominant with
   * non-negative diagonal entries.
   * A symmetric matrix X is diagonally dominant with non-negative diagonal
   * entries if
   * X(i, i) >= ∑ⱼ |X(i, j)| ∀ j ≠ i
   * namely in each row, the diagonal entry is larger than the sum of the
   * absolute values of all other entries in the same row. A matrix being
   * diagonally dominant with non-negative diagonals is a sufficient (but not
   * necessary) condition of a matrix being positive semidefinite.
   * Internally we will create a matrix Y as slack variables, such that Y(i, j)
   * represents the absolute value |X(i, j)| ∀ j ≠ i. The diagonal entries
   * Y(i, i) = X(i, i)
   * The users can refer to "DSOS and SDSOS Optimization: More Tractable
   * Alternatives to Sum of Squares and Semidefinite Optimization" by Amir Ali
   * Ahmadi and Anirudha Majumdar, with arXiv link
   * https://arxiv.org/abs/1706.02586
   * @param X The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
   * X.
   * @return Y The slack variable. Y(i, j) represents |X(i, j)| ∀ j ≠ i, with
   * the constraint Y(i, j) >= X(i, j) and Y(i, j) >= -X(i, j). Y is a symmetric
   * matrix. The diagonal entries Y(i, i) = X(i, i)
   */
  MatrixX<symbolic::Expression> AddPositiveDiagonallyDominantMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * @anchor addsdd
   * @name     scaled diagonally dominant matrix constraint
   * Adds the constraint that a symmetric matrix is scaled diagonally dominant
   * (sdd). A matrix X is sdd if there exists a diagonal matrix D, such that
   * the product DXD is diagonally dominant with non-negative diagonal entries,
   * namely
   * d(i)X(i, i) ≥ ∑ⱼ |d(j)X(i, j)| ∀ j ≠ i
   * where d(i) = D(i, i).
   * X being sdd is equivalent to the existence of symmetric matrices Mⁱʲ∈ ℝⁿˣⁿ,
   * i < j, such that all entries in Mⁱʲ are 0, except Mⁱʲ(i, i), Mⁱʲ(i, j),
   * Mⁱʲ(j, j). (Mⁱʲ(i, i), Mⁱʲ(j, j), Mⁱʲ(i, j)) is in the rotated
   * Lorentz cone, and X = ∑ᵢⱼ Mⁱʲ.
   *
   * The users can refer to "DSOS and SDSOS Optimization: More Tractable
   * Alternatives to Sum of Squares and Semidefinite Optimization" by Amir Ali
   * Ahmadi and Anirudha Majumdar, with arXiv link
   * https://arxiv.org/abs/1706.02586.
   */
  //@{
  /**
   * This is an overloaded variant of @ref addsdd
   * "scaled diagonally dominant matrix constraint"
   * @param X The matrix X to be constrained scaled diagonally dominant.
   * X.
   * @pre X(i, j) should be a linear expression of decision variables.
   * @return M A vector of vectors of 2 x 2 symmetric matrices M. For i < j,
   * M[i][j] is
   * <pre>
   * [Mⁱʲ(i, i), Mⁱʲ(i, j)]
   * [Mⁱʲ(i, j), Mⁱʲ(j, j)].
   * </pre>
   * Note that M[i][j](0, 1) = Mⁱʲ(i, j) = (X(i, j) + X(j, i)) / 2
   * for i >= j, M[i][j] is the zero matrix.
   */
  std::vector<std::vector<Matrix2<symbolic::Expression>>>
  AddScaledDiagonallyDominantMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * This is an overloaded variant of @ref addsdd
   * "scaled diagonally dominant matrix constraint"
   * @param X The symmetric matrix X to be constrained scaled diagonally
   * dominant.
   * @return M For i < j M[i][j] contains the slack variables, mentioned in
   * @ref addsdd "scaled diagonally dominant matrix constraint". For i >= j,
   * M[i][j] contains dummy variables.
   */
  std::vector<std::vector<Matrix2<symbolic::Variable>>>
  AddScaledDiagonallyDominantMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Variable>>& X);
  //@}

  /**
   * Adds constraints that a given polynomial @p p is a sums-of-squares (SOS),
   * that is, @p p can be decomposed into `mᵀQm`, where m is the @p
   * monomial_basis. It returns the coefficients matrix Q, which is positive
   * semidefinite.
   *
   * @note It calls `Reparse` to enforce `p` to have this MathematicalProgram's
   * indeterminates if necessary.
   */
  MatrixXDecisionVariable AddSosConstraint(
      const symbolic::Polynomial& p,
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis);

  /**
   * Adds constraints that a given polynomial @p p is a sums-of-squares (SOS),
   * that is, @p p can be decomposed into `mᵀQm`, where m is a monomial
   * basis selected from the sparsity of @p p. It returns a pair of constraint
   * bindings expressing:
   *
   * @note It calls `Reparse` to enforce `p` to have this MathematicalProgram's
   * indeterminates if necessary.
   *
   *  - The coefficients matrix Q, which is positive semidefinite.
   *  - The monomial basis m.
   */
  std::pair<MatrixXDecisionVariable, VectorX<symbolic::Monomial>>
  AddSosConstraint(const symbolic::Polynomial& p);

  /**
   * Adds constraints that a given symbolic expression @p e is a
   * sums-of-squares (SOS), that is, @p p can be decomposed into `mᵀQm`,
   * where m is the @p monomial_basis.  Note that it decomposes @p e into a
   * polynomial with respect to `indeterminates()` in this mathematical
   * program. It returns the coefficients matrix Q, which is positive
   * semidefinite.
   */
  MatrixXDecisionVariable AddSosConstraint(
      const symbolic::Expression& e,
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis);

  /**
   * Adds constraints that a given symbolic expression @p e is a sums-of-squares
   * (SOS), that is, @p e can be decomposed into `mᵀQm`. Note that it decomposes
   * @p e into a polynomial with respect to `indeterminates()` in this
   * mathematical program. It returns a pair expressing:
   *
   *  - The coefficients matrix Q, which is positive semidefinite.
   *  - The monomial basis m.
   */
  std::pair<MatrixXDecisionVariable, VectorX<symbolic::Monomial>>
  AddSosConstraint(const symbolic::Expression& e);

  /**
   * Constraining that two polynomials are the same (i.e., they have the same
   * coefficients for each monomial). This function is often used in
   * sum-of-squares optimization.
   * We will impose the linear equality constraint that the coefficient of a
   * monomial in @p p1 is the same as the coefficient of the same monomial in @p
   * p2.
   * @param p1 Note that p1's indeterminates should have been registered as
   * indeterminates in this MathematicalProgram object, and p1's coefficients
   * are affine functions of decision variables in this MathematicalProgram
   * object.
   * @param p2 Note that p2's indeterminates should have been registered as
   * indeterminates in this MathematicalProgram object, and p2's coefficients
   * are affine functions of decision variables in this MathematicalProgram
   * object.
   *
   * @note It calls `Reparse` to enforce `p1` and `p2` to have this
   * MathematicalProgram's indeterminates.
   */
  void AddEqualityConstraintBetweenPolynomials(const symbolic::Polynomial& p1,
                                               const symbolic::Polynomial& p2);

  /**
   * Adds the exponential cone constraint that
   * z = binding.evaluator()->A() * binding.variables() +
   *     binding.evaluator()->b()
   * should be in the exponential cone. Namely
   * {(z₀, z₁, z₂) | z₀ ≥ z₁ * exp(z₂ / z₁), z₁ > 0}.
   * @param binding The binding of ExponentialConeConstraint and its bound
   * variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<ExponentialConeConstraint> AddConstraint(
      const Binding<ExponentialConeConstraint>& binding);

  /**
   * Adds an exponential cone constraint, that z = A * vars + b should be in
   * the exponential cone. Namely {z₀, z₁, z₂ | z₀ ≥ z₁ * exp(z₂ / z₁), z₁ >
   * 0}.
   * @param A The A matrix in the documentation above. A must have 3 rows.
   * @param b The b vector in the documentation above.
   * @param vars The variables bound with this constraint.
   */
  Binding<ExponentialConeConstraint> AddExponentialConeConstraint(
      const Eigen::Ref<const Eigen::SparseMatrix<double>>& A,
      const Eigen::Ref<const Eigen::Vector3d>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Add the constraint that z is in the exponential cone.
   * @param z The expression in the exponential cone.
   * @pre each entry in `z` is a linear expression of the decision variables.
   */
  Binding<ExponentialConeConstraint> AddExponentialConeConstraint(
      const Eigen::Ref<const Vector3<symbolic::Expression>>& z);

  /**
   * Gets the initial guess for a single variable.
   * @pre @p decision_variable has been registered in the optimization program.
   * @throws std::runtime_error if the pre condition is not satisfied.
   */
  double GetInitialGuess(const symbolic::Variable& decision_variable) const;

  /**
   * Gets the initial guess for some variables.
   * @pre Each variable in @p decision_variable_mat has been registered in the
   * optimization program.
   * @throws std::runtime_error if the pre condition is not satisfied.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>::type
  GetInitialGuess(
      const Eigen::MatrixBase<Derived>& decision_variable_mat) const {
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>
        decision_variable_values(decision_variable_mat.rows(),
                                 decision_variable_mat.cols());
    for (int i = 0; i < decision_variable_mat.rows(); ++i) {
      for (int j = 0; j < decision_variable_mat.cols(); ++j) {
        decision_variable_values(i, j) =
            GetInitialGuess(decision_variable_mat(i, j));
      }
    }
    return decision_variable_values;
  }

  /**
   * Sets the initial guess for a single variable @p decision_variable.
   * The guess is stored as part of this program.
   * @pre decision_variable is a registered decision variable in the program.
   * @throws std::runtime_error if precondition is not satisfied.
   */
  void SetInitialGuess(const symbolic::Variable& decision_variable,
                       double variable_guess_value);

  /**
   * Sets the initial guess for the decision variables stored in
   * @p decision_variable_mat to be @p x0.
   * The guess is stored as part of this program.
   */
  template <typename DerivedA, typename DerivedB>
  void SetInitialGuess(const Eigen::MatrixBase<DerivedA>& decision_variable_mat,
                       const Eigen::MatrixBase<DerivedB>& x0) {
    DRAKE_DEMAND(decision_variable_mat.rows() == x0.rows());
    DRAKE_DEMAND(decision_variable_mat.cols() == x0.cols());
    for (int i = 0; i < decision_variable_mat.rows(); ++i) {
      for (int j = 0; j < decision_variable_mat.cols(); ++j) {
        SetInitialGuess(decision_variable_mat(i, j), x0(i, j));
      }
    }
  }

  /**
   * Set the initial guess for ALL decision variables.
   * Note that variables begin with a default initial guess of NaN to indicate
   * that no guess is available.
   * @param x0 A vector of appropriate size (num_vars() x 1).
   */
  template <typename Derived>
  void SetInitialGuessForAllVariables(const Eigen::MatrixBase<Derived>& x0) {
    DRAKE_ASSERT(x0.rows() == num_vars() && x0.cols() == 1);
    x_initial_guess_ = x0;
  }

  /**
   * Updates the value of a single @p decision_variable inside the @p values
   * vector to be @p decision_variable_new_value.
   * The other decision variables' values in @p values are unchanged.
   * @param decision_variable a registered decision variable in this program.
   * @param decision_variable_new_value the variable's new values.
   * @param[in,out] values The vector to be tweaked; must be of size num_vars().
   */
  void SetDecisionVariableValueInVector(
      const symbolic::Variable& decision_variable,
      double decision_variable_new_value,
      EigenPtr<Eigen::VectorXd> values) const;

  /**
   * Updates the values of some @p decision_variables inside the @p values
   * vector to be @p decision_variables_new_values.
   * The other decision variables' values in @p values are unchanged.
   * @param decision_variables registered decision variables in this program.
   * @param decision_variables_new_values the variables' respective new values;
   *   must have the same rows() and cols() sizes and @p decision_variables.
   * @param[in,out] values The vector to be tweaked; must be of size num_vars().
   */
  void SetDecisionVariableValueInVector(
      const Eigen::Ref<const MatrixXDecisionVariable>& decision_variables,
      const Eigen::Ref<const Eigen::MatrixXd>& decision_variables_new_values,
      EigenPtr<Eigen::VectorXd> values) const;

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, double option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, int option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option,
                       const std::string& option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  /**
   * Overwrite the stored solver options inside MathematicalProgram with the
   * provided solver options.
   */
  void SetSolverOptions(const SolverOptions& solver_options) {
    solver_options_ = solver_options;
  }

  /**
   * Returns the solver options stored inside MathematicalProgram.
   */
  const SolverOptions& solver_options() const { return solver_options_; }

  const std::unordered_map<std::string, double>& GetSolverOptionsDouble(
      const SolverId& solver_id) const {
    return solver_options_.GetOptionsDouble(solver_id);
  }

  const std::unordered_map<std::string, int>& GetSolverOptionsInt(
      const SolverId& solver_id) const {
    return solver_options_.GetOptionsInt(solver_id);
  }

  const std::unordered_map<std::string, std::string>& GetSolverOptionsStr(
      const SolverId& solver_id) const {
    return solver_options_.GetOptionsStr(solver_id);
  }

  /**
   * Getter for all callbacks.
   */
  const std::vector<Binding<VisualizationCallback>>& visualization_callbacks()
      const {
    return visualization_callbacks_;
  }

  /**
   * Getter for all generic costs.
   */
  const std::vector<Binding<Cost>>& generic_costs() const {
    return generic_costs_;
  }  // e.g. for snopt_user_fun

  /**
   * Getter for all generic constraints
   */
  const std::vector<Binding<Constraint>>& generic_constraints() const {
    return generic_constraints_;
  }  // e.g. for snopt_user_fun

  /**
   * Getter for linear equality constraints.
   */
  const std::vector<Binding<LinearEqualityConstraint>>&
  linear_equality_constraints() const {
    return linear_equality_constraints_;
  }

  /** Getter for linear costs. */
  const std::vector<Binding<LinearCost>>& linear_costs() const {
    return linear_costs_;
  }

  /** Getter for quadratic costs. */
  const std::vector<Binding<QuadraticCost>>& quadratic_costs() const {
    return quadratic_costs_;
  }

  /** Getter for linear constraints. */
  const std::vector<Binding<LinearConstraint>>& linear_constraints() const {
    return linear_constraints_;
  }

  /** Getter for Lorentz cone constraints. */
  const std::vector<Binding<LorentzConeConstraint>>& lorentz_cone_constraints()
      const {
    return lorentz_cone_constraint_;
  }

  /** Getter for rotated Lorentz cone constraints. */
  const std::vector<Binding<RotatedLorentzConeConstraint>>&
  rotated_lorentz_cone_constraints() const {
    return rotated_lorentz_cone_constraint_;
  }

  /** Getter for positive semidefinite constraints. */
  const std::vector<Binding<PositiveSemidefiniteConstraint>>&
  positive_semidefinite_constraints() const {
    return positive_semidefinite_constraint_;
  }

  /** Getter for linear matrix inequality constraints. */
  const std::vector<Binding<LinearMatrixInequalityConstraint>>&
  linear_matrix_inequality_constraints() const {
    return linear_matrix_inequality_constraint_;
  }

  /** Getter for exponential cone constraints. */
  const std::vector<Binding<ExponentialConeConstraint>>&
  exponential_cone_constraints() const {
    return exponential_cone_constraints_;
  }

  /**
   * Getter returning all costs.
   * @returns Vector of all cost bindings.
   * @note The group ordering may change as more cost types are added.
   */
  std::vector<Binding<Cost>> GetAllCosts() const {
    auto costlist = generic_costs_;
    costlist.insert(costlist.end(), linear_costs_.begin(), linear_costs_.end());
    costlist.insert(costlist.end(), quadratic_costs_.begin(),
                    quadratic_costs_.end());
    return costlist;
  }

  /**
   * Getter returning all linear constraints (both linear equality and
   * inequality constraints).
   * @returns Vector of all linear constraint bindings.
   */
  std::vector<Binding<LinearConstraint>> GetAllLinearConstraints() const {
    std::vector<Binding<LinearConstraint>> conlist = linear_constraints_;
    conlist.insert(conlist.end(), linear_equality_constraints_.begin(),
                   linear_equality_constraints_.end());
    return conlist;
  }

  /**
   * Getter for returning all constraints.
   * @returns Vector of all constraint bindings.
   * @note The group ordering may change as more constraint types are added.
   */
  std::vector<Binding<Constraint>> GetAllConstraints() const {
    std::vector<Binding<Constraint>> conlist = generic_constraints_;
    auto extend = [&conlist](auto container) {
      conlist.insert(conlist.end(), container.begin(), container.end());
    };
    extend(linear_constraints_);
    extend(linear_equality_constraints_);
    extend(bbox_constraints_);
    extend(lorentz_cone_constraint_);
    extend(rotated_lorentz_cone_constraint_);
    extend(linear_matrix_inequality_constraint_);
    extend(linear_complementarity_constraints_);
    return conlist;
  }

  /** Getter for all bounding box constraints */
  const std::vector<Binding<BoundingBoxConstraint>>& bounding_box_constraints()
      const {
    return bbox_constraints_;
  }

  /** Getter for all linear complementarity constraints.*/
  const std::vector<Binding<LinearComplementarityConstraint>>&
  linear_complementarity_constraints() const {
    return linear_complementarity_constraints_;
  }

  /** Getter for number of variables in the optimization program */
  int num_vars() const { return decision_variables_.rows(); }

  /** Getter for the initial guess */
  const Eigen::VectorXd& initial_guess() const { return x_initial_guess_; }

  /** Returns the index of the decision variable. Internally the solvers thinks
   * all variables are stored in an array, and it accesses each individual
   * variable using its index. This index is used when adding constraints
   * and costs for each solver.
   * @pre{@p var is a decision variable in the mathematical program, otherwise
   * this function throws a runtime error.}
   */
  int FindDecisionVariableIndex(const symbolic::Variable& var) const;

  /**
   * Returns the indices of the decision variables. Internally the solvers
   * thinks all variables are stored in an array, and it accesses each
   * individual
   * variable using its index. This index is used when adding constraints
   * and costs for each solver.
   * @pre{@p vars are decision variables in the mathematical program, otherwise
   * this function throws a runtime error.}
   */
  std::vector<int> FindDecisionVariableIndices(
      const Eigen::Ref<const VectorXDecisionVariable>& vars) const;

  /** Gets the number of indeterminates in the optimization program */
  int num_indeterminates() const { return indeterminates_.rows(); }

  /** Returns the index of the indeterminate. Internally a solver
   * thinks all indeterminates are stored in an array, and it accesses each
   * individual indeterminate using its index. This index is used when adding
   * constraints and costs for each solver.
   * @pre @p var is a indeterminate in the mathematical program,
   * otherwise this function throws a runtime error.
   */
  size_t FindIndeterminateIndex(const symbolic::Variable& var) const;

  /**
   * Evaluates the value of some binding, for some input value for all
   * decision variables.
   * @param binding A Binding whose variables are decision variables in this
   * program.
   * @param prog_var_vals The value of all the decision variables in this
   * program.
   * @throws std::logic_error if the size of `prog_var_vals` is invalid.
   */
  template <typename C, typename DerivedX>
  typename std::enable_if<is_eigen_vector<DerivedX>::value,
                          VectorX<typename DerivedX::Scalar>>::type
  EvalBinding(const Binding<C>& binding,
              const Eigen::MatrixBase<DerivedX>& prog_var_vals) const {
    using Scalar = typename DerivedX::Scalar;
    if (prog_var_vals.rows() != num_vars()) {
      std::ostringstream oss;
      oss << "The input binding variable is not in the right size. Expects "
          << num_vars() << " rows, but it actually has " << prog_var_vals.rows()
          << " rows.\n";
      throw std::logic_error(oss.str());
    }
    VectorX<Scalar> binding_x(binding.GetNumElements());
    VectorX<Scalar> binding_y(binding.evaluator()->num_outputs());
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      binding_x(i) =
          prog_var_vals(FindDecisionVariableIndex(binding.variables()(i)));
    }
    binding.evaluator()->Eval(binding_x, &binding_y);
    return binding_y;
  }

  /**
   * Evaluates a set of bindings (plural version of `EvalBinding`).
   * @param bindings List of bindings.
   * @param prog
   * @param prog_var_vals The value of all the decision variables in this
   * program.
   * @return All binding values, concatenated into a single vector.
   * @throws std::logic_error if the size of `prog_var_vals` is invalid.
   */
  template <typename C, typename DerivedX>
  typename std::enable_if<is_eigen_vector<DerivedX>::value,
                          VectorX<typename DerivedX::Scalar>>::type
  EvalBindings(const std::vector<Binding<C>>& bindings,
               const Eigen::MatrixBase<DerivedX>& prog_var_vals) const {
    // TODO(eric.cousineau): Minimize memory allocations when it becomes a
    // major performance bottleneck.
    using Scalar = typename DerivedX::Scalar;
    int num_y{};
    for (auto& binding : bindings) {
      num_y += binding.evaluator()->num_outputs();
    }
    VectorX<Scalar> y(num_y);
    int offset_y{};
    for (auto& binding : bindings) {
      VectorX<Scalar> binding_y = EvalBinding(binding, prog_var_vals);
      y.segment(offset_y, binding_y.size()) = binding_y;
      offset_y += binding_y.size();
    }
    DRAKE_DEMAND(offset_y == num_y);
    return y;
  }

  /**
   * Given the value of all decision variables, namely
   * this.decision_variable(i) takes the value prog_var_vals(i), returns the
   * vector that contains the value of the variables in binding.variables().
   * @param binding binding.variables() must be decision variables in this
   * MathematicalProgram.
   * @param prog_var_vals The value of ALL the decision variables in this
   * program.
   * @return binding_variable_vals binding_variable_vals(i) is the value of
   * binding.variables()(i) in prog_var_vals.
   */
  template <typename C, typename DerivedX>
  typename std::enable_if<is_eigen_vector<DerivedX>::value,
                          VectorX<typename DerivedX::Scalar>>::type
  GetBindingVariableValues(
      const Binding<C>& binding,
      const Eigen::MatrixBase<DerivedX>& prog_var_vals) const {
    DRAKE_DEMAND(prog_var_vals.rows() == num_vars());
    VectorX<typename DerivedX::Scalar> result(binding.GetNumElements());
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      result(i) =
          prog_var_vals(FindDecisionVariableIndex(binding.variables()(i)));
    }
    return result;
  }

  /** Evaluates all visualization callbacks registered with the
   * MathematicalProgram.
   *
   * @param prog_var_vals The value of all the decision variables in this
   * program.
   * @throws std::logic_error if the size does not match.
   */
  void EvalVisualizationCallbacks(
      const Eigen::Ref<const Eigen::VectorXd>& prog_var_vals) const {
    if (prog_var_vals.rows() != num_vars()) {
      std::ostringstream oss;
      oss << "The input binding variable is not in the right size. Expects "
          << num_vars() << " rows, but it actually has " << prog_var_vals.rows()
          << " rows.\n";
      throw std::logic_error(oss.str());
    }

    Eigen::VectorXd this_x;

    for (auto const& binding : visualization_callbacks_) {
      auto const& obj = binding.evaluator();

      const int num_v_variables = binding.GetNumElements();
      this_x.resize(num_v_variables);
      for (int j = 0; j < num_v_variables; ++j) {
        this_x(j) =
            prog_var_vals(FindDecisionVariableIndex(binding.variables()(j)));
      }

      obj->EvalCallback(this_x);
    }
  }

  /**
   * Evaluates the evaluator in @p binding at the initial guess.
   * @return The value of @p binding at the initial guess.
   */
  template <typename C>
  Eigen::VectorXd EvalBindingAtInitialGuess(const Binding<C>& binding) const {
    return EvalBinding(binding, x_initial_guess_);
  }

  /** Getter for all decision variables in the program. */
  const VectorXDecisionVariable& decision_variables() const {
    return decision_variables_;
  }

  /** Getter for the decision variable with index @p i in the program. */
  const symbolic::Variable& decision_variable(int i) const {
    return decision_variables_(i);
  }

  /** Getter for all indeterminates in the program. */
  const VectorXIndeterminate& indeterminates() const { return indeterminates_; }

  /** Getter for the indeterminate with index @p i in the program. */
  const symbolic::Variable& indeterminate(int i) const {
    return indeterminates_(i);
  }

  /// Getter for the required capability on the solver, given the
  /// cost/constraint/variable types in the program.
  const ProgramAttributes& required_capabilities() const {
    return required_capabilities_;
  }

  /**
   * Returns the mapping from a decision variable ID to its index in the vector
   * containing all the decision variables in the mathematical program.
   */
  const std::unordered_map<symbolic::Variable::Id, int>&
  decision_variable_index() const {
    return decision_variable_index_;
  }

  /**
   * Returns the mapping from an indeterminate ID to its index in the vector
   * containing all the indeterminates in the mathematical program.
   */
  const std::unordered_map<symbolic::Variable::Id, int>& indeterminates_index()
      const {
    return indeterminates_index_;
  }

  /**
   * @anchor variable_scaling
   * @name Variable scaling
   * Some solvers (e.g. SNOPT) work better if the decision variables values
   * are on the same scale. Hence, internally we scale the variable as
   * snopt_var_value = var_value / scaling_factor.
   * This scaling factor is only used inside the solve, so
   * users don't need to manually scale the variables every time they appears in
   * cost and constraints. When the users set the initial guess, or getting the
   * result from MathematicalProgramResult::GetSolution(), the values are
   * unscaled. Namely, MathematicalProgramResult::GetSolution(var) returns the
   * value of var, not var_value / scaling_factor.
   *
   * The feature of variable scaling is currently only implemented for SNOPT.
   */
  //@{
  /**
   * Returns the mapping from a decision variable index to its scaling factor.
   *
   * See @ref variable_scaling "Variable scaling" for more information.
   */
  const std::unordered_map<int, double>& GetVariableScaling() const {
    return var_scaling_map_;
  }

  /**
   * Setter for the scaling of decision variables starting from index @p
   * idx_start to @p idx_end (including @p idx_end).
   * @param var the decision variable to be scaled.
   * @param s scaling factor (must be positive).
   *
   * See @ref variable_scaling "Variable scaling" for more information.
   */
  void SetVariableScaling(const symbolic::Variable& var, double s);
  //@}

 private:
  static void AppendNanToEnd(int new_var_size, Eigen::VectorXd* vector);

  // maps the ID of a symbolic variable to the index of the variable stored in
  // the optimization program.
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index_{};

  VectorXDecisionVariable decision_variables_;

  std::unordered_map<symbolic::Variable::Id, int> indeterminates_index_;
  VectorXIndeterminate indeterminates_;

  std::vector<Binding<VisualizationCallback>> visualization_callbacks_;

  std::vector<Binding<Cost>> generic_costs_;
  std::vector<Binding<QuadraticCost>> quadratic_costs_;
  std::vector<Binding<LinearCost>> linear_costs_;
  // TODO(naveenoid) : quadratic_constraints_

  // note: linear_constraints_ does not include linear_equality_constraints_
  std::vector<Binding<Constraint>> generic_constraints_;
  std::vector<Binding<LinearConstraint>> linear_constraints_;
  std::vector<Binding<LinearEqualityConstraint>> linear_equality_constraints_;
  std::vector<Binding<BoundingBoxConstraint>> bbox_constraints_;
  std::vector<Binding<LorentzConeConstraint>> lorentz_cone_constraint_;
  std::vector<Binding<RotatedLorentzConeConstraint>>
      rotated_lorentz_cone_constraint_;
  std::vector<Binding<PositiveSemidefiniteConstraint>>
      positive_semidefinite_constraint_;
  std::vector<Binding<LinearMatrixInequalityConstraint>>
      linear_matrix_inequality_constraint_;
  std::vector<Binding<ExponentialConeConstraint>> exponential_cone_constraints_;

  // Invariant:  The bindings in this list must be non-overlapping, when calling
  // Linear Complementarity solver like Moby. If this constraint is solved
  // through a nonlinear optimization solver (like SNOPT) instead, then we allow
  // the bindings to be overlapping.
  // TODO(ggould-tri) can this constraint be relaxed?
  std::vector<Binding<LinearComplementarityConstraint>>
      linear_complementarity_constraints_;

  Eigen::VectorXd x_initial_guess_;
  Eigen::VectorXd x_values_;
  std::optional<SolverId> solver_id_;
  double optimal_cost_{};
  // The lower bound of the objective found by the solver, during the
  // optimization process.
  double lower_bound_cost_{};

  // The actual per-solver customization options.
  SolverOptions solver_options_;

  ProgramAttributes required_capabilities_{};

  template <typename T>
  void NewVariables_impl(
      VarType type, const T& names, bool is_symmetric,
      Eigen::Ref<MatrixXDecisionVariable> decision_variable_matrix) {
    switch (type) {
      case VarType::CONTINUOUS:
        break;
      case VarType::BINARY:
        required_capabilities_.insert(ProgramAttribute::kBinaryVariable);
        break;
      case VarType::INTEGER:
        throw std::runtime_error(
            "MathematicalProgram does not support integer variables yet.");
      case VarType::BOOLEAN:
        throw std::runtime_error(
            "MathematicalProgram does not support Boolean variables.");
      case VarType::RANDOM_UNIFORM:
        throw std::runtime_error(
            "MathematicalProgram does not support random uniform variables.");
      case VarType::RANDOM_GAUSSIAN:
        throw std::runtime_error(
            "MathematicalProgram does not support random Gaussian variables.");
      case VarType::RANDOM_EXPONENTIAL:
        throw std::runtime_error(
            "MathematicalProgram does not support random exponential "
            "variables.");
    }
    int rows = decision_variable_matrix.rows();
    int cols = decision_variable_matrix.cols();
    DRAKE_ASSERT(!is_symmetric || rows == cols);
    int num_new_vars = 0;
    if (!is_symmetric) {
      num_new_vars = rows * cols;
    } else {
      num_new_vars = rows * (rows + 1) / 2;
    }
    DRAKE_ASSERT(static_cast<int>(names.size()) == num_new_vars);
    decision_variables_.conservativeResize(num_vars() + num_new_vars,
                                           Eigen::NoChange);
    AppendNanToEnd(num_new_vars, &x_values_);
    int row_index = 0;
    int col_index = 0;
    for (int i = 0; i < num_new_vars; ++i) {
      decision_variables_(num_vars() - num_new_vars + i) =
          symbolic::Variable(names[i], type);
      const int new_var_index = num_vars() - num_new_vars + i;
      decision_variable_index_.insert(std::pair<int, int>(
          decision_variables_(new_var_index).get_id(), new_var_index));
      decision_variable_matrix(row_index, col_index) =
          decision_variables_(num_vars() - num_new_vars + i);
      // If the matrix is not symmetric, then store the variable in column
      // major.
      if (!is_symmetric) {
        if (row_index + 1 < rows) {
          ++row_index;
        } else {
          ++col_index;
          row_index = 0;
        }
      } else {
        // If the matrix is symmetric, then the decision variables are the lower
        // triangular part of the symmetric matrix, also stored in column major.
        if (row_index != col_index) {
          decision_variable_matrix(col_index, row_index) =
              decision_variable_matrix(row_index, col_index);
        }
        if (row_index + 1 < rows) {
          ++row_index;
        } else {
          ++col_index;
          row_index = col_index;
        }
      }
    }

    AppendNanToEnd(num_new_vars, &x_initial_guess_);
  }

  MatrixXDecisionVariable NewVariables(VarType type, int rows, int cols,
                                       bool is_symmetric,
                                       const std::vector<std::string>& names);

  template <typename T>
  void NewIndeterminates_impl(
      const T& names, Eigen::Ref<MatrixXIndeterminate> indeterminates_matrix) {
    int rows = indeterminates_matrix.rows();
    int cols = indeterminates_matrix.cols();
    int num_new_vars = rows * cols;

    DRAKE_ASSERT(static_cast<int>(names.size()) == num_new_vars);
    indeterminates_.conservativeResize(indeterminates_.rows() + num_new_vars,
                                       Eigen::NoChange);
    int row_index = 0;
    int col_index = 0;
    for (int i = 0; i < num_new_vars; ++i) {
      indeterminates_(indeterminates_.rows() - num_new_vars + i) =
          symbolic::Variable(names[i]);

      const int new_var_index = indeterminates_.rows() - num_new_vars + i;
      indeterminates_index_.insert(std::pair<size_t, size_t>(
          indeterminates_(new_var_index).get_id(), new_var_index));
      indeterminates_matrix(row_index, col_index) =
          indeterminates_(indeterminates_.rows() - num_new_vars + i);

      // store the indeterminate in column major.
      if (row_index + 1 < rows) {
        ++row_index;
      } else {
        ++col_index;
        row_index = 0;
      }
    }
  }

  /*
   * Given a matrix of decision variables, checks if every entry in the
   * matrix is a decision variable in the program; throws a runtime
   * error if any variable is not a decision variable in the program.
   * @tparam Derived An Eigen::Matrix type of symbolic Variable.
   * @param vars A matrix of variables.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value>::type
  CheckIsDecisionVariable(const Eigen::MatrixBase<Derived>& vars) const {
    for (int i = 0; i < vars.rows(); ++i) {
      for (int j = 0; j < vars.cols(); ++j) {
        if (decision_variable_index_.find(vars(i, j).get_id()) ==
            decision_variable_index_.end()) {
          std::ostringstream oss;
          oss << vars(i, j)
              << " is not a decision variable of the mathematical program.\n";
          throw std::runtime_error(oss.str());
        }
      }
    }
  }

  /*
   * Ensure a binding is valid *before* adding it to the program.
   * @pre The binding has not yet been registered.
   * @pre The decision variables have been registered.
   * @throws std::runtime_error if the binding is invalid.
   */
  template <typename C>
  void CheckBinding(const Binding<C>& binding) const {
    // TODO(eric.cousineau): In addition to identifiers, hash bindings by
    // their constraints and their variables, to prevent duplicates.
    // TODO(eric.cousineau): Once bindings have identifiers (perhaps
    // retrofitting `description`), ensure that they have unique names.
    CheckIsDecisionVariable(binding.variables());
  }

  // Adds a constraint represented by a set of symbolic formulas to the
  // program.
  //
  // Precondition: ∀ f ∈ formulas, is_relational(f).
  Binding<Constraint> AddConstraint(
      const std::set<symbolic::Formula>& formulas);

  // Adds a linear-equality constraint represented by a set of symbolic formulas
  // to the program.
  //
  // Precondition: ∀ f ∈ formulas, is_equal_to(f).
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const std::set<symbolic::Formula>& formulas);

  /**
   * Adds new variables to MathematicalProgram.
   */
  template <int Rows, int Cols>
  MatrixDecisionVariable<Rows, Cols> NewVariables(
      VarType type, const typename NewVariableNames<Rows, Cols>::type& names,
      int rows, int cols) {
    DRAKE_DEMAND(rows >= 0 && cols >= 0);
    MatrixDecisionVariable<Rows, Cols> decision_variable_matrix;
    decision_variable_matrix.resize(rows, cols);
    NewVariables_impl(type, names, false, decision_variable_matrix);
    return decision_variable_matrix;
  }

  /**
   * Adds symmetric matrix variables to optimization program. Only the lower
   * triangular part of the matrix is used as decision variables.
   * @param names The names of the stacked columns of the lower triangular part
   * of the matrix.
   */
  template <int Rows>
  MatrixDecisionVariable<Rows, Rows> NewSymmetricVariables(
      VarType type, const typename NewSymmetricVariableNames<Rows>::type& names,
      int rows = Rows) {
    MatrixDecisionVariable<Rows, Rows> decision_variable_matrix(rows, rows);
    NewVariables_impl(type, names, true, decision_variable_matrix);
    return decision_variable_matrix;
  }

  std::unordered_map<int, double> var_scaling_map_{};
};

}  // namespace solvers
}  // namespace drake
