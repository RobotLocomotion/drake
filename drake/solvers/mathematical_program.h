#pragma once

#include <array>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_variable.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/function.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

/** @defgroup solvers Formulating and Solving Optimization Problems
 * @{
 * Drake wraps a number of commercial solvers (+ a few custom solvers) to
 * provide a common interface for convex optimization, mixed-integer convex
 * optimization, and other non-convex mathematical programs.
 *
 * The MathematicalProgram class handles the coordination of decision variables,
 * objectives, and constraints.  The MathematicalProgram::Solve() method
 * reflects on the accumulated objectives and constraints and will dispatch to
 * the most appropriate solver.  Alternatively, one can invoke specific solver
 * by instantiating its MathematicalProgramSolverInterface and passing the
 * MathematicalProgram directly to the
 * MathematicalProgramSolverInterface::Solve() method.
 *
 * Our solver coverage still has many gaps, but is under active development.
 *
 * <b>Closed-form solutions</b>
 *
 * The LinearSystemSolver and EqualityConstrainedQPSolver classes provide
 * efficient closed-form solutions to these special cases.
 *
 * <b>Convex Optimization</b>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td><a href="https://en.wikipedia.org/wiki/Linear_programming">LP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Quadratic_programming">
 *     QP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Second-order_cone_programming">
 *     SOCP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Semidefinite_programming">
 *     SDP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Sum-of-squares_optimization">
 *     SOS</a></td>
 * </tr>
 * <tr><td>&dagger; <a href="http://www.gurobi.com/products/gurobi-optimizer">
 *    Gurobi</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 *  </tr>
 * <tr><td>&dagger; <a href="https://www.mosek.com/products/mosek">
 *    Mosek</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * </tr>
 * </table>
 *
 * <b>Mixed-Integer Convex Optimization</b>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td>MILP</a></td>
 *   <td>MIQP</a></td>
 *   <td>MISOCP</a></td>
 *   <td>MISDP</a></td>
 * </tr>
 * <tr><td>&dagger; <a href="http://www.gurobi.com/products/gurobi-optimizer">
 *    Gurobi</a></td>
 *    <td></td>
 *    <td></td>
 *    <td></td>
 *    <td></td>
 *  </tr>
 * <tr><td>&dagger; <a href="https://www.mosek.com/products/mosek">
 *    Mosek</a></td>
 *    <td></td>
 *    <td></td>
 *    <td></td>
 *    <td></td>
 * </tr>
 * </table>
 *
 * <b>Nonconvex Programming</b>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td><a href="https://en.wikipedia.org/wiki/Nonlinear_programming">
 *     Nonlinear Program</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Linear_complementarity_problem">
 *   LCP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Satisfiability_modulo_theories">
 *     SMT</a></td>
 * </tr>
 * <tr><td>&dagger;
 *   <a href="http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm">
 *    SNOPT</a></td></tr>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * <tr><td><a href="https://projects.coin-or.org/Ipopt">Ipopt</a></td></tr>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * <tr><td><a href="http://ab-initio.mit.edu/wiki/index.php/NLopt">
 *    NLopt</a></td></tr>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * <tr><td><a href="https://github.com/PositronicsLab/Moby">
 *    Moby LCP</a></td>
 *    <td></td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 * <tr><td><a href="https://dreal.github.io/">dReal</a></td>
 *    <td></td>
 *    <td></td>
 *    <td align="center">&diams;</td>
 * </tr>
 * </table>
 *
 * &dagger; indicates that this is a commercial solver which requires a license
 * (note that some have free licenses for academics).
 *
 * Note: Drake must be able to locate each solver on your system during the
 * configuration step (when you run cmake), otherwise that solver will be
 * disabled.  To simplify this process, we have attempted to make solvers
 * available as a part of the Drake superbuild, but we are unable to publicly
 * share the distributions for commercially licensed solvers.
 *
 * @}
 */

class MathematicalProgram;

enum ProgramAttributes {
  kNoCapabilities = 0,
  kError = 1 << 0,  ///< Do not use, to avoid & vs. && typos.
  kGenericCost = 1 << 1,
  kGenericConstraint = 1 << 2,
  kQuadraticCost = 1 << 3,
  kQuadraticConstraint = 1 << 4,
  kLinearCost = 1 << 5,
  kLinearConstraint = 1 << 6,
  kLinearEqualityConstraint = 1 << 7,
  kLinearComplementarityConstraint = 1 << 8,
  kLorentzConeConstraint = 1 << 9,
  kRotatedLorentzConeConstraint = 1 << 10,
  kPositiveSemidefiniteConstraint = 1 << 11,
  kBinaryVariable = 1 << 12
};
typedef uint32_t AttributesSet;

class MathematicalProgram {
  /**
   * A binding on constraint type C is a mapping of the decision
   * variables onto the inputs of C.  This allows the constraint to operate
   * on a vector made up of different elements of the decision variables.
   */
  template <typename C>
  class Binding {
   public:
    Binding(const std::shared_ptr<C>& c, const VariableList& v)
        : constraint_(c), variable_list_(v) {}

    Binding(const std::shared_ptr<C>& c, const VariableListRef& v)
        : constraint_(c), variable_list_(v) {}
    template <typename U>
    Binding(
        const Binding<U>& b,
        typename std::enable_if<std::is_convertible<
            std::shared_ptr<U>, std::shared_ptr<C>>::value>::type* = nullptr)
        : Binding(b.constraint(), b.variable_list()) {}

    const std::shared_ptr<C>& constraint() const { return constraint_; }

    const VariableList& variable_list() const { return variable_list_; }

    /**
     * Returns true iff the given @p var is included in this Binding.*/
    bool ContainsVariable(const symbolic::Variable& var) const {
      for (const auto& v : variable_list_.variables()) {
        for (int i = 0; i < v.rows(); ++i) {
          for (int j = 0; j < v.cols(); ++j) {
            if (v(i, j) == var) {
              return true;
            }
          }
        }
      }
      return false;
    }

    size_t GetNumElements() const {
      // TODO(ggould-tri) assumes that no index appears more than once in the
      // view, which is nowhere asserted (but seems assumed elsewhere).
      return variable_list_.size();
    }

   private:
    std::shared_ptr<C> constraint_;
    VariableList variable_list_;
  };

  template <typename F>
  class ConstraintImpl : public Constraint {
    F const f_;

   public:
    // Construct by copying from an lvalue.
    template <typename... Args>
    ConstraintImpl(const F& f, Args&&... args)
        : Constraint(detail::FunctionTraits<F>::numOutputs(f),
                     std::forward<Args>(args)...),
          f_(f) {}

    // Construct by moving from an rvalue.
    template <typename... Args>
    ConstraintImpl(F&& f, Args&&... args)
        : Constraint(detail::FunctionTraits<F>::numOutputs(f),
                     std::forward<Args>(args)...),
          f_(std::forward<F>(f)) {}

    void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
      y.resize(detail::FunctionTraits<F>::numOutputs(f_));
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                   detail::FunctionTraits<F>::numInputs(f_));
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                   detail::FunctionTraits<F>::numOutputs(f_));
      detail::FunctionTraits<F>::eval(f_, x, y);
    }
    void Eval(const Eigen::Ref<const TaylorVecXd>& x,
              TaylorVecXd& y) const override {
      y.resize(detail::FunctionTraits<F>::numOutputs(f_));
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                   detail::FunctionTraits<F>::numInputs(f_));
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                   detail::FunctionTraits<F>::numOutputs(f_));
      detail::FunctionTraits<F>::eval(f_, x, y);
    }
  };

 public:
  enum class VarType { CONTINUOUS, INTEGER, BINARY };

  MathematicalProgram();

  /** MathematicalProgram is not copyable. */
  MathematicalProgram(const MathematicalProgram& rhs) = delete;

  /** MathematicalProgram is not assinable. */
  MathematicalProgram& operator=(const MathematicalProgram& rhs) = delete;

  /** MathematicalProgram is not movable. */
  MathematicalProgram(MathematicalProgram&& rhs) = delete;

  /** MathematicalProgram is not move-assignable. */
  MathematicalProgram& operator=(MathematicalProgram&& rhs) = delete;

  /**
   * Adds new variables to MathematicalProgram.
   * Appending new variables to an internal vector of any existing vars.
   * The new variables are initialized to zero.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows Number of rows in the variables.
   * @tparam cols Number of cols in the variables.
   * @param name An array containing the name of each variable.
   * @return The DecisionVariableMatrix<rows, cols> containing rows * cols new
   * variables (not
   * all the variables stored in MathematicalProgram).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewVariables<2, 3>(
   *      VarType::CONTINUOUS,
   *      {"x1", "x2", "x3", "x4", "x5", "x6"});
   * @endcode
   * This adds a matrix of size 2 x 3 as new variables into the optimization
   * program.
   * The name of the variable is only used for the user to understand.
   */
  template <int rows, int cols>
  DecisionVariableMatrix<rows, cols> NewVariables(
      VarType type, const std::array<std::string, rows * cols>& names) {
    DecisionVariableMatrix<rows, cols> decision_variable_matrix;
    NewVariables_impl(type, names, false, decision_variable_matrix);
    return decision_variable_matrix;
  }

  /**
   * Adds column vector variables to the optimization program.
   */
  template <int rows>
  DecisionVariableVector<rows> NewVariables(
      VarType type, const std::array<std::string, rows>& names) {
    return NewVariables<rows, 1>(type, names);
  }

  /**
   * Adds symmetric matrix variables to optimization program. Only the lower
   * triangular
   * part of the matrix is used as decision variables.
   * @param names The names of the stacked columns of the lower triangular part
   * of the matrix.
   */
  template <int rows>
  DecisionVariableMatrix<rows, rows> NewSymmetricVariables(
      VarType type, const std::array<std::string, rows*(rows + 1) / 2>& names) {
    DecisionVariableMatrix<rows, rows> decision_variable_matrix;
    NewVariables_impl(type, names, true, decision_variable_matrix);
    return decision_variable_matrix;
  }

  /**
   * Adds continuous variables to this MathematicalProgram.
   * @see NewContinuousVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  DecisionVariableVectorX NewContinuousVariables(
      std::size_t rows, const std::vector<std::string>& names);

  /**
   * Adds continuous variables to this MathematicalProgram, with default name
   * "x".
   * @see NewContinuousVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  DecisionVariableVectorX NewContinuousVariables(std::size_t rows,
                                                 const std::string& name = "x");

  /// Adds continuous variables to this MathematicalProgram.
  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The new variables are initialized to zero.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @param rows  The number of rows in the new variables.
   * @param cols  The number of columns in the new variables.
   * @param names A vector of strings containing the name for each variable.
   * @return The DecisionVariableMatrix of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables(2, 3, {"x1", "x2", "x3", "x4", "x5",
   * "x6"});
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user for understand.
   */
  DecisionVariableMatrixX NewContinuousVariables(
      std::size_t rows, std::size_t cols,
      const std::vector<std::string>& names);

  /**
   * Adds continuous variables to this MathematicalProgram, with default name
   * "X". The new variables are returned and viewed as a matrix, with size
   * @p rows x @p cols.
   * @see NewContinuousVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  DecisionVariableMatrixX NewContinuousVariables(std::size_t rows,
                                                 std::size_t cols,
                                                 const std::string& name = "X");

  /// Adds continuous variables to this MathematicalProgram.
  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The new variables are initialized to zero.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows  The number of rows in the new variables.
   * @tparam cols  The number of columns in the new variables.
   * @param names An array of strings containing the name for each variable.
   * @return The DecisionVariableMatrix of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * std::array<std::string, 6> names = {"x1", "x2", "x3", "x4", "x5", "x6"};
   * auto x = prog.NewContinuousVariables<2, 3>(names);
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user for understand.
   */
  template <int rows, int cols>
  DecisionVariableMatrix<rows, cols> NewContinuousVariables(
      const std::array<std::string, rows * cols>& names) {
    return NewVariables<rows, cols>(VarType::CONTINUOUS, names);
  }

  /// Adds continuous variables to this MathematicalProgram.
  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The new variables are initialized to zero.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows  The number of rows in the new variables.
   * @tparam cols  The number of columns in the new variables.
   * @param name All variables will share the same name, but different index.
   * @return The DecisionVariableMatrix of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables<2, 3>("X");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user for understand.
   */
  template <int rows, int cols>
  DecisionVariableMatrix<rows, cols> NewContinuousVariables(
      const std::string& name = "X") {
    std::array<std::string, rows * cols> names;
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        names[j * rows + i] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      }
    }
    return NewVariables<rows, cols>(VarType::CONTINUOUS, names);
  }

  /// Adds continuous variables to this MathematicalProgram.
  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The new variables are initialized to zero.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows  The number of rows in the new variables.
   * @param names An array of strings containing the name for each variable.
   * @return The DecisionVariableMatrix of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * std::array<std::string, 2> names = {"x1", "x2"};
   * auto x = prog.NewContinuousVariables<2>(names);
   * @endcode
   * This adds a 2 x 1 vector containing decision variables into the program.
   *
   * The name of the variable is only used for the user for understand.
   */
  template <int rows>
  DecisionVariableVector<rows> NewContinuousVariables(
      const std::array<std::string, rows>& names) {
    return NewContinuousVariables<rows, 1>(names);
  }

  /**
   * Adds continuous variables to the program.
   * The name for all newly added variables are set to @p name. The default name
   * is "x"
   * @see NewContinuousVariables(const std::array<std::string, rows>& names)
   */
  template <int rows>
  DecisionVariableVector<rows> NewContinuousVariables(
      const std::string& name = "x") {
    std::array<std::string, rows> names;
    int offset = (name.compare("x") == 0) ? num_vars_ : 0;
    for (int i = 0; i < rows; ++i) {
      names[i] = name + "(" + std::to_string(offset + i) + ")";
    }
    return NewContinuousVariables<rows>(names);
  }

  /// Adds binary variables to this MathematicalProgram.
  /**
   * Adds binary variables, appending them to an internal vector of any
   * existing vars.
   * The new variables are initialized to zero.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows  The number of rows in the new variables.
   * @tparam cols  The number of columns in the new variables.
   * @param names An array of strings containing the name for each variable.
   * @return The DecisionVariableMatrix of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * std::array<std::string, 6> names = {"b1", "b2", "b3", "b4", "b5", "b6"};
   * auto b = prog.NewBinaryVariables<2, 3>(names);
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user for understand.
   */
  template <int rows, int cols>
  DecisionVariableMatrix<rows, cols> NewBinaryVariables(
      const std::array<std::string, rows * cols>& names) {
    return NewVariables<rows, cols>(VarType::BINARY, names);
  }

  /**
   * Adds vector of binary variables into the optimization program.
   * @tparam rows The number of rows in the newly added binary variables.
   * @param names An array of strings containing the name of each variable.
   * @return A vector containing the newly added variables.
   */
  template <int rows>
  DecisionVariableVector<rows> NewBinaryVariables(
      const std::array<std::string, rows>& names) {
    return NewBinaryVariables<rows, 1>(names);
  }

  /**
   * Adds vector of binary variables into the optimization program.
   * @tparam rows The number of rows in the newly added binary variables.
   * @param name Each newly added binary variable will share the same name. The
   * default name is "b".
   * @return A vector containing the newly added variables.
   */
  template <int rows>
  DecisionVariableVector<rows> NewBinaryVariables(
      const std::string& name = "b") {
    std::array<std::string, rows> names;
    int offset = (name.compare("b") == 0) ? num_vars_ : 0;
    for (int i = 0; i < rows; ++i) {
      names[i] = name + "(" + std::to_string(offset + i) + ")";
    }
    return NewBinaryVariables<rows, 1>(names);
  }

  /// Adds binary variables to this MathematicalProgram.
  /**
   * Adds binary variables, appending them to an internal vector of any
   * existing vars.
   * The new variables are initialized to zero.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @param rows  The number of rows in the new variables.
   * @param cols  The number of columns in the new variables.
   * @param names A vector of strings containing the name for each variable.
   * @return The DecisionVariableMatrix of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto b = prog.NewBinaryVariables(2, 3, {"b1", "b2", "b3", "b4", "b5",
   * "b6");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user for understand.
   */
  DecisionVariableMatrixX NewBinaryVariables(
      size_t rows, size_t cols, const std::vector<std::string>& names);

  /**
   * Adds binary variables to this MathematicalProgram, with default name "b".
   * The new variables are returned and viewed as a matrix, with size
   * \param rows x \param cols.
   * @see NewBinaryVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  DecisionVariableMatrixX NewBinaryVariables(size_t rows, size_t cols,
                                             const std::string& name = "b");

  /**
   * Adds binary variables to this MathematicalProgram. The new variables are
   * viewed as a column vector, with size @p rows x 1.
   * @see NewBinaryVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  DecisionVariableVectorX NewBinaryVariables(size_t rows,
                                             const std::string& name = "b");

  /**
   * Adds a symmetric matrix as decision variables to this MathematicalProgram.
   * The optimization will only use the stacked columns of the
   * lower triangular part of the symmetric matrix as decision variables.
   * @param rows The rows of the symmetric matrix.
   * @param names A std::vector containing the names of each entry in the lower
   * triagular part of the symmetric matrix. The length of @p names is
   * @p rows * (rows+1) / 2.
   * @return The newly added decision variables.
   */
  DecisionVariableMatrixX NewSymmetricContinuousVariables(
      size_t rows, const std::vector<std::string>& names);

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
  DecisionVariableMatrixX NewSymmetricContinuousVariables(
      size_t rows, const std::string& name = "Symmetric");

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
  DecisionVariableMatrix<rows, rows> NewSymmetricContinuousVariables(
      const std::string& name = "Symmetric") {
    std::array<std::string, rows*(rows + 1) / 2> names;
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

  /**
   * Adds a generic cost to the optimization program.
   * @param obj The added objective.
   * @param vars The decision variables on which the cost depend.
   */
  void AddCost(const std::shared_ptr<Constraint>& obj,
               const VariableListRef& vars);

  /**
   * Adds a cost to the problem which covers all decision
   * variables created at the time the cost was added.
   */
  template <typename ConstraintT>
  void AddCost(std::shared_ptr<ConstraintT> constraint) {
    AddCost(constraint, {decision_variables_});
  }

  template <typename F>
  static std::shared_ptr<Constraint> MakeCost(F&& f) {
    return std::make_shared<
        ConstraintImpl<typename std::remove_reference<F>::type>>(
        std::forward<F>(f));
  }

  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  AddCost(F&& f, const VariableListRef& vars) {
    auto c = MakeCost(std::forward<F>(f));
    AddCost(c, vars);
    return c;
  }

  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  AddCost(F&& f) {
    return AddCost(std::forward<F>(f), {decision_variables_});
  }

  // libstdc++ 4.9 evaluates
  // `std::is_convertible<std::unique_ptr<Unrelated>,
  // std::shared_ptr<Constraint>>::value`
  // incorrectly as `true` so our enable_if overload is not used.
  // Provide an explicit alternative for this case.
  template <typename F>
  std::shared_ptr<Constraint> AddCost(std::unique_ptr<F>&& f,
                                      const VariableListRef& vars) {
    auto c = std::make_shared<ConstraintImpl<std::unique_ptr<F>>>(
        std::forward<std::unique_ptr<F>>(f));
    AddCost(c, vars);
    return c;
  }
  template <typename F>
  std::shared_ptr<Constraint> AddCost(std::unique_ptr<F>&& f) {
    return AddCost(std::forward<std::unique_ptr<F>>(f), {decision_variables_});
  }

  /**
   * Adds a cost term of the form c'*x.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  void AddCost(const std::shared_ptr<LinearConstraint>& obj,
               const VariableListRef& vars);

  /**
   * Adds a linear cost term of the form c'*x.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  template <typename DerivedC>
  std::shared_ptr<LinearConstraint> AddLinearCost(
      const Eigen::MatrixBase<DerivedC>& c, const VariableListRef& vars) {
    using Scalar = typename DerivedC::Scalar;
    auto cost = std::make_shared<LinearConstraint>(
        c.transpose(), drake::Vector1<Scalar>::Constant(
                           -std::numeric_limits<Scalar>::infinity()),
        drake::Vector1<Scalar>::Constant(
            std::numeric_limits<Scalar>::infinity()));
    AddCost(cost, vars);
    return cost;
  }

  /**
   * Adds a linear cost term of the form c'*x.
   * Applied to all decision variables existing at the time when
   * the cost is added, and pushes onto
   * the linear cost data structure.
   */
  template <typename DerivedC>
  std::shared_ptr<LinearConstraint> AddLinearCost(
      const Eigen::MatrixBase<DerivedC>& c) {
    return AddLinearCost(c, {decision_variables_});
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x.
   * Applied to subset of the variables and pushes onto
   * the quadratic cost data structure.
   */
  void AddCost(const std::shared_ptr<QuadraticConstraint>& obj,
               const VariableListRef& vars);

  /**
   * Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   */
  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> AddQuadraticErrorCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& x_desired,
      const VariableListRef& vars) {
    auto cost = std::make_shared<QuadraticConstraint>(
        2 * Q, -2 * Q * x_desired, -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity());
    AddCost(cost, vars);
    return cost;
  }

  /**
   * Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   * Applied to all (currently existing) variables.
   */
  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> AddQuadraticErrorCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& x_desired) {
    return AddQuadraticErrorCost(Q, x_desired, {decision_variables_});
  }

  /**
   * Adds a cost term of the form | Ax - b |^2.
   */
  template <typename DerivedA, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> AddL2NormCost(
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<Derivedb>& b, const VariableListRef& vars) {
    return AddQuadraticCost(2 * A.transpose() * A, -2 * A.transpose() * b,
                            vars);
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x
   * Applied to subset of the variables.
   */
  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> AddQuadraticCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& b, const VariableListRef& vars) {
    auto cost = std::make_shared<QuadraticConstraint>(
        Q, b, -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity());
    AddCost(cost, vars);
    return cost;
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x.
   * Applies to all (currently existing) variables.
   */
  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> AddQuadraticCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& b) {
    return AddQuadraticCost(Q, b, {decision_variables_});
  }

  /**
   * Adds a constraint to the problem which covers all decision.
   * variables created at the time the constraint was added.
   */
  template <typename ConstraintT>
  void AddConstraint(std::shared_ptr<ConstraintT> constraint) {
    AddConstraint(constraint, {decision_variables_});
  }

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   */
  void AddConstraint(std::shared_ptr<Constraint> con,
                     const VariableListRef& vars);

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   */
  void AddConstraint(std::shared_ptr<LinearConstraint> con,
                     const VariableListRef& vars);

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   */
  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  std::shared_ptr<LinearConstraint> AddLinearConstraint(
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub, const VariableListRef& vars) {
    auto constraint = std::make_shared<LinearConstraint>(A, lb, ub);
    AddConstraint(constraint, vars);
    return constraint;
  }

  /**
   * Adds linear constraints to the program for all (currently existing)
   * variables.
   */
  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  std::shared_ptr<LinearConstraint> AddLinearConstraint(
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub) {
    return AddLinearConstraint(A, lb, ub, {decision_variables_});
  }

  /**
   * Adds one row of linear constraint referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * lb <= a*vars <= ub
   * @param a A row vector.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   * @param vars Each element in the container
   * is a DecisionVariableMatrix object, which contains
   * a matrix of decision variables.
   */
  template <typename DerivedA>
  std::shared_ptr<LinearConstraint> AddLinearConstraint(
      const Eigen::MatrixBase<DerivedA>& a, double lb, double ub,
      const VariableListRef& vars) {
    DRAKE_ASSERT(a.rows() == 1);
    return AddLinearConstraint(a, drake::Vector1d(lb), drake::Vector1d(ub),
                               vars);
  }

  /**
   * Adds one row of linear constraint on all variables.
   * lb <= a*vars <= ub
   * @param a A row vector.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound
   */
  template <typename DerivedA>
  std::shared_ptr<LinearConstraint> AddLinearConstraint(
      const Eigen::MatrixBase<DerivedA>& a, double lb, double ub) {
    DRAKE_ASSERT(a.rows() == 1);
    return AddLinearConstraint(a, drake::Vector1d(lb), drake::Vector1d(ub),
                               {decision_variables_});
  }

  /**
   * Adds linear equality constraints referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   */
  void AddConstraint(std::shared_ptr<LinearEqualityConstraint> con,
                     const VariableListRef& vars);

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints referencing potentially a subset of
   * the decision variables.
   *
   * Example: to add two equality constraints which only depend on two of the
   * elements of x, you could use
   * @code{.cc}
   *   auto x = prog.NewContinuousDecisionVariable(6,"myvar");
   *   Eigen::Matrix2d Aeq;
   *   Aeq << -1, 2,
   *           1, 1;
   *   Eigen::Vector2d beq(1, 3);
   *   prog.AddLinearEqualityConstraint(Aeq, beq,{x(2), x(5)});
   * @endcode
   * The code above imposes constraints
   * @f[-x(2) + 2x(5) = 1 @f]
   * @f[ x(2) +  x(5) = 3 @f]
   */
  template <typename DerivedA, typename DerivedB>
  std::shared_ptr<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedA>& Aeq,
      const Eigen::MatrixBase<DerivedB>& beq, const VariableListRef& vars) {
    auto constraint = std::make_shared<LinearEqualityConstraint>(Aeq, beq);
    AddConstraint(constraint, vars);
    return constraint;
  }

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints to the program for all
   * (currently existing) variables.
   */
  template <typename DerivedA, typename DerivedB>
  std::shared_ptr<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedA>& Aeq,
      const Eigen::MatrixBase<DerivedB>& beq) {
    return AddLinearEqualityConstraint(Aeq, beq, {decision_variables_});
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
   */
  template <typename DerivedA>
  std::shared_ptr<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedA>& a, double beq,
      const VariableListRef& vars) {
    DRAKE_ASSERT(a.rows() == 1);
    return AddLinearEqualityConstraint(a, drake::Vector1d(beq), vars);
  }

  /**
   * Adds one row of linear equality constraint referencing all
   * decision variables.
   * @f[
   * ax = beq
   * @f]
   * @param a A row vector.
   * @param beq A scalar.
   */
  template <typename DerivedA>
  std::shared_ptr<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedA>& a, double beq) {
    DRAKE_ASSERT(a.rows() == 1);
    return AddLinearEqualityConstraint(a, drake::Vector1d(beq),
                                       {decision_variables_});
  }
  /**
   * Adds bounding box constraints referencing potentially a subset of
   * the decision variables.
   */
  void AddConstraint(std::shared_ptr<BoundingBoxConstraint> con,
                     const VariableListRef& vars) {
    VariableList var_list(vars);
    DRAKE_ASSERT(var_list.column_vectors_only());
    required_capabilities_ |= kLinearConstraint;
    int var_dim = var_list.size();
    DRAKE_ASSERT(con->num_constraints() == static_cast<size_t>(var_dim));
    bbox_constraints_.push_back(Binding<BoundingBoxConstraint>(con, var_list));
  }

  /** AddBoundingBoxConstraint
   *
   * Adds bounding box constraints referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   */
  template <typename DerivedLB, typename DerivedUB>
  std::shared_ptr<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub, const VariableListRef& vars) {
    auto constraint = std::make_shared<BoundingBoxConstraint>(lb, ub);
    AddConstraint(constraint, vars);
    return constraint;
  }

  /** AddBoundingBoxConstraint
   *
   * Adds bounding box constraints to the program for all
   * (currently existing) variables.
   */
  template <typename DerivedLB, typename DerivedUB>
  std::shared_ptr<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub) {
    return AddBoundingBoxConstraint(lb, ub, {decision_variables_});
  }

  /**
   * Adds bounds for a single variable.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param var The decision variable.
   */
  std::shared_ptr<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const symbolic::Variable& var) {
    DecisionVariableMatrix<1, 1> var_matrix(var);
    return AddBoundingBoxConstraint(drake::Vector1d(lb), drake::Vector1d(ub),
                                    {var_matrix});
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in the list.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   *
   * Note: This version of the interface *does* accept references to
   * non-column-vector decision variable matrices.
   */
  std::shared_ptr<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const VariableListRef& vars) {
    VariableList var_list(vars);
    int var_dim = var_list.size();
    if (var_list.column_vectors_only()) {
      return AddBoundingBoxConstraint(Eigen::VectorXd::Constant(var_dim, lb),
                                      Eigen::VectorXd::Constant(var_dim, ub),
                                      vars);
    } else {  // Support non-column-vector decision variable matrices.
      VariableListRef flattened_vars;
      for (const auto& v : vars) {
        if (v.cols() == 1) {
          flattened_vars.push_back(v);
        } else {
          Eigen::Map<const DecisionVariableVectorX> vec(v.data(), v.size());
          flattened_vars.push_back(vec);
        }
      }
      return AddBoundingBoxConstraint(Eigen::VectorXd::Constant(var_dim, lb),
                                      Eigen::VectorXd::Constant(var_dim, ub),
                                      flattened_vars);
    }
  }

  /**
   * Adds Lorentz cone constraint referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)^2 + ... + z(n-1)^2}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   */
  void AddConstraint(std::shared_ptr<LorentzConeConstraint> con,
                     const VariableListRef& vars);

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables (defined in the vars parameter).
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)^2 + ... + z(n-1)^2}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   * @param A A @f$\mathbb{R}^{n\times m}@f$ matrix, whose number of columns
   * equals to the size of the decision variables.
   * @param b A @f$\mathbb{R}^n@f$ vector, whose number of rows equals to the
   * size of the decision variables.
   * @param vars The list of containing @f$ m @f$ decision variables.
   * @return The newly added Lorentz cone constraint.
   */
  std::shared_ptr<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableListRef& vars) {
    auto constraint = std::make_shared<LorentzConeConstraint>(A, b);
    AddConstraint(constraint, vars);
    return constraint;
  }

  /**
   * Adds Lorentz cone constraint to the program for all
   * (currently existing) variables.
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)^2 + ... + z(n-1)^2}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   * @param A A @f$\mathbb{R}^{n \times m}@f$ matrix.
   * @param b A @f$ \mathbb{R}^{n} @f$ vector.
   * @return The newly added Lorentz cone constraint.
   */
  std::shared_ptr<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b) {
    return AddLorentzConeConstraint(A, b, {decision_variables_});
  }

  /**
   * Imposes that a vector @f$ x\in\mathbb{R}^m @f$ lies in Lorentz cone. Namely
   * @f[
   * x_0 \ge \sqrt{x_1^2 + .. + x_{m-1}^2}
   * @f]
   * <!-->
   * x(0) >= sqrt(x(1)^2 + ... + x(m-1)^2)
   * <-->
   * @param vars The stacked column of vars should lie within the Lorentz cone.
   * @return The newly added Lorentz cone constraint.
   */
  std::shared_ptr<LorentzConeConstraint> AddLorentzConeConstraint(
      const VariableListRef& vars);

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables. The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)^2 + ... + z(n-1)^2
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   * @param con A pointer to a RotatedLorentzConeConstraint object.
   * @param vars The decision variables on which the constraint is imposed.
   */
  void AddConstraint(std::shared_ptr<RotatedLorentzConeConstraint> con,
                     const VariableListRef& vars);

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables, The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)^2 + ... + z(n-1)^2
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   * where @f$ A\in\mathbb{R}^{n\times m}, b\in\mathbb{R}^n@f$ are given
   * matrices.
   *
   * you can call
   * @code{.cc}
   *   auto x = prog.NewContinuousVariables(n,'x');
   *   auto con = prog.AddRotatedLorentzConeConstraint(A, b, {x});
   * @endcode
   *
   * @param A A matrix whose number of columns equals to the size of the
   * decision variables.
   * @param b A vector whose number of rows equals to the size fo the decision
   * variables.
   * @param vars The decision variables on which the constraint is imposed.
   * Each DecisionVariableMatrix object should have only one column.
   */
  std::shared_ptr<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableListRef& vars) {
    auto constraint = std::make_shared<RotatedLorentzConeConstraint>(A, b);
    AddConstraint(constraint, vars);
    return constraint;
  }

  /**
   * Adds a rotated Lorentz constraint to the program for all
   * (currently existing) variables, the linear expression @f$ z=Ax+b @f$ is in
   * rotated Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)^2 + ... + z(n-1)^2
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
   */
  std::shared_ptr<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b) {
    return AddRotatedLorentzConeConstraint(A, b, {decision_variables_});
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
  std::shared_ptr<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const VariableListRef& vars);

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  template <typename DerivedM, typename Derivedq>
  std::shared_ptr<LinearComplementarityConstraint>
  AddLinearComplementarityConstraint(const Eigen::MatrixBase<DerivedM>& M,
                                     const Eigen::MatrixBase<Derivedq>& q,
                                     const VariableListRef& vars) {
    VariableList var_list(vars);
    DRAKE_ASSERT(var_list.column_vectors_only());
    required_capabilities_ |= kLinearComplementarityConstraint;

    // Linear Complementarity Constraint cannot currently coexist with any
    // other types of constraint or cost.
    // (TODO(ggould-tri) relax this to non-overlapping bindings, possibly by
    // calling multiple solvers.)
    DRAKE_ASSERT(generic_constraints_.empty());
    DRAKE_ASSERT(generic_costs_.empty());
    DRAKE_ASSERT(quadratic_costs_.empty());
    DRAKE_ASSERT(linear_costs_.empty());
    DRAKE_ASSERT(linear_constraints_.empty());
    DRAKE_ASSERT(linear_equality_constraints_.empty());
    DRAKE_ASSERT(bbox_constraints_.empty());
    DRAKE_ASSERT(lorentz_cone_constraint_.empty());
    DRAKE_ASSERT(rotated_lorentz_cone_constraint_.empty());

    auto constraint = std::make_shared<LinearComplementarityConstraint>(M, q);
    linear_complementarity_constraints_.push_back(
        Binding<LinearComplementarityConstraint>(constraint, var_list));
    return constraint;
  }

  /**
   * Adds a linear complementarity constraint to the program for all
   * (currently existing) variables.
   */
  template <typename DerivedM, typename Derivedq>
  std::shared_ptr<LinearComplementarityConstraint>
  AddLinearComplementarityConstraint(const Eigen::MatrixBase<DerivedM>& M,
                                     const Eigen::MatrixBase<Derivedq>& q) {
    return AddLinearComplementarityConstraint(M, q, {decision_variables_});
  }

  /**
   * Adds a polynomial constraint to the program referencing a subset
   * of the decision variables (defined in the vars parameter).
   */
  std::shared_ptr<Constraint> AddPolynomialConstraint(
      const VectorXPoly& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const VariableListRef& vars);

  /**
   * Adds a polynomial constraint to the program referencing all of the
   * decision variables.
   */
  std::shared_ptr<Constraint> AddPolynomialConstraint(
      const VectorXPoly& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) {
    return AddPolynomialConstraint(polynomials, poly_vars, lb, ub,
                                   {decision_variables_});
  }

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   * @param symmetric_matrix_var A symmetric DecisionVariableMatrix object.
   */
  void AddConstraint(
      std::shared_ptr<PositiveSemidefiniteConstraint> con,
      const Eigen::Ref<const DecisionVariableMatrixX> symmetric_matrix_var);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   * In Debug mode, @throws error if
   * @p symmetric_matrix_var is not symmetric.
   * @param symmetric_matrix_var A symmetric DecisionVariableMatrix object.
   */
  std::shared_ptr<PositiveSemidefiniteConstraint>
  AddPositiveSemidefiniteConstraint(
      const Eigen::Ref<const DecisionVariableMatrixX> symmetric_matrix_var);

  /**
   * Adds a linear matrix inequality constraint to the program.
   */
  void AddConstraint(std::shared_ptr<LinearMatrixInequalityConstraint> con,
                     const VariableListRef& vars);

  /**
   * Adds a linear matrix inequality constraint to the program.
   */
  std::shared_ptr<LinearMatrixInequalityConstraint>
  AddLinearMatrixInequalityConstraint(
      const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
      const VariableListRef& vars);

  // template <typename FunctionType>
  // void AddCost(std::function..);
  // void AddLinearCost(const Eigen::MatrixBase<Derived>& c, const vector<const
  // DecisionVariable&>& vars)
  // void addQuadraticCost ...

  /**
   * Set the initial guess for the decision variables stored in @p var to be x0.
   */
  template <typename DerivedA, typename DerivedB>
  void SetInitialGuess(const Eigen::MatrixBase<DerivedA>& decision_variable_mat,
                       const Eigen::MatrixBase<DerivedB>& x0) {
    DRAKE_ASSERT(decision_variable_mat.rows() == x0.rows());
    DRAKE_ASSERT(decision_variable_mat.cols() == x0.cols());
    for (int i = 0; i < decision_variable_mat.rows(); ++i) {
      for (int j = 0; j < decision_variable_mat.cols(); ++j) {
        x_initial_guess_(
            FindDecisionVariableIndex(decision_variable_mat(i, j))) = x0(i, j);
      }
    }
  }

  /**
   * Set the intial guess for ALL decision variables.
   * @param x0 A vector of appropriate size (num_vars() x 1).
   */
  template <typename Derived>
  void SetInitialGuessForAllVariables(const Eigen::MatrixBase<Derived>& x0) {
    DRAKE_ASSERT(x0.rows() == static_cast<int>(num_vars_) && x0.cols() == 1);
    x_initial_guess_ = x0;
  }

  /**
   * Solve the MathematicalProgram.
   *
   * @return SolutionResult indicating if the solution was successful.
   */
  SolutionResult Solve();
  // TODO(naveenoid) : add argument for options

  //    template <typename Derived>
  //    bool solve(const Eigen::MatrixBase<Derived>& x0);

  //    getCostValue();
  //    getExitFlag();
  //    getInfeasibleConstraintNames();

  void PrintSolution() {
    for (int i = 0; i < static_cast<int>(num_vars_); ++i) {
      std::cout << decision_variables_(i).get_name() << " = "
                << GetSolution(decision_variables_(i)) << std::endl;
    }
  }

  template <typename Derived>
  void SetDecisionVariableValues(const Eigen::MatrixBase<Derived>& x) {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == num_vars_);
    for (int i = 0; i < static_cast<int>(num_vars_); ++i) {
      x_values_[i] = x(i);
    }
  }

  /**
   * Set an option for a particular solver.  This interface does not
   * do any verification of solver parameters beyond what an
   * individual solver does for itself.  It does not even verify that
   * the specifed solver exists.  Use this only when you have
   * particular knowledge of what solver is being invoked, and exactly
   * what tuning is required.
   *
   * Supported solver names/options:
   *
   * "SNOPT" -- Paramater names and values as specified in SNOPT
   * User's Guide section 7.7 "Description of the optional parameters",
   * used as described in section 7.5 for snSet().
   *
   * "IPOPT" -- Paramater names and values as specified in IPOPT users
   * guide section "Options Reference"
   * http://www.coin-or.org/Ipopt/documentation/node40.html
   *
   * "GUROBI" -- Parameter name and values as specified in GUROBI Reference
   * Manual, section 10.2 "Parameter Descriptions"
   * https://www.gurobi.com/documentation/6.5/refman/parameters.html
   */
  void SetSolverOption(const std::string& solver_name,
                       const std::string& solver_option, double option_value) {
    solver_options_double_[solver_name][solver_option] = option_value;
  }

  void SetSolverOption(const std::string& solver_name,
                       const std::string& solver_option, int option_value) {
    solver_options_int_[solver_name][solver_option] = option_value;
  }

  void SetSolverOption(const std::string& solver_name,
                       const std::string& solver_option,
                       const std::string& option_value) {
    solver_options_str_[solver_name][solver_option] = option_value;
  }

  const std::map<std::string, double>& GetSolverOptionsDouble(
      const std::string& solver_name) {
    return solver_options_double_[solver_name];
  }

  const std::map<std::string, int>& GetSolverOptionsInt(
      const std::string& solver_name) {
    return solver_options_int_[solver_name];
  }

  const std::map<std::string, std::string>& GetSolverOptionsStr(
      const std::string& solver_name) {
    return solver_options_str_[solver_name];
  }

  /**
   * Get the name and result code of the particular solver which was
   * used to solve this MathematicalProgram.  The solver names and
   * results are not documented here as this function is only intended
   * for debugging, testing, and support of certain legacy
   * APIs.
   */
  void GetSolverResult(std::string* solver_name, int* solver_result) const {
    *solver_name = solver_name_;
    *solver_result = solver_result_;
  }

  void SetSolverResult(const std::string& solver_name, int solver_result) {
    solver_name_ = solver_name;
    solver_result_ = solver_result;
  }

  /**
   * Getter for all generic costs.
   */
  const std::vector<Binding<Constraint>>& generic_costs() const {
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
  const std::vector<Binding<LinearConstraint>>& linear_costs() const {
    return linear_costs_;
  }

  /** Getter for quadratic costs. */
  const std::vector<Binding<QuadraticConstraint>>& quadratic_costs() const {
    return quadratic_costs_;
  }

  /** Getter for linear constraints. */
  const std::vector<Binding<LinearConstraint>>& linear_constraints() const {
    return linear_constraints_;
  }

  /** Getter for Lorentz cone constraint */
  const std::vector<Binding<LorentzConeConstraint>>& lorentz_cone_constraints()
      const {
    return lorentz_cone_constraint_;
  }

  /** Getter for rotated Lorentz cone constraint */
  const std::vector<Binding<RotatedLorentzConeConstraint>>&
  rotated_lorentz_cone_constraints() const {
    return rotated_lorentz_cone_constraint_;
  }

  /** Getter for positive semidefinite constraint */
  const std::vector<Binding<PositiveSemidefiniteConstraint>>&
  positive_semidefinite_constraints() const {
    return positive_semidefinite_constraint_;
  }

  /** Getter for linear matrix inequality constraint */
  const std::vector<Binding<LinearMatrixInequalityConstraint>>&
  linear_matrix_inequality_constraints() const {
    return linear_matrix_inequality_constraint_;
  }

  /**
   * Getter returning all costs (for now linear costs appended to
   * generic costs, then quadratic costs appended to
   * generic costs).
   */
  std::vector<Binding<Constraint>> GetAllCosts() const {
    std::vector<Binding<Constraint>> costlist = generic_costs_;
    costlist.insert(costlist.end(), linear_costs_.begin(), linear_costs_.end());
    costlist.insert(costlist.end(), quadratic_costs_.begin(),
                    quadratic_costs_.end());
    return costlist;
  }

  /**
   * Getter returning all linear constraints (both linear equality and
   * inequality constraints).
   */
  std::vector<Binding<LinearConstraint>> GetAllLinearConstraints() const {
    std::vector<Binding<LinearConstraint>> conlist = linear_constraints_;
    conlist.insert(conlist.end(), linear_equality_constraints_.begin(),
                   linear_equality_constraints_.end());
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

  // Base class for solver-specific data.  A solver implementation may derive
  // a helper class from this for use with getSolverData.
  struct SolverData {
    virtual ~SolverData() {}
  };

  // Call from solver implementations to get a persistently-stored
  // helper structure of type T (derived from SolverData).  If no
  // data of type T is already stored then a new one will be created
  // and stored, replacing data from any other solver in this problem
  // instance.
  template <typename T>
  std::shared_ptr<T> GetSolverData() {
    auto p = std::dynamic_pointer_cast<T>(solver_data_);
    if (!p) {
      p = std::make_shared<T>();
      solver_data_ = p;
    }
    return p;
  }

  /** Getter for number of variables in the optimization program */
  size_t num_vars() const { return num_vars_; }

  /**
   * Returns a vector containing the type of each decision variable.
   * The length of the vector is the same as
   * MathematicalProgram::num_vars(). variable_type[i] is the type
   * of x(i) in the MathematicalProgram, where x is the vector containing all
   * decision variables.
   */
  const std::vector<VarType>& DecisionVariableTypes() const {
    return decision_variable_type_;
  }

  /** Returns the type of the decision variable. */
  VarType DecisionVariableType(const symbolic::Variable& var) const;

  /** Getter for the initial guess */
  const Eigen::VectorXd& initial_guess() const { return x_initial_guess_; }

  /**
   * Returns the solution in a flat Eigen::VectorXd. The caller needs to
   * compute the solution first by calling Solve.
   * @return a flat Eigen vector that represents the solution.
   */
  const Eigen::VectorXd GetSolutionVectorValues() const {
    return GetSolution(decision_variables_);
  }

  /** Returns the index of the decision variable. Internally the solvers thinks
   * all variables are stored in an array, and it accesess each individual
   * variable using its index. This index is used when adding constraints
   * and costs for each solver.
   */
  size_t FindDecisionVariableIndex(const symbolic::Variable& var) const;

  /**
   * Gets the solution of an Eigen matrix of decision variables.
   * @tparam Derived An Eigen matrix containing symbolic::Variable.
   * @param var The decision variables.
   * @return The value of the decision variable after solving the problem.
   */
  template <typename Derived>
  Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
  GetSolution(const Eigen::MatrixBase<Derived>& var) const {
    static_assert(
        std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
        "The input should be an Eigen matrix of symbolic::Variable object.");
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>
        value(var.rows(), var.cols());
    for (int i = 0; i < var.rows(); ++i) {
      for (int j = 0; j < var.cols(); ++j) {
        auto it = decision_variable_index_.find(var(i, j).get_id());
        DRAKE_ASSERT(it != decision_variable_index_.end());
        value(i, j) = x_values_[it->second];
      }
    }
    return value;
  }

  /**
   * Gets the value of a single decision variable.
   */
  double GetSolution(const symbolic::Variable& var) const;

  /**
   * Evaluate the constraint in the Binding at the solution value.
   * @return The value of the constraint in the binding.
   * TODO(hongkai.dai): Do not use teample function, when the Binding is moved
   * to a public class.
   */
  template <typename _Binding>
  Eigen::VectorXd EvalBindingAtSolution(const _Binding& binding) const {
    Eigen::VectorXd val(binding.constraint()->num_constraints());
    size_t dim = 0;
    Eigen::VectorXd flat_solution(binding.GetNumElements());
    for (const auto& var : binding.variable_list().variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      flat_solution.segment(dim, var.rows()) = GetSolution(var);
      dim += var.rows();
    }
    binding.constraint()->Eval(flat_solution, val);
    return val;
  }

  /**
   * Sets the values of the decision variables, bound by the elements in @p
   * binding.
   * @tparam _Binding should be MathematicalProgram::Binding class
   * @param binding_solution The value of the variables bound in the @p binding.
   * @param binding A binding containing the constraint and bound variables.
   * The value of the bound variable will be changed by calling this function.
   */
  template <typename _Binding>
  void SetDecisionVariableValueFromBinding(
      const Eigen::Ref<const Eigen::VectorXd> &binding_solution,
      const _Binding &binding) {
    DRAKE_ASSERT(static_cast<size_t>(binding_solution.rows()) ==
                 binding.GetNumElements());

    size_t solution_index = 0;
    for (const auto& var : binding.variable_list().variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      for (int i = 0; i < var.rows(); ++i) {
        x_values_[FindDecisionVariableIndex(var(i))] =
            binding_solution(solution_index + i);
      }
      solution_index += var.rows();
    }
  }

  /** Getter for all decision variables in the program. */
  const DecisionVariableVectorX& decision_variables() const {
    return decision_variables_;
  }

  /** Getter for the decision variable with index @p i in the program. */
  const symbolic::Variable& decision_variable(int i) const {
    return decision_variables_(i);
  }

 private:
  // maps the ID of a symbolic variable to the index of the variable stored in
  // the optimization program.
  std::unordered_map<size_t, size_t> decision_variable_index_{};

  std::vector<VarType> decision_variable_type_;  // decision_variable_type_[i]
                                                 // stores the type of the
                                                 // variable with index i.

  DecisionVariableVectorX decision_variables_;
  std::vector<Binding<Constraint>> generic_costs_;
  std::vector<Binding<Constraint>> generic_constraints_;
  std::vector<Binding<QuadraticConstraint>> quadratic_costs_;
  std::vector<Binding<LinearConstraint>> linear_costs_;
  // TODO(naveenoid) : quadratic_constraints_

  // note: linear_constraints_ does not include linear_equality_constraints_
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

  // Invariant:  The bindings in this list must be non-overlapping.
  // TODO(ggould-tri) can this constraint be relaxed?
  std::vector<Binding<LinearComplementarityConstraint>>
      linear_complementarity_constraints_;

  size_t num_vars_;
  Eigen::VectorXd x_initial_guess_;
  std::vector<double> x_values_;
  std::shared_ptr<SolverData> solver_data_;
  std::string solver_name_;
  int solver_result_;
  std::map<std::string, std::map<std::string, double>> solver_options_double_;
  std::map<std::string, std::map<std::string, int>> solver_options_int_;
  std::map<std::string, std::map<std::string, std::string>> solver_options_str_;

  AttributesSet required_capabilities_{0};

  std::unique_ptr<MathematicalProgramSolverInterface> ipopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> nlopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> snopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> moby_lcp_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> linear_system_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface>
      equality_constrained_qp_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> gurobi_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> mosek_solver_;

  template <typename T>
  void NewVariables_impl(
      VarType type, const T& names, bool is_symmetric,
      Eigen::Ref<DecisionVariableMatrixX> decision_variable_matrix) {
    switch (type) {
      case VarType::CONTINUOUS:
        break;
      case VarType::BINARY:
        required_capabilities_ |= kBinaryVariable;
        break;
      default:
        throw std::runtime_error("Unknown variable type");
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
    decision_variables_.conservativeResize(num_vars_ + num_new_vars,
                                           Eigen::NoChange);
    x_values_.resize(num_vars_ + num_new_vars, NAN);
    decision_variable_type_.resize(num_vars_ + num_new_vars);
    int row_index = 0;
    int col_index = 0;
    for (int i = 0; i < num_new_vars; ++i) {
      decision_variables_(num_vars_ + i) = symbolic::Variable(names[i]);
      const size_t new_var_index = num_vars_ + i;
      decision_variable_index_.insert(std::pair<size_t, size_t>(
          decision_variables_(new_var_index).get_id(), new_var_index));
      decision_variable_type_[new_var_index] = type;
      decision_variable_matrix(row_index, col_index) =
          decision_variables_(num_vars_ + i);
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

    num_vars_ += num_new_vars;
    x_initial_guess_.conservativeResize(num_vars_);
    x_initial_guess_.tail(num_new_vars) = Eigen::VectorXd::Zero(num_new_vars);
  }

  DecisionVariableMatrixX NewVariables(VarType type, int rows, int cols,
                                       bool is_symmetric,
                                       const std::vector<std::string>& names);

  DecisionVariableVectorX NewVariables(VarType type, int rows,
                                       const std::vector<std::string>& names);
};
}  // namespace solvers
}  // namespace drake
