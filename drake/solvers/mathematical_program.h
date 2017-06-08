#pragma once

#include <array>
#include <cstddef>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/create_constraint.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/function.h"
#include "drake/solvers/indeterminate.h"
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
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MathematicalProgram)

  using VarType = symbolic::Variable::Type;

  MathematicalProgram();
  virtual ~MathematicalProgram() {}

  /**
   * Adds new variables to MathematicalProgram.
   * Appending new variables to an internal vector of any existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows Number of rows in the variables.
   * @tparam cols Number of cols in the variables.
   * @param name An array containing the name of each variable.
   * @return The MatrixDecisionVariable<rows, cols> containing rows * cols new
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
  MatrixDecisionVariable<rows, cols> NewVariables(
      VarType type, const std::array<std::string, rows * cols>& names) {
    MatrixDecisionVariable<rows, cols> decision_variable_matrix;
    NewVariables_impl(type, names, false, decision_variable_matrix);
    return decision_variable_matrix;
  }

  /**
   * Adds column vector variables to the optimization program.
   */
  template <int rows>
  VectorDecisionVariable<rows> NewVariables(
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
  MatrixDecisionVariable<rows, rows> NewSymmetricVariables(
      VarType type, const std::array<std::string, rows*(rows + 1) / 2>& names) {
    MatrixDecisionVariable<rows, rows> decision_variable_matrix;
    NewVariables_impl(type, names, true, decision_variable_matrix);
    return decision_variable_matrix;
  }

  /**
   * Adds continuous variables to this MathematicalProgram.
   * @see NewContinuousVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  VectorXDecisionVariable NewContinuousVariables(
      size_t rows, const std::vector<std::string>& names);

  /**
   * Adds continuous variables to this MathematicalProgram, with default name
   * "x".
   * @see NewContinuousVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  VectorXDecisionVariable NewContinuousVariables(size_t rows,
                                                 const std::string& name = "x");

  /// Adds continuous variables to this MathematicalProgram.
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
   * @param cols  The number of columns in the new variables.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
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
  MatrixXDecisionVariable NewContinuousVariables(
      size_t rows, size_t cols, const std::vector<std::string>& names);

  /**
   * Adds continuous variables to this MathematicalProgram, with default name
   * "X". The new variables are returned and viewed as a matrix, with size
   * @p rows x @p cols.
   * @see NewContinuousVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  MatrixXDecisionVariable NewContinuousVariables(size_t rows, size_t cols,
                                                 const std::string& name = "X");

  /// Adds continuous variables to this MathematicalProgram.
  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows  The number of rows in the new variables.
   * @tparam cols  The number of columns in the new variables.
   * @param names An array of strings containing the name for each variable.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
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
  MatrixDecisionVariable<rows, cols> NewContinuousVariables(
      const std::array<std::string, rows * cols>& names) {
    return NewVariables<rows, cols>(VarType::CONTINUOUS, names);
  }

  /// Adds continuous variables to this MathematicalProgram.
  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows  The number of rows in the new variables.
   * @tparam cols  The number of columns in the new variables.
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
   * The name of the variable is only used for the user for understand.
   */
  template <int rows, int cols>
  MatrixDecisionVariable<rows, cols> NewContinuousVariables(
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
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows  The number of rows in the new variables.
   * @param names An array of strings containing the name for each variable.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
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
  VectorDecisionVariable<rows> NewContinuousVariables(
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
  VectorDecisionVariable<rows> NewContinuousVariables(
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
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam rows  The number of rows in the new variables.
   * @tparam cols  The number of columns in the new variables.
   * @param names An array of strings containing the name for each variable.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
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
  MatrixDecisionVariable<rows, cols> NewBinaryVariables(
      const std::array<std::string, rows * cols>& names) {
    return NewVariables<rows, cols>(VarType::BINARY, names);
  }

  /**
   * Adds a matrix of binary variables into the optimization program.
   * @tparam rows The number of rows in the newly added binary variables.
   * @tparam cols  The number of columns in the new variables.
   * @param name Each newly added binary variable will share the same name. The
   * default name is "b".
   * @return A matrix containing the newly added variables.
   */
  template <int rows, int cols>
  MatrixDecisionVariable<rows, cols> NewBinaryVariables(
      const std::string& name = "b") {
    std::array<std::string, rows * cols> names;
    int count = 0;
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        names[count] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
        ++count;
      }
    }
    return NewBinaryVariables<rows, cols>(names);
  }

  /**
   * Adds vector of binary variables into the optimization program.
   * @tparam rows The number of rows in the newly added binary variables.
   * @param names An array of strings containing the name of each variable.
   * @return A vector containing the newly added variables.
   */
  template <int rows>
  VectorDecisionVariable<rows> NewBinaryVariables(
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
  VectorDecisionVariable<rows> NewBinaryVariables(
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
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @param rows  The number of rows in the new variables.
   * @param cols  The number of columns in the new variables.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
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
  MatrixXDecisionVariable NewBinaryVariables(
      size_t rows, size_t cols, const std::vector<std::string>& names);

  /**
   * Adds binary variables to this MathematicalProgram, with default name "b".
   * The new variables are returned and viewed as a matrix, with size
   * \param rows x \param cols.
   * @see NewBinaryVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  MatrixXDecisionVariable NewBinaryVariables(size_t rows, size_t cols,
                                             const std::string& name = "b");

  /**
   * Adds binary variables to this MathematicalProgram. The new variables are
   * viewed as a column vector, with size @p rows x 1.
   * @see NewBinaryVariables(size_t rows, size_t cols, const
   * std::vector<std::string>& names);
   */
  VectorXDecisionVariable NewBinaryVariables(size_t rows,
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
  MatrixXDecisionVariable NewSymmetricContinuousVariables(
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
  MatrixXDecisionVariable NewSymmetricContinuousVariables(
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
  MatrixDecisionVariable<rows, rows> NewSymmetricContinuousVariables(
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
   */
  template <int rows>
  VectorIndeterminate<rows> NewIndeterminates(const std::string& name = "x") {
    std::array<std::string, rows> names;
    int offset = (name.compare("x") == 0) ? num_vars_ : 0;
    for (int i = 0; i < rows; ++i) {
      names[i] = name + "(" + std::to_string(offset + i) + ")";
    }
    return NewIndeterminates<rows>(names);
  }

  /**
   * Adds indeterminates to this MathematicalProgram.
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
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
   * @see drake::solvers::detail::FunctionTraits.
   */
  template <typename F>
  static std::shared_ptr<Cost> MakeCost(F&& f) {
    return MakeFunctionCost(f);
  }

  /**
   * Adds a cost to the optimization program on a list of variables.
   * @tparam F it should define functions numInputs, numOutputs and eval. Check
   * drake::solvers::detail::FunctionTraits for more detail.
   */
  template <typename F>
  typename std::enable_if<detail::is_cost_functor_candidate<F>::value,
                          Binding<Cost>>::type
  AddCost(F&& f, const VariableRefList& vars) {
    return AddCost(f, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost to the optimization program on an Eigen::Vector containing
   * decision variables.
   * @tparam F Type that defines functions numInputs, numOutputs and eval.
   * @see drake::solvers::detail::FunctionTraits.
   */
  template <typename F>
  typename std::enable_if<detail::is_cost_functor_candidate<F>::value,
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
  typename std::enable_if<detail::assert_if_is_constraint<F>::value,
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
   * @param e A quadratic symbolic expression. Throws a runtime error if the
   * expression is not quadratic.
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
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   */
  Binding<Constraint> AddConstraint(const Binding<Constraint>& binding);

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
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
   */
  template <typename C>
  auto AddConstraint(std::shared_ptr<C> con,
                     const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddConstraint(internal::CreateBinding(con, vars));
  }

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
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
   * expression. Throws an exception if
   *  1. @p e is a non-linear expression.
   *  2. <tt>lb <= e <= ub</tt> is a trivial constraint such as 1 <= 2 <= 3.
   *  3. <tt>lb <= e <= ub</tt> is unsatisfiable such as 1 <= -5 <= 3
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
   *  1. e1  <=  e2 , which is 0 <= e2 - e1 <= ∞
   *  2. e1  >=  e2 , which is 0 <= e1 - e2 <= ∞
   *  3. e1  ==  e2
   *  4. A conjunction of relational formulas where each conjunct is
   *     a relational formula matched by 1, 2, or 3.
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
   *  1. e1 <= e2 , which is -∞ <= e1 - e2 <= 0
   *  2. e1 >= e2 , which is  0 <= e1 - e2 <= ∞
   *  3. e1 == e2 , which is  0 <= e1 - e2 <= 0
   *
   * It throws an exception if
   *  1. A formula in @p formulas is not matched with one of the above
   *     patterns. Especially, strict inequalities (<, >) are not allowed.
   *  2. A formula in @p formulas includes a non-linear expression.
   *  3. A formula in @p formulas is either a trivial constraint such
   *     as "1 <= 2" or an unsatisfiable constraint such as "2 <= 1".
   *
   * @tparam Derived An Eigen Array type of Formula.
   */
  template <typename Derived>
  typename std::enable_if<
      detail::is_eigen_scalar_same<Derived, symbolic::Formula>::value,
      Binding<LinearConstraint>>::type
  AddLinearConstraint(const Eigen::ArrayBase<Derived>& formulas) {
    return AddConstraint(internal::ParseLinearConstraint(formulas));
  }

  /**
   * Adds linear equality constraints referencing potentially a
   * subset of the decision variables.
   */
  Binding<LinearEqualityConstraint> AddConstraint(
      const Binding<LinearEqualityConstraint>& binding);

  /**
   * Adds one row of linear constraint e = b where @p e is a symbolic
   * expression. Throws an exception if
   *  1. @p e is a non-linear expression.
   *  2. @p e is a constant.
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
   *  1. @p f is neither an equality formula nor a conjunction of equalities.
   *  2. @p f includes a non-linear expression.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const symbolic::Formula& f);

  /**
   * Adds linear equality constraints \f$ v = b \f$, where \p v(i) is a symbolic
   * linear expression. Throws an exception if
   * 1. @p v(i) is a non-linear expression.
   * 2. @p v(i) is a constant.
   * @tparam DerivedV An Eigen Matrix type of Expression. A column vector.
   * @tparam DerivedB An Eigen Matrix type of double. A column vector.
   * @param v v(i) is a linear symbolic expression in the form of
   * <tt> c0 + c1 * x1 + ... + cn * xn </tt> where ci is a constant and @xi is
   * a variable.
   * @param b A vector of doubles.
   * @return The newly added linear equality constraint, together with the
   * bound variables.
   */
  template <typename DerivedV, typename DerivedB>
  typename std::enable_if<
      detail::is_eigen_vector_expression_double_pair<DerivedV, DerivedB>::value,
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
   */
  template <typename DerivedV, typename DerivedB>
  typename std::enable_if<
      detail::is_eigen_matrix_expression_double_pair<DerivedV, DerivedB>::value,
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
   *   auto x = prog.NewContinuousDecisionVariable(6,"myvar");
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
   *   auto x = prog.NewContinuousDecisionVariable(6,"myvar");
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
   * binding.constraint()->lower_bound()(i) <= binding.variables()(i)
   *                   <= binding.constraint().upper_bound()(i)
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
   * auto x = prog.NewContinuousDecisionVariables<2>("x");
   * auto y = prog.NewContinuousDecisionVariables<1>("y");
   * Eigen::Vector3d lb(0, 1, 2);
   * Eigen::Vector3d ub(1, 2, 3);
   * // Imposes the constraint
   * // 0 ≤ x(0) ≤ 1
   * // 1 ≤ x(1) ≤ 2
   * // 2 ≤ y    ≤ 3
   * prog.AddBoundingBoxConstraint(lb, ub, {x, y});
   * \endcode
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
   * @param v1     The linear expression.
   * @param v2  The quadratic expression.
   * @retval binding The newly added Lorentz cone constraint, together with the
   * bound variables.
   * @pre
   * 1. `linear_expr` is a linear expression, in the form of c'*x + d.
   * 2. `quadratic_expr` is a quadratic expression, in the form of
   *    <pre>
   *          0.5 * x'*Q*x + b'x + a
   *    </pre>
   *    Also the quadratic expression has to be convex, namely Q is a
   *    positive semidefinite matrix, and the quadratic expression needs
   *    to be non-negative for any x.
   * Throws a runtime_error if the preconditions are not satisfied.
   *
   * Notice this constraint is equivalent to the vector [z;y] is within a
   * Lorentz cone, where
   * <pre>
   *  z = v1
   *  y = [1 / sqrt(2) * R * x + R⁻ᵀb; sqrt(a - 0.5 * bᵀ * Q⁻¹ * a)]
   * </pre>
   * while R satisfies Rᵀ * R = Q
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const symbolic::Expression& v1, const symbolic::Expression& v2);

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
   */
  Binding<RotatedLorentzConeConstraint> AddConstraint(
      const Binding<RotatedLorentzConeConstraint>& binding);

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
   */
  Binding<PositiveSemidefiniteConstraint> AddConstraint(
      const Binding<PositiveSemidefiniteConstraint>& binding);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   */
  Binding<PositiveSemidefiniteConstraint> AddConstraint(
      std::shared_ptr<PositiveSemidefiniteConstraint> con,
      const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   * In Debug mode, @throws error if
   * @p symmetric_matrix_var is not symmetric.
   * @param symmetric_matrix_var A symmetric MatrixDecisionVariable object.
   */
  Binding<PositiveSemidefiniteConstraint> AddPositiveSemidefiniteConstraint(
      const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix of symbolic
   * espressions @p e. We create a new symmetric matrix of variables M being
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
    if (Derived::RowsAtCompileTime == Eigen::Dynamic) {
      M = NewSymmetricContinuousVariables(e_rows);
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

  // template <typename FunctionType>
  // void AddCost(std::function..);
  // void AddLinearCost(const Eigen::MatrixBase<Derived>& c, const vector<const
  // DecisionVariable&>& vars)
  // void addQuadraticCost ...

  /**
   * Set the initial guess for the decision variables stored in @p var to be x0.
   * Variables begin with a default initial guess of NaN to indicate that no
   * guess is available.
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
   * Note that variables begin with a default initial guess of NaN to indicate
   * that no guess is available.
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

  /**
   * Sets the values of all decision variables, such that the value of
   * \p decision_variables_(i) is \p values(i).
   * @param values The values set to all the decision variables.
   */
  void SetDecisionVariableValues(
      const Eigen::Ref<const Eigen::VectorXd>& values);

  /**
   * Sets the value of some decision variables, such that the value of
   * \p variables(i) is \p values(i).
   * @param variables The value of these decision variables will be set.
   * @param values The values set to the decision variables.
   */
  void SetDecisionVariableValues(
      const Eigen::Ref<const VectorXDecisionVariable>& variables,
      const Eigen::Ref<const Eigen::VectorXd>& values);

  /**
   * Sets the value of a single decision variable in the optimization program.
   * @param var A decision variable in the program.
   * @param value The value of that decision variable.
   */
  void SetDecisionVariableValue(const symbolic::Variable& var, double value);

  /**
   * Set an option for a particular solver.  This interface does not
   * do any verification of solver parameters beyond what an
   * individual solver does for itself.  It does not even verify that
   * the specified solver exists.  Use this only when you have
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
  void SetSolverOption(SolverType solver_type, const std::string& solver_option,
                       double option_value) {
    solver_options_double_[solver_type][solver_option] = option_value;
  }

  void SetSolverOption(SolverType solver_type, const std::string& solver_option,
                       int option_value) {
    solver_options_int_[solver_type][solver_option] = option_value;
  }

  void SetSolverOption(SolverType solver_type, const std::string& solver_option,
                       const std::string& option_value) {
    solver_options_str_[solver_type][solver_option] = option_value;
  }

  const std::map<std::string, double>& GetSolverOptionsDouble(
      SolverType solver_type) {
    return solver_options_double_[solver_type];
  }

  const std::map<std::string, int>& GetSolverOptionsInt(
      SolverType solver_type) {
    return solver_options_int_[solver_type];
  }

  const std::map<std::string, std::string>& GetSolverOptionsStr(
      SolverType solver_type) {
    return solver_options_str_[solver_type];
  }

  /**
   * Get the name and result code of the particular solver which was
   * used to solve this MathematicalProgram.  The solver names and
   * results are not documented here as this function is only intended
   * for debugging, testing, and support of certain legacy
   * APIs.
   */
  void GetSolverResult(SolverType* solver_type, int* solver_result) const {
    *solver_type = solver_type_;
    *solver_result = solver_result_;
  }

  void SetSolverResult(SolverType solver_type, int solver_result) {
    solver_type_ = solver_type;
    solver_result_ = solver_result;
  }

  /**
   * Getter for optimal cost at the solution. Will return NaN if there has
   * been no successful solution.
   */
  double GetOptimalCost() const { return optimal_cost_; }

  void SetOptimalCost(double optimal_cost) { optimal_cost_ = optimal_cost; }

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
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolverData)
    SolverData() = default;
    virtual ~SolverData() = default;
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
   * all variables are stored in an array, and it acceses each individual
   * variable using its index. This index is used when adding constraints
   * and costs for each solver.
   * @pre{@p var is a decision variable in the mathematical program, otherwise
   * this function throws a runtime error.}
   */
  size_t FindDecisionVariableIndex(const symbolic::Variable& var) const;

  /**
   * Gets the solution of an Eigen matrix of decision variables.
   * @tparam Derived An Eigen matrix containing Variable.
   * @param var The decision variables.
   * @return The value of the decision variable after solving the problem.
   */

  /** Gets the number of indeterminates in the optimization program */
  int num_indeterminates() const { return indeterminates_.rows(); }

  /** Returns the index of the indeterminate. Internally a solver
   * thinks all indeterminates are stored in an array, and it acceses each
   * individual indeterminate using its index. This index is used when adding
   * constraints and costs for each solver.
   * @pre @p var is a indeterminate in the mathematical program,
   * otherwise this function throws a runtime error.
   */
  size_t FindIndeterminateIndex(const symbolic::Variable& var) const;

  /**
   * Gets the solution of an Eigen matrix of decision variables.
   * @tparam Derived An Eigen matrix containing Variable.
   * @param var The decision variables.
   * @return The value of the decision variable after solving the problem.
   */

  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>::type
  GetSolution(const Eigen::MatrixBase<Derived>& var) const {
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>
        value(var.rows(), var.cols());
    for (int i = 0; i < var.rows(); ++i) {
      for (int j = 0; j < var.cols(); ++j) {
        value(i, j) = GetSolution(var(i, j));
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
  template <typename C>
  Eigen::VectorXd EvalBindingAtSolution(const Binding<C>& binding) const {
    Eigen::VectorXd val(binding.constraint()->num_constraints());
    Eigen::VectorXd binding_var_vals = GetSolution(binding.variables());
    binding.constraint()->Eval(binding_var_vals, val);
    return val;
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

 private:
  // maps the ID of a symbolic variable to the index of the variable stored in
  // the optimization program.
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index_{};

  VectorXDecisionVariable decision_variables_;

  std::unordered_map<symbolic::Variable::Id, int> indeterminates_index_;
  VectorXIndeterminate indeterminates_;

  std::vector<Binding<Cost>> generic_costs_;
  std::vector<Binding<Constraint>> generic_constraints_;
  std::vector<Binding<QuadraticCost>> quadratic_costs_;
  std::vector<Binding<LinearCost>> linear_costs_;
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

  // TODO(fischergundlach) : remove member num_vars_ and use vars_.row()
  // instead.
  size_t num_vars_;
  Eigen::VectorXd x_initial_guess_;
  std::vector<double> x_values_;
  std::shared_ptr<SolverData> solver_data_;
  SolverType solver_type_;
  int solver_result_;
  double optimal_cost_;
  std::map<SolverType, std::map<std::string, double>> solver_options_double_;
  std::map<SolverType, std::map<std::string, int>> solver_options_int_;
  std::map<SolverType, std::map<std::string, std::string>> solver_options_str_;

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
      Eigen::Ref<MatrixXDecisionVariable> decision_variable_matrix) {
    switch (type) {
      case VarType::CONTINUOUS:
        break;
      case VarType::BINARY:
        required_capabilities_ |= kBinaryVariable;
        break;
      case VarType::INTEGER:
        throw std::runtime_error(
            "MathematicalProgram does not support integer variables yet.");
      case VarType::BOOLEAN:
        throw std::runtime_error(
            "MathematicalProgram does not support Boolean variables.");
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
    int row_index = 0;
    int col_index = 0;
    for (int i = 0; i < num_new_vars; ++i) {
      decision_variables_(num_vars_ + i) = symbolic::Variable(names[i], type);
      const size_t new_var_index = num_vars_ + i;
      decision_variable_index_.insert(std::pair<size_t, size_t>(
          decision_variables_(new_var_index).get_id(), new_var_index));
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
    x_initial_guess_.tail(num_new_vars)
        .fill(std::numeric_limits<double>::quiet_NaN());
  }

  MatrixXDecisionVariable NewVariables(VarType type, int rows, int cols,
                                       bool is_symmetric,
                                       const std::vector<std::string>& names);

  VectorXDecisionVariable NewVariables(VarType type, int rows,
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
  CheckIsDecisionVariable(const Eigen::MatrixBase<Derived>& vars) {
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

  // Adds a linear constraint represented by a set of symbolic formulas to the
  // program.
  //
  // Precondition: ∀ f ∈ formulas, is_relational(f).
  Binding<LinearConstraint> AddLinearConstraint(
      const std::set<symbolic::Formula>& formulas);

  // Adds a linear-equality constraint represented by a set of symbolic formulas
  // to the program.
  //
  // Precondition: ∀ f ∈ formulas, is_equal_to(f).
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const std::set<symbolic::Formula>& formulas);
};

}  // namespace solvers
}  // namespace drake
