// Copyright 2016. Alex Dunyak
#pragma once

extern "C" {
  #include <mosek/mosek.h>
}

#include <vector>
#include <string>

#include <Eigen/Sparse>
#include <Eigen/Core>

#include "drake/solvers/solution_result.h"
#include "drake/solvers/Optimization.h"
#include "drake/solvers/MathematicalProgram.h"

/**Definitions and program flow taken from
http://docs.mosek.com/7.1/capi/Linear_optimization.html

For further definition of terms, see
http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
*/
namespace drake {
namespace solvers {

/** MosekLP solves a linear program when given a correctly formatted program.
 *  Specifically, the program options:
 *  -- "maxormin" -- must be set to "max" or "min"
 *  -- "problemtype" -- must be set to "linear"
 *
 *  It is created by a MosekSolver object.
 */
class DRAKEOPTIMIZATION_EXPORT MosekLP {
  /** MosekLP
   *  @brief this class allows the creation and solution of a linear programming
   *  problem using the mosek solver.
   */
 public:
  MosekLP() {
    task_ = NULL;
    env_ = NULL;
  }

  /** Create a mosek linear programming environment using a constraint matrix
  * Takes the number of variables and constraints, the linear eqn to
  * optimize, the constraint matrix, and the constraint and variable bounds
  * @p environment is created and must be told to optimize.
  */
  MosekLP(int num_variables, int num_constraints,
      std::vector<double> equationScalars,
      Eigen::MatrixXd cons,
      std::vector<MSKboundkeye> mosek_constraint_bounds,
      std::vector<double> upper_constraint_bounds,
      std::vector<double> lower_constraint_bounds,
      std::vector<MSKboundkeye> mosek_variable_bounds,
      std::vector<double> upper_variable_bounds,
      std::vector<double> lower_variable_bounds);

  ~MosekLP() {
    if (task_ != NULL)
      MSK_deletetask(&task_);
    if (env_ != NULL)
      MSK_deleteenv(&env_);
  }


    /** Solve()
   * @brief optimizes variables in given linear constraints, works with either
   * of the two previous object declarations.
   * */
  SolutionResult Solve(OptimizationProblem &prog) const;

  std::vector<double>  GetSolution() const { return solutions_; }

  Eigen::VectorXd GetEigenVectorSolutions() const;

 private:
  /**The following names are consistent with the example programs given by
   *http://docs.mosek.com/7.1/capi/Linear_optimization.html
   *and by
   *http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
   *to ensure ease of translation and understanding.
   */

   /**AddLinearConstraintMatrix()
   *@brief adds linear constraints to mosek environment
   */
  void AddLinearConstraintMatrix(const Eigen::MatrixXd& cons_);

  /**addLinearConstraintSparseColumnMatrix()
  *@brief adds linear constraints in sparse column matrix form
  *just uses Eigen's sparse matrix library to implement expected format
  */
  void AddLinearConstraintSparseColumnMatrix(
    const Eigen::SparseMatrix<double>& sparsecons_);

  /**AddLinearConstraintBounds()
  *@brief bounds constraints, see http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
  *for details on how to set mosek_bounds_
  */
  void AddLinearConstraintBounds(const std::vector<MSKboundkeye>& mosek_bounds_,
      const std::vector<double>& upper_bounds,
      const std::vector<double>& lower_bounds);

  /**AddVariableBounds()
   * @brief bounds variables, see http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
   * for details on how to set mosek_bounds_
   */
  void AddVariableBounds(const std::vector<MSKboundkeye>& mosek_bounds,
                         const std::vector<double>& upper_bounds,
                         const std::vector<double>& lower_bounds);

  /**FindMosekBounds()
   * Given upper and lower bounds for a variable or constraint, finds the
   * equivalent Mosek bound keys.
   */
  std::vector<MSKboundkeye> FindMosekBounds(
      const std::vector<double>& upper_bounds,
      const std::vector<double>& lower_bounds) const;

  SolutionResult OptimizeTask(const std::string& maxormin,
                              const std::string& ptype);

  MSKint32t numvar_, numcon_;
  MSKenv_t env_;       // Internal environment, used to check if problem formed
                      // correctly
  MSKtask_t task_;     // internal definition of task
  MSKrescodee r_;      // used for validity checking
  std::vector<double> solutions_;  // Contains the solutions of the system
};

}  // namespace solvers
}  // namespace drake
