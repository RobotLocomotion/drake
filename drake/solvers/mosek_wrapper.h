#pragma once

extern "C" {
  #include <mosek/mosek.h>
}

#include <vector>
#include <string>

#include <Eigen/Sparse>
#include <Eigen/Core>

#include "drake/solvers/solution_result.h"
#include "drake/solvers/mathematical_program.h"

/** Definitions and program flow taken from
`http://docs.mosek.com/7.1/capi/Linear_optimization.html`_

For further definition of terms, see
`http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html`_  **/
namespace drake {
namespace solvers {

/** This class allows the creation and solution of a linear programming
problem using the Mosek solver.
MosekWrapper solves a linear program when given a correctly formatted
program.  Specifically, the program options:
 - "maxormin" -- must be set to "max" or "min"
 - "problemtype" -- must be set to "linear", "quadratic", or "sdp"

 It is created by a MosekSolver object.
 Use the strictest constraint and variable bounds available, unbounded precision
 precision varies from bounded precision by about 10^-3.  **/
class DRAKEOPTIMIZATION_EXPORT MosekWrapper {
 public:
  /** Create a Mosek linear programming environment using a constraint matrix
  Takes the number of variables and constraints, the linear eqn to
  optimize, the constraint matrix, and the constraint and variable bounds.
  The Mosek environment is created and must be told to optimize.  **/
  MosekWrapper(int num_variables, int num_constraints,
    const std::vector<double>& equation_scalars,
    const Eigen::MatrixXd& linear_cons,
    const std::vector<MSKboundkeye>& mosek_constraint_bounds,
    const std::vector<double>& upper_constraint_bounds,
    const std::vector<double>& lower_constraint_bounds,
    const std::vector<MSKboundkeye>& mosek_variable_bounds,
    const std::vector<double>& upper_variable_bounds,
    const std::vector<double>& lower_variable_bounds,
    double constant_eqn_term);

  /** Create a Mosek environment for quadratic programming problems.  **/
  MosekWrapper(int num_variables, int num_constraints,
    const std::vector<double>& equation_scalars,
    const Eigen::MatrixXd& linear_cons,
    const std::vector<MSKboundkeye>& mosek_constraint_bounds,
    const std::vector<double>& upper_constraint_bounds,
    const std::vector<double>& lower_constraint_bounds,
    const std::vector<MSKboundkeye>& mosek_variable_bounds,
    const std::vector<double>& upper_variable_bounds,
    const std::vector<double>& lower_variable_bounds,
    double constant_eqn_term,
    const Eigen::MatrixXd& quad_objective,
    const Eigen::MatrixXd& quad_cons);

  /** Create a Mosek environment for SDP problems.  **/
  MosekWrapper(int num_variables, int num_constraints,
    const std::vector<MSKboundkeye>& mosek_constraint_bounds,
    const std::vector<double>& upper_constraint_bounds,
    const std::vector<double>& lower_constraint_bounds,
    const std::vector<MSKboundkeye>& mosek_variable_bounds,
    const std::vector<double>& upper_variable_bounds,
    const std::vector<double>& lower_variable_bounds,
    double constant_eqn_term,
    const SemidefiniteConstraint& sdp_objective,
    const std::vector<SemidefiniteConstraint>& sdp_constraints,
    const std::vector<int>& lorentz_cone_subscripts,
    int numbarvar);

  ~MosekWrapper() {
    if (task_ != NULL)
      MSK_deletetask(&task_);
    if (env_ != NULL)
      MSK_deleteenv(&env_);
  }

  /** Optimizes variables in given linear constraints, works with either
  of the two previous object declarations.  **/
  static SolutionResult Solve(MathematicalProgram &prog);

  std::vector<double> GetSolution() const { return solutions_; }

  Eigen::VectorXd GetEigenVectorSolutions() const;

 private:
  /** The following names are consistent with the example programs given by
  http://docs.mosek.com/7.1/capi/Linear_optimization.html
  and by
  http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
  to ensure ease of translation and understanding.  **/

  /** Adds linear constraints to mosek environment.  **/
  void AddLinearConstraintMatrix(const Eigen::MatrixXd& cons_);

  /** Adds linear constraints in sparse column matrix form.
  Just uses Eigen's sparse matrix library to implement expected format.  **/
  void AddLinearConstraintSparseColumnMatrix(
    const Eigen::SparseMatrix<double>& sparsecons_);

  /** Bounds constraints, see:
  `http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html`_
  for details on how to set mosek_bounds_  **/
  void AddLinearConstraintBounds(const std::vector<MSKboundkeye>& mosek_bounds_,
      const std::vector<double>& upper_bounds,
      const std::vector<double>& lower_bounds);

  /** Add a single quadratic matrix to a Mosek constraint.  **/
  void AddQuadraticConstraintMatrix(const Eigen::MatrixXd& cons);

  /** Adds a single quadratic matrix to a Mosek objective.  **/
  void AddQuadraticObjective(const Eigen::MatrixXd& obj);

  /** Adds a cone to Mosek for solving. Currently does not handle rotated
  cones.  **/
  void AppendLorentzCone(const std::vector<int>& sdp_cone_subscripts);

  /** Adds a single SDP objective to Mosek for solving, will not work if
  called multiple times.
  The mathematical formulation is: <pre>
  minimize the function
  sum(c_j * x_j) + Trace(C'*X) + c
  </pre>
  where x is contained in a cone, and X is a positive semidefinite matrix,
  subject to SDP constraints below.
  See: http://docs.mosek.com/7.1/capi/Semidefinite_optimization.html  **/
  void AddSDPObjective(const SemidefiniteConstraint& sdp_objective);

  /** Adds multiple SDP constraints to Mosek. Only call once per program.
  The mathematical formulation is to minimize the objective (above) with the
  constraints:  <pre>
  l_i <= sum(a_j * x_j) + Trace(A_i' * X_i) <= u_i
  </pre>
  See: http://docs.mosek.com/7.1/capi/Semidefinite_optimization.html  **/
  void AddSDPConstraints(
      const std::vector<SemidefiniteConstraint>& sdp_constraints);

  /** Binds variables, see http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
  for details on how to set mosek_bounds_  **/
  void AddVariableBounds(const std::vector<MSKboundkeye>& mosek_bounds,
                         const std::vector<double>& upper_bounds,
                         const std::vector<double>& lower_bounds);

  /** Given upper and lower bounds for a variable or constraint, finds the
  equivalent Mosek bound keys.  **/
  static std::vector<MSKboundkeye> FindMosekBounds(
      const std::vector<double>& upper_bounds,
      const std::vector<double>& lower_bounds);

  SolutionResult OptimizeTask(const std::string& maxormin,
                              const std::string& ptype);

  // NOTE: numbarvar only used for sdp.
  MSKint32t numvar_, numcon_, numbarvar_;
  MSKenv_t env_;       //< Internal environment, used to check if the problem
                       // is well-formed.
  MSKtask_t task_;     //< internal definition of task
  MSKrescodee result_;      //< used for validity checking
  std::vector<double> solutions_;  //< Contains the solutions of the system
};

}  // namespace solvers
}  // namespace drake
