#pragma once

#include <functional>
#include <string>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class GurobiSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GurobiSolver)

  GurobiSolver() = default;
  ~GurobiSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // Gurobi was available during compilation.
  bool available() const override;

  /// Contains info returned to a user function that handles
  /// a Node or Solution callback.
  /// @see MipNodeCallbackFunction
  /// @see MipSolCallbackFunction
  struct SolveStatusInfo {
    /// Runtime as of this callback.
    double reported_runtime{};
    /// Objective of current solution.
    double current_objective{};
    /// Objective of best solution yet.
    double best_objective{};
    /// Best known objective lower bound.
    double best_bound{};
    /// Number of nodes explored so far.
    int explored_node_count{};
    /// Number of feasible sols found so far.
    int feasible_solutions_count{};
  };

  /// Users can supply a callback to be called when the Gurobi solver
  /// finds an intermediate solution node, which may not be feasible.
  /// See Gurobi reference manual for more detail on callbacks:
  /// https://www.gurobi.com/documentation/7.0/refman/callback_codes.html.
  /// The user may supply a partial solution in the VectorXd and
  /// VectorXDecisionVariable arguments that will be passed to Gurobi
  /// as a candidate feasible solution.
  /// See gurobi_solver_test.cc for an example of using std::bind
  /// to create a callback of this signature, while allowing
  /// additional data to be passed through.
  /// @param MathematicalProgram& The optimization wrapper, whose
  /// current variable values (accessible via
  /// MathematicalProgram::GetSolution) will be set to the intermediate
  /// solution values.
  /// @param SolveStatusInfo& Intermediate solution status information values
  /// queried from Gurobi.
  /// @param VectorXd* User may assign this to be the values of the variable
  /// assignments.
  /// @param VectorXDecisionVariable* User may assign this to be the decision
  /// variables being assigned. Must have the same number of elements as
  /// the VectorXd assignment.
  typedef std::function<void(const MathematicalProgram&,
                             const SolveStatusInfo& callback_info,
                             Eigen::VectorXd*, VectorXDecisionVariable*)>
      MipNodeCallbackFunction;

  /// Registers a callback to be called at intermediate solutions
  /// during the solve.
  /// @param callback User callback function.
  /// @param user_data Arbitrary data that will be passed to the user
  /// callback function.
  void AddMipNodeCallback(const MipNodeCallbackFunction& callback) {
    mip_node_callback_ = callback;
  }

  /// Users can supply a callback to be called when the Gurobi solver
  /// finds a feasible solution.
  /// See Gurobi reference manual for more detail on callbacks:
  /// https://www.gurobi.com/documentation/7.0/refman/callback_codes.html.
  /// See gurobi_solver_test.cc for an example of using std::bind
  /// to create a callback of this signature, while allowing
  /// additional data to be passed through.
  /// @param MathematicalProgram& The optimization wrapper, whose
  /// current variable values (accessible via
  /// MathematicalProgram::GetSolution) will be set to the intermediate
  /// solution values.
  /// @param SolveStatusInfo& Intermediate solution status information values
  /// queried from Gurobi.
  /// @param void* Arbitrary data supplied during callback registration.
  typedef std::function<void(const MathematicalProgram&,
                             const SolveStatusInfo& callback_info)>
      MipSolCallbackFunction;

  /// Registers a callback to be called at feasible solutions
  /// during the solve.
  /// @param callback User callback function.
  /// @param usrdata Arbitrary data that will be passed to the user
  /// callback function.
  void AddMipSolCallback(const MipSolCallbackFunction& callback) {
    mip_sol_callback_ = callback;
  }

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

 private:
  // Callbacks and generic user data to pass through,
  // or NULL if no callback has been supplied.
  MipNodeCallbackFunction mip_node_callback_;
  MipSolCallbackFunction mip_sol_callback_;
};

}  // end namespace solvers
}  // end namespace drake
