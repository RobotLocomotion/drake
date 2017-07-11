#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
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

  struct SolveStatusInfo {
      double reported_runtime;
      double current_objective;
      double best_objective;
      double best_bound;
      int explored_node_count;
      int feasible_solutions_count;
  };

  // Users can supply a callback to be called when the Gurobi solver
  // finds an intermediate solution node, which may not be feasible.
  // The user may supply a partial solution in the VectorXd and
  // VectorXDecisionVariable arguments that will be passed to Gurobi
  // as a candidate feasible solution. In that case, the user function
  // should assign the VectorXd& to be the values of the variable
  // assignments, and the VectorXDecisionVariable& to be the decision
  // variables being assigned.
  // See Gurobi reference manual for more detail on callbacks:
  // https://www.gurobi.com/documentation/7.0/refman/refman.html.
  typedef void (*mipNodeCallbackFunction)(const MathematicalProgram&,
    const SolveStatusInfo& callback_info, void *, Eigen::VectorXd&,
    VectorXDecisionVariable&);
  void addMIPNodeCallback(mipNodeCallbackFunction fnc, void * usrdata) {
    mip_node_callback_ = fnc;
    mip_node_callback_usrdata_ = usrdata;
  }
  // Users can supply a callback to be called when the Gurobi solver
  // finds a feasible solution.
  typedef void (*mipSolCallbackFunction)(const MathematicalProgram&,
    const SolveStatusInfo& callback_info, void *);
  void addMIPSolCallback(mipSolCallbackFunction fnc, void * usrdata) {
    mip_sol_callback_ = fnc;
    mip_sol_callback_usrdata_ = usrdata;
  }

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

 private:
  // Callbacks and generic user data to pass through,
  // or NULL if no callback has been supplied.
  mipNodeCallbackFunction mip_node_callback_ = NULL;
  mipSolCallbackFunction mip_sol_callback_ = NULL;
  void * mip_node_callback_usrdata_ = NULL;
  void * mip_sol_callback_usrdata_ = NULL;
};

}  // end namespace solvers
}  // end namespace drake
