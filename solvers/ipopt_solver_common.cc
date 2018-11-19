/* clang-format off to disable clang-format-includes */
#include "drake/solvers/ipopt_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

std::string to_string(IpoptSolverReturn status) {
  switch(status) {
    case IpoptSolverReturn::SUCCESS: {
      return "Success";
    }
    case IpoptSolverReturn::MAXITER_EXCEEDED : {
      return "Max iteration exceeded";
    }
    case IpoptSolverReturn::CPUTIME_EXCEEDED: {
      return "CPU time exceeded";
    }
    case IpoptSolverReturn::STOP_AT_TINY_STEP: {
      return "Stop at tiny step";
    }
    case IpoptSolverReturn::STOP_AT_ACCEPTABLE_POINT: {
      return "Stop at acceptable point";
    }
    case IpoptSolverReturn::LOCAL_INFEASIBILITY: {
      return "Local infeasibility";
    }
    case IpoptSolverReturn::USER_REQUESTED_STOP: {
      return "User requested stop";
    }
    case IpoptSolverReturn::FEASIBLE_POINT_FOUND: {
      return "Feasible point found";
    }
    case IpoptSolverReturn::DIVERGING_ITERATES: {
      return "Divergent iterates";
    }
    case IpoptSolverReturn::RESTORATION_FAILURE: {
      return "Restoration failure";
    }
    case IpoptSolverReturn::ERROR_IN_STEP_COMPUTATION: {
      return "Error in step computation";
    }
    case IpoptSolverReturn::INVALID_NUMBER_DETECTED: {
      return "Invalid number detected";
    }
    case IpoptSolverReturn::TOO_FEW_DEGREES_OF_FREEDOM: {
      return "Too few degrees of freedom";
    }
    case IpoptSolverReturn::INVALID_OPTION: {
      return "Invalid option";
    }
    case IpoptSolverReturn::OUT_OF_MEMORY: {
      return "Out of memory";
    }
    case IpoptSolverReturn::INTERNAL_ERROR: {
      return "Internal error";
    }
    case IpoptSolverReturn::UNASSIGNED: {
      return "Unassigned";
    }
    default : {
      throw std::runtime_error("Should not reach here.");
    }
  }
}

std::ostream& operator<<(std::ostream& os, IpoptSolverReturn solver_return) {
  os << to_string(solver_return);
  return os;
}

SolverId IpoptSolver::solver_id() const {
  return id();
}

SolverId IpoptSolver::id() {
  static const never_destroyed<SolverId> singleton{"IPOPT"};
  return singleton.access();
}

bool IpoptSolver::AreProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  return IpoptSolver::ProgramAttributesSatisfied(prog);
}

bool IpoptSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kGenericConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kQuadraticConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kGenericCost, ProgramAttribute::kLinearCost,
          ProgramAttribute::kQuadraticCost, ProgramAttribute::kCallback});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}
}  // namespace solvers
}  // namespace drake
